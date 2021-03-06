#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "sysemu/char.h"
#include "atlantronic_cpu.h"
#include "atlantronic_model.h"
#include "atlantronic_tools.h"
#include "atlantronic_can_motor.h"
#include "atlantronic_canopen.h"
#include "atlantronic_dynamixel.h"
#include "atlantronic_hokuyo.h"
#include "atlantronic_can.h"

#include "kernel/robot_parameters.h"
#include "kernel/rcc.h"

#define MODEL_DT           0.001f       //!< modele a 1ms
#define MODEL_PERIOD_TICK  (int)(RCC_SYSCLK*MODEL_DT)

#define PWM_NUM            4
#define ENCODER_NUM        3
#define CAN_MOTOR_NUM      6
#define AX12_NUM           6
#define RX24_NUM           6
#define HOKUYO_NUM         2

enum
{
	EVENT_CLOCK_FACTOR = 1,
	EVENT_NEW_OBJECT,
	EVENT_MOVE_OBJECT,
	EVENT_MANAGE_CANOPEN_NODE,
};

enum
{
	EVENT_MANAGE_CANOPEN_NODE_CONNECT,
	EVENT_MANAGE_CANOPEN_NODE_DISCONNECT,
};

static const float steering_coupling[3] = { DRIVING1_WHEEL_RADIUS, DRIVING2_WHEEL_RADIUS, DRIVING3_WHEEL_RADIUS};

struct atlantronic_model_rx_event
{
	uint32_t type;        //!< type
	union
	{
		uint8_t data[256];     //!< données
		uint32_t data32[64];   //!< données
	};
};

struct atlantronic_motor
{
	float pwm;      //!< pwm
	float gain_pwm; //!< gain entre la pwm et u (V)
	float f;        //!< coefficient de frottement dynamique du moteur
	float r;        //!< résistance du moteur Ohm
	float j;        //!< moment d'inertie du robot par rapport à l'axe de rotation du moteur kgm²
	float k;        //!< constante du moteur en Nm/A
	float l;        //!< inductance du moteur en H
	float cp;       //!< couple de pertes (frottements de la roue sur le sol + frottements / transmission)
	float red;      //!< reducteur
	float i_max;    //!< courant maximal (A)

	int block;     //!< bloquage du moteur

	float i_old;         //!< intensite du moteur du cycle precedent (A)
	float w_old;         //!< vitesse de rotation du moteur (avant reducteur) du cycle precedent
	float theta_old;     //!< angle du moteur (après reducteur) du cycle precedent

	float i;             //!< intensite du moteur (A)
	float w;             //!< vitesse de rotation du moteur (avant reducteur)
	float theta;         //!< angle du moteur (après reducteur)
};

struct atlantronic_model_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq[MODEL_IRQ_OUT_NUM];
	CharDriverState* chr;
	QEMUTimer* timer;
	uint64_t timer_count;
	float encoder[ENCODER_NUM];
	struct can_msg can_msg;
	struct atlantronic_canopen canopen;
	struct atlantronic_can_motor can_motor[CAN_MOTOR_NUM];
	int pwm_dir[PWM_NUM];
	struct atlantronic_motor motor[PWM_NUM];

	struct atlantronic_hokuyo_state hokuyo[HOKUYO_NUM];
	struct atlantronic_dynamixel_state ax12[AX12_NUM];
	struct atlantronic_dynamixel_state rx24[RX24_NUM];

	struct atlantronic_vect3 pos_robot;
	struct atlantronic_vect3 npSpeed;
};

static void atlantronic_motor_init(struct atlantronic_motor* motor)
{
	motor->pwm  = 0;
	motor->gain_pwm = 24.0f;
	motor->f  = 0;
	motor->r  = 1.11;
	motor->j  = 0.0001;
	motor->k  = 0.0364;
	motor->l  = 201e-6;
	motor->cp = 0;
	motor->red = 1 / 21.0f;
	motor->i_max = 2.25;

	motor->block = 0;

	motor->i_old = 0;
	motor->theta_old = 0;
	motor->w_old = 0;

	motor->i = 0;
	motor->theta = 0;
	motor->w = 0;
}
#if 0
//! Calcule la dérivée de l'état du moteur à partir de l'état et des paramètres du moteur
//! La dérivée est calculée par unité de temps (dt = 1)
//!
//! @param x etat du moteur (i, w, theta)
//! @param dx réponse : dérivée de l'état par unité de temps
static void atlantronic_motor_compute_dx(struct atlantronic_motor* motor, double *x, double* dx)
{
	// di/dt = ( u - ri - kw ) / L
	dx[0] = ( motor->gain_pwm * motor->pwm - motor->r * x[0] - motor->k * x[1]) / motor->l;

	// dw/dt = (ki - cp - fw) / J (ou 0 en cas de bloquage)
	dx[1] = (motor->k * x[0] - motor->cp - motor->f * x[1]) / motor->j;
	if( motor->block )
	{
		dx[1] = 0;
	}

	// dtheta/dt = w
	dx[2] = motor->red * x[1];
}

static void atlantronic_motor_update(struct atlantronic_motor* motor)
{
	int i = 0;
	int j = 0;
	double te = 0.1f / CONTROL_HZ;
	double X[3] = { motor->i, motor->w, motor->theta};
	double x[3];
	double k1[3];
	double k2[3];
	double k3[3];
	double k4[3];

	motor->i_old = motor->i;
	motor->w_old = motor->w;
	motor->theta_old = motor->theta;

	// bloquage du moteur
	if( motor->block )
	{
		X[1] = 0;
	}

	for(i = 0; i < 10; i++)
	{
		// intégration runge-kutta 4
		atlantronic_motor_compute_dx(motor, X, k1);
		for(j=0; j<3; j++)
		{
			x[j] = X[j] + k1[j] * te / 2;
			x[0] = sat(x[0], -motor->i_max, motor->i_max);
		}

		atlantronic_motor_compute_dx(motor, x, k2);
		for(j=0; j<3; j++)
		{
			x[j] = X[j] + k2[j] * te / 2;
			x[0] = sat(x[0], -motor->i_max, motor->i_max);
		}

		atlantronic_motor_compute_dx(motor, x, k3);
		for(j=0; j<3; j++)
		{
			x[j] = X[j] + k3[j] * te;
			x[0] = sat(x[0], -motor->i_max, motor->i_max);
		}

		atlantronic_motor_compute_dx(motor, x, k4);
		for(j=0; j<3; j++)
		{
			X[j] += (k1[j] + 2* k2[j] + 2*k3[j] + k4[j]) * te / 6.0f;
			X[0] = sat(X[0], -motor->i_max, motor->i_max);
		}
	}

	motor->i = X[0];
	motor->w = X[1];
	motor->theta = fmod(X[2], 2 * M_PI * 65536 / (1<< PARAM_ENCODERS_BIT_RES));
}

static void atlantronic_motor_cancel_update(struct atlantronic_motor* motor)
{
	motor->i = motor->i_old;
	motor->w = motor->w_old;
	motor->theta = motor->theta_old;
}
#endif
static void atlantronic_model_reset(struct atlantronic_model_state* s)
{
	int i = 0;

	s->pos_robot.x = 0;
	s->pos_robot.y = 700;
	s->pos_robot.theta = - M_PI / 2;

	s->npSpeed.x = 0;
	s->npSpeed.y = 0;
	s->npSpeed.theta = 0;

	atlantronic_canopen_init(&s->canopen, &s->irq[MODEL_IRQ_OUT_CAN1_MSG_ID], &s->irq[MODEL_IRQ_OUT_CAN1_MSG_SIZE],
			&s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_L], &s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_H]);

	for(i = 0; i < AX12_NUM; i++)
	{
		atlantronic_dynamixel_init(&s->ax12[i], &s->irq[MODEL_IRQ_OUT_USART_AX12], 2+i, DYNAMIXEL_AX12);
	}

	for(i = 0; i < RX24_NUM; i++)
	{
		atlantronic_dynamixel_init(&s->rx24[i], &s->irq[MODEL_IRQ_OUT_USART_RX24], 2+i, DYNAMIXEL_RX24F);
	}

	struct atlantronic_vect3 pos_hokuto = { 0, 0, 0}; // TODO
	for(i = 0; i < HOKUYO_NUM; i++)
	{
		atlantronic_hokuyo_init(&s->hokuyo[i], &s->irq[MODEL_IRQ_OUT_USART_HOKUYO1+i], pos_hokuto);
	}

	for(i = 0; i < ENCODER_NUM; i++)
	{
		s->encoder[i] = 0;
	}


	for(i = 0; i < PWM_NUM; i++)
	{
		s->pwm_dir[i] = 0;
		atlantronic_motor_init(&s->motor[i]);
	}

	float outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
	atlantronic_can_motor_init(&s->can_motor[0], outputGain, 0);
	atlantronic_canopen_register_node(&s->canopen, 0x31, &s->can_motor[0].node, atlantronic_can_motor_callback);

	outputGain = 2 * M_PI / (float)(MOTOR_STEERING1_RED * MOTOR_ENCODER_RESOLUTION);
	atlantronic_can_motor_init(&s->can_motor[1], outputGain, MOTOR_STEERING1_OFFSET);
	atlantronic_canopen_register_node(&s->canopen, 0x21, &s->can_motor[1].node, atlantronic_can_motor_callback);

	outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING2_RED);
	atlantronic_can_motor_init(&s->can_motor[2], outputGain, 0);
	atlantronic_canopen_register_node(&s->canopen, 0x32, &s->can_motor[2].node, atlantronic_can_motor_callback);

	outputGain = 2 * M_PI / (float)(MOTOR_STEERING2_RED * MOTOR_ENCODER_RESOLUTION);
	atlantronic_can_motor_init(&s->can_motor[3], outputGain, MOTOR_STEERING2_OFFSET);
	atlantronic_canopen_register_node(&s->canopen, 0x22, &s->can_motor[3].node, atlantronic_can_motor_callback);

	outputGain = 2 * M_PI * DRIVING3_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING3_RED);
	atlantronic_can_motor_init(&s->can_motor[4], outputGain, 0);
	atlantronic_canopen_register_node(&s->canopen, 0x33, &s->can_motor[4].node, atlantronic_can_motor_callback);

	outputGain = 2 * M_PI / (float)(MOTOR_STEERING3_RED * MOTOR_ENCODER_RESOLUTION);
	atlantronic_can_motor_init(&s->can_motor[5], outputGain, MOTOR_STEERING3_OFFSET);
	atlantronic_canopen_register_node(&s->canopen, 0x23, &s->can_motor[5].node, atlantronic_can_motor_callback);
}

#if 0
static void atlantronic_model_compute(struct atlantronic_model_state* s)
{
	int i = 0;

	// test avec moteurs non bloques
	s->motor[0].block = 0;
	s->motor[1].block = 0;
	atlantronic_motor_update(&s->motor[0]);
	atlantronic_motor_update(&s->motor[1]);

	float v1 = s->motor[0].w * s->motor[0].red * PARAM_RIGHT_ODO_WHEEL_RADIUS_FX / 65536.0f;
	float v2 = s->motor[1].w * s->motor[1].red * PARAM_LEFT_ODO_WHEEL_RADIUS_FX / 65536.0f;
	struct atlantronic_vect3 pos_new = s->pos;

	s->v = (v1 + v2) / 2;
	s->w = ((v1 - v2) * PARAM_INVERTED_VOIE_FX39) / ((uint64_t)1 << 39);
	pos_new.x += s->v * s->pos.ca / CONTROL_HZ;
	pos_new.y += s->v * s->pos.sa / CONTROL_HZ;
	pos_new.alpha += s->w / CONTROL_HZ;
	pos_new.ca = cos(pos_new.alpha);
	pos_new.sa = sin(pos_new.alpha);

#if 0
	if( fabs(s->v) > 0.01 || fabs(s->w) > 0.00001)
	{
		printf("%.2f %.2f %.2f v %.2f w %f\n", s->pos.x, s->pos.y, s->pos.alpha, s->v, s->w);
	}
#endif

	struct atlantronic_vect2 corner_abs_old[CORNER_NUM];
	struct atlantronic_vect2 corner_abs_new[CORNER_NUM];
	int res = -1;

	for(i = 0; i < CORNER_NUM && res; i++)
	{
		int j = 0;
		atlantronic_vect2_loc_to_abs(&s->pos, &corner_loc[i], &corner_abs_old[i]);
		atlantronic_vect2_loc_to_abs(&pos_new, &corner_loc[i], &corner_abs_new[i]);
		struct atlantronic_vect2 h;

		for(j = 0; j < atlantronic_static_obj_count && res ; j++)
		{
			int k = 0;
			for(k = 0; k < atlantronic_static_obj[j].size - 1 && res ; k++)
			{
				res = atlantronic_segment_intersection(atlantronic_static_obj[j].pt[k], atlantronic_static_obj[j].pt[k+1], corner_abs_old[i], corner_abs_new[i], &h);
			}
		}
	}

	if( ! res )
	{
		s->motor[0].block = 1;
		s->motor[1].block = 1;

		// bloquage d'un des moteurs, on refait le calcul
		atlantronic_motor_cancel_update(&s->motor[0]);
		atlantronic_motor_cancel_update(&s->motor[1]);
		atlantronic_motor_update(&s->motor[0]);
		atlantronic_motor_update(&s->motor[1]);

		pos_new = s->pos;
		s->v = 0;
		s->w = 0;
	}

	s->pos = pos_new;
	s->enc[0] = s->motor[0].theta * (1<< PARAM_ENCODERS_BIT_RES) / (2 * M_PI);
	s->enc[0] -= (floor((s->enc[0] - 65536) / 65536) + 1 ) * 65536;
	s->enc[1] = s->motor[1].theta * (1<< PARAM_ENCODERS_BIT_RES) / (2 * M_PI);
	s->enc[1] -= (floor((s->enc[1] - 65536) / 65536) + 1 ) * 65536;

	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER_RIGHT], ((int32_t) s->enc[0])&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER_LEFT], ((int32_t) s->enc[1])&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_I_RIGHT], ((int32_t) fabs(s->motor[0].i / s->motor[0].i_max * 65536))&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_I_LEFT], ((int32_t) fabs(s->motor[1].i / s->motor[1].i_max * 65536))&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_X], (int32_t)(s->pos.x * 65536.0f));
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_Y], (int32_t)(s->pos.y * 65536.0f));
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ALPHA], (int32_t)(s->pos.alpha / (2 * M_PI) * (1<<26)));
}
#endif
static void atlantronic_model_in_recv(void * opaque, int numPin, int level)
{
	struct atlantronic_model_state *s = opaque;
	int i = 0;
	int index;

	switch(numPin)
	{
		case MODEL_IRQ_IN_CAN1_MSG_ID:
			s->can_msg.id = level;
			atlantronic_canopen_tx(&s->canopen, s->can_msg);
			break;
		case MODEL_IRQ_IN_CAN1_MSG_SIZE:
			s->can_msg.size = level;
			break;
		case MODEL_IRQ_IN_CAN1_MSG_DATA_L:
			s->can_msg._data.low = level;
			break;
		case MODEL_IRQ_IN_CAN1_MSG_DATA_H:
			s->can_msg._data.high = level;
			break;
		case MODEL_IRQ_IN_ENCODER1:
		case MODEL_IRQ_IN_ENCODER2:
		case MODEL_IRQ_IN_ENCODER3:
			index = numPin - MODEL_IRQ_IN_ENCODER1;
			if( index < ENCODER_NUM && index >= 0)
			{
				s->encoder[index] = level;
			}
			break;
		case MODEL_IRQ_IN_PWM1:
		case MODEL_IRQ_IN_PWM2:
		case MODEL_IRQ_IN_PWM3:
		case MODEL_IRQ_IN_PWM4:
			index = numPin - MODEL_IRQ_IN_PWM1;
			if( index < PWM_NUM && index >= 0)
			{
				// mise a jour du moteur i
				if( ! s->pwm_dir[index])
				{
					level = -level;
				}

				s->motor[index].pwm = level / 65536.0f;
			}
			break;
		case MODEL_IRQ_IN_PWM_DIR1:
		case MODEL_IRQ_IN_PWM_DIR2:
		case MODEL_IRQ_IN_PWM_DIR3:
		case MODEL_IRQ_IN_PWM_DIR4:
			index = numPin - MODEL_IRQ_IN_PWM_DIR1;
			if( index < PWM_NUM && index >= 0)
			{
				s->pwm_dir[index] = level;
			}
			break;
		case MODEL_IRQ_IN_USART_AX12:
			for(i=0; i < AX12_NUM; i++)
			{
				atlantronic_dynamixel_in_recv_usart(&s->ax12[i], level&0xff);
			}
			break;
		case MODEL_IRQ_IN_USART_RX24:
			for(i=0; i < RX24_NUM; i++)
			{
				atlantronic_dynamixel_in_recv_usart(&s->rx24[i], level&0xff);
			}
			break;
		case MODEL_IRQ_IN_USART_HOKUYO1:
		case MODEL_IRQ_IN_USART_HOKUYO2:
			index = numPin - MODEL_IRQ_IN_USART_HOKUYO1;
			if( index < HOKUYO_NUM && index >= 0)
			{
				atlantronic_hokuyo_in_recv_usart(&s->hokuyo[index], level&0xff);
			}
			break;
	}
}

static int atlantronic_model_can_receive(void *opaque)
{
	return sizeof(struct atlantronic_model_rx_event);
}

static void atlantronic_model_receive(void *opaque, const uint8_t* buf, int size)
{
	struct atlantronic_model_state* model = opaque;
	struct atlantronic_model_rx_event* event = (struct atlantronic_model_rx_event*) buf;

	if(size != sizeof(struct atlantronic_model_rx_event))
	{
		hw_error("atlantronic_model_receive - incorrect size");
	}

	switch(event->type)
	{
		case EVENT_CLOCK_FACTOR:
			if(event->data32[0] > 0)
			{
				system_clock_scale = event->data32[0];
			}
			else
			{
				system_clock_scale = INT_MAX/2;
			}
			char buffer[256];
			sprintf(buffer, "%d", event->data32[1]);
			configure_icount(buffer);
			break;
		case EVENT_NEW_OBJECT:
			atlantronic_add_object(MIN(event->data[0], (sizeof(event->data)-1)/sizeof(struct atlantronic_vect2)), (struct atlantronic_vect2*)&event->data[1]);
			break;
		case EVENT_MOVE_OBJECT:
			atlantronic_move_object(event->data[0], *((struct atlantronic_vect2*)&event->data[1]), *((struct atlantronic_vect3*)&event->data[1+sizeof(struct atlantronic_vect2)]));
			break;
		case EVENT_MANAGE_CANOPEN_NODE:
			atlantronic_canopen_manage_node_connextion(&model->canopen, event->data32[0], event->data32[1] == EVENT_MANAGE_CANOPEN_NODE_CONNECT ? 1 : 0);
			break;
	}
}

static void atlantronic_model_event(void *opaque, int event)
{

}

static struct atlantronic_vect3 odometry2turret(const struct atlantronic_vect3* cp, const struct atlantronic_vect3* A, const struct atlantronic_vect3* B, const struct atlantronic_vect3* v1, const struct atlantronic_vect3* v2, float* slippageSpeed)
{
	struct atlantronic_vect3 res = {0, 0, 0};

	float dx = B->x - A->x;
	float dy = B->y - A->y;
	float dv = 0;

	// on divise par le plus grand pour eviter les pb numeriques
	if( fabsf(dx) > fabsf(dy) )
	{
		res.theta = (v2->y - v1->y) / dx;
		dv = fabsf(v1->x - v2->x - dy * res.theta);
	}
	else if( fabsf(dy) > 0 )
	{
		res.theta = (v1->x - v2->x) / dy;
		dv = fabsf(v2->y - v1->y - dx * res.theta);
	}
	else
	{
		// calcul non realisable, A et B sont au même endroit
		// on retourne une vitesse nulle
		goto end;
	}

	res.x = 0.5 * (v1->x + v2->x + res.theta * (A->y + B->y - 2 * cp->y));
	res.y = 0.5 * (v1->y + v2->y + res.theta * ( 2 * cp->x - A->x - B->x));

end:
	if( slippageSpeed )
	{
		*slippageSpeed = dv;
	}

	return res;
}

static void atlantronic_model_update_odometry(struct atlantronic_model_state *s, float dt)
{
	struct atlantronic_vect3 turret[3] =
	{
		{   0,  155, 0},
		{   0, -155, 0},
		{-175,    0, 0}
	};

	int i = 0;
	struct atlantronic_vect3 cp = {0,0,0};
	struct atlantronic_vect3 v[3];

	for(i = 0; i < 3; i++)
	{
		v[i].theta = s->can_motor[2*i+1].pos;
		float speed = s->can_motor[2*i].v - steering_coupling[i] * s->can_motor[2*i+1].v;
		v[i].x = speed * cosf(v[i].theta);
		v[i].y = speed * sinf(v[i].theta);
	}

	struct atlantronic_vect3 npSpeed1 = odometry2turret(&cp, &turret[0], &turret[1], &v[0], &v[1], NULL);
	struct atlantronic_vect3 npSpeed2 = odometry2turret(&cp, &turret[1], &turret[2], &v[1], &v[2], NULL);
	struct atlantronic_vect3 npSpeed3 = odometry2turret(&cp, &turret[0], &turret[2], &v[0], &v[2], NULL);

	s->npSpeed.x = (npSpeed1.x + npSpeed2.x + npSpeed3.x) / 3;
	s->npSpeed.y = (npSpeed1.y + npSpeed2.y + npSpeed3.y) / 3;
	s->npSpeed.theta = (npSpeed1.theta + npSpeed2.theta + npSpeed3.theta) / 3;

	struct atlantronic_vect3 npSpeedAbs = atlantronic_vect3_loc_to_abs_speed(s->pos_robot.theta, &s->npSpeed);

	s->pos_robot.x += npSpeedAbs.x * dt;
	s->pos_robot.y += npSpeedAbs.y * dt;
	s->pos_robot.theta += npSpeedAbs.theta * dt;
}

static void atlantronic_model_timer_cb(void* arg)
{
	struct atlantronic_model_state *s = arg;
	int i = 0;
	float dt = MODEL_DT/system_clock_scale;

	s->timer_count += MODEL_PERIOD_TICK;
	timer_mod(s->timer, s->timer_count);

	// mise a jour des moteurs
	for(i = 0; i < CAN_MOTOR_NUM; i++)
	{
		atlantronic_can_motor_update(&s->can_motor[i], dt);
	}

	atlantronic_model_update_odometry(s, dt);
	for(i = 0; i < HOKUYO_NUM; i++)
	{
		s->hokuyo[i].pos_robot = s->pos_robot;
	}
}

static int atlantronic_model_init(SysBusDevice * dev)
{
    struct atlantronic_model_state *s = OBJECT_CHECK(struct atlantronic_model_state, dev, "atlantronic-model");

	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, MODEL_IRQ_OUT_NUM);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_model_in_recv, MODEL_IRQ_IN_NUM);

	s->chr = qemu_chr_find("foo_model");
	if( s->chr == NULL)
	{
		hw_error("chardev foo_model not found");
	}
	else
	{
		qemu_chr_add_handlers(s->chr, atlantronic_model_can_receive, atlantronic_model_receive, atlantronic_model_event, s);
	}

	atlantronic_model_reset(s);

	s->timer = timer_new(QEMU_CLOCK_VIRTUAL, 1, atlantronic_model_timer_cb, s);
	s->timer_count = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MODEL_PERIOD_TICK;
	timer_mod(s->timer, s->timer_count);

    return 0;
}

static void atlantronic_model_class_init(ObjectClass *klass, void *data)
{
	SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

	sdc->init = atlantronic_model_init;
}

static TypeInfo atlantronic_model_info =
{
	.name          = "atlantronic-model",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(struct atlantronic_model_state),
	.class_init    = atlantronic_model_class_init,
};

static void atlantronic_model_register_types(void)
{
	type_register_static(&atlantronic_model_info);
}

type_init(atlantronic_model_register_types);
