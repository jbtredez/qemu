#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "sysemu/char.h"
#include "atlantronic_cpu.h"
#include "atlantronic_model.h"
#include "atlantronic_tools.h"
#include "atlantronic_can_motor_mip.h"
#include "atlantronic_dynamixel.h"
#include "atlantronic_hokuyo.h"
#include "atlantronic_can.h"

#define __disco__
#include "kernel/robot_parameters.h"
#include "kernel/rcc.h"
#define LINUX
#include "kernel/driver/io.h"
#undef LINUX

#define MODEL_DT           0.001f       //!< modele a 1ms
#define MODEL_PERIOD_TICK  (int)(RCC_SYSCLK*MODEL_DT)

#define PWM_NUM            4
#define ENCODER_NUM        3
#define CAN_MOTOR_NUM      2
#define AX12_NUM           8
#define RX24_NUM           6
#define HOKUYO_NUM         2

enum
{
	EVENT_CLOCK_FACTOR = 1,
	EVENT_NEW_OBJECT,
	EVENT_MOVE_OBJECT,
	EVENT_MANAGE_CANOPEN_NODE,
	EVENT_SET_IO,
};

enum
{
	EVENT_MANAGE_CANOPEN_NODE_CONNECT,
	EVENT_MANAGE_CANOPEN_NODE_DISCONNECT,
};

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
	struct atlantronic_can_bus can;
	struct can_msg can_msg;
	struct atlantronic_can_motor_mip can_motor[CAN_MOTOR_NUM];
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

	s->pos_robot.x = -1300;
	s->pos_robot.y = 0;
	s->pos_robot.theta = 0;

	s->npSpeed.x = 0;
	s->npSpeed.y = 0;
	s->npSpeed.theta = 0;

	s->can.irq_id = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_ID];
	s->can.irq_size = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_SIZE];
	s->can.irq_data_h = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_H];
	s->can.irq_data_l = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_L];

	for(i = 0; i < AX12_NUM; i++)
	{
		atlantronic_dynamixel_init(&s->ax12[i], &s->irq[MODEL_IRQ_OUT_USART_AX12], 2+i, DYNAMIXEL_AX12);
	}

	for(i = 0; i < RX24_NUM; i++)
	{
		atlantronic_dynamixel_init(&s->rx24[i], &s->irq[MODEL_IRQ_OUT_USART_RX24], 2+i, DYNAMIXEL_RX24F);
	}

	struct atlantronic_vect3 pos_hokuyo[HOKUYO_NUM];
	pos_hokuyo[0].x = 0;
	pos_hokuyo[0].y = 0;
	pos_hokuyo[0].theta = 0;
	pos_hokuyo[1].x = 0;
	pos_hokuyo[1].y = 0;
	pos_hokuyo[1].theta = 0;
	for(i = 0; i < HOKUYO_NUM; i++)
	{
		atlantronic_hokuyo_init(&s->hokuyo[i], &s->irq[MODEL_IRQ_OUT_USART_HOKUYO1+i], pos_hokuyo[i]);
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

	float outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MIP_MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
	atlantronic_can_motor_mip_init(&s->can_motor[0], 1, outputGain, 0, &s->can);

	outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MIP_MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING2_RED);
	atlantronic_can_motor_mip_init(&s->can_motor[1], 2, outputGain, 0, &s->can);
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
			for(i = 0; i < CAN_MOTOR_NUM; i++)
			{
				atlantronic_can_motor_mip_rx(&s->can_motor[i], s->can_msg);
			}
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
		case MODEL_IRQ_IN_LCD:
			// TODO decoder trame lcd + envoi ihm
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
//			atlantronic_canopen_manage_node_connextion(&model->canopen, event->data32[0], event->data32[1] == EVENT_MANAGE_CANOPEN_NODE_CONNECT ? 1 : 0);
			break;
		case EVENT_SET_IO:
			{
				if(event->data32[0] & GPIO_MASK_0) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_0], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_1) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_1], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_2) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_2], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_3) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_3], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_4) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_4], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_5) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_5], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_6) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_6], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_7) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_7], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_8) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_8], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_9) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_9], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_10) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_10], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_11) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_11], event->data32[1]);
				if(event->data32[0] & GPIO_MASK_GO) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_GO], event->data32[1]);
			}
			break;
	}
}

static void atlantronic_model_event(void *opaque, int event)
{

}

static void atlantronic_model_update_odometry(struct atlantronic_model_state *s, float dt)
{
	s->npSpeed.x = 0.5 * (s->can_motor[0].v + s->can_motor[1].v);
	s->npSpeed.y = 0;
	s->npSpeed.theta = (s->can_motor[1].v - s->can_motor[0].v) * VOIE_MOT_INV;
	struct atlantronic_vect3 npSpeedAbs = atlantronic_vect3_loc_to_abs_speed(s->pos_robot.theta, &s->npSpeed);

	s->pos_robot.x += npSpeedAbs.x * dt;
	s->pos_robot.y += npSpeedAbs.y * dt;
	s->pos_robot.theta += npSpeedAbs.theta * dt;

	float v[2];
	v[0] = s->npSpeed.x - 0.5 * VOIE_ODO * s->npSpeed.theta;
	v[1] = s->npSpeed.x + 0.5 * VOIE_ODO * s->npSpeed.theta;

	s->encoder[0] += v[0] * dt * ODO_ENCODER_RESOLUTION / (2 * M_PI * ODO1_WHEEL_RADIUS) ;
	s->encoder[0] -= (floor((s->encoder[0] - 65536) / 65536) + 1 ) * 65536;
	s->encoder[1] += v[1] * dt * ODO_ENCODER_RESOLUTION / (2 * M_PI * ODO2_WHEEL_RADIUS) ;
	s->encoder[1] -= (floor((s->encoder[1] - 65536) / 65536) + 1 ) * 65536;
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER1], ((int32_t) s->encoder[0])&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER2], ((int32_t) s->encoder[1])&0xffff );

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
		atlantronic_can_motor_mip_update(&s->can_motor[i], dt);
	}

	// mise a jour des dynamixels
	for(i = 0; i < AX12_NUM; i++)
	{
		atlantronic_dynamixel_update(&s->ax12[i], dt);
	}
	for(i = 0; i < RX24_NUM; i++)
	{
		atlantronic_dynamixel_update(&s->rx24[i], dt);
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
