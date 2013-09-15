#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "sysemu/char.h"

#define LINUX
#define STM32F10X_CL
#undef FALSE
#undef TRUE
#undef bool
#include "kernel/cpu/cpu.h"
#undef LINUX

#include "kernel/driver/usb/stm32f1xx/otgd_fs_regs.h"
#include "atlantronic_model.h"
#include "atlantronic_tools.h"

#include "kernel/robot_parameters.h"
#include "foo/control/control.h"

#define IRQ_IN_NUM        10
#define PWM_NUM            4
#define ENCODER_NUM        2
#define MODEL_FACTOR      10      //!< calcul du modele a 10x la frequence d'utilisation

#define EVENT_CLOCK_FACTOR         1
#define EVENT_NEW_OBJECT           2
#define EVENT_MOVE_OBJECT          3

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

	bool block;     //!< bloquage du moteur

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
	int32_t pwm_dir[PWM_NUM];
	float enc[ENCODER_NUM];
	struct atlantronic_motor motor[PWM_NUM];

	struct atlantronic_vect3 pos;
	float v;           //!< vitesse linéaire du robot (mm/s)
	float w;           //!< vitesse angulaire du robot (rd/s)
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

	motor->block = false;

	motor->i_old = 0;
	motor->theta_old = 0;
	motor->w_old = 0;

	motor->i = 0;
	motor->theta = 0;
	motor->w = 0;
}

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

static void atlantronic_model_reset(struct atlantronic_model_state* s)
{
	int i = 0;
	for(i = 0; i < PWM_NUM; i++)
	{
		s->pwm_dir[i] = 0;
		atlantronic_motor_init(&s->motor[i]);
	}

	for(i = 0; i < ENCODER_NUM; i++)
	{
		s->enc[i] = 0;
	}

	s->pos.x = 0;
	s->pos.y = 700;
	s->pos.alpha = - M_PI / 2;
	s->pos.ca = cos(s->pos.alpha);
	s->pos.sa = sin(s->pos.alpha);
	s->v = 0;
	s->w = 0;
}

static void atlantronic_model_compute(struct atlantronic_model_state* s)
{
	int i = 0;

	// test avec moteurs non bloques
	s->motor[0].block = false;
	s->motor[1].block = false;
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
		s->motor[0].block = true;
		s->motor[1].block = true;

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

static void atlantronic_model_in_recv(void * opaque, int numPin, int level)
{
    struct atlantronic_model_state *s = opaque;

	if(numPin < ENCODER_NUM)
	{
		s->enc[numPin] = level;
	}
	else if(numPin < PWM_NUM + ENCODER_NUM)
	{
		int id = numPin - ENCODER_NUM;
		switch(id)
		{
			case 0:
			case 1:
				if( ! s->pwm_dir[id])
				{
					level = -level;
				}

				s->motor[id].pwm = level / 65536.0f;

				// dans le code, c'est la derniere pwm..., on en profite pour calculer le modele cinematique
				if(id == 1)
				{
					atlantronic_model_compute(s);
				}
				break;
			case 2:
			case 3:
				//	qemu_set_irq(s->irq[MODEL_IRQ_I_MOT3], ((int32_t) fabs(s->motor[1].i / s->motor[1].i_max * 65536))&0xffff );
				//	qemu_set_irq(s->irq[MODEL_IRQ_I_MOT4], ((int32_t) fabs(s->motor[1].i / s->motor[1].i_max * 65536))&0xffff );
			default:
				break;
		}
	}
	else if(numPin < 2 * PWM_NUM + ENCODER_NUM)
	{
		int id = numPin - PWM_NUM - ENCODER_NUM;
		s->pwm_dir[id] = level;
	}
}

static int atlantronic_model_can_receive(void *opaque)
{
	return sizeof(struct atlantronic_model_rx_event);
}

static void atlantronic_model_receive(void *opaque, const uint8_t* buf, int size)
{
//	struct atlantronic_model_state *model = opaque;
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
			break;
		case EVENT_NEW_OBJECT:
			atlantronic_add_object(MIN(event->data[0], (sizeof(event->data)-1)/sizeof(struct atlantronic_vect2)), (struct atlantronic_vect2*)&event->data[1]);
			break;
		case EVENT_MOVE_OBJECT:
			atlantronic_move_object(event->data[0], *((struct atlantronic_vect2*)&event->data[1]), *((struct atlantronic_vect3*)&event->data[1+sizeof(struct atlantronic_vect2)]));
			break;
	}
}

static void atlantronic_model_event(void *opaque, int event)
{

}

static int atlantronic_model_init(SysBusDevice * dev)
{
    struct atlantronic_model_state *s = OBJECT_CHECK(struct atlantronic_model_state, dev, "atlantronic-model");

	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(DEVICE(dev), s->irq, MODEL_IRQ_OUT_NUM);
	qdev_init_gpio_in(DEVICE(dev), atlantronic_model_in_recv, IRQ_IN_NUM);

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
