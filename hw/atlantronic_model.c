#define LINUX
#include "kernel/cpu/cpu.h"
#undef LINUX
#include "kernel/driver/usb/otgd_fs_regs.h"

#include "sysbus.h"
#include "arm-misc.h"
#include "boards.h"
#include "exec-memory.h"
#include "kernel/robot_parameters.h"
#include "foo/control/control.h"

#define IRQ_OUT_NUM        2
#define IRQ_IN_NUM        10
#define PWM_NUM            4
#define ENCODER_NUM        2
#define MODEL_FACTOR      10      //!< calcul du modele a 10x la frequence d'utilisation

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


	float i;          //!< intensite du moteur (A)
	float w;          //!< vitesse de rotation du moteur (avant reducteur)
	float theta;      //!< angle du moteur (après reducteur)
};

struct atlantronic_model_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq[IRQ_OUT_NUM];
	int32_t pwm_dir[PWM_NUM];
	float enc[ENCODER_NUM];
	struct atlantronic_motor motor[PWM_NUM];

	float x;           //!< position du robot (axe x, mm)
	float y;           //!< position du robot (axe y, mm)
	float alpha;       //!< angle du robot (rd)
	float ca;          //!< cos(alpha)
	float sa;          //!< sin(alpha)
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

	// dw/dt = (ki - cp - fw) / J
	dx[1] = (motor->k * x[0] - motor->cp - motor->f * x[1]) / motor->j;

	// dtheta/dt = w
	dx[2] = motor->red * x[1];
}

static float sat(float x, float min, float max)
{
	if(x > max)
	{
		x = max;
	}
	else if(x < min)
	{
		x = min;
	}

	return x;
}

static void atlantronic_motor_update(struct atlantronic_motor* motor, double pwm)
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

	motor->pwm = pwm;

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

	s->x = 0;
	s->y = 0;
	s->alpha = 0;
	s->ca = 1;
	s->sa = 0;
	s->v = 0;
	s->w = 0;
}

static void atlantronic_model_compute(struct atlantronic_model_state* s)
{
	float v1 = s->motor[0].w * s->motor[0].red * PARAM_RIGHT_ODO_WHEEL_RADIUS_FX / 65536.0f;
	float v2 = s->motor[1].w * s->motor[1].red * PARAM_LEFT_ODO_WHEEL_RADIUS_FX / 65536.0f;
	s->v = (v1 + v2) / 2;
	s->w = ((v1 - v2) * PARAM_INVERTED_VOIE_FX39) / ((uint64_t)1 << 39);
	s->x += s->v * s->ca / CONTROL_HZ;
	s->y += s->v * s->sa / CONTROL_HZ;
	s->alpha += s->w / CONTROL_HZ;
	s->ca = cos(s->alpha);
	s->sa = sin(s->alpha);

#if 0
	if( fabs(s->v) > 0.01 || fabs(s->w) > 0.00001)
	{
		printf("%.2f %.2f %.2f v %.2f w %f\n", s->x, s->y, s->alpha, s->v, s->w);
	}
#endif
	// TODO :simulation bordures de la table
	// TEST
#if 0
	if(s->x > 1500 + PARAM_NP_X / 65536.0f)
	{
		s->x = 1500 + PARAM_NP_X / 65536.0f;
		s->motor[0].w = 0;
		s->motor[1].w = 0;
	}
#endif
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

				atlantronic_motor_update(&s->motor[id], level / 65536.0f);
				s->enc[id] = s->motor[id].theta * (1<< PARAM_ENCODERS_BIT_RES) / (2 * M_PI);
				s->enc[id] -= (floor((s->enc[id] - 65536) / 65536) + 1 ) * 65536;
				qemu_set_irq(s->irq[id], ((int32_t) s->enc[id])&0xffff );

				// dans le code, c'est la derniere pwm..., on en profite pour calculer le modele cinematique
				if(id == 1)
				{
					atlantronic_model_compute(s);
				}
				break;
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

static int atlantronic_model_init(SysBusDevice * dev)
{
    struct atlantronic_model_state *s = FROM_SYSBUS(struct atlantronic_model_state, dev);

	// memory_region_init_ram_ptr
	sysbus_init_mmio(dev, &s->iomem);

	qdev_init_gpio_out(&dev->qdev, s->irq, IRQ_OUT_NUM);
	qdev_init_gpio_in(&dev->qdev, atlantronic_model_in_recv, IRQ_IN_NUM);

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
