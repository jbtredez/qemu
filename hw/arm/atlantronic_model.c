#include <math.h>
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "sysemu/char.h"
#include "sysemu/cpus.h"
#include "atlantronic_cpu.h"
#include "atlantronic_model.h"
#include "atlantronic_tools.h"
#include "atlantronic_can_motor_mip.h"
#include "atlantronic_dynamixel.h"
#include "atlantronic_hokuyo.h"
#include "atlantronic_can.h"
#include "atlantronic_omron.h"

#define __disco__
#include "kernel/rcc.h"
#include "disco/robot_parameters.h"
#include "kernel/can/can_id.h"
#define LINUX
#include "kernel/driver/io.h"
#undef LINUX

#define MODEL_DT           0.001f       //!< modele a 1ms

#define PWM_NUM            4
#define ENCODER_NUM        3
#define CAN_MOTOR_NUM      2
#define AX12_NUM           8
#define RX24_NUM           6
#define HOKUYO_NUM         2
#define OMRON_NUM          4

struct atlantronic_motor
{
	float pwm;         //!< pwm
	float gain_pwm;    //!< gain entre la pwm et u (V)
	float uToRpmGain;  //!< gain de conversion entre u (V) et la vitesse du moteur en rpm
};

struct atlantronic_model_state
{
	SysBusDevice busdev;
	MemoryRegion iomem;
	qemu_irq irq[MODEL_IRQ_OUT_NUM];
	CharDriverState* chr;
	float encoder[ENCODER_NUM];
	struct atlantronic_can_bus can;
	struct can_msg can_msg;
	struct atlantronic_can_motor_mip can_motor[CAN_MOTOR_NUM];
	int pwm_dir[PWM_NUM];
	struct atlantronic_motor motor[PWM_NUM];
	struct atlantronic_omron omron[OMRON_NUM];

	struct atlantronic_hokuyo_state hokuyo[HOKUYO_NUM];
	struct atlantronic_dynamixel_state ax12[AX12_NUM];
	struct atlantronic_dynamixel_state rx24[RX24_NUM];

	struct atlantronic_vect3 pos_robot;
	struct atlantronic_vect3 npSpeed;
	unsigned int cycle_count;
	QemuRobotParameters robotParameters;
};

static void atlantronic_motor_init(struct atlantronic_motor* motor)
{
	motor->pwm  = 0;
	motor->gain_pwm = 24.0f;
	motor->uToRpmGain = 163.5/3;
}

static void atlantronic_model_reset(struct atlantronic_model_state* s)
{
	int i = 0;

	s->robotParameters.odoWheel1Radius = 39.7;
	s->robotParameters.odoWheel2Radius = 39.7;
	s->robotParameters.odoWheel1Way = 1;
	s->robotParameters.odoWheel2Way = -1;
	s->robotParameters.odoEncoderResolution = 4096;
	s->robotParameters.voieOdo = 110;
	s->robotParameters.voieMot = 164;
	s->robotParameters.driving1WheelRaduis = 100;
	s->robotParameters.driving2WheelRaduis = 100;
	s->robotParameters.motorEncoderResolution = 1024;
	s->robotParameters.drivingMotor1Red = -5.2*88/25.0f;
	s->robotParameters.drivingMotor2Red = 5.2*88/25.0f;

	s->pos_robot.x = 0;
	s->pos_robot.y = 0;
	s->pos_robot.theta = 0;

	s->npSpeed.x = 0;
	s->npSpeed.y = 0;
	s->npSpeed.theta = 0;

	s->can.irq_id = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_ID];
	s->can.irq_size = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_SIZE];
	s->can.irq_data_h = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_H];
	s->can.irq_data_l = &s->irq[MODEL_IRQ_OUT_CAN1_MSG_DATA_L];
	s->cycle_count = 0;

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


	struct atlantronic_vect3 pos_omron[OMRON_NUM];
	pos_omron[0].x = -s->robotParameters.halfLength;
	pos_omron[0].y = 0;
	pos_omron[0].theta = M_PI;

	pos_omron[1].x = -s->robotParameters.halfLength;
	pos_omron[1].y = s->robotParameters.halfWidth;
	pos_omron[1].theta = M_PI;

	pos_omron[2].x = -s->robotParameters.halfLength;
	pos_omron[2].y = -s->robotParameters.halfWidth;
	pos_omron[2].theta = M_PI;

	pos_omron[3].x = 100;
	pos_omron[3].y = 10;
	pos_omron[3].theta = -M_PI / 4;

	atlantronic_omron_init(&s->omron[0], &s->irq[MODEL_IRQ_OUT_GPIO_1], pos_omron[0], REAR_OMRON_RANGE, OBJECT_SEEN_BY_OMRON, 1);
	atlantronic_omron_init(&s->omron[1], &s->irq[MODEL_IRQ_OUT_GPIO_2], pos_omron[1], REAR_OMRON_RANGE, OBJECT_SEEN_BY_OMRON, 1);
	atlantronic_omron_init(&s->omron[2], &s->irq[MODEL_IRQ_OUT_GPIO_3], pos_omron[2], REAR_OMRON_RANGE, OBJECT_SEEN_BY_OMRON, 1);
	atlantronic_omron_init(&s->omron[3], &s->irq[MODEL_IRQ_OUT_GPIO_4], pos_omron[3], 100, OBJECT_SEEN_BY_OMRON, 0);

	float outputGain = 2 * M_PI * s->robotParameters.driving1WheelRaduis / (float)(s->robotParameters.motorEncoderResolution * s->robotParameters.drivingMotor1Red);
	atlantronic_can_motor_mip_init(&s->can_motor[0], CAN_MOTOR_LEFT_NODEID, outputGain, 0, &s->can);

	outputGain = 2 * M_PI * s->robotParameters.driving2WheelRaduis / (float)(s->robotParameters.motorEncoderResolution * s->robotParameters.drivingMotor2Red);
	atlantronic_can_motor_mip_init(&s->can_motor[1], CAN_MOTOR_RIGHT_NODEID, outputGain, 0, &s->can);
}

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
	return sizeof(QemuAtlantronicModelEvent);
}

static void atlantronic_model_receive(void *opaque, const uint8_t* buf, int size)
{
	struct atlantronic_model_state* model = opaque;
	QemuAtlantronicModelEvent* event = (QemuAtlantronicModelEvent*) buf;

	if(size != sizeof(QemuAtlantronicModelEvent))
	{
		hw_error("atlantronic_model_receive - incorrect size");
	}

	switch(event->type)
	{
		case EVENT_NEW_OBJECT:
			atlantronic_add_object(event->data[0], MIN(event->data[1], (sizeof(event->data)-2)/sizeof(struct atlantronic_vect2)), (struct atlantronic_vect2*)&event->data[2]);
			break;
		case EVENT_MOVE_OBJECT:
			atlantronic_move_object(event->data[0], *((struct atlantronic_vect2*)&event->data[1]), *((struct atlantronic_vect3*)&event->data[1+sizeof(struct atlantronic_vect2)]));
			break;
		case EVENT_MANAGE_CAN_MOTOR:
//			atlantronic_canopen_manage_node_connextion(&model->canopen, event->data32[0], event->data32[1] == EVENT_MANAGE_CAN_MOTOR_CONNECT ? 1 : 0);
			break;
		case EVENT_SET_IO:
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
			if(event->data32[0] & GPIO_MASK_IN_GO) qemu_set_irq(model->irq[MODEL_IRQ_OUT_GPIO_GO], event->data32[1]);
			break;
		case EVENT_SET_POSITION:
			memcpy(&model->pos_robot.x, &event->data32[0], sizeof(model->pos_robot.x));
			memcpy(&model->pos_robot.y, &event->data32[1], sizeof(model->pos_robot.y));
			memcpy(&model->pos_robot.theta, &event->data32[2], sizeof(model->pos_robot.theta));
			break;
		case EVENT_SET_MAX_CYCLE_COUNT:
			if( event->data32[0] < model->cycle_count )
			{
				pause_all_vcpus();
			}
			else
			{
				resume_all_vcpus();
			}
			break;
		case EVENT_SET_ROBOT_PARAMETERS:
			memcpy(&model->robotParameters, &event->data[0], sizeof(model->robotParameters));
			model->can_motor[0].outputGain = 2 * M_PI * model->robotParameters.driving1WheelRaduis / (float)(model->robotParameters.motorEncoderResolution * model->robotParameters.drivingMotor1Red);
			model->can_motor[1].outputGain = 2 * M_PI * model->robotParameters.driving2WheelRaduis / (float)(model->robotParameters.motorEncoderResolution * model->robotParameters.drivingMotor2Red);
			break;
	}
}

static void atlantronic_model_event(void *opaque, int event)
{

}

static void atlantronic_model_update_odometry(struct atlantronic_model_state *s, float dt)
{
	// calcul de la nouvelle position
	float v1;
	float v2;
	if( s->robotParameters.canDrivingMotors )
	{
		v1 = s->can_motor[0].v;
		v2 = s->can_motor[1].v;
	}
	else
	{
		v1 = s->motor[0].pwm * s->motor[0].gain_pwm * s->motor[0].uToRpmGain / 60.0f * 2 * M_PI * s->robotParameters.driving1WheelRaduis / s->robotParameters.drivingMotor1Red;
		v2 = s->motor[1].pwm * s->motor[1].gain_pwm * s->motor[1].uToRpmGain / 60.0f * 2 * M_PI * s->robotParameters.driving2WheelRaduis / s->robotParameters.drivingMotor2Red;
	}

	s->npSpeed.x = 0.5 * (v1 + v2);
	s->npSpeed.y = 0;
	s->npSpeed.theta = (v2 - v1) / s->robotParameters.voieMot;

	struct atlantronic_vect3 npSpeedAbs = atlantronic_vect3_loc_to_abs_speed(s->pos_robot.theta, &s->npSpeed);

	struct atlantronic_vect3 pos_new = s->pos_robot;
	pos_new.x += npSpeedAbs.x * dt;
	pos_new.y += npSpeedAbs.y * dt;
	pos_new.theta += npSpeedAbs.theta * dt;

	// detection collisions
#if 0
	struct atlantronic_vect2 corner_abs_old[CORNER_NUM];
	struct atlantronic_vect2 corner_abs_new[CORNER_NUM];
	int res = -1;
	int i;

	for(i = 0; i < CORNER_NUM && res; i++)
	{
		int j = 0;
		atlantronic_vect2_loc_to_abs(&s->pos_robot, &corner_loc[i], &corner_abs_old[i]);
		atlantronic_vect2_loc_to_abs(&pos_new, &corner_loc[i], &corner_abs_new[i]);
		struct atlantronic_vect2 h;

		for(j = 0; j < atlantronic_static_obj_count && res ; j++)
		{
			// on regarde uniquement les objets consideres comme fixe
// TODO revoir algo de detection de collision (et aussi dans code robot?) pour le cas ou un segment petit est compris dans le tube du trajet
			if( ! (atlantronic_static_obj[j].flags & OBJECT_MOBILE) )
			{
				int k = 0;
				for(k = 0; k < atlantronic_static_obj[j].polyline.size - 1 && res ; k++)
				{
					res = atlantronic_segment_intersection(atlantronic_static_obj[j].polyline.pt[k], atlantronic_static_obj[j].polyline.pt[k+1], corner_abs_old[i], corner_abs_new[i], &h);
				}
			}
		}
	}
#else
	int res = -1;
	int j;
	struct atlantronic_vect2 a;// = {s->pos_robot.x, s->pos_robot.y};
	struct atlantronic_vect2 b;// = {pos_new.x, pos_new.y};
	struct atlantronic_vect2 aLoc = {0, 0};
	struct atlantronic_vect2 bLoc = {0, 0};
	atlantronic_vect2_loc_to_abs(&s->pos_robot, &aLoc, &a);
	atlantronic_vect2_loc_to_abs(&pos_new, &bLoc, &b);
	for(j = 0; j < atlantronic_static_obj_count && res ; j++)
	{
		// on regarde uniquement les objets consideres comme fixe
		if( ! (atlantronic_static_obj[j].flags & OBJECT_MOBILE) )
		{
			int k = 0;
			for(k = 0; k < atlantronic_static_obj[j].polyline.size - 1 && res ; k++)
			{
				float d = atlantronic_segment_distance(atlantronic_static_obj[j].polyline.pt[k], atlantronic_static_obj[j].polyline.pt[k+1], a, b);
				if( d < s->robotParameters.halfWidth )
				{
					res = 0;
				}
			}
		}
	}
#endif
	if( ! res )
	{
		// bloquage d'un des moteurs - TODO : on refait le calcul pour bloquer les moteurs
		//s->can_motor[0].block = 1;
		//s->can_motor[1].block = 1;
		// pour le moment, on va juste bloquer l'odometrie
		s->npSpeed.x = 0;
		s->npSpeed.y = 0;
		s->npSpeed.theta = 0;
		pos_new = s->pos_robot;
	}

	s->pos_robot = pos_new;

	// mise a jour des codeurs
	float v[2];
	v[0] = s->npSpeed.x - 0.5 * s->robotParameters.voieOdo * s->npSpeed.theta;
	v[1] = s->npSpeed.x + 0.5 * s->robotParameters.voieOdo * s->npSpeed.theta;

	s->encoder[0] += s->robotParameters.odoWheel1Way * v[0] * dt * s->robotParameters.odoEncoderResolution / (2 * M_PI * s->robotParameters.odoWheel1Radius) ;
	s->encoder[0] -= (floor((s->encoder[0] - 65536) / 65536) + 1 ) * 65536;
	s->encoder[1] += s->robotParameters.odoWheel2Way * v[1] * dt * s->robotParameters.odoEncoderResolution / (2 * M_PI * s->robotParameters.odoWheel2Radius) ;
	s->encoder[1] -= (floor((s->encoder[1] - 65536) / 65536) + 1 ) * 65536;
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER1], ((int32_t) s->encoder[0])&0xffff );
	qemu_set_irq(s->irq[MODEL_IRQ_OUT_ENCODER2], ((int32_t) s->encoder[1])&0xffff );
}

static void atlantronic_model_systick_cb(void* arg)
{
	struct atlantronic_model_state *s = arg;
	int i = 0;
	float dt = MODEL_DT;

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

	// mise a jour des hokuyos
	for(i = 0; i < HOKUYO_NUM; i++)
	{
		s->hokuyo[i].pos_robot = s->pos_robot;
	}

	// mise a jour des omron
	for(i = 0; i < OMRON_NUM; i++)
	{
		atlantronic_omron_update(&s->omron[i], s->pos_robot);
	}

	s->cycle_count++;
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

	systick_add_cb(atlantronic_model_systick_cb, s);

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
