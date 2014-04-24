#ifndef ATLANTRONIC_MODEL_H
#define ATLANTRONIC_MODEL_H

enum
{
	MODEL_IRQ_OUT_CAN1_MSG_ID,
	MODEL_IRQ_OUT_CAN1_MSG_SIZE,
	MODEL_IRQ_OUT_CAN1_MSG_DATA_L,
	MODEL_IRQ_OUT_CAN1_MSG_DATA_H,
	MODEL_IRQ_OUT_USART_HOKUYO1,
	MODEL_IRQ_OUT_USART_HOKUYO2,
	MODEL_IRQ_OUT_USART_AX12,
	MODEL_IRQ_OUT_USART_RX24,
	MODEL_IRQ_OUT_GPIO_1,
	MODEL_IRQ_OUT_GPIO_2,
	MODEL_IRQ_OUT_GPIO_3,
	MODEL_IRQ_OUT_GPIO_4,
	MODEL_IRQ_OUT_GPIO_5,
	MODEL_IRQ_OUT_GPIO_6,
	MODEL_IRQ_OUT_GPIO_7,
	MODEL_IRQ_OUT_GPIO_8,
	MODEL_IRQ_OUT_GPIO_9,
	MODEL_IRQ_OUT_GPIO_10,
	MODEL_IRQ_OUT_GPIO_11,
	MODEL_IRQ_OUT_GPIO_12,
	MODEL_IRQ_OUT_GPIO_13,
	MODEL_IRQ_OUT_GPIO_14,
	MODEL_IRQ_OUT_GPIO_BTN1,
	MODEL_IRQ_OUT_GPIO_BTN2,
	MODEL_IRQ_OUT_GPIO_GO,
	MODEL_IRQ_OUT_NUM
};

enum
{
	MODEL_IRQ_IN_CAN1_MSG_ID,
	MODEL_IRQ_IN_CAN1_MSG_SIZE,
	MODEL_IRQ_IN_CAN1_MSG_DATA_L,
	MODEL_IRQ_IN_CAN1_MSG_DATA_H,
	MODEL_IRQ_IN_ENCODER1,
	MODEL_IRQ_IN_ENCODER2,
	MODEL_IRQ_IN_ENCODER3,
	MODEL_IRQ_IN_PWM1,
	MODEL_IRQ_IN_PWM2,
	MODEL_IRQ_IN_PWM3,
	MODEL_IRQ_IN_PWM4,
	MODEL_IRQ_IN_PWM_DIR1,
	MODEL_IRQ_IN_PWM_DIR2,
	MODEL_IRQ_IN_PWM_DIR3,
	MODEL_IRQ_IN_PWM_DIR4,
	MODEL_IRQ_IN_USART_HOKUYO1,
	MODEL_IRQ_IN_USART_HOKUYO2,
	MODEL_IRQ_IN_USART_AX12,
	MODEL_IRQ_IN_USART_RX24,
	MODEL_IRQ_IN_NUM,
};

#endif
