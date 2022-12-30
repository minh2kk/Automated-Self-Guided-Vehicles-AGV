#include "HCSR04.h"

uint8_t
TM_HCSR04_Init(TM_HCSR04_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIGGER_GPIOx, uint16_t TRIGGER_GPIO_Pin) {
  GPIO_InitTypeDef gpio;
	/* Init Delay functions */
	delay_init();

	/* Save everything */
	HCSR04->ECHO_GPIOx = ECHO_GPIOx;
	HCSR04->ECHO_GPIO_Pin = ECHO_GPIO_Pin;
	HCSR04->TRIGGER_GPIOx = TRIGGER_GPIOx;
	HCSR04->TRIGGER_GPIO_Pin = TRIGGER_GPIO_Pin;

	/* Initialize pins */
	/* Trigger pin */
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio
	TM_GPIO_Init(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_Medium);

	/* Echo pin */
	TM_GPIO_Init(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_Medium);

	/* Trigger set to low */
	TM_GPIO_SetPinLow(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin);

	/* Start measurement, check if sensor is working */
	if (TM_HCSR04_Read(HCSR04) >= 0) {
			/* Sensor OK */
			return 1;
	}

	/* Sensor error */
	return 0;
}

float
TM_HCSR04_Read(TM_HCSR04_t* HCSR04) {
    uint32_t time, timeout;
    /* Trigger low */
    TM_GPIO_SetPinLow(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin);
    /* Delay 2 us */
    Delay(2);
    /* Trigger high for 10us */
    TM_GPIO_SetPinHigh(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin);
    /* Delay 10 us */
    Delay(10);
    /* Trigger low */
    TM_GPIO_SetPinLow(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin);

    /* Give some time for response */
    timeout = HCSR04_TIMEOUT;
    while (!TM_GPIO_GetInputPinValue(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) {
        if (timeout-- == 0x00) {
            return -1;
        }
    }

    /* Start time */
    time = 0;
    /* Wait till signal is low */
    while (TM_GPIO_GetInputPinValue(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) {
        /* Increase time */
        time++;
        /* Delay 1us */
        Delay(1);
    }

    /* Convert us to cm */
    HCSR04->Distance =  (float)time * HCSR04_NUMBER;

    /* Return distance */
    return HCSR04->Distance;
}
