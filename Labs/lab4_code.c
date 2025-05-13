/* Include header files */
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */
#define BTN1_PIN            GPIO_PORT_P1,GPIO_PIN1
#define BTN2_PIN            GPIO_PORT_P1,GPIO_PIN4

'w'

/* Variables globales */
uint8_t speed_level = 0; // 0 = 0%, 1 = 33%, 2 = 66%, 3 = 100%
uint8_t direction = 0; // 0 = Avant, 1 = Arri�re

void main(void) {
    WDT_A_holdTimer();
    mechrev_setup();

    /* Initialisation GPIO */
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN1_PIN);
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN2_PIN);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig3);
    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig4);

    MACRO_LAB4_INIT();

    while (1) {
            static uint8_t lastStateS1 = GPIO_INPUT_PIN_HIGH;
            static uint8_t lastStateS2 = GPIO_INPUT_PIN_HIGH;
            uint8_t currentStateS1 = MAP_GPIO_getInputPinValue(BTN1_PIN);
            uint8_t currentStateS2 = MAP_GPIO_getInputPinValue(BTN2_PIN);

            if (currentStateS1 == GPIO_INPUT_PIN_LOW && lastStateS1 == GPIO_INPUT_PIN_HIGH) {
                speed_level = (speed_level + 1) % 4;
                direction = 0;
            }
            lastStateS1 = currentStateS1;

            if (currentStateS2 == GPIO_INPUT_PIN_LOW && lastStateS2 == GPIO_INPUT_PIN_HIGH) {
                speed_level = (speed_level + 1) % 4;
                direction = 1;
            }
            lastStateS2 = currentStateS2;

            uint16_t dutyCycle = (speed_level * 1000);
            if (direction == 0) {
                TA0CCR1 = dutyCycle;
                printf("TACCR1: %d\n", TA0CCR1);
                TA0CCR2 = 0;
                printf("TACCR2: %d\n", TA0CCR2);
                TA0CCR3 = dutyCycle;
                printf("TACCR3: %d\n", TA0CCR3);
                TA1CCR1 = 0;
                printf("TACCR4: %d\n\n", TA0CCR4);
            } else {
                TA0CCR1 = 0;
                TA0CCR2 = dutyCycle;
                TA0CCR3 = 0;
                TA1CCR1 = dutyCycle;
            }

            MACRO_LAB4_EVENT();
        }
}
