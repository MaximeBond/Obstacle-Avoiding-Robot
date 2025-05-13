/* Include header files */
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */
#define LED1_PIN            GPIO_PORT_P1,GPIO_PIN0
#define LEDR_PIN            GPIO_PORT_P2,GPIO_PIN0
#define LEDG_PIN            GPIO_PORT_P2,GPIO_PIN1
#define LEDB_PIN            GPIO_PORT_P2,GPIO_PIN2
#define BTN1_PIN            GPIO_PORT_P1,GPIO_PIN1
#define BTN2_PIN            GPIO_PORT_P1,GPIO_PIN4

/* Declare global and volatile variables if needed */


/* Main program */
void main(void)
{
    /* Stop Watchdog Timer */
    WDT_A_holdTimer();

    /* Call the mechrev_setup function included in the mechrev.h header file */
    mechrev_setup();

    /* Initialize GPIOs P1.1 and P1.4 for PushButtons (S1 and S2 switches) as inputs with pull-up resitors*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN1_PIN);
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN2_PIN);

    /* Initialize GPIOs P1.0, P2.0, P2.1 and P2.2 for LED1 and LED2 */
    MAP_GPIO_setAsOutputPin(LED1_PIN);
    MAP_GPIO_setAsOutputPin(LEDR_PIN);
    MAP_GPIO_setAsOutputPin(LEDG_PIN);
    MAP_GPIO_setAsOutputPin(LEDB_PIN);

    MAP_GPIO_setOutputLowOnPin(LED1_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDR_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDB_PIN);

    /* Initialize GPIOs P4.0, P4.2, P4.3, P4.5, P4.6 and P4.7 for Bump Sensors */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    /* Enable interrupts for Bump Sensors' GPIOs */
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);
    Interrupt_enableInterrupt(INT_PORT4);

    /* Declare local variables if needed */
    int i;

    /* Call the initialization grading macro */
    MACRO_LAB3_INIT();

    while(1)
    {
        /* Design a Polling process to detect PushButtons press and turn on or off LED1 accordingly */
        // Pushbuttons 1&2 and LED 1
                if (MAP_GPIO_getInputPinValue(BTN1_PIN) == GPIO_INPUT_PIN_LOW || MAP_GPIO_getInputPinValue(BTN2_PIN) == GPIO_INPUT_PIN_LOW)
                {
                    MAP_GPIO_setOutputHighOnPin(LED1_PIN);

                    for (i=0; i<10000; i++); // switch debouncing
                }
                else
                {
                    MAP_GPIO_setOutputLowOnPin(LED1_PIN);
                }

            /* Note: Call the event grading macro after turning on LED1 */
            MACRO_LAB3_EVENT();
    }
}

/* Interrupt Service Routine for PORT4 to handle Bump Sensors */
void PORT4_IRQHandler(void)
{
    /* Check the interrupt status */
    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

    /* Change the color of LED2 according to the interrupt status */
    if (status & GPIO_PIN0 || status & GPIO_PIN7) {
    // set LED2 to red color
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0); // red on
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1); // green off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2); // blue off
    }
    else if (status & GPIO_PIN2 || status & GPIO_PIN6) {
    // set LED2 to green color
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0); // red off
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1); // green on
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2); // blue off
    }
    else if (status & GPIO_PIN3 || status & GPIO_PIN5){
    // set LED2 to blue color
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0); // red off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1); // green off
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2); // blue on
    }

    /* Note: Call the event grading macro after changing the color of LED2 */
    MACRO_LAB3_EVENT();

    /* Clear the PORT4 interrupt flag */
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}
