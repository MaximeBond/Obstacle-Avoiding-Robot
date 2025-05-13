#include <stdio.h>
#include "driverlib.h"
#include "mechrev.h"
#include <math.h>

/* Define macros and function prototypes if needed */

/* Define configuration structs if needed */

    // Clock rate: 3 MHz. Baud rate 57600
    const eUSCI_UART_Config UART_init =
    {
         EUSCI_A_UART_CLOCKSOURCE_SMCLK,
         3,
         4,
         2,
         EUSCI_A_UART_NO_PARITY,
         EUSCI_A_UART_LSB_FIRST,
         EUSCI_A_UART_ONE_STOP_BIT,
         EUSCI_A_UART_MODE,
         EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const Timer_A_UpModeConfig upConfig =        // UP CONFIG - GENERATES PERIODIC INTERRUPT EVERY 1 SEC
    {
          TIMER_A_CLOCKSOURCE_SMCLK,             // Tie Timer A to SMCLK
          TIMER_A_CLOCKSOURCE_DIVIDER_40,        // Increment counter every 40 clock cycles
          75000,                                 // Period of Timer A
          TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer A roll over interrupt
          TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,    // Enable Capture Compare interrupt
          TIMER_A_DO_CLEAR                       // Clear counter upon initialization
    };


/* Declare global and volatile variables if needed */
volatile float distance_var;


/* Main program */
void main(void)

{
    /* Stop Watchdog Timer */
    WDT_A_holdTimer();

    /* Call the mechrev_setup function included in the mechrev.h header file */
    mechrev_setup();
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);       //Set LED2 as output (pin 2.1)
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);    //Turn off LED2
    /* Initialize GPIOs P1.2 and P1.3 for UART receive and transmit functionality */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initialize and enable UART EUSCI_A0 module */
    UART_initModule(EUSCI_A0_BASE, &UART_init);
    UART_enableModule(EUSCI_A0_BASE);

    /* Initialize UART RX Interrupt */
    Interrupt_disableMaster();
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);                 // Arm interrupt in peripheral register
    Interrupt_enableInterrupt(INT_EUSCIA0);                                              // Arm interrupt in NVIC
    Interrupt_enableMaster();                                                            // Enable all interrupts in interrupt controller (exit critical section)

    /* Initialize GPIOs P6.1 for ADC functionality */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initialize ADC14 module */
    ADC14_enableModule();
    ADC14_setResolution(ADC_10BIT);
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);                          //TBD DIVIDERS   WARNING
    ADC14_configureSingleSampleMode(ADC_MEM0, false);                                                     //TBD TRUE?      WARNING
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, 0);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();

    /* Initialize Timer A1 to generate periodic interrupts for for ADC readings */
    CS_setDCOFrequency(3E+6);                                    //Set DCO clock source frequency
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);  //Tie SMCLK to DCO

    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);              // Configure timer in Up mode
    Interrupt_enableInterrupt(INT_TA1_0);                           // Enable Timer A interrupt
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);           // Start Timer A

    /* Declare local variables if needed */

    /* Call the initialization grading macro */
    MACRO_LAB5_INIT();

    while(1)
    {
        // Empty while loop
    }
}


/* Interrupt Service Routine for Timer A1 */
void TA1_0_IRQHandler(void)
{
    /* Toggle an ADC conversion, wait for the results and read the ADC module */
    ADC14_toggleConversionTrigger();

    //Polls on BUSY flag
    while(ADC14_isBusy()) {}                                        // Returns a boolean value that tells if a conversion/sample is in progress

    //Conversion is done. Print results
    int adc_result = ADC14_getResult(ADC_MEM0);           // Returns the conversion result for a specified memory channel

    printf("adc_result --> %d\n", adc_result);


    /* Convert the ADC result to the actual voltage, */
    float voltage = adc_result * (3.3/1023.0);
    //printf("Voltage --> %f\n", voltage);

    /* and then convert the voltage to the actual distance value in cm. */
    distance_var = 27.726 * pow(voltage,-1.2045);
    printf("distance_var (cm) -- > %f\n", distance_var);
    /* Note: Store the result in the "distance_var" variable */                                          //WARNING

    /* Call the ADC grading macro with the distance variable */
    MACRO_LAB5_ADC_EVENT(distance_var);

    /* Clear the Timer A1 Capture/Compare interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/* Interrupt Service Routine for UART */
void EUSCIA0_IRQHandler(void)
{
    /* Read the received data from UART RX */
    uint8_t receivedValue = UART_receiveData(EUSCI_A0_BASE);
    /* Call the UART RX grading macro with the distance variable */
    MACRO_LAB5_UART_RX_EVENT(distance_var);

    /* Design a code so that when the byte received through UART RX is a
     * carriage return (HEX code 0x0D), convert the distance variable to
     * ASCII characters using "sprintf" function and
     * transmit those characters through UART TX.
     */
    if (receivedValue == 0x0D)
        {
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);    //led2
        char stringData[16]; // to hold the ASCII characters to be transmitted out
        int numChars = sprintf(stringData, "%.2f", distance_var);
        int i;

            for(i = 0; i < numChars; i++)
            {
                UART_transmitData(EUSCI_A0_BASE, stringData[i]);

                /* Note: Call the UART TX grading macro after transmitting each character */
                MACRO_LAB5_UART_TX_EVENT();
            }
        }

    /* Append a carriage return character (0x0D) and a newline character (0x0A)
     * at the end and transmit those characters through UART TX.
     */
        while((UCA0IFG & 0x02) == 0) {};
        UART_transmitData(EUSCI_A0_BASE, 0x0D); // Write carriage return '\r'
        while((UCA0IFG & 0x02) == 0) {};
        UART_transmitData(EUSCI_A0_BASE, 0x0A); // Write new line '\n'

    /* Clear the UART RX interrupt flag */
        UART_clearInterruptFlag(EUSCI_A0_BASE, INT_EUSCIA0);
}
