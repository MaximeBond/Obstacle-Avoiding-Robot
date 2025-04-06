#include "driverlib.h"
#include "mechrev.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// --- Robot States ---
typedef enum {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
} RobotState;

volatile RobotState current_state = STOP;
volatile float distance_var = 0.0;
volatile bool collision_detected = false;

// --- PWM period ---
#define PWM_PERIOD 3000

// --- Function Prototypes ---
void driveForward(void);
void driveBackward(void);
void turnLeft(void);
void turnRight(void);
void stopMotors(void);

void main(void)
{
    WDT_A_holdTimer();
    mechrev_setup();

    // Enable H-Bridge driver
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7);

    // Set PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    // Timer_A0 PWM config
    Timer_A_PWMConfig pwmConfig1 = {TIMER_A_CLOCKSOURCE_SMCLK, TIMER_A_CLOCKSOURCE_DIVIDER_1, PWM_PERIOD, TIMER_A_CAPTURECOMPARE_REGISTER_1, TIMER_A_OUTPUTMODE_RESET_SET, 0};
    Timer_A_PWMConfig pwmConfig2 = pwmConfig1; pwmConfig2.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    Timer_A_PWMConfig pwmConfig3 = pwmConfig1; pwmConfig3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    Timer_A_PWMConfig pwmConfig4 = pwmConfig1; pwmConfig4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig3);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig4);

    // UART setup
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        3, 4, 2,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);

    // ADC setup (P6.1 -> A14)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    ADC14_enableModule();
    ADC14_setResolution(ADC_10BIT);
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
    ADC14_configureSingleSampleMode(ADC_MEM0, false);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();

    // Timer_A1 setup for ADC sampling every 0.5 sec
    const Timer_A_UpModeConfig upConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        23438,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
    };
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    // Bump sensors P4
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, 0xFF);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    Interrupt_enableInterrupt(INT_PORT4);

    MACRO_LAB6_INIT();

    while (1) {
/*      if (collision_detected || distance_var < 20.0) {
            stopMotors();
            current_state = STOP;
        }
*/
        if (collision_detected) {
                    stopMotors();
                    current_state = STOP;
                }

        switch (current_state) {
            case FORWARD:  driveForward(); break;
            case BACKWARD: driveBackward(); break;
            case LEFT:     turnLeft(); break;
            case RIGHT:    turnRight(); break;
            case STOP:
            default:       stopMotors(); break;
        }
    }
}

// --- ISRs ---
void TA1_0_IRQHandler(void)
{
    ADC14_toggleConversionTrigger();
    while (ADC14_isBusy());
    uint16_t adc_result = ADC14_getResult(ADC_MEM0);
    float voltage = adc_result * (3.3 / 1023.0);
    distance_var = 27.726 * pow(voltage, -1.2045);
    MACRO_LAB6_ADC_EVENT(distance_var);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void EUSCIA0_IRQHandler(void)
{
    uint8_t command = UART_receiveData(EUSCI_A0_BASE);
    MACRO_LAB6_UART_RX_EVENT(distance_var);

    if (command == 0x0D) {
        char str[10];
        int i;
        int len = sprintf(str, "%.2f", distance_var);
        for (i = 0; i < len; i++) {
            UART_transmitData(EUSCI_A0_BASE, str[i]);
            MACRO_LAB6_UART_TX_EVENT();
        }
        UART_transmitData(EUSCI_A0_BASE, 0x0D);
        UART_transmitData(EUSCI_A0_BASE, 0x0A);
    }
    else {
        switch (command) {
            case 'w': current_state = FORWARD; break;
            case 's': current_state = STOP; break;
            case 'z': current_state = BACKWARD; break;
            case 'a': current_state = LEFT; break;
            case 'd': current_state = RIGHT; break;
            default: break;
        }
    }

    UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
}

void PORT4_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MACRO_LAB6_SWITCH_EVENT();
    collision_detected = true;
    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}

// --- Motor control helpers ---
void driveForward() {
    TA0CCR1 = 3000; TA0CCR2 = 0;
    TA0CCR3 = 1000; TA0CCR4 = 0;
}

void driveBackward() {
    TA0CCR1 = 0; TA0CCR2 = 3000;
    TA0CCR3 = 0; TA0CCR4 = 500;
}

void turnLeft() {
    TA0CCR1 = 750; TA0CCR2 = 0;
    TA0CCR3 = 500; TA0CCR4 = 0;
}

void turnRight() {
    TA0CCR1 = 3000; TA0CCR2 = 0;
    TA0CCR3 = 250; TA0CCR4 = 0;
}

void stopMotors() {
    TA0CCR1 = TA0CCR2 = TA0CCR3 = TA0CCR4 = 0;
}
