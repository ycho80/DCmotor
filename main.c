#include "msp.h"
#include "driverlib.h"
#include "stdio.h"
#include <stdlib.h>


char receivedBuffer[200]; // All letters from Putty



//Set configure UART communication
const eUSCI_UART_Config uartConfig =
{
EUSCI_A_UART_CLOCKSOURCE_SMCLK, //  Use SMCLK as clock source for this UART
// 9600 baud rate
19, // clockPrescalar
8, // firstModReg
85, // secondModReg
EUSCI_A_UART_NO_PARITY, // No Parity
EUSCI_A_UART_LSB_FIRST, // LSB First
EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
EUSCI_A_UART_MODE, // UART mode
1// Oversampling
};

//Timer_A UpMode Configuration Parameter
const Timer_A_UpModeConfig upConfig_0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,         // SMCLK/1 = 3MHz
        15625,                                  // Period of 15625
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

void main(void)
{
    //Stop watchdog timer
    WDT_A_holdTimer();

    //SYSTEM CLOCK SETUP
    // Set DCO clock source frequency
    CS_setDCOFrequency(3E+6);
    // Tie SMCLK to DC
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Set pins 2 and 3 of port 1 to the primary module function (UART)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    // Initialize UART
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
    // Disable all interrupts
    Interrupt_disableMaster();
    // Clear interrupt flag(UCA0IFG)
    UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    // Arm interrupts when RXIFG is set
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    // Set interrupt Priority
    Interrupt_setPriority(INT_EUSCIA0, 0);
    // Enable interrupt
    Interrupt_enableInterrupt(INT_EUSCIA0);
    // Enable NVIC
    Interrupt_enableMaster();

    //TIMER SET UP
    // Configure Timer A using above struct
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_0);
    // Start Timer A
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN2); //Setting output pin for P3.2
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN3); //Setting output pin for P3.3
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW


    //PWM signal on P2.4
    P2SEL0 |= 0x10 ; // Set bit 4 of P2SEL0 to enable TA0.1 functionality on P2.4
    P2SEL1 &= ~0x10 ; // Clear bit 4 of P2SEL1 to enable TA0.1 functionality on P2.4
    P2DIR |= 0x10 ; // Set pin 2.4 as an output pin


    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN5); //Setting output pin for P4.5
    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN6); //Setting output pin for P4.6
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW


    //PWM signal on P2.6
    P2SEL0 |= BIT6;     // Set bit 6 of P2SEL0 to enable TA0.3 functionality on P2.6
    P2SEL1 &= ~BIT6;    // Clear bit 6 of P2SEL1 to enable TA0.3 functionality on P2.6
    P2DIR |= BIT6;      // Set pin 2.6 as an output pin

    while(1){}
}

volatile int a = 0;
int b;

// ISR
void EUSCIA0_IRQHandler(void)
{
        uint8_t letters; // storing letters in UCA0RXBUF
        letters = UART_receiveData(EUSCI_A0_BASE);

            if (letters == 0x61)
            {
                receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
                a++;                       // Increasing the index
                receivedBuffer[a] = 0xA; // Change the line for new coming letters

                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
                // Set Timer A period (PWM signal period)
                TA0CCR0 = 3000 ;
                // Set Duty cycle
                TA0CCR3 = atoi(receivedBuffer) ;
                // Set output mode to Reset/Set
                TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                // Initialize Timer A
                TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                for(b=0;b<=a;b++)
                {
                    if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                    {
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                    }
                    else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                    {
                        while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                            {
                                break;
                            }
                        }
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                    }
                }
                a = 0;
                for(b=0;b<=199;b++){
                    receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                }
            }

        else if (letters == 0x62) //tab for CCW
        {
            receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
            a++;                       // Increasing the index
            receivedBuffer[a] = 0xA; // Change the line for new coming letters

            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 3000 ;
            // Set Duty cycle
            TA0CCR3 = atoi(receivedBuffer) ;
            // Set output mode to Reset/Set
            TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                for(b=0;b<=a;b++)
                {
                    if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                    {
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                    }
                    else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                    {
                        while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                            {
                               break;
                            }
                        }
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                    }
                }
                a = 0;
                for(b=0;b<=199;b++){
                    receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                }
            }
        else if (letters == 0x63) //tab for CCW
                {
                    receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
                    a++;                       // Increasing the index
                    receivedBuffer[a] = 0xA; // Change the line for new coming letters

                    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
                    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
                    // Set Timer A period (PWM signal period)
                    TA0CCR0 = 3000 ;
                    // Set Duty cycle
                    TA0CCR3 = 0;
                    // Set output mode to Reset/Set
                    TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                    // Initialize Timer A
                    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                        for(b=0;b<=a;b++)
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                            {
                                UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                            }
                            else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                            {
                                while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                                {
                                    if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                                    {
                                       break;
                                    }
                                }
                                UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                            }
                        }
                        a = 0;
                        for(b=0;b<=199;b++){
                            receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                        }
                    }
        else if (letters == 0x64) //tab for CCW
        {
            receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
            a++;                       // Increasing the index
            receivedBuffer[a] = 0xA; // Change the line for new coming letters

            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 3000 ;
            // Set Duty cycle
            TA0CCR1 = atoi(receivedBuffer) ;
            // Set output mode to Reset/Set
            TA0CCTL1 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                for(b=0;b<=a;b++)
                {
                    if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                    {
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                    }
                    else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                    {
                        while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                            {
                               break;
                            }
                        }
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                    }
                }
                a = 0;
                for(b=0;b<=199;b++){
                    receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                }
            }
        else if (letters == 0x65) //tab for CCW
        {
            receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
            a++;                       // Increasing the index
            receivedBuffer[a] = 0xA; // Change the line for new coming letters

            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 3000 ;
            // Set Duty cycle
            TA0CCR1 = atoi(receivedBuffer) ;
            // Set output mode to Reset/Set
            TA0CCTL1 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                for(b=0;b<=a;b++)
                {
                    if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                    {
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                    }
                    else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                    {
                        while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                            {
                               break;
                            }
                        }
                        UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                    }
                }
                a = 0;
                for(b=0;b<=199;b++){
                    receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                }
            }
        else if (letters == 0x66) //tab for CCW
                {
                    receivedBuffer[a] = letters; // Store the letters in UCA0RXBUF in reveriveBuffer
                    a++;                       // Increasing the index
                    receivedBuffer[a] = 0xA; // Change the line for new coming letters

                    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
                    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
                    // Set Timer A period (PWM signal period)
                    TA0CCR0 = 3000 ;
                    // Set Duty cycle
                    TA0CCR1 = 0;
                    // Set output mode to Reset/Set
                    TA0CCTL1 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                    // Initialize Timer A
                    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                        for(b=0;b<=a;b++)
                        {
                            if((UCA0IFG & 0x0002) == 0x0002) // End of transmission. TXIFG is set
                            {
                                UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Sends the character stored in receiveBuffer to the A0 module
                            }
                            else if((UCA0IFG & 0x0002) == 0) // TXBUF is full
                            {
                                while ((UCA0IFG & 0x0002) == 0) // Polling until when TXIFG is set
                                {
                                    if((UCA0IFG & 0x0002) == 0x0002) // TXIFG is set
                                    {
                                       break;
                                    }
                                }
                                UART_transmitData(EUSCI_A0_BASE, receivedBuffer[b]); // Send the character stored in receiveBuffer to the A0 module
                            }
                        }
                        a = 0;
                        for(b=0;b<=199;b++){
                            receivedBuffer[b]= 0x00; // Clearing out all the buffer with NULL

                        }
                }
            else
            {

                receivedBuffer[a] = letters;
                a++;
            }

}
