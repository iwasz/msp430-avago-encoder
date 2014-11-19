/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F5529LP:  simpleUsbBackchannel example
//
//   Description: 	Demonstrates simple sending over USB, as well as the F5529's
//                  backchannel UART.
//
//   Texas Instruments Inc.
//   August 2013
//******************************************************************************
// Basic MSP430 and driverLib #includes
#include "msp430.h"
#include "wdt_a.h"
#include "ucs.h"
#include "pmm.h"
#include "sfr.h"
#include "bcUart.h"
#include "hal.h"
#include "usci_a_spi.h"
#include "gpio.h"

#define SPICLK                          500000
uint8_t transmitData = 0x00, receiveData = 0x00;
uint8_t returnValue = 0x00;

// Global variables
uint32_t rxByteCount; // Momentarily stores the number of bytes received
int8_t buf_bcuartToUsb[BC_RXBUF_SIZE]; // Same size as the UART's rcv buffer
int8_t buf_usbToBcuart[128]; // This can be any size
void initSpi (void);

int main (void)
{
        WDTCTL = WDTPW + WDTHOLD; // Halt the dog

        // MSP430 USB requires a Vcore setting of at least 2.  2 is high enough
        // for 8MHz MCLK, below.
        PMM_setVCore (PMM_CORE_LEVEL_2);

        initPorts (); // Config all the GPIOS for low-power (output low)
        initClocks (8000000); // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
        bcUartInit (); // Init the back-channel UART
        initSpi ();
        __enable_interrupt();
        // Enable interrupts globally

        while (1) {
//                bcUartSend ("Hello world\r\n", 13);
//
//                for (int i = 0; i < 8192; ++i)
//                        ;
        }
}

void initSpi (void)
{
        //Set P1.1 for slave reset
        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN1);

        //Set P1.1 for slave reset
        //Set P1.0 to output direction
        GPIO_setAsOutputPin (GPIO_PORT_P1, GPIO_PIN0);

        //P3.5,4,0 option select
        GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P3, GPIO_PIN5 + GPIO_PIN4 + GPIO_PIN0);

        //Initialize Master
        returnValue = USCI_A_SPI_masterInit (
                        USCI_A0_BASE,
                        USCI_A_SPI_CLOCKSOURCE_SMCLK,
                        UCS_getSMCLK (),
                        SPICLK,
                        USCI_A_SPI_MSB_FIRST,
                        USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                        USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH);

        if (STATUS_FAIL == returnValue) {
                return;
        }

        //Enable SPI module
        USCI_A_SPI_enable (USCI_A0_BASE);

        //Enable Receive interrupt
        USCI_A_SPI_clearInterruptFlag (USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);
        USCI_A_SPI_enableInterrupt (USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);

        //Now with SPI signals initialized, reset slave
        GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN1);

        //LED On
        GPIO_setOutputHighOnPin (GPIO_PORT_P1, GPIO_PIN0);

        //Wait for slave to initialize
        __delay_cycles (100);

        //Initialize data values
        transmitData = 0x00;

        //USCI_A0 TX buffer ready?
        while (!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT)) ;

        //Transmit Data to slave
        USCI_A_SPI_transmitData(USCI_A0_BASE, transmitData);
}

//******************************************************************************
//
//This is the USCI_B0 interrupt vector service routine.
//
//******************************************************************************
__attribute__((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR (void)
{
        switch (__even_in_range(UCA0IV, 4)) {
        //Vector 2 - RXIFG
        case 2:
                //USCI_A0 TX buffer ready?
                while (!USCI_A_SPI_getInterruptStatus (USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT))
                        ;

                receiveData = USCI_A_SPI_receiveData (USCI_A0_BASE);

//                buf_bcuartToUsb[0] = receiveData;
                __disable_interrupt ();
                bcUartSend (&receiveData, 1);
                __enable_interrupt ();

                //Increment data
                transmitData++;

                //Send next value
                USCI_A_SPI_transmitData (USCI_A0_BASE, transmitData);

                //Delay between transmissions for slave to process information
                __delay_cycles (40);

                break;
        default:
                break;
        }
}
