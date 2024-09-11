#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

volatile unsigned int lastPWM = 200;
volatile unsigned int distance = 0;
bool flag = false;

#define FWD 1
#define REV 2
#define LEFT 3
#define RIGHT 4

void setSW1() {

	SIM->SCGC5 |= 1 << 11; // Enable Port C
	PORTC->PCR[3] &= ~0x703; // Clear mux
	PORTC->PCR[3] |= 0x703 & ((1 << 8) | 0x3); // Set mux
	GPIOC->PDDR &= ~(1 << 3); // Set to input mode

}

bool sw1Pressed() {
	return GPIOC->PDIR & 0x8;
}

void delay(unsigned int time) {
	unsigned int i, j;

	for (i = 1; i < time; i++)
		for (j = 0; j < 3825; j++)
			;
}

void setupMotor() {

	SIM->SCGC5 |= 1 << 10; // Enable Port B
	SIM->SCGC5 |= 1 << 11; // Enable Port C

	// Enable PTB0 as GPIO
	PORTB->PCR[0] &= ~0x700;
	PORTB->PCR[0] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 0);

	// Enable PTB1 as GPIO
	PORTB->PCR[1] &= ~0x700;
	PORTB->PCR[1] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 1);

	// Enable PTB2 as GPIO
	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 2);

	// Enable PTB3 as GPIO
	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 3);

	// Enable PTC1 as GPIO
	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 1);

	// Enable PTC2 as GPIO
	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[2] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 2);

}

void forward() {

	//Left Motor
	GPIOB->PDOR |= (1 << 0); //AI2
	GPIOB->PDOR &= ~(1 << 1); //AI1
	GPIOB->PDOR |= (1 << 2); //PWMA

	//Right Motor
	GPIOC->PDOR &= ~(1 << 1); //BI2
	GPIOC->PDOR |= (1 << 2); //BI1
	GPIOB->PDOR |= (1 << 3); //PWMB

}

void left() {

	//Left Motor
	GPIOB->PDOR &= ~(1 << 0); //AI2
	GPIOB->PDOR |= (1 << 1); //AI1

	//Right Motor
	GPIOC->PDOR &= ~(1 << 1); //BI2
	GPIOC->PDOR |= (1 << 2); //BI1
}

void right() {

	//Left Motor
	GPIOB->PDOR |= (1 << 0); //AI2
	GPIOB->PDOR &= ~(1 << 1); //AI1

	//Right Motor
	GPIOC->PDOR |= (1 << 1); //BI2
	GPIOC->PDOR &= ~(1 << 2); //BI1
}

void stop() {

	//Left Motor
	GPIOB->PDOR &= ~(1 << 0); //AI2
	GPIOB->PDOR &= ~(1 << 1); //AI1

	//Right Motor
	GPIOC->PDOR &= ~(1 << 1); //BI2
	GPIOC->PDOR &= ~(1 << 2); //BI1
}

void setupServo() {

	SIM->SCGC5 |= 1 << 9; // Enable Port A

	SIM->SCGC6 |= 1 << 25; // Enable TPM1
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK

	PORTA->PCR[12] &= ~0x700; // Clear MUX
	PORTA->PCR[12] |= 0x300; //Setting mux as ALT3

	TPM1->MOD = 625; // Setting mod register

	// Initialize TPM1 for PWM
	TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Edge PWM
	TPM1->CONTROLS[0].CnV = 200;

	// Start the Clock
	TPM1->SC |= 0xE; // Setting the frequency
}

void servoControl(int direction) {

	switch (direction) {

	case (1): //forward
		while (lastPWM != 200) {
			if (lastPWM < 200) {
				lastPWM += 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
			if (lastPWM > 200) {
				lastPWM -= 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
		}
		break;
	case (4): //right
		while (lastPWM != 80) {
			if (lastPWM < 80) {
				lastPWM += 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
			if (lastPWM > 80) {
				lastPWM -= 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
		}
		break;
	case (3): //left
		while (lastPWM != 330) {
			if (lastPWM < 330) {
				lastPWM += 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
			if (lastPWM > 330) {
				lastPWM -= 2;
				TPM1->CONTROLS[0].CnV = lastPWM;
				delay(10);
			}
		}
		break;

	}
}

void setupSonar() {

	SIM->SCGC5 |= (1 << 9);	// Enable port A
	SIM->SCGC5 |= (1 << 12); // Enable Port D

	// PTD2 GPIO
	PORTD->PCR[2] &= ~0x700; // Clear Mux
	PORTD->PCR[2] |= 0x700 & (1 << 8); // Set Mux
	GPIOD->PDDR |= (1 << 2); // GPIO output

	// PTA13 Interrupt
	PORTA->PCR[13] &= ~(0xF0703); // Clear MUX
	PORTA->PCR[13] |= 0xB0103; // Set interrupt to rising and falling edge.
	GPIOA->PDDR &= ~(1 << 13); // GPIO input

	// TODO: Leave as priority 0

	// TODO: Call Core API to Enable IRQ
	NVIC_EnableIRQ(30);

}

void delayUs(unsigned short delay_t) {
	SIM->SCGC6 |= (1 << 24); // Clock Enable TPM0
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM0->CONF |= (0x1 << 17); // Stop on Overflow
	TPM0->SC = (0x1 << 7) | (0x3); // Reset Timer Overflow Flag, Set Prescaler 8
	TPM0->MOD = delay_t - 1; // 10 microseconds

	TPM0->SC |= 0x01 << 3; // Start the clock!

	while (!(TPM0->SC & 0x80)) {
	} // Wait until Overflow Flag
	return;

}

void PORTA_IRQHandler(void) {
	// Sonar
	if (PORTA->PCR[13] & (1 << 24)) {

		distance = 0;
		while (GPIOA->PDIR & (1 << 13)) {
			delayUs(1);
			distance++;
		}
		PORTA->PCR[13] |= (1 << 24); // Clears interrupt flag
	}
}

void sendPulse() {
	GPIOD->PDOR |= (1 << 2);
	delayUs(10);
	GPIOD->PDOR &= ~(1 << 2);
}

int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	setSW1();
	setupMotor();
	setupServo();
	setupSonar();

	bool flag = false;

	while (1) {

		if (sw1Pressed()) {
			//Do nothing
			distance = 0;
			int distanceLeft, distanceRight;
			delay(500);
			sendPulse();
			delay(15);

			if (distance <= 250 && flag) {
				stop();

				servoControl(LEFT);
				sendPulse();
				delay(500);
				distanceLeft = distance;

				servoControl(RIGHT);
				sendPulse();
				delay(500);
				distanceRight = distance;

				servoControl(FWD);
				delay(500);

				if (distanceLeft <= 300 && distanceRight <= 300) {
					left();
					delay(950);
					stop();
					delay(500);
				} else if (distanceLeft > 250 && distanceLeft > distanceRight) {
					left();
					delay(475);
					stop();
					delay(500);
				} else if (distanceRight > 250
						&& distanceRight > distanceLeft) {
					right();
					delay(475);
					stop();
					delay(500);
				} else {
					stop();
					break;
				}
				forward();
			}
		} else {
			//Do something when switched is pressed.
			forward();
			flag = true;
		}
	}

	return 0;
}
