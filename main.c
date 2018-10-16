/*
 * acube.c
 *
 * Created: 19.03.2017 19:24:13
 * Author : Sergey Shelepin
 */ 


#define F_CPU 1000000UL

#define TRUE  1
#define FALSE 0

//#define DEBUG_C

#ifdef DEBUG_C
	#define DP(x)           UARTPrint(x)
	#define DPln(x)         UARTPrintln(x)
	#define DPUi16(x, b)    UARTPrintInt16(x, b)
	#define DL_on()         sled_on()
	#define DL_off()        sled_off()	
	#define SIDE_LIGHT_DUR  2
	#define MAIN_TIME_BIT 500
	
#else
	#define DP(x)
	#define DPln(x)
	#define DPUi16(x, b)
	#define DL_on()        
	#define DL_off()       		
	#define SIDE_LIGHT_DUR  25
	#define MAIN_TIME_BIT 50

	
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include "UARTSI.h"
#include "SPIsi.h"
#include "ADXL345.h"
#include "pins_config.h"

#define sr_latch_low()   SR_LATCH_PORT &= ~_BV(SR_LATCH_PIN)					// set shift register latch pin low
#define sr_latch_high()  SR_LATCH_PORT |= _BV(SR_LATCH_PIN)						// set shift register latch pin high

uint8_t sides[] = {0, 0b10, 0b100, 0b1000, 0b10000, 0b100000, 0b1000000};		// sides to shift reg outs array: off, 1st side, 2nd side, ...
#define light_side(s_num) sr_update(sides[s_num])								// side light up macro

#define sled_on()  SLED_PORT |= _BV(SLED_PIN)									// signal debug led macro
#define sled_off() SLED_PORT &= ~_BV(SLED_PIN)

#define int0_enable()  EIMSK |= _BV(INT0)										// enabling INT0 interrupt			
#define int0_disable() EIMSK &= ~_BV(INT0)										// disabling INT0 interrupt			

#define vdd_turn_on()  VDD_EN_PORT |= _BV(PD4)									// leds power regulators on/off macro
#define vdd_turn_off() VDD_EN_PORT &= ~_BV(PD4)
																				// buzzer on timer2 macro
#define buzzer_on()      TCCR2B = _BV(CS20);									// start timer2 with no pre-scaling
#define buzzer_off()     TCCR2B = 0;											// set timer 2 off
#define buzzer_pin_low() BUZZER_PORT &= _BV(BUZZER_PIN)							// force buzzer pin to low

																				// macro for 0 and 1 timers
#define timer0_start() TCCR0B |= _BV(CS01)										// starting with set pre-scaler 8 
#define timer0_stop()  TCCR0B &= ~_BV(CS01)
#define timer1_start() TCCR1B |= _BV(CS11)										// starting with set pre-scaler 8 
#define timer1_stop()  TCCR1B &= ~_BV(CS11)
	
																				// RGB PWM setup: mapping to Output Compare Registers
																				// (!) it's fully not configurable!!! due to different ISRs
#define OCREG_R_LED OCR0B														
#define OCREG_G_LED OCR0A
#define OCREG_B_LED OCR1A
																				// variables for RGB flashing randomly on different speed, direction and initial states  
uint8_t volatile r_led_ocreg; //OC0B
uint8_t volatile g_led_ocreg; //OC0A
uint8_t volatile b_led_ocreg; //OC1A

uint8_t volatile r_led_inc_down; //OC0B
uint8_t volatile g_led_inc_down; //OC0A
uint8_t volatile b_led_inc_down; //OC1A

uint8_t volatile r_led_slower;
uint8_t volatile g_led_slower; 
uint8_t volatile b_led_slower; 

uint8_t volatile r_led_slower_max;
uint8_t volatile g_led_slower_max;
uint8_t volatile b_led_slower_max;

uint8_t volatile smooth_light_t0;
uint8_t volatile smooth_light_t0_slower;
uint8_t volatile smooth_light_t1;
uint8_t volatile smooth_light_t1_slower;
																			// defines for smooth light
#define SL_OFF			    0
#define SL_UP			    1
#define SL_DOWN			    2
#define SL_THRESH		  110
#define SL_SLOWER_MAX       1

#define PRR_CONFIG		   _BV(PRTWI)										// TWI is switched off all the time

#define FREE_FLOAT_ADC_CH  1												// ADC channel for rand function	
#define ADMUX_MASK         ( _BV(REFS1) | _BV(REFS0) )						// set internal ref for ADC

#define ADC_enable()	   ADCSRA |= _BV(ADEN)								// ADC on/off macro
#define ADC_disable() 	   ADCSRA &= ~_BV(ADEN) 

#define ADC_PWR_VOLTAGE_CH 0												// power supply ADC channel 

#define PWR_VLT_ALARM      869												// 3.67 volts
#define PWR_VLT_CRITICAL   852												// 3.60 volts
#define PWR_VLT_BLOCKING   817												// 3.45 volts

int16_t adxl_xyz_data[3];													// ADXL axis values array

volatile uint8_t adxl_int_flag;												// ADXL int flag
uint8_t cube_is_active;														// cube main state

uint8_t side;																// cur side
uint8_t prev_side;															// prev side
uint8_t side_light_flag;													// side light on/off flag


//----------------------------------------------declarations---------------------------------------------
void inline adxl_setup();
void inline ADC_init();
uint16_t	ADC_measure_10bit(uint8_t ch);
uint8_t		ADC_measure_8bit(uint8_t ch);
uint8_t		a_rand(uint8_t bit_maks);										
uint8_t		color_start_rand();
void		RGB_rand_and_start();
void inline RGB_stop();
void		sr_update(uint8_t val);
void inline fading();
void inline fading_low_power(uint8_t is_red);
uint16_t	measure_power_voltage();
void		get_side();
void		buzz();
uint8_t		check_power();

/**************************************************************************************************************/
//   MAIN                                                                                                     //
/**************************************************************************************************************/

int main(void) {
	
	// external interrupt setup------
	EIMSK |= _BV(INT0);												// enabling INT0 interrupt
	EICRA = 0;														// low level generates interrupt
	
	//------------------pins setup------------------------------------
	//leds pins setup
	SLED_DDR |= _BV(SLED_PIN);									   // signal led setup
	sled_off();
	
	RGB_RLED_DDR |= _BV(RGB_RLED_PIN);							   // set red RGB pin led as output
	RGB_GLED_DDR |= _BV(RGB_GLED_PIN);							   // set green RGB pin led as output
	RGB_BLED_DDR |= _BV(RGB_BLED_PIN);							   // set green RGB pin led as output
	
	VDD_EN_DDR |= _BV(VDD_EN_PIN);								   // voltage regulator DDR set as output
	
	adxl345_cs_pin_as_output();									   // adxl pins setup
	adxl345_cs_high();
																   // shift reg pins setup
	SR_LATCH_DDR |= _BV(SR_LATCH_PIN);							   // latch pin as output
	sr_latch_high();
		
	//------------end of pins setup------------------------------------
	
	SPI_set_mode3();												// SPI mode3 is needed to communicate with ADXL345
	SPI_master_start();												// start spi bus
	sr_update(0);													// ensure all sides leds are off
	adxl_setup();													//call adxl setup
	
																	// power management setup
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);							// set sleep mode POWER DOWN
	PRR = PRR_CONFIG;												// set PRR register 	
	
#ifdef DEBUG_C
	UARTInitRXTX();													// UART init if debug mode
#endif	
	
	cube_is_active = TRUE;											// cube main flag init	
	
	//-----------------------------timers and PWM setup-----------------------------------------------
	//timer	0: PWM initialization: mode 3 Fast PWM, TOV interrupt enable
	TCCR0A =_BV(WGM01) | _BV(WGM00);						   // turn mode3 fast PWM
	TIMSK0 = _BV(TOIE0);									   // enable Timer/Counter0 Overflow Interrupt

	// timer 1: 
	TCCR1A = _BV(WGM10);									   // set mode 5 Fast 8 bit PWM. 
	TCCR1B = _BV(WGM12); 	                                   // continue with mode setting
	OCR1B =  127;	                                           // duty cycle 50% 
	TIMSK1 = _BV(TOIE1);                                       // enable timer1 overflow interrupt

	//timer2: 
	TCCR2A = _BV(WGM21);									   // set CTC mode
//	TCCR2B = _BV(CS20);										   // no prescaling
	OCR2A = 165;											   // set ~6x kHz timer to get about ~3 kHz square wave frequency
	TIMSK2 = _BV(OCIE2A);									   // enabling compare match A interrupt
	//-----------------------------end of PWM setup------------------------------------------------
	ADC_init();												   // call ADC setup
	sei();													   // enable interrupts

	// ---------------starting-----------------------
	DP("Privet!");
#ifdef DEBUG_C
	ADXL345_print_details();
	_delay_ms(100);  // ?
#endif

	ADC_enable();												// enable ADC
	RGB_rand_and_start();										// get random start RBG values and start timers 
	vdd_turn_on();												// turn on leds power supply
	buzz();														// make short beep

    /* Replace with your application code */
    while (1) {
		
		//------------------ ADXL INT FLAG PROCESSING-----------------------------------------------------------------------------------------------------------
		
		if (adxl_int_flag) {															// if ADXL flag is raised
			uint8_t int_source_reg = ADXL345_read_register(ADXL345_INT_SOURCE);			// reading int source and clearing interrupts

			if (int_source_reg & _BV(ADXL_ACTIVITY)) {								    // 1. check if ADXL reports for activity detection
				DL_on();																// turn on debug led 

				if(!cube_is_active) {													// if cube is not active, changing cube state. starting RGB leds
					cube_is_active = TRUE;												// change cube state to active
					RGB_rand_and_start();												// initialize RGB initial values and change rate with "hardware random"
					vdd_turn_on();														// turn on leds power regulators
				}
				DPln("activity int!!!");												// debug print
			
			} else if (int_source_reg & _BV(ADXL_INACTIVITY)) {							// 2. if ADXL reports for inactivity detection	
				DL_off();																// turn off debug led 
				cube_is_active = FALSE;													// change cube state to inactive
				DPln("inactivity int!");												// debug print
				
			} else {																	// 3. if ADXL reports for some unexpected int...	
				DPln("some other int..."); 
			}
			
			adxl_int_flag = FALSE;														// drop adxl int flag 
			int0_enable();																// enable INT0 interrupt
		}		
		//------------------ END OF ADXL INT FLAG PROCESSING------------------------------------------------------------------------------------------------------
		

		//------------------ CUBE STATES PROCESSING---------------------------------------------------------------------------------------------------------------		
		if (!cube_is_active) {															// if switched to inactive cube state - put mcu to sleep
			DPln("go to sleep...zzzz");													// debug print
			_delay_ms(100);																// waiting till UART finished with processing its buffer
			
			fading();																	// 1. call led fading function
																						// 2. check if we need to indicate low battery power by yellow or red fading at the end. 
			uint16_t vlt = measure_power_voltage();										// get power voltage 
			if(vlt < PWR_VLT_ALARM) {													// if less than power voltage alarm threshold		
				if(vlt < PWR_VLT_CRITICAL) {											// if less than critical 
					fading_low_power(TRUE);												// make red color flash and fading 
				} else {																// otherwise 
					fading_low_power(FALSE);											// make yellow color flash and fading 
				}
			}
																						// 3. turn all peripherals off
			sr_update(0);																// make sure all all sides leds are off
			vdd_turn_off();																// turn off leds voltage regulators
			RGB_stop();																 	// stop RGB timers, disconnecting OCR pins, force rgb pins to low state
			ADC_disable();															 	// disable ADC
			PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRADC) | _BV(PRUSART0);// shutdown timers 0,1,2, ADC, UART
																						
																						// 4. go to sleep
			sleep_enable();																// set sleep enable
			if(!adxl_int_flag) {														// couldn't go to sleep if ADXL interrupt have been occurred while all those fading procedures and other pre-sleep tasks		
				MCUCR |= (1<<BODS) | (1<<BODSE);									    // 5. turn bod off
				MCUCR &= ~(1<<BODSE);													// must be done right before sleep
				sleep_cpu();															// 6. sleeping....zzz
			}				
																						// 7. after waking-up
			sleep_disable();															// set sleep disable
			PRR = PRR_CONFIG;															// restore PRR register
			ADCSRA |= _BV(ADEN);														// enable ADC
#ifdef DEBUG_C
			UARTInitRXTX();																// reinit UART
			DPln("I'm back!");															// debug print
#endif			
		}
		//------------------ END OF CUBE STATES PROCESSING---------------------------------------------------------------------------------------------------------

		//------------------ MAIN CYCLE PROCESSING-----------------------------------------------------------------------------------------------------------------
		
		if(!check_power()) {															// check for low power
			fading();																	// just fading if low power detected
		} else {																		// else start main active cycle
										
			ADXL345_read_xyz(adxl_xyz_data);											//1. reading adxl data

	#ifdef DEBUG_C																		// debug print voltage and xyz data
			DPUi16(measure_power_voltage(), 10);
			DP("   |   ");
			for (uint8_t i = 0; i<3; i++) {
				UARTPrintInt16(adxl_xyz_data[i], 10);
				UARTPrint(" ");
			}
	#endif			
			get_side();																	// 2. evaluate current active side and put side code into global side variable 

			if(prev_side != side){														// 3. process active side change
				if(side) {																// if we have some side at the end
					light_side(side);													// light that side up
					buzz();																// make beep 
					side_light_flag = SIDE_LIGHT_DUR;									// set count down timer to control duration 
				} else {																// if we have no side after change
					light_side(0);														// turn off all sides 
					side_light_flag = 0;												// set count down timer to zero
				}
			}
			
			prev_side = side;															// save side to prev side
			
			if(side_light_flag) {														// if light countdown flag is not zero
				--side_light_flag;														// decrease the flag
			}
		
	#ifdef DEBUG_C																		// debug print
			UARTPrint("   |   ");
			UARTPrint("ps=");
			UARTPrintUint(prev_side, 10);
			UARTPrint("  s=");
			UARTPrintUint(side, 10);
			UARTPrintln("");
	#endif				
	
			_delay_ms(MAIN_TIME_BIT);													// wait cycle time bit. can afford such approach due to simple program structure.
			
			if(side_light_flag == 1) {													// if countdown timer is int one step to be zero
				light_side(0);															// turn off the side light 
			}
		}
		//------------------ END OF MAIN CYCLE PROCESSING----------------------------------------------------------------------------------------------------------
    }
}

//==================================================================================================================================================================
//																			FUNCTIONS
//==================================================================================================================================================================

//-------------------------------------------------------------------------------------------
// checking if battery power is enough to start up
// @todo: need to be finalized and some code to be refactored 
//-------------------------------------------------------------------------------------------
uint8_t check_power() {
	return TRUE;
	/*
	uint16_t vlt = measure_power_voltage()
	DP("power check  ");
	DPUi16(vlt, 10);
	DPln("");
	if (vlt < PWR_VLT_BLOCKING) {
		return FALSE;
	} else {
		return TRUE;
	}
	*/
}

//-------------------------------------------------------------------------------------------
// makes short beep sound
//-------------------------------------------------------------------------------------------
void buzz() {
	buzzer_on();
	_delay_ms(50);
	buzzer_off();
	buzzer_pin_low();																// have to force buzzer pin to low, as per buzzer data sheet
}


//-------------------------------------------------------------------------------------------
// defines for get side function
// many of the below is needed because pcb (and so adxl) diagonal orientation inside the cube
//-------------------------------------------------------------------------------------------
																					// "in the side" detection
#define X_DISP 18																	// x axis dispersion in adxl units
#define Y_DISP 15																	// y axis dispersion 
#define Z_DISP 15																	// z axis dispersion 
																					// "out of the side" detection
#define X_SO_DISP  60																// x axis dispersion
#define Y_SO_DISP  45																// y axis dispersion
#define Z_SO_DISP  45																// z axis dispersion
																					// x axis average center, high and low point 
#define X_AVG_C -3
#define X_AVG_H 125
#define X_AVG_L -128
																					// y axis average center, high and low point 
#define Y_AVG_C -10
#define Y_AVG_H 80
#define Y_AVG_L -98
																					// z axis average center, high and low point 
#define Z_AVG_C 15
#define Z_AVG_H 103
#define Z_AVG_L -73
																					// axis data indexes in adxl_xyz_data array
#define XVAL 0 
#define YVAL 1
#define ZVAL 2

//macros to check whether the axis value is equal to average +/- dispersion, i.e. fulfills  "in the side" condition
#define check_val_x(xyz_val_index, xyz_avg, xyz_disp)   ((adxl_xyz_data[xyz_val_index] < (xyz_avg + xyz_disp)) && (adxl_xyz_data[xyz_val_index] > (xyz_avg - xyz_disp)))

// macros for "out of the side" condition (side axis out condition -saoc)
#define saoc(xyz_val_index, xyz_avg, xyz_disp ) (adxl_xyz_data[xyz_val_index] > (xyz_avg + xyz_disp) ||  adxl_xyz_data[xyz_val_index] < (xyz_avg - xyz_disp))

//-------------------------------------------------------------------------------------------
// get side function
// algorithm is based on experimental measurements. 
// Each side has assigned color based excel graph, where measurements were placed
//-------------------------------------------------------------------------------------------
void get_side() {
	// if no active side 
	if (!side) {
	//	4 sides
		if ( check_val_x(XVAL, X_AVG_C, X_DISP) ) {
			DP("X_AVG_C!  ");
		
			// orange and grey
			if( check_val_x(YVAL, Y_AVG_H, Y_DISP) ) {
				DP("Y_AVG_H!  ");
				//grey
				if (check_val_x(ZVAL, Z_AVG_H, Z_DISP) ) {
					DP("Z_AVG_H! GREY ");
					side = 3; 
					return;
				}
				//orange
				if (check_val_x(ZVAL, Z_AVG_L, Z_DISP) ) {
					DP("Z_AVG_L! ORANGE ");
					side = 2;
					return;
				}			
			
			}
		
			//light blue and yellow
			if( check_val_x(YVAL, Y_AVG_L, Y_DISP) ){
				DP("Y_AVG_L!  ");		
				//yellow			
				if (check_val_x(ZVAL, Z_AVG_H, Z_DISP) ) {
					DP("Z_AVG_H! YELLOW ");
					side = 4;
					return;
				}
				//light blue
				if (check_val_x(ZVAL, Z_AVG_L, Z_DISP) ) {
					DP("Z_AVG_L! LIGHT BLUE ");
					side = 1;
					return;
				}			
				
			}
		
		}

	// 2 sides: blue and green 
		if (check_val_x(ZVAL, Z_AVG_C, Z_DISP) ){
			UARTPrint("Z_AVG_C!  ");
		
			if (check_val_x(YVAL, Y_AVG_C, Y_DISP) ){
				UARTPrint("Z_AVG_C!  ");
				//blue
				if(check_val_x(XVAL, X_AVG_H,X_DISP)){
					UARTPrint("Z_AVG_H! BLUE ");	
					side = 5;		
					return;	
				}
				//green
				if(check_val_x(XVAL, X_AVG_L, X_DISP)){
					UARTPrint("Z_AVG_L! GREEN ");				
					side = 6;
					return;
				}
			
			}		

		
		}
		side = 0;
		return;		
	} else { // if some side is active
		
		switch (side) {
			case 1: //light blue
				if (saoc(XVAL, X_AVG_C, X_SO_DISP) || saoc(YVAL, Y_AVG_L, Y_SO_DISP) || saoc(ZVAL, Z_AVG_L, Z_SO_DISP)) {
					side = 0;
				}
			break;
			case 2: //orange
				if (saoc(XVAL, X_AVG_C, X_SO_DISP) || saoc(YVAL, Y_AVG_H, Y_SO_DISP) || saoc(ZVAL, Z_AVG_L, Z_SO_DISP)) {
					side = 0;
				}			
			break;
			
			case 3: //grey
				if (saoc(XVAL, X_AVG_C, X_SO_DISP) || saoc(YVAL, Y_AVG_H, Y_SO_DISP) || saoc(ZVAL, Z_AVG_H, Z_SO_DISP)) {
					side = 0;
				}
			break;
						
			case 4: //yellow
				if (saoc(XVAL, X_AVG_C, X_SO_DISP) || saoc(YVAL, Y_AVG_L, Y_SO_DISP) || saoc(ZVAL, Z_AVG_H, Z_SO_DISP)) {
					side = 0;
				}
			break;


			case 5: // blue
				if (saoc(XVAL, X_AVG_H, X_SO_DISP) || saoc(YVAL, Y_AVG_C, Y_SO_DISP) || saoc(ZVAL, Z_AVG_C, Z_SO_DISP)) {
					side = 0;
				}
			break;

			case 6: //green
				if (saoc(XVAL, X_AVG_L, X_SO_DISP) || saoc(YVAL, Y_AVG_C, Y_SO_DISP) || saoc(ZVAL, Z_AVG_C, Z_SO_DISP)) {
					side = 0;
				}
			break;
			
			default:
			break;
						
		}

		
	}
}

//-------------------------------------------------------------------------------------------
// gets power voltage by measure it 8 times in a row. 
// returns average value by shifting measurement sum by 3 bits to the right
//-------------------------------------------------------------------------------------------
uint16_t measure_power_voltage() {
	uint16_t vlt = 0;
	for (uint8_t i = 0; i < 8; i++) {
		vlt += ADC_measure_10bit(ADC_PWR_VOLTAGE_CH);
	}
	return vlt >>= 3;
}

//-------------------------------------------------------------------------------------------
// do smooth lighting off
//-------------------------------------------------------------------------------------------
void inline fading(){
	smooth_light_t0 = SL_DOWN;
	smooth_light_t1 = SL_DOWN;
		
	cli();
	OCREG_R_LED = SL_THRESH;
	OCREG_G_LED = SL_THRESH;
	OCREG_B_LED = SL_THRESH;
	sei();
			
	while (smooth_light_t0 || smooth_light_t0 ); // waiting  till sl worked out
}

//-------------------------------------------------------------------------------------------
// need tp somehow indicate if the power is low
// if is_red - slowly fading with red color, otherwise - with yellow
//-------------------------------------------------------------------------------------------
void inline fading_low_power(uint8_t is_red) {
	cli();													// disable interrupts  
															
															// we need to turn off blue led
	TCCR1A &= ~_BV(COM1A1);									// disconnecting OC1A channel using by blue led
	RGB_BLED_PORT &= ~_BV(RGB_BLED_PIN);					// put blue led pin to low 

	if(is_red)	{											// if need to fade in red
		TCCR0A &= ~_BV(COM0A1);								// disconnecting OC0A channel using by green led
		RGB_GLED_PORT &= ~_BV(RGB_GLED_PIN);				// put green led pin to low 
	} else {
		OCREG_G_LED = 255;									// put to max brightness otherwise
	}

	OCREG_R_LED = 255;										// set red to max 

	sei();													// enable interrupts
	
	smooth_light_t0 = SL_DOWN;								// set fading time 
	while (smooth_light_t0 );								// waiting till sl worked out	
	
}

//-------------------------------------------------------------------------------------------
// ADXL345 setup
//-------------------------------------------------------------------------------------------
void inline adxl_setup() {

	ADXL345_write_register(ADXL345_BW_RATE, _BV(ADXL_LOW_POWER) | _BV(ADXL_RATE_B3) | _BV(ADXL_RATE_B1)| _BV(ADXL_RATE_B0));  	//set output rate 200 Hz and LOW_POWER bit
//	ADXL345_write_register(ADXL345_BW_RATE, _BV(ADXL_RATE_B3) | _BV(ADXL_RATE_B2));				// set output rate 400 Hz with NO(!) LOW_POWER bit
	ADXL345_write_register(ADXL345_DATA_FORMAT, _BV(ADXL_INT_INVERT) | _BV(ADXL_RANGE_B0) );    // put the ADXL345 into +/- 4G range and set INT_INVERT flag
	ADXL345_write_register(ADXL345_TIME_INACT, 10);												// Set time of inactivity to 10 sec
	ADXL345_write_register(ADXL345_THRESH_ACT, 0x02);											// Set act interrupt threshold with 62.5 mg/LSB steps
	ADXL345_write_register(ADXL345_THRESH_INACT, 0x02);										    // Set Inactivity threshold as 0.1875 g
	ADXL345_write_register(ADXL345_ACT_INACT_CTL, 0xFF);  										// Enable Activity and Inactivity of X-, Y-, Z-axis, wherein Inactivity is ac-coupled mode, Activity is c-coupled mode
	ADXL345_write_register(ADXL345_INT_ENABLE, _BV(ADXL_ACTIVITY) | _BV(ADXL_INACTIVITY));      // enabling ADXL345 interrupts; all interrupts mapped to INT1 pin - 0 byt default in ADXL345 INT_NAP register
	ADXL345_read_register(ADXL345_INT_SOURCE);													// clear all interrupts to be on the safe side
	uint8_t register reg = _BV(ADXL_LINK) | _BV(ADXL_AUTO_SLEEP);								// set link bit (activity <-> inactivity detection ), set auto_sleep bit, wake_up bits are 0 and 0 - i.e. 8 Hz update freq in sleep mode.
	ADXL345_write_register(ADXL345_POWER_CTL, reg);
	ADXL345_write_register(ADXL345_POWER_CTL, reg |= _BV(ADXL_MEASURE));						// switch to measurement mode.
}

//-------------------------------------------------------------------------------------------
// ADXL interrupt ISR 
//-------------------------------------------------------------------------------------------
ISR(INT0_vect) {
	EIMSK &= ~_BV(INT0);																		// disabling INT0 interrupt
	adxl_int_flag = TRUE;																		// raise adxl flag
}

//-------------------------------------------------------------------------------------------
// ISR of green and red leds PWM processing which is on OCR0A and OCR0B controlled by Counter/Timer 0
//-------------------------------------------------------------------------------------------
ISR(TIMER0_OVF_vect) {
	
	if (!smooth_light_t0)	{
	
	// ----------------------------green led--------------------
		if(!g_led_slower) {

			g_led_inc_down? --OCREG_G_LED: ++OCREG_G_LED;
		
			if (OCR0A == 0xFF) {
				g_led_inc_down = TRUE;
			} else 	if (OCR0A == 0) {
				g_led_inc_down = FALSE;
			}
		
			g_led_slower = g_led_slower_max;
		
		} else {
			--g_led_slower;
		}

	// ----------------------------red led--------------------	
		if(!r_led_slower) {
			r_led_inc_down? --OCREG_R_LED: ++OCREG_R_LED;
		
			if (OCR0B == 0xFF) {
				r_led_inc_down = TRUE;
			} else if (OCR0B == 0) {
				r_led_inc_down = FALSE;
			}
		
			r_led_slower = r_led_slower_max;
		} else {
			--r_led_slower;
		}
		
		
	} else if(smooth_light_t0 == SL_UP) {
		
		if (OCREG_R_LED == SL_THRESH) {  // only red (!) is here
			OCREG_R_LED = r_led_ocreg;
			OCREG_G_LED = g_led_ocreg;
			smooth_light_t0 = SL_OFF;
		} else {
			
			if(!smooth_light_t0_slower){
				++OCREG_R_LED;
				++OCREG_G_LED;
				smooth_light_t0_slower = SL_SLOWER_MAX;
			} else {
				--smooth_light_t0_slower;
			}
		}
		
	} else if(smooth_light_t0 == SL_DOWN) {

		if(!smooth_light_t0_slower){		
			if(OCREG_R_LED == 0) { // only red (!) is here
				smooth_light_t0 = SL_OFF;
			} else {
				--OCREG_R_LED;
				--OCREG_G_LED;
				smooth_light_t0_slower = SL_SLOWER_MAX+2;
			}
		}else {
			--smooth_light_t0_slower;
		}
	}
}

//-------------------------------------------------------------------------------------------
// ISR of blue led PWM on Counter/Timer 1
//-------------------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect) {
	
	if(!smooth_light_t1) {
		if(!b_led_slower){
			b_led_inc_down? --OCREG_B_LED: ++OCREG_B_LED;

			if (OCR1AL == 0xFF) {
				b_led_inc_down = TRUE;
			}
			if (OCR1AL == 0) {
				b_led_inc_down = FALSE;
			}
			b_led_slower = b_led_slower_max;
		} else {
			--b_led_slower;
		}
	} else if(smooth_light_t1 == SL_UP) {
		
		if (OCREG_B_LED == SL_THRESH) { // if reach the threshold 
			OCREG_B_LED = b_led_ocreg;
			smooth_light_t1 = SL_OFF;
		} else {						// make fading with slower
			if(!smooth_light_t1_slower) {
				++OCREG_B_LED;
				smooth_light_t1_slower = SL_SLOWER_MAX;
			} else {
				--smooth_light_t1_slower;
			}
		}
		
	} else if(smooth_light_t1 == SL_DOWN) {
	
		if(OCREG_B_LED == 0) {
			smooth_light_t1 = SL_OFF;
		} else {
			--OCREG_B_LED;
		}
	}
}

//-------------------------------------------------------------------------------------------
// ISR to make buzzer sound wave
//-------------------------------------------------------------------------------------------
ISR(TIMER2_COMPA_vect) {
	BUZZER_PORT ^= _BV(BUZZER_PIN);
}

//-------------------------------------------------------------------------------------------
// ADC init
//-------------------------------------------------------------------------------------------
void inline ADC_init(){
	ACSR &= ~_BV(ACIE);							// disable ADC interrupt
	ACSR |= _BV(ACD);							// disable the comparator	
	DIDR0 = _BV(ADC0D);							// block digital input buffer on channel 0 (vcc probe) to save power
	ADMUX =  ADMUX_MASK;						// set internal ref
	ADCSRA = _BV(ADPS2);						// set prescaler to 16 
}

//-------------------------------------------------------------------------------------------
// ADC 10 bit measurement on channel ch
//-------------------------------------------------------------------------------------------
uint16_t ADC_measure_10bit(uint8_t ch) {

	ADMUX  = ADMUX_MASK | ch;					// set ADC channel

	ADCSRA |= _BV(ADSC);						// start measurement
	while(!(ADCSRA & _BV(ADIF))) {}				// wait till it's done
	ADCSRA |= _BV(ADIF);						// clear flag

	return ADCW;								// there is no ADCW reg in AVR. Compiler substitute it by reading first from ADCL and then from ADCH
}

//-------------------------------------------------------------------------------------------
// ADC 8 bit measurement on channel ch
//-------------------------------------------------------------------------------------------
uint8_t ADC_measure_8bit(uint8_t ch) {
	
	ADMUX  = ADMUX_MASK |_BV(ADLAR) | ch;       // set channel and left result adjustment
	
	ADCSRA |= _BV(ADSC);						// start measurement
	while(!(ADCSRA & _BV(ADIF))) {}
	ADCSRA |= _BV(ADIF);
	
	return ADCH;								// return only high byte
}

//-------------------------------------------------------------------------------------------
// get random val function based on free float pin voltage measurement. 
// works well for low bits
//-------------------------------------------------------------------------------------------
uint8_t a_rand(uint8_t bit_maks) {
	
	uint8_t val = (uint8_t) ADC_measure_10bit(FREE_FLOAT_ADC_CH);   // take lower byte of ADC 10 bit measurement
	val &= bit_maks;							                    // apply the mask
	return val;
}

//-------------------------------------------------------------------------------------------
// make 8 bit random val for led initial brightness
//-------------------------------------------------------------------------------------------
uint8_t color_start_rand() {
	return (a_rand(0b11) << 6) | (a_rand(0b11) << 4) | (a_rand(0b11) << 4 ) | (a_rand(0b11));
}

//-------------------------------------------------------------------------------------------
// evaluate  initial parameters for RGB leds and start timers for PWM processing
//-------------------------------------------------------------------------------------------
void RGB_rand_and_start() {
	r_led_ocreg = color_start_rand();
	g_led_ocreg = color_start_rand();
	b_led_ocreg = color_start_rand();
	
	smooth_light_t0 = SL_UP;
	smooth_light_t1 = SL_UP;
	
	OCREG_R_LED = 0;
	OCREG_G_LED = 0;
	OCREG_B_LED = 0;
	
	TCNT0 = color_start_rand();
	TCNT1 = color_start_rand();	

/*	
	uint8_t ttt = a_rand(1);
	r_led_inc_down = ttt;
		UARTPrintUint(ttt, 10);
		UARTPrint(" ");

		ttt = a_rand(1);
		g_led_inc_down = ttt;
		UARTPrintUint(ttt, 10);
		UARTPrint(" ");

		ttt = a_rand(1);
		b_led_inc_down = ttt;
		UARTPrintUint(ttt,10);
		UARTPrint(" ");
	
		ttt = a_rand(0b11);
		r_led_slower_max = ttt;
		UARTPrintUint(ttt, 10);
		UARTPrint(" ");

		ttt = a_rand(0b11);
		g_led_slower_max = ttt;
		UARTPrintUint(ttt, 10);
		UARTPrint(" ");

		ttt = a_rand(0b11);
		b_led_slower_max = ttt;
		UARTPrintUint(ttt, 10);
		UARTPrint(" ");
	
		UARTPrintln("");
*/
	r_led_inc_down = a_rand(1);
	g_led_inc_down = a_rand(1);
	b_led_inc_down = a_rand(1);

	r_led_slower_max =  a_rand(0b11);
	g_led_slower_max =  a_rand(0b11);
	b_led_slower_max =  a_rand(0b11);
	
	
	TCCR1A |= _BV(COM1A1);							// connecting OC1A channel in non-inverting mode
	TCCR0A |= _BV(COM0A1)| _BV(COM0B1);				// connecting OC0A and OC0B channels in non-inverting mode

	timer1_start();
	timer0_start();

}

//-------------------------------------------------------------------------------------------
// stop RGB leds flashing
//-------------------------------------------------------------------------------------------
void inline RGB_stop() {

	TCCR1A &= ~_BV(COM1A1);							// disconnecting OC1A channel 
	TCCR0A &= ~(_BV(COM0A1) | _BV(COM0B1));			// disconnecting OC0A and OC0B channels

	timer1_stop();
	timer0_stop();
	
	RGB_RLED_PORT &= ~_BV(RGB_RLED_PIN);
	RGB_GLED_PORT &= ~_BV(RGB_GLED_PIN);
	RGB_BLED_PORT &= ~_BV(RGB_BLED_PIN);
	
}

//-------------------------------------------------------------------------------------------
// shift register update via SPI bus
//-------------------------------------------------------------------------------------------
void sr_update(uint8_t val) {
	sr_latch_low();
	SPI_MasterTransmit(val);
	sr_latch_high();	
}






