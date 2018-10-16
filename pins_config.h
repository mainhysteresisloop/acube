/*
 * pins_config.h
 *
 * Created: 19.03.2017 22:18:27
 * Author: Sergey Shelepin
 */ 
#include <avr/io.h>

#ifndef PINS_CONFIG_H_
#define PINS_CONFIG_H_

#define ADXL345_CS_DDR   DDRD
#define ADXL345_CS_PORT  PORTD
#define ADXL345_CS_PIN   PD7  

#define SLED_DDR		PORTC
#define SLED_PORT		PORTC
#define SLED_PIN		PC4

#define VDD_EN_DDR		DDRD
#define VDD_EN_PORT		PORTD
#define VDD_EN_PIN		PD4

#define RGB_RLED_DDR	DDRD
#define RGB_RLED_PORT	PORTD
#define RGB_RLED_PIN	PD5

#define RGB_GLED_DDR	DDRD
#define RGB_GLED_PORT	PORTD
#define RGB_GLED_PIN	PD6

#define RGB_BLED_DDR	DDRB
#define RGB_BLED_PORT	PORTB
#define RGB_BLED_PIN	PB1

#define SR_LATCH_DDR	DDRB
#define SR_LATCH_PORT	PORTB
#define SR_LATCH_PIN	PB0

#define BUZZER_DDR		DDRB
#define BUZZER_PORT		PORTB
#define BUZZER_PIN		PB2



#endif /* PINS_CONFIG_H_ */