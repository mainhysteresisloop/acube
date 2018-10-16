/*
 * ADXL345.c
 *
 * Created: 19.03.2017 22:17:22
 * Author: Sergey Shelepin
 */ 
#include "ADXL345.h"

extern uint8_t SPI_MasterTransmit(uint8_t data); 
extern void	   UARTPrintUint(uint8_t ui, uint8_t raddix);
extern void    UARTPrint( const char *str );
extern void    UARTPrintUintBits(uint8_t ui);
extern void    UARTPrintln( const char *str );

//-------------------------------------------------------------------------------------------------------------------------------------
//read a certain adxl345 register via SPI
uint8_t ADXL345_read_register(uint8_t addr){
	adxl345_cs_low();                                  //Set the Chip select pin low to start an SPI packet.
	SPI_MasterTransmit(ADXL345_READ_REG_MASK | addr);  // 7 bit has to be 1 for reading
	uint8_t res = SPI_MasterTransmit(0x00);			   // getting reg value
	adxl345_cs_high();                                 //end of communication
	return res;
}

//-------------------------------------------------------------------------------------------------------------------------------------
void ADXL345_read_xyz_row(uint8_t *ad) {
	adxl345_cs_low();																// Set the Chip select pin low to start an SPI packet.
	SPI_MasterTransmit(ADXL345_READ_REG_MASK | ADXL345_MB_MASK | ADXL345_DATAX0 );  // starting with ADXL345_DATAX0
	for (uint8_t i = 0; i < 6; i++) {
		*(ad + i) = SPI_MasterTransmit(0x00);										// getting reg value
	}
	adxl345_cs_high();																// end of communication
}
//-------------------------------------------------------------------------------------------------------------------------------------
void ADXL345_read_xyz(int16_t *xyz) {
	
	uint8_t row_data[6];
	
	ADXL345_read_xyz_row(row_data);	
	
	*xyz = row_data[0] | (row_data[1] << 8);
	*(xyz + 1) = row_data[2] | (row_data[3] << 8);
	*(xyz + 2) = row_data[4] | (row_data[5] << 8);	
	
}
//-------------------------------------------------------------------------------------------------------------------------------------
// writes adxl345 register
void ADXL345_write_register(char addr, char value) {
	adxl345_cs_low();
	SPI_MasterTransmit(addr);														// 7th bit is zero for writing
	SPI_MasterTransmit(value);
	adxl345_cs_high();
}

//-------------------------------------------------------------------------------------------------------------------------------------
void ADXL345_print_details() {
	uint8_t reg_val = ADXL345_read_register(0x00);
	UARTPrint("DEVID=");
	UARTPrintUint(reg_val, 16);

	reg_val = ADXL345_read_register(ADXL345_BW_RATE);
	UARTPrint("   BW_RATE=");
	UARTPrintUint(reg_val, 16);
		
// 	UARTPrint(" 0b");
// 	UARTPrintUintBits(reg_val);
// 	UARTPrint("   ");   

	reg_val = ADXL345_read_register(ADXL345_POWER_CTL);
	UARTPrint("   POWER_CTL=");
	UARTPrintUint(reg_val, 16);
	UARTPrintln("");
	
}
