#include <avr/io.h>
#include <avr/eeprom.h>

// Network settings
#define IP_ADDR     150,140,5,120
#define SUBNET_MASK 255,255,255,128
#define GW_ADDR     150,140,5,1
#define MAC_ADDR    0xDE,0xAD,0xBE,0xEF,0xFE,0xED

// Offset of pins inside PORTB
#define SCK_PIN   5  //PB5  Pin 13
#define MISO_PIN  4  //PB4  Pin 12
#define MOSI_PIN  3  //PB3  Pin 11
#define SS_PIN    2  //PB2  Pin 10

#define EEPROM_SIG_1 ((uint8_t*)0)
#define EEPROM_SIG_2 ((uint8_t*)1)
#define EEPROM_DATA ((uint8_t*)2)
#define EEPROM_SIG_1_VALUE (0x55)
#define EEPROM_SIG_2_VALUE (0xAA)

#define SS_LOW() PORTB &= ~_BV(SS_PIN)
#define SS_HIGH() PORTB |= _BV(SS_PIN)

#define SPI_WRITE (0xF0)
#define SPI_READ (0x0F)

#define REGISTER_BLOCK_SIZE 28


void netWriteReg(uint16_t address, uint8_t value);
uint8_t netReadReg(uint16_t address);
uint16_t netReadWord(uint16_t address);
void netWriteWord(uint16_t address, uint16_t value);

void netInit();