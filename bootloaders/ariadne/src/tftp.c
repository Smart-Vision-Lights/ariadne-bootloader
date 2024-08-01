/* Name: tftp.c
 * Author: .
 * Copyright: Arduino
 * License: GPL http://www.gnu.org/licenses/gpl-2.0.html
 * Project: ethboot
 * Function: tftp implementation and flasher
 * Version: 0.2 tftp / flashing functional
 */

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/boot.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "util.h"
#include "spi.h"
#include "net.h"
#include "neteeprom.h"
#include "tftp.h"
#include "validate.h"
#include "serial.h"
#include "debug.h"
#include "debug_tftp.h"

/** Opcode?: tftp operation is unsupported. The bootloader only supports 'put' */
#define TFTP_OPCODE_ERROR_LEN 12
const unsigned char tftp_opcode_error_packet[]  PROGMEM = "\0\5" "\0\4" "Opcode?";
/** Full: Binary image file is larger than the available space. */
#define TFTP_FULL_ERROR_LEN 9
const unsigned char tftp_full_error_packet[]    PROGMEM = "\0\5" "\0\3" "Full";
/** General catch-all error for unknown errors */
#define TFTP_UNKNOWN_ERROR_LEN 10
const unsigned char tftp_unknown_error_packet[] PROGMEM = "\0\5" "\0\0" "Error";
/** Invalid image file: Doesn't look like a binary image file */
#define TFTP_INVALID_IMAGE_LEN 23
const unsigned char tftp_invalid_image_packet[] PROGMEM = "\0\5" "\0\0" "Invalid image file";

uint16_t lastPacket = 0, highPacket = 0;
uint8_t tftpFlashing = FALSE;
#ifndef TFTP_RANDOM_PORT
uint16_t tftpTransferPort;
#endif

// How big of a buffer to fill with binary data from the .hex file
#define BINARY_BUFFER_SIZE 16

// How long to wait for an ack from the serial bus
#define SERIAL_BUS_FLASH_TIMEOUT 10


static void sockInit(uint16_t port)
{
	DBG_TFTP(
		tracePGMlnTftp(mDebugTftp_SOCK);
		tracenum(port);
	)

	spiWriteReg(REG_S3_CR, S3_W_CB, CR_CLOSE);
    while(spiReadReg(REG_S3_CR, S3_R_CB)) {
		//wait for command to complete
	}

	do {
        // Write interrupt
		spiWriteReg(REG_S3_IR, S3_W_CB, 0xFF);
		// Write mode
		spiWriteReg(REG_S3_MR, S3_W_CB, MR_UDP);
		// Write TFTP Port
		spiWriteWord(REG_S3_PORT0, S3_W_CB, port);
		// Open Socket
		spiWriteReg(REG_S3_CR, S3_W_CB, CR_OPEN);
		while(spiReadReg(REG_S3_CR, S3_R_CB)) {
			//wait for command to complete
 		}
		// Read Status
		if(spiReadReg(REG_S3_SR, S3_R_CB) != SOCK_UDP)
			// Close Socket if it wasn't initialized correctly
			spiWriteReg(REG_S3_CR, S3_W_CB, CR_CLOSE);

		// If socket correctly opened continue
	} while(spiReadReg(REG_S3_SR, S3_R_CB) != SOCK_UDP);
}


uint8_t hexCharToInt(char c) {
    c = toupper(c); // Convert to uppercase to handle both 'a'-'f' and 'A'-'F'
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    } else {
        // Handle invalid character error
        return 0; // Or any appropriate error handling
    }
}

uint32_t hexStringToUint(const char *hexString, uint8_t size) {
    
    uint8_t nibbles[8] = {hexCharToInt(hexString[0]), hexCharToInt(hexString[1]), hexCharToInt(hexString[2]), hexCharToInt(hexString[3]),
                            hexCharToInt(hexString[4], hexCharToInt(hexString[5]), hexCharToInt(hexString[6]), hexCharToInt(hexString[7]))}; 
    // uint8_t highNibble = hexCharToInt(hexString[0]);
    // uint8_t lowNibble = hexCharToInt(hexString[1]);
    
    if ( size == 2 )
    {
      return (nibbles[0] << 4) | nibbles[1];
    }else if (size == 8)
    {
      return (nibbles[0] << 28) | (nibbles[1] << 24) | (nibbles[2] << 20) | (nibbles[3] << 16) |
               (nibbles[4] << 12) | (nibbles[5] << 8) | (nibbles[6] << 4) | (nibbles[7]);
    }

    return 0;
}

// Validates that this is a char 0 through F, or a '\r', which we still need to use for indexing
uint8_t isHexChar(uint8_t value)
{
  return value == '0' || value == '1' || value == '2' || value == '3' || value == '4' || 
          value == '5' || value == '6' || value == '7' || value == '8' || value == '9' ||
           value == 'A' || value == 'B' || value == 'C' || value == 'D' || value == 'E' ||
            value == 'F' || value == '\r' || value == '\n' || value == ':';
}


// Buffer to hold the data to be written to the Atmel
uint8_t binaryBuffer[BINARY_BUFFER_SIZE] = {'\0'};
uint16_t binaryBufferIndex = 0;
// Buffer to hold any incomplete hex chars
uint8_t overflowBuffer[2] = {'\0', '\0'};

// Overflow for binary buffer
uint8_t binaryBufferOverflow[BINARY_BUFFER_SIZE] = {'\0'};
uint16_t binaryBufferOverflowIndex = 0;
// Number of times the binary buffer has been filled
uint16_t fillCount = 0;

// Index to tell us how many chars we've let pass by
uint8_t waitIndex = 0;
// Whether or not we are waiting
uint8_t areWaiting = 0;
// Whether or not the binary buffer is full and ready to be written
uint8_t writeData = 0;
// Record type of hex line
uint8_t recordType = '\0';
// Buffer to hold the current hex line in its complete char form
#define HEX_LINE_BUFFER_SIZE 64
uint8_t hexLine[HEX_LINE_BUFFER_SIZE] = {'0'};
uint8_t hexLineIndex = 0;
// Buffer to hold the address chars of the current hex line
char hexAddressChars[8] = {'0','0','0','0','\0','\0','\0','\0'};
// Used to extend the address if necessary
char hexAddressExtensionChars[4] = {'\0','\0','\0','\0'};

uint8_t hexAddressIndex = 4;
// How big the current line of data is
char hexSizeChars[2] = {'\0', '\0'};
uint8_t hexSizeCharsIndex = 0;
uint16_t hexSize = 0;
// Actual location in memory to write to
uint32_t hexAddress = 0;
// Last write reached (it will probably be smaller than 512 bytes)
uint8_t lastWrite = 0;
uint8_t doneLastWrite = 0;
uint16_t offset = 0; // Block offset


#if (DEBUG_TFTP > 0)
static uint8_t processPacket(uint16_t packetSize)
#else
static uint8_t processPacket(void)
#endif
{
  // Added 1 to account for any overflow from the previous hex line
	uint8_t buffer[TFTP_PACKET_MAX_SIZE+4] = {'\0'};


	uint16_t readPointer;
	address_t writeAddr;
	// Transfer entire packet to RAM
	uint8_t* bufPtr = buffer;
	uint16_t count;



	DBG_TFTP(
		tracePGMlnTftp(mDebugTftp_START);
		tracenum(packetSize);

		if(packetSize >= 0x800) tracePGMlnTftp(mDebugTftp_OVFL);

		DBG_BTN(button();)
	)

	// Read data from chip to buffer
	readPointer = spiReadWord(REG_S3_RX_RD0, S3_R_CB);

	DBG_TFTP_EX(
		tracePGMlnTftp(mDebugTftp_RPTR);
		tracenum(readPointer);
	)

#if defined(__WIZ_W5500__)
	//W5500 auto increments the readpointer by memory mapping a 16bit addr
#else
	if(readPointer == 0) readPointer += S3_RX_START;
#endif

	for(count = TFTP_PACKET_MAX_SIZE; count--;) {

		DBG_TFTP_EX(
			if((count == TFTP_PACKET_MAX_SIZE - 1) || (count == 0)) {
				tracePGMlnTftp(mDebugTftp_RPOS);
				tracenum(readPointer);
			}
		)

#if defined(__WIZ_W5500__)
		*bufPtr++ = spiReadReg(readPointer++, S3_RXBUF_CB);
		//W5500 auto increments the readpointer by memory mapping a 16bit addr
		//Use uint16_t overflow from 0xFFFF to 0x10000 to follow W5500 internal pointer
#else
		*bufPtr++ = spiReadReg(readPointer++, 0);
		if(readPointer == S3_RX_END) readPointer = S3_RX_START;
#endif

	}

	spiWriteWord(REG_S3_RX_RD0, S3_W_CB, readPointer);     // Write back new pointer
	spiWriteReg(REG_S3_CR, S3_W_CB, CR_RECV);

	while(spiReadReg(REG_S3_CR, S3_R_CB));

	DBG_TFTP_EX(
		tracePGMlnTftp(mDebugTftp_BLEFT);
		tracenum(spiReadWord(REG_S3_RX_RSR0, S3_R_CB));
	)

	// Dump packet
	DBG_TFTP_EX(
		bufPtr = buffer;
		tracePGM(mDebugTftp_NEWLINE);

		for(count = TFTP_PACKET_MAX_SIZE / 2; count--;) {
			uint16_t val = *bufPtr++;
			val |= (*bufPtr++) << 8;
			tracenum(val);

			if((count % 8) == 0 && count != 0) tracePGM(mDebugTftp_NEWLINE);
			else putch(0x20); //Print space
		}
	)

	// Set up return IP address and port
	uint16_t i;

	for(i = 0; i < 6; i++) spiWriteReg(REG_S3_DIPR0 + i, S3_W_CB, buffer[i]);

	DBG_TFTP(tracePGMlnTftp(mDebugTftp_RADDR);)




	// Parse packet
	uint16_t tftpDataLen = (buffer[6] << 8) + buffer[7];
	uint16_t tftpOpcode  = (buffer[8] << 8) + buffer[9];
	uint16_t tftpBlock   = (buffer[10] << 8) + buffer[11];

  #define HEX_HEADER_SIZE 9


	DBG_TFTP(
		tracePGMlnTftp(mDebugTftp_BLOCK);
		tracenum(tftpBlock);
		tracePGM(mDebugTftp_OPCODE);
		tracenum(tftpOpcode);
		tracePGM(mDebugTftp_DLEN);
		tracenum(tftpDataLen - (TFTP_OPCODE_SIZE + TFTP_BLOCKNO_SIZE));
	)

	if((tftpOpcode == TFTP_OPCODE_DATA)
		&& ((fillCount > MAX_ADDR / 0x200) || (tftpBlock < highPacket) || (tftpBlock > highPacket + 1)))
		tftpOpcode = TFTP_OPCODE_UKN;

	if(tftpDataLen > (0x200 + TFTP_OPCODE_SIZE + TFTP_BLOCKNO_SIZE))
		tftpOpcode = TFTP_OPCODE_UKN;

	uint8_t returnCode = ERROR_UNKNOWN;
	uint16_t packetLength;


	switch(tftpOpcode) {

		case TFTP_OPCODE_RRQ: // Read request
			DBG_TFTP(tracePGMlnTftp(mDebugTftp_OPRRQ);)
			break;

		case TFTP_OPCODE_WRQ: // Write request
			// Valid WRQ -> reset timer
			resetTick();

			DBG_TFTP(tracePGMlnTftp(mDebugTftp_OPWRQ);)

			// Flagging image as invalid since the flashing process has started
			eeprom_write_byte(EEPROM_IMG_STAT, EEPROM_IMG_BAD_VALUE);

#if defined(RANDOM_TFTP_DATA_PORT)
			sockInit((buffer[4] << 8) | ~buffer[5]); // Generate a 'random' TID (RFC1350)
#else
			sockInit(tftpTransferPort);
#endif

			DBG_TFTP(
				tracePGMlnTftp(mDebugTftp_NPORT);
#if defined(RANDOM_TFTP_DATA_PORT)
				tracenum((buffer[4] << 8) | (buffer[5] ^ 0x55));
#else
				tracenum(tftpTransferPort);
#endif
			)

			lastPacket = highPacket = 0;
			returnCode = ACK; // Send back acknowledge for packet 0
			break;

		case TFTP_OPCODE_DATA:


      // Cycle through the data in the buffer
      for (i = 12; i < TFTP_PACKET_MAX_SIZE; i++)
      {

        // Current char to process
        uint8_t curChar;
        // Next char
        uint8_t nextChar;
        // Carriage return check
        uint8_t thirdChar;

        // Check for any char left over from last buffer
        if ( overflowBuffer[0] != '\0' )
        {
          // Process left over char
          curChar = overflowBuffer[0];
          // Reset overflow
          overflowBuffer[0] = '\0';

          // Check for a second overflow char
          if ( overflowBuffer[1] != '\0' )
          {
            // Process left over char
            nextChar = overflowBuffer[1];
            // Reset overflow
            overflowBuffer[1] = '\0';
            // Carriage return check
            thirdChar = buffer[i];
            // Ensure we don't increment i this time around
            i-=2;
          }else{
            // Next char
            nextChar = buffer[i];
            // Carriage return check
            thirdChar = buffer[i+1];
            // Ensure we only increment i by 1 and not 2
            i--;
          }


        }else
        {
          // Get current char in buffer
          curChar = buffer[i];
          // Get next char
          nextChar = buffer[i+1];
          // Carriage return check
          thirdChar = buffer[i+2];
        }


        // Look for a ':'
        if ( curChar == ':' )
        {
          // Set flag
          areWaiting = 1;
        }

        // Increment waiting counter
        if ( 1 == areWaiting && waitIndex < HEX_HEADER_SIZE )
        {
          // if ( 1 == lastWrite )
          // {
          //   hexLine[hexLineIndex++] = curChar;
          // }
          // If the current char is one of the 2 size chars
          if ( waitIndex >= HEX_HEADER_SIZE - 8 && waitIndex <= HEX_HEADER_SIZE - 7 )
          {
            // Get the current char
            hexSizeChars[hexSizeCharsIndex++] = curChar;

          // If the current char is one of the 4 address chars
          }else if ( waitIndex >= HEX_HEADER_SIZE - 6 && waitIndex <= HEX_HEADER_SIZE - 3 && hexAddressIndex < 8 )
          {
            if ( hexSizeChars[0] != '\0' )
            {
              // Get size of line
              hexSize = hexStringToUint(hexSizeChars, 2);

              // Reset char and index buffers
              hexSizeChars[0] = '\0';
              hexSizeChars[1] = '\0';
              hexSizeCharsIndex = 0;
            }
            // Get the current char
            hexAddressChars[hexAddressIndex++] = curChar;
          // Reset buffers when necessary
          }else if ( hexAddressIndex >= 8 )
          {

          }
          // If the current char is the last one of the record type
          if ( waitIndex == HEX_HEADER_SIZE-1 )
          {
            // Get it
            recordType = curChar;
            // Add memory extension if we need to
            if ( recordType == '4' )
            {
              hexAddressChars[0] = hexAddressChars[4];
              hexAddressChars[1] = hexAddressChars[5];
              hexAddressChars[2] = hexAddressChars[6];
              hexAddressChars[3] = hexAddressChars[7];
              hexAddressChars[4] = '\0';
              hexAddressChars[5] = '\0';
              hexAddressChars[6] = '\0';
              hexAddressChars[7] = '\0';
            }else
            {
              // Get memory location
              hexAddress = hexStringToUint(hexAddressChars, 8);
            }

          }
          // Increment wait index
          waitIndex++;

        }else if ( 1 == areWaiting && waitIndex >= HEX_HEADER_SIZE )
        {
          // Increment i a second time, to process chars by 2's
          i++;

          // Check to make sure there is data to be read, and that we haven't reached the end of the file
          if ( isHexChar(curChar) && isHexChar(nextChar) && isHexChar(thirdChar) && i < TFTP_PACKET_MAX_SIZE-1 )
          {
            // Check to make sure this isn't the checksum, and that we haven't already gathered all the data we need
            if ( thirdChar != '\r' && binaryBufferIndex < hexSize && recordType == '0' )
            {

                // // Add chars to hex line buffer
                // hexLine[hexLineIndex++] = curChar;
                // hexLine[hexLineIndex++] = nextChar;

                // Convert the current hex char pair to binary
                char hexString[] = {curChar, nextChar};

                // Get value from chars
                uint8_t binaryVal = hexStringToUint(hexString, 2);

                // Append it to the binary array
                binaryBuffer[binaryBufferIndex++] = binaryVal;

            // If it is, move the index to the next ':'
            }else
            {
              // Ensure this isn't an address extension
              if ( recordType != '4' )
              {
                // if ( 1 == lastWrite )
                // {
                //   // Add chars to hex line buffer
                //   // hexLine[hexLineIndex++] = curChar;
                //   // hexLine[hexLineIndex++] = nextChar;
                //   // hexLine[hexLineIndex++] = thirdChar;
                //   // hexLine[hexLineIndex++] = '\n';
                // }
                // Set packet length to size of hex line
                packetLength = hexSize;

                // Get address where to write current line
                writeAddr = hexAddress;

                

                uint16_t writeValue;
                static uint16_t prevAddress;
                static uint8_t prevIndex;
                uint8_t index = 0;


                // Check to see if we've filled the Atmel's memory yet
                if( (writeAddr + packetLength) > MAX_ADDR || 1 == lastWrite )  {

                  // // Flash is full - abort with an error before a bootloader overwrite occurs
                  // // Application is now corrupt, so do not hand over.

                  // DBG_TFTP(tracePGMlnTftp(mDebugTftp_FULL);)

                  // returnCode = ERROR_FULL;


                  // Finish writing the last page if we haven't already
                  if ( 0 == lastWrite )
                  {

                    // Round up to the next page
                    while(offset % SPM_PAGESIZE)
                    {
                      // Fill the rest of the last page with null chars
                      writeValue = '\0';
                      boot_page_fill(prevAddress + prevIndex, writeValue);

                      offset+=2;
                      prevIndex+=2;
                    }

                    // Write the last page
                    boot_page_erase(prevAddress + prevIndex - SPM_PAGESIZE);
                    boot_spm_busy_wait();
                    boot_page_write(prevAddress + prevIndex - SPM_PAGESIZE);
                    boot_spm_busy_wait();
        #if defined(RWWSRE)
                    // Reenable read access to flash
                    boot_rww_enable();
                    // // Print out what we just wrote
                    // for (int m = prevAddress+prevIndex-SPM_PAGESIZE; m < prevAddress+prevIndex; m++)
                    // {
                    //   putch(pgm_read_byte_near(m));
                    // }
        #endif

                    lastWrite = 1;
                    // doneLastWrite = 1;
                    // putch('@');
                  }

                  // Pass hex data to the serial port

                  // Add label
                  putch('H');
                  putch('E');
                  putch('X');
                  putch(',');
                  // Add hex address chars
                  putch(hexAddressChars[4]);
                  putch(hexAddressChars[5]);
                  putch(hexAddressChars[6]);
                  putch(hexAddressChars[7]);
                  putch(',');
                  // Add record type
                  putch(recordType);
                  putch(',');
                  // Add size (number of bytes in the line)
                  putch(hexSize);
                  // Cycle through the entire packet
                  for ( uint8_t n = 0; n < binaryBufferIndex; n++ )
                  {
                    // Send that data on the serial bus
                    putch(binaryBuffer[n]);
                  }
                  // End line with a newline
                  putch('\n');

                  // Wait for an ack back on the serial bus
                  while ( getch() != 'K' && getch() != '\0' ) {}

                  // // Abort if we timed out
                  // if ( getTick() >= SERIAL_BUS_FLASH_TIMEOUT )
                  // {
                  //   returnCode = ERROR_UNKNOWN;
                  //   break;
                  // }

                  // // Break out of the loop if we've reached the end of the file
                  // if ( recordType == '1' )
                  // {
                  //   break;
                  // }

                  // // Reset hex line
                  // for ( uint8_t n = 0; n < HEX_LINE_BUFFER_SIZE; n++ )
                  // {
                  //   hexLine[n] = '\0';
                  // }
                  // // Reset hex line index
                  // hexLineIndex = 0;

                  // Either the Atmel is full, or else the memory address is outside our range,
                  // so move on to just printing out the hex data on the serial bus



                  // putch('!');


                } else {

                  // uint8_t* pageBase = buffer + (UDP_HEADER_SIZE + TFTP_OPCODE_SIZE + TFTP_BLOCKNO_SIZE); // Start of block data
                  uint8_t* pageBase = binaryBuffer; // Start of block data

                                  // Add label
                  // putch('H');
                  // putch('E');
                  // putch('X');
                  // putch(',');
                  // // Add hex address chars
                  // putch(hexAddressChars[4]);
                  // putch(hexAddressChars[5]);
                  // putch(hexAddressChars[6]);
                  // putch(hexAddressChars[7]);
                  // putch(',');
                  // // Add record type
                  // putch(recordType);
                  // putch(',');
                  // // Add size (number of bytes in the line)
                  // putch(hexSize);
                  // // Cycle through the entire packet
                  // for ( uint8_t n = 0; n < binaryBufferIndex; n++ )
                  // {
                  //   // Send that data on the serial bus
                  //   putch(binaryBuffer[n]);
                  // }
                  // // End line with a newline
                  // putch('\n');



          //         if(writeAddr == 0) {
          //           // First sector - validate
          // //           if(!validImage(pageBase)) {

          // // #if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
          // //             /* FIXME: Validity checks. Small programms (under 512 bytes?) don't
          // //             * have the the JMP sections and that is why app.bin was failing.
          // //             * When flashing big binaries is fixed, uncomment the break below.*/
          // //             returnCode = INVALID_IMAGE;
          // //             break;
          // // #endif
          // //           }
          //         }
          
                  do {
                    
                    
                    if ( recordType != '1' )
                    {
                      writeValue = (pageBase[index]) | (pageBase[index + 1] << 8);
                      boot_page_fill(writeAddr + index, writeValue);

                      offset+=2;
                      index+=2;

                      // Only write the page if it is full, or we are done writing all data
                      if( offset % SPM_PAGESIZE == 0 ) {
                        boot_page_erase(writeAddr + index - SPM_PAGESIZE);
                        boot_spm_busy_wait();
                        boot_page_write(writeAddr + index - SPM_PAGESIZE);
                        boot_spm_busy_wait();
            #if defined(RWWSRE)
                        // Reenable read access to flash
                        boot_rww_enable();
                        
                        // // Print out what we just wrote
                        // for (int m = writeAddr+index-SPM_PAGESIZE; m < writeAddr+index; m++)
                        // {
                        //   putch(pgm_read_byte_near(m));
                        // }
            #endif
                      }
                    }else
                    {
                      break;
                    }
                  } while( index < packetLength );



                  if ( recordType == '1' )
                  {
                      // Round up to the next page
                      while(offset % SPM_PAGESIZE)
                      {
                        // Fill the rest of the last page with null chars
                        writeValue = '\0';
                        boot_page_fill(prevAddress + prevIndex, writeValue);

                        offset+=2;
                        prevIndex+=2;
                      }

                      // Write the last page
                      boot_page_erase(prevAddress + prevIndex - SPM_PAGESIZE);
                      boot_spm_busy_wait();
                      boot_page_write(prevAddress + prevIndex - SPM_PAGESIZE);
                      boot_spm_busy_wait();
          #if defined(RWWSRE)
                      // Reenable read access to flash
                      boot_rww_enable();
                      // // Print out what we just wrote
                      // for (int m = prevAddress+prevIndex-SPM_PAGESIZE; m < prevAddress+prevIndex; m++)
                      // {
                      //   putch(pgm_read_byte_near(m));
                      // }
          #endif

                      break;
                  }




                  // Store last address and index
                  prevAddress = writeAddr;
                  prevIndex = index;


                }

              
              
              // If this is an address extension, store it for all future hex addresses
              }else
              {
                // hexAddressExtension = hexAddress;
              }

              // Reset binary buffer
              for (uint8_t k = 0; k < BINARY_BUFFER_SIZE; k++)
              {
                binaryBuffer[k] = '\0';
              }

              binaryBufferIndex = 0;


              // Reset hex address buffer
              // hexAddressChars[0] = '\0';
              // hexAddressChars[1] = '\0';
              // hexAddressChars[2] = '\0';
              // hexAddressChars[3] = '\0';
              hexAddressChars[4] = '\0';
              hexAddressChars[5] = '\0';
              hexAddressChars[6] = '\0';
              hexAddressChars[7] = '\0';
              hexAddressIndex = 4;
              
              i+=2;
              // Reset index
              waitIndex = 0;
              areWaiting = 0;
              // Reset record type
              recordType = '\0';

            }
          // Otherwise, store the current char(s) for the next time around
          }else
          {
            if ( isHexChar(curChar) )
            {
              overflowBuffer[0] = curChar;
            }
            if ( isHexChar(nextChar) )
            {
              overflowBuffer[1] = nextChar;
            }

            // Break
            break;
          }
        }
      }


      // Valid Data Packet -> reset timer
      resetTick();

      // Set the return code before packetLength gets rounded up
      if (tftpDataLen - (TFTP_OPCODE_SIZE + TFTP_BLOCKNO_SIZE) < TFTP_DATA_SIZE) returnCode = FINAL_ACK;
      else returnCode = ACK;

      DBG_TFTP(tracePGMlnTftp(mDebugTftp_OPDATA);)

      lastPacket = tftpBlock;

      if(returnCode == FINAL_ACK) {
        // Flash is complete
        // Hand over to application
        // offset = 0;

        // lastWrite = 0;
        DBG_TFTP(tracePGMlnTftp(mDebugTftp_DONE);)

        // Flag the image as valid since we received the last packet
        eeprom_write_byte(EEPROM_IMG_STAT, EEPROM_IMG_OK_VALUE);
      }

			break;

		// Acknowledgment
		case TFTP_OPCODE_ACK:

			DBG_TFTP(tracePGMlnTftp(mDebugTftp_OPACK);)

			break;

		// Error signal
		case TFTP_OPCODE_ERROR:

			DBG_TFTP(tracePGMlnTftp(mDebugTftp_OPERR);)

			/* FIXME: Resetting might be needed here too */
			break;

		default:
			DBG_TFTP(
				tracePGMlnTftp(mDebugTftp_INVOP);
				tracenum(tftpOpcode);
			)

#if defined(RANDOM_TFTP_DATA_PORT)
			sockInit((buffer[4] << 8) | ~buffer[5]); // Generate a 'random' TID (RFC1350)
#else
			sockInit(tftpTransferPort);
#endif
			/* FIXME: This is where the tftp server should be resetted.
			 * It can be done by reinitializig the tftpd or
			 * by resetting the device. I should find out which is best...
			 * Right now it is being done by resetting the timer if we have a
			 * data packet. */
			// Invalid - return error
			returnCode = ERROR_INVALID;
			break;

	}

	return(returnCode);
}


static void sendResponse(uint16_t response)
{
	uint8_t txBuffer[100];
	uint8_t* txPtr = txBuffer;
	uint8_t packetLength;
	uint16_t writePointer;

#if defined(__WIZ_W5500__)
	writePointer = spiReadWord(REG_S3_TX_WR0, S3_R_CB);
#else
	writePointer = spiReadWord(REG_S3_TX_WR0, 0) + S3_TX_START;
#endif

	switch(response) {
		default:

		case ERROR_UNKNOWN:
			// Send unknown error packet
			packetLength = TFTP_UNKNOWN_ERROR_LEN;
#if (FLASHEND > 0x10000)
			memcpy_PF(txBuffer, PROGMEM_OFFSET + (uint32_t)(uint16_t)tftp_unknown_error_packet, packetLength);
#else
			memcpy_P(txBuffer, tftp_unknown_error_packet, packetLength);
#endif
			break;

		case ERROR_INVALID:
			// Send invalid opcode packet
			packetLength = TFTP_OPCODE_ERROR_LEN;
#if (FLASHEND > 0x10000)
			memcpy_PF(txBuffer, PROGMEM_OFFSET + (uint32_t)(uint16_t)tftp_opcode_error_packet, packetLength);
#else
			memcpy_P(txBuffer, tftp_opcode_error_packet, packetLength);
#endif
			break;

		case ERROR_FULL:
			// Send unknown error packet
			packetLength = TFTP_FULL_ERROR_LEN;
#if (FLASHEND > 0x10000)
			memcpy_PF(txBuffer, PROGMEM_OFFSET + (uint32_t)(uint16_t)tftp_full_error_packet, packetLength);
#else
			memcpy_P(txBuffer, tftp_full_error_packet, packetLength);
#endif
			break;

		case ACK:
			if(lastPacket > highPacket) highPacket = lastPacket;

			DBG_TFTP(tracePGMlnTftp(mDebugTftp_SACK);)
			/* no break */

		case FINAL_ACK:

			DBG_TFTP(
				if(response == FINAL_ACK)
					tracePGMlnTftp(mDebugTftp_SFACK);
			)

			packetLength = 4;
			*txPtr++ = TFTP_OPCODE_ACK >> 8;
			*txPtr++ = TFTP_OPCODE_ACK & 0xff;
			// lastPacket is block code
			*txPtr++ = lastPacket >> 8;
			*txPtr = lastPacket & 0xff;
			break;
	}

	txPtr = txBuffer;

	while(packetLength--) {
		spiWriteReg(writePointer++, S3_TXBUF_CB, *txPtr++);
#if defined(__WIZ_W5500__)
		//W5500 auto increments the readpointer by memory mapping a 16bit addr
		//Use uint16_t overflow from 0xFFFF to 0x10000 to follow W5500 internal pointer
	}
	spiWriteWord(REG_S3_TX_WR0, S3_W_CB, writePointer);
#else
		if(writePointer == S3_TX_END) writePointer = S3_TX_START;
	}
	spiWriteWord(REG_S3_TX_WR0, S3_W_CB, writePointer - S3_TX_START);
#endif

	spiWriteReg(REG_S3_CR, S3_W_CB, CR_SEND);

	while(spiReadReg(REG_S3_CR, S3_R_CB));

	DBG_TFTP(tracePGMlnTftp(mDebugTftp_RESP);)
}


/**
 * Initializes the network controller
 */
void tftpInit(void)
{
	// Open socket
	sockInit(TFTP_PORT);

#if defined(RANDOM_TFTP_DATA_PORT)
#else
	if(eeprom_read_byte(EEPROM_SIG_3) == EEPROM_SIG_3_VALUE)
		tftpTransferPort = ((eeprom_read_byte(EEPROM_PORT + 1) << 8) + eeprom_read_byte(EEPROM_PORT));
	else
		tftpTransferPort = TFTP_DATA_PORT;
#endif

	DBG_TFTP(
		tracePGMlnTftp(mDebugTftp_INIT);
#if defined(RANDOM_TFTP_DATA_PORT)
#else
		tracePGMlnTftp(mDebugTftp_PORT);
		tracenum(tftpTransferPort);
#endif
	)
}


/**
 * Looks for a connection
 */
uint8_t tftpPoll(void)
{
	uint8_t response = ACK;
	// Get the size of the recieved data
	uint16_t packetSize = spiReadWord(REG_S3_RX_RSR0, S3_R_CB);
//	uint16_t packetSize = 0, incSize = 0;

// 	do {
// 		incSize = spiReadWord(REG_S3_RX_RSR0);
// 		if(incSize != 0) {
// 			_delay_ms(400);
// 			packetSize = spiReadWord(REG_S3_RX_RSR0);
// 		}
// 	} while (packetSize != incSize);

	if(packetSize) {
		tftpFlashing = TRUE;

		while((spiReadReg(REG_S3_IR, S3_R_CB) & IR_RECV)) {
			spiWriteReg(REG_S3_IR, S3_W_CB, IR_RECV);
			//FIXME: is this right after all? smaller delay but
			//still a delay and it still breaks occasionally
			_delay_ms(TFTP_PACKET_DELAY);
		}

		// Process Packet and get TFTP response code
#if (DEBUG_TFTP > 0)
		packetSize = spiReadWord(REG_S3_RX_RSR0, S3_R_CB);
		response = processPacket(packetSize);
#else
		response = processPacket();
#endif
		// Send the response
		sendResponse(response);
	}

	if(response == FINAL_ACK) {
		spiWriteReg(REG_S3_CR, S3_W_CB, CR_CLOSE);
		// Complete
		return(0);
	}

	// Tftp continues
	return(1);
}

