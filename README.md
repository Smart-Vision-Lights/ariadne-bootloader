# [Ariadne Bootloader for Arduino][1]

## Hex branch
This branch allows for TFTP uploading of HEX data instead of binary data. **Note: If built using the `atmega32u4_w5500_alwaysBootload_passthrough_9600` option (which is the version that is described in most of my following additions to this Readme), ensure the EEPROM of your ATMEGA32U4 (which is the chip this branch was developed for) has been programmed with the appropriate bits set so that the bootloader knows there is a valid program in memory and doesn't sit waiting forever to bootload. A sample EEPROM `.hex` that should work can be found in `ariadne-bootloader\bootloaders\ariadne\src\32u4_eeprom.hex`**

Everything works basically the same as the binary version, only instead of sending a `.bin` file, you send a `.hex` file using the same TFTP protocol (however there are quite a few additional features described below).

All of the `.hex` data received up to the maximum memory size of the 32u4 (the micro I developed this branch for) is written to the Atmel's program memory. If the `.hex` file exceeds the maximum memory size, instead of throwing an error, it passes the rest of the `.hex` file along the serial port with the format `HEX,<LineOfHexData>\n`, an example of this would be `HEX,:107030002EC300002CC300002AC3000028C3000098\n`. The Atmel then waits for an acknowledgement message back from the secondary processor in the form of the character `K` (the Atmel will also throw an error to the TFTP client if the secondary processor provides the character `E` instead). Once the Atmel receives this acknowledgement message (provided it is not an error message), it then sends the next line of `.hex` data to the serial port. This process continues until the end of the TFTP transaction is reached.

The main application of this feature is as follows. If you have a secondary processor connected to the Atmel's serial port, and if this secondary processor has a bootloader partition in its program memory that is **equal to or greater in size** to the Atmel's program memory, then you can combine the `.hex` file for the compiled Atmel code, with the `.hex` file for the compiled code for the secondary processor, and so long as there is an empty partition in the secondary processor's code where its bootloader normally resides, the Atmel code should fit nicely in that gap. Then, when you upload this combined `.hex` file to the Atmel's TFTP hex bootloader, so long as the bootloader on the secondary processor is set up to take the `.hex` data that the Atmel provides it over the serial port, and write it to it's program memory (and provide the appropriate acknowledgement messages), this allows you to effectively bootload 2 microprocessors using 1 TFTP port and 1 `.hex` file.

**Note: At the beginning of the process of passing the `.hex` data to the serial port, there is 1 message sent with no data (i.e. it looks like this `HEX,\n`). This message is intended to signal to the external processor that the Atmel is ready to pass it the rest of the `.hex` file. If I recall correctly, the Atmel then waits for an acknowledgement character `K` as usual. After it receives this acknowledgement, it then proceeds to pass the `.hex` data over the serial port as described previously.**

**If the bootloader is built with serial passthrough enabled, right before it exits to the main program, it will send "EXIT\n" on the serial port to tell any external devices connected to this port to exit their own bootloader and run their main program**


### Bootloader for Arduino with Ethernet
This is a beta stage bootloader for Arduino Ethernet board and the regular
Arduino with Ethernet Shield. It is based on previous unfinished work by the
Arduino developers. The bootloader implements a TFTP server on the Arduino board
and flashing works using any regular TFTP client.

### License
This is free software and it is released under the
[GPLv2, GNU General Public License][99]

[1]: http://loathingkernel.github.io/ariadne-bootloader/
[99]: https://www.gnu.org/licenses/gpl-2.0.html
