take the file you want to upload to your board named "ariadne_atmega32u4_w5500<type>.hex", copy it to the
directory above this one (ariadne), rename that file to remove the type, "ariadne_atmega32u4_w5500.hex", then
you'll be able to burn that bootloader using the Arduino IDE config.


To convert .hex to .bin, which is required to upload via ethernet. You can usually find avr-objcopy (at least for pre-electron based IDE's) in "C:\"Program Files (x86)"\Arduino\hardware\tools\avr\bin"

C:\"Program Files (x86)"\Arduino\hardware\tools\avr\bin\avr-objcopy -I ihex [sketch].hex -O binary [sketch].bin