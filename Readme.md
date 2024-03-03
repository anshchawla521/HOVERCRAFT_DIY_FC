# DIY Hovercraft firmware for stm32F405 board

https://github.dev/betaflight/betaflight/betaflight/src/main/drivers/dshot_command.h
https://github.dev/betaflight/betaflight/betaflight/src/main/drivers/dshot.h
https://github.com/Carbon225/esp32-dshot/blob/master/DShotRMT.cpp

## comments
Beep working for cmd setting why telem needs to be set 1 i dont know
reverse and normal direction working
3d mode working when used with esc configurator.com (have to flash betaflight for that)
basic hovercraft working
PRE ARM Added 
Basic Failsafe working on RX lost

# next steps

-> implement filter
-> improve crc,receving algo , baud rate as f1000
-> implement crsf telemetry
-> implement msp for esc configurator.com