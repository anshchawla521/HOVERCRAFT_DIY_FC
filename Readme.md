# DIY Hovercraft firmware for stm32F405 board

Hovercraft firmware for mamba f405 mk2 based stm32 based flight controller
![Hovercraft image](assets/hover.jpg)

## comments

Beep working for cmd setting why telem needs to be set 1 i dont know
reverse and normal direction working
3d mode working when used with esc configurator.com (have to flash betaflight for that)
basic hovercraft working
PRE ARM Added
Basic Failsafe working on RX lost
CRC algo not touching as betaflight also uses same only
proper telemetry working

## changes

Telemetry only sent just after when packet received

## next steps

- implement filter
- receving algo , baud rate as f1000
- implement msp for esc configurator.com

## imp links

https://github.dev/betaflight/betaflight/betaflight/src/main/drivers/dshot_command.h
https://github.dev/betaflight/betaflight/betaflight/src/main/drivers/dshot.h
https://github.com/Carbon225/esp32-dshot/blob/master/DShotRMT.cpp
https://github.com/crsf-wg/crsf/wiki/Physical-Layer -> default baud rate is 416666