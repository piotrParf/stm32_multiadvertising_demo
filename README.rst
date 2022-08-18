.. _bluetooth-ibeacon-sample:

Bluetooth: iBeacon
##################

Overview
********
Based on iBeacon zephyr demo to demonstrate two beacons advertised simultaneously

Requirements
************

tried on booth

* stm32wb55 nucleo
* nrf5340dk - does not run changing of public adress

TIPS 
********************
For STM32wb55 there is need to turn on extended advertising in config options and install extended adv firmware into stm32wb55.

use hal_stm32_fork with commits making compatible with stm32 cube 1.14(https://github.com/zephyrproject-rtos/hal_stm32/pull/146). Also apply patches hal_stm32_update_ext.diff to hal_stm32 module of zephyr
and main_ext_zephyr.diff to zephyr main (3217d97)

