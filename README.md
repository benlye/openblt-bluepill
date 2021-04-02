# OpenBLT-BluePill
OpenBLT Bootloader for STM32F103CB Blue Pill board

* 8KB is reserved for the bootloader, program flash begins at `0x8002000`
* RS232 is enabled on USART2, pins `PA2` and `PA3`
* USB and CAN are disabled
* LED pin is `PC13`

Based on [demo examples](https://www.feaser.com/openblt/doku.php?id=manual:demos) from the OpenBLT documentation.

## More Information
https://www.feaser.com/openblt/