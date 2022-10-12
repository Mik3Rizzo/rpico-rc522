# Library rpico-rc522

[rpico-rc522](https://github.com/Mik3Rizzo/rpico-rc522) is a **CircuitPython** library that manages  the RC522 reader 
(chip MFRC522) on the Raspberry Pico microcontroller.

It is a porting of [rpi-rc522](https://github.com/Mik3Rizzo/rpi-rc522). Please see there for detailed 
documentation and usage examples.

## Library structure

The library offers two handy objects:
- **RC522**: low level class that manages the RC522.
- **RC522Manager**: high level class to easily read/write data from/to the NFC tag.

There is also a collection of utils functions.


## Requirements and install

The library is works with CircuitPython.

To install CircuitPython on the Pico:
- Download it [here](https://circuitpython.org/board/raspberry_pi_pico/)
- Push the BOOTSEL button on the Pico while connecting the usb to the computer. 
- Drag and drop the downloaded file directly into the Pico, that is seen as a USB storage.

To install the library, simply copy the `rpico_rc522` folder in the `lib` directory of the Pico.



## Wiring

Default wiring between the Pico and the rc522.

| Name  | Pin #  | Pin name |
|:-----:|:------:|:--------:|
|  SDA  |   2    |   GP1    |
|  SCK  |   4    |   GP2    |
| MOSI  |   5    |   GP3    |
| MISO  |   6    |   GP4    |
|  IRQ  |  None  |   None   |
|  GND  | Ground |  Ground  |
|  RST  |   1    |   GP0    |
| 3.3V  |   36   |   3V3    |

You can use [this](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg) 
image for the Raspberry Pico pinout.

The wiring configuration can be changed via software.


## About

**Michele Rizzo**, *Master's Degree Computer Engineering student at University of Brescia*.

Feel free to contact me by [mail](mailto:m.rizzo006@studenti.unibs.it) or visit my [Github](https://github.com/Mik3Rizzo/).