# Library rpico-rc522

[rpico-rc522](https://github.com/Mik3Rizzo/rpico-rc522) is a **CircuitPython** library written to use the RC522 reader (chip MFRC522) 
on the Raspberry Pico microcontroller.

The library is a porting of [rpi-rc522](https://github.com/Mik3Rizzo/rpi-rc522). Please see there for detailed 
documentation and usage examples.

## Library structure

The library offers two handy objects:
- **RC522**: low level class that manages the RC522.
- **RC522Manager**: high level class to easily read/write data from/to the NFC tag.

There is also a collection of utils functions.


## Requisites and install

The library is written for CircuitPython.

To install CircuitPython on the Pico:
- Download it [here](https://circuitpython.org/board/raspberry_pi_pico/)
- Push the BOOTSEL button on the Pico while connecting the usb to the computer. 
- Drag and drop the downloaded file directly into the Pico, that is seen as a USB storage.


To use the library, simply copy `rc522.py` and `rc522manager.py` in the Raspberry Pico microcontroller (started normally,
not pushing BOOTSEL).



## Pinout

Default pinout for the connection between the Pico and the rc522.

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

The pinout configuration can be changed via software.


## About

**Michele Rizzo**, *Master's Degree Computer Engineering student at University of Brescia*.

Feel free to contact me by [mail](mailto:m.rizzo006@studenti.unibs.it) or visit my [Github](https://github.com/Mik3Rizzo/).