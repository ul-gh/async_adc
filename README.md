# Async ADC
# Incomplete, work-in-progress!
Python package providing async API for Texas Instruments ADS1255 and ADS1256
SPI bus based analog-to-digital converters connected to a Raspberry Pi.

Async reads and writes are implemented using the asyncio package.

The Raspberry Pi SPI hardware is accessed through the pigpio system service.

On a Raspberry Pi 3, multi-channel ADC reads require a time overhead in
the order of one millisecond per sample which makes this library suitable
for low-speed, high resolution data acquisition.

As the ADS125x devices feature a sophisticated, configurable, hardware-based
down-sampling filter, there is no penalty in terms of accuracy.

Limitation:
* This does not feature high-speed or low delays. This is a low-speed solution!

Download: https://github.com/ul-gh/async_adc

Documentation of pigpio library: https://abyz.me.uk/rpi/pigpio/python.html  

License: GNU LGPLv2.1, see:
https://www.gnu.org/licenses/old-licenses/lgpl-2.1-standalone.html

Ulrich Lukas, 2025-06-05

## Run example on Raspberry Pi OS:
### Install:
	# numpy is optional for running the Isoflux hardware examples
	sudo apt install python3-pip python3-numpy pigpio python3-pigpio git
	pip3 install pipyadc
### Enable pigpio system service (started at boot)
	sudo systemctl enable pigpiod.service
### Activate SPI bus and reboot system
	# Using raspi-config or, alternatively, using following command:
	sudo sed -E -i s/"(#)(dtparam=spi).*"/"\2=on"/ /boot/config.txt
	sudo reboot
### Run example:
	git clone https://github.com/ul-gh/PiPyADC
	cd PiPyADC/examples/waveshare_board
	./waveshare_example.py


