# FireFly-Remote v2.0

Control your electric skateboard with an Arduino controlled remote. This repository contains the needed software for the remote and the receiver, but you will need to install a few Arduino Libraries in order to compile the Arduino sketches. The Arduino IDE comes with most of the needed libraries, but you will ned to manually install VescUartControl from RollingGecko: https://github.com/RollingGecko/VescUartControl.

You can find the 3D-models for the remote (STL files) on Thingiverse: https://www.thingiverse.com/thing:2454391 and read more about the project on: https://www.electric-skateboard.builders/t/simple-3d-printed-nrf-remote-arduino-controlled/28543

I have made a Wiki here on Github, with a few tips and guides on how to build the remote. The Wiki can be found here: https://github.com/SolidGeek/nRF24-Esk8-Remote/wiki

Donation link: https://www.paypal.me/solidgeek

## Update v2.0

Update 2.0 introduces quite a few new interesting features to the remote, and fixes some minor issues from the past. The update includes:

* The possibility to generate a new address
* Receiver settings (transmitable from the remote)
* Trigger- and controlmode is handled on the receiver
* Calibration of throttle
* Remote now uses channel 108


To-do:
* UART throttle control
* Introducing cruise control

