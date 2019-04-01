# Self Balancing Robot

Short description

## Installing

There are several steps to be done. Please follow all the steps in order to get your Raspberry up and running.

### Raspberry Pi

 1. Download [Image](https://www.raspberrypi.org/downloads/raspbian/).

 2. Flash image on SD-Card using [Etcher](https://www.balena.io/etcher/).

 3. Add ssh file with no extension in the boot directory.

You should now insert the SD-Card into the raspberry and connect it into your network and give it power.

### Configuring Raspberry Pi

1. ```bash
   sudo raspi-config
   ```
   Go to Interface Options and enable both VNC and I2C.

2. ```bash
   sudo apt-get update
   sudo apt-get upgrade
   ```

3. ```bash
   sudo apt install libsdl1.2-dev
   ```

Restart your Raspberry and proceed to next step.

### Python configurations

1. ```bash
   git clone https://github.com/magnusoy/SelfBalancingRobot.git
   ```

2. ```bash
   pip3 install --upgrade setuptools
   ```

3. ```bash
   cd ~
   sudo pip3 install -r SelfBalancingRobot/requirements.txt   
   ```

## Setup

Short description

## Usage

Short description

## Built With

* [Python](https://www.python.org/) - Python
* [Arduino](https://www.arduino.cc/) - Arduino

## Contributing

If you want to contribute or find anything wrong, please create a Pull request, or issue addressing the change, or issue.

## Author

* **Magnus Øye** - [magnusoy](https://github.com/magnusoy)

## License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/magnusoy/SelfBalancingRobot/blob/master/LICENSE) file for details.

## Libraries

[Ds4drv](https://github.com/chrippa/ds4drv)

[Numpy](http://www.numpy.org/)

[Pyserial](https://pythonhosted.org/pyserial/)

[Pygame](https://www.pygame.org/news)

[OpenCV](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)

[Adafruit Circuitpython Motorkit](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/python-circuitpython)
