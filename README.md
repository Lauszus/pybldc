# pybldc

#### Developed by Kristian Sloth Lauszus, 2024-2025

The code is released under the GNU General Public License v3.0.
_________
[![PyPI](https://img.shields.io/pypi/v/pybldc.svg)](https://pypi.org/project/pybldc)
[![CI](https://github.com/Lauszus/pybldc/actions/workflows/ci.yml/badge.svg)](https://github.com/Lauszus/pybldc/actions/workflows/ci.yml)
[![Coverage badge](https://raw.githubusercontent.com/Lauszus/pybldc/python-coverage-comment-action-data/badge.svg)](https://htmlpreview.github.io/?https://github.com/Lauszus/pybldc/blob/python-coverage-comment-action-data/htmlcov/index.html)

## Purpose

This tool allows you to flash a VESC via CAN-bus or serial/USB using Python.

## Installation and running

To install:

```bash
pip install .
```

Or install the latest release from PyPI:

```bash
pip install -U pybldc
```

### CLI

A handy CLI tool is included.

I recommend using [uvx](https://github.com/astral-sh/uv) to run the CLI tool:

```bash
uvx pybldc --help
```

Or you can install it in your path:

```bash
uv tool install pybldc
pybldc --help
```

__Usage__

```
usage: pybldc [-id CONTROLLER_ID] [-i INTERFACE] [-l CHANNEL] [-p PORT] [-h]
              [--version] [-B BINARY] [--bootloader] [-t TIMEOUT]
              [-r PING_REPEAT] [-b BAUDRATE] [--debug]
              {can,serial} {upload,ping,reset}

positional arguments:
  {can,serial}          Communicate with the target via either CAN or serial
  {upload,ping,reset}   upload: write BINARY
                        ping: send a ping command to the target and check for a response
                        reset: send a reset command to the target and check for a response

required CAN arguments:
  -id CONTROLLER_ID, --controller-id CONTROLLER_ID
                        The VESC ID used for communication

optional CAN arguments:
  -i INTERFACE, --interface INTERFACE
                        The CAN-Bus interface to use (default "socketcan")
  -l CHANNEL, --channel CHANNEL
                        The CAN-Bus channel to use (default "can0")

required serial arguments:
  -p PORT, --port PORT  The port to use for serial

optional arguments:
  -h, --help            Show this help message and exit
  --version             Show program's version number and exit
  -B BINARY, --binary BINARY
                        The binary to upload
  --bootloader          The binary to upload is a bootloader
  -t TIMEOUT, --timeout TIMEOUT
                        The time to wait in seconds for a response (default 5.0)
  -r PING_REPEAT, --ping-repeat PING_REPEAT
                        The number of times to try to establish a connection by pinging the VESC (default 3)
  -b BAUDRATE, --baudrate BAUDRATE, --bitrate BAUDRATE
                        The baudrate/bitrate to use for serial/can (defaults to 115200 for serial and 500000 for CAN)
  --debug               Turn on debug logs
```

__Upload__

```bash
uvx pybldc can upload -id 1 -B VESC.bin
```

```bash
uvx pybldc serial upload -p /dev/ttyACM0 -B VESC.bin
```

__Ping__

```bash
uvx pybldc can ping -id 1
```

```bash
uvx pybldc serial ping -p /dev/ttyACM0
```

__Reset__

```bash
uvx pybldc can reset -id 1
```

```bash
uvx pybldc serial reset -p /dev/ttyACM0
```

#### Increase TX buffer

On Linux you might need to increase the TX buffer size:

```bash
sudo ip link set can0 txqueuelen 1000
```

## Credit

* VESC firmware: https://github.com/vedderb/bldc
* VESC bootloader: https://github.com/vedderb/bldc-bootloader
* VESC Tool: https://github.com/vedderb/vesc_tool
