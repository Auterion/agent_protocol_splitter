<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# agent_protocol_splitter
[![GitHub](https://img.shields.io/github/license/Auterion/agent_protocol_splitter?style=for-the-badge)](https://github.com/Auterion/agent_protocol_splitter/blob/master/LICENSE)

microRTPS/DDS agent protocol splitter. Allows to stream and parse MAVLink and RTPS packets in the same serial link.

## Build

```sh
cmake ..
make
```

## Install

To install system wise on an UNIX system (might require `sudo` permissions):

```sh
make install
```

## Usage

Make sure to have the UART link up and the cabling right (if using an TTL-USB adapter).

Run the splitter using:

```sh
$ protocol_splitter [options]
```

Options available:

```sh
  -b <baudrate>			UART device baudrate. Default 460800
  -d <uart_device>		UART device. Default /dev/ttyUSB0
  -i <host_ip>			Host IP for UDP. Default 127.0.0.1
  -w <mavlink_udp_recv_port>	UDP port for receiving. Default 5800
  -x <mavlink_udp_send_port>	UDP port for receiving. Default 5801
  -y <rtps_udp_recv_port>	UDP port for receiving. Default 5900
  -z <rtps_udp_send_port>	UDP port for receiving. Default 5901
  -f <sw_flow_control>		Activates UART link SW flow control
  -h <hw_flow_control>		Activates UART link HW flow control
  -v <verbose_debug>		Add more verbosity
```

At the same time, the `protocol_splitter` should also be run in the client side, so the stream can be multiplexed bidirectionally.
