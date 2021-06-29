/****************************************************************************
 *
 * Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <protocol_splitter.hpp>


DevSerial::DevSerial(const char *device_name, const uint32_t baudrate, const bool hw_flow_control,
		     const bool sw_flow_control, const double passthrough_timeout_ms)
	: _baudrate(baudrate),
	  _hw_flow_control(hw_flow_control),
	  _sw_flow_control(sw_flow_control),
	  _passthrough_timeout_ms(passthrough_timeout_ms)
{
	strncpy(_uart_name, device_name, sizeof(_uart_name));
}

DevSerial::~DevSerial()
{
	if (_uart_fd >= 0) {
		close();
	}
}

int DevSerial::open_uart()
{
	// Open a serial port, if not opened already
	if (_uart_fd < 0) {
		_uart_fd = open(_uart_name, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);

		if (_uart_fd < 0) {
			printf("\033[0;31m[ protocol__splitter ]\tUART link: Failed to open device: %s (%d)\033[0m\n", _uart_name, errno);
			return -errno;
		}

		// If using shared UART, no need to set it up
		if (_baudrate == 0) {
			return _uart_fd;
		}

		// Try to set baud rate
		struct termios uart_config;
		int termios_state;

		// Back up the original uart configuration to restore it after exit
		if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tUART link: Error getting config %s: %d (%d)\033[0m\n", _uart_name,
			       termios_state, errno);
			close();
			return -errno_bkp;
		}

		uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
		uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

		uart_config.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

		// never send SIGTTOU
		uart_config.c_lflag &= ~(TOSTOP);

		// ignore modem control lines
		uart_config.c_cflag |= CLOCAL;

		// 8 bits
		uart_config.c_cflag |= CS8;

		// Flow control
		if (_hw_flow_control) {
			// HW flow control
			uart_config.c_lflag |= CRTSCTS;

		} else if (_sw_flow_control) {
			// SW flow control
			uart_config.c_lflag |= (IXON | IXOFF | IXANY);
		}

		// Set baud rate
		speed_t speed;

		if (!baudrate_to_speed(_baudrate, &speed)) {
			printf("\033[0;31m[ protocol__splitter ]\tUART link: Error setting baudrate %s: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\033[0m\n",
			       _uart_name, _baudrate);
			close();
			return -EINVAL;
		}

		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tUART link: Error setting baudrate %s: %d (%d)\033[0m\n", _uart_name,
			       termios_state, errno);
			close();
			return -errno_bkp;
		}

		if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ protocol__splitter ]\tUART link: ERR SET CONF %s (%d)\033[0m\n", _uart_name, errno);
			close();
			return -errno_bkp;
		}

#ifdef __linux__
		// For Linux, set high speed polling at the chip level. Since this routine relies on a USB latency
		// change at the chip level it may fail on certain chip sets if their driver does not support this
		// configuration request
		{
			struct serial_struct serial_ctl;

			if (ioctl(_uart_fd, TIOCGSERIAL, &serial_ctl) < 0) {
				printf("\033[0;31m[ protocol__splitter ]\tError while trying to read serial port configuration: %d\033[0m\n", errno);
				goto set_latency_failed;
			}

			serial_ctl.flags |= ASYNC_LOW_LATENCY;

			if (ioctl(_uart_fd, TIOCSSERIAL, &serial_ctl) < 0) {
				int errno_bkp = errno;
				printf("\033[0;31m[ protocol__splitter ]\tError while trying to write serial port latency: %d\033[0m\n", errno);
				close();
				return -errno_bkp;
			}
		}
#endif

		printf("[ protocol__splitter ]\tUART link: device: %s; baudrate: %d; flow_control: %s\n",
		       _uart_name, _baudrate, _sw_flow_control ? "SW enabled" : (_hw_flow_control ? "HW enabled" : "No"));

		char aux[64];

		while (0 < ::read(_uart_fd, (void *)&aux, 64)) {
			printf("[ protocol__splitter ]\tUART link: Flushed\n");
			usleep(1000);
		}
	}

	return _uart_fd;

#ifdef __linux__
set_latency_failed:

	if (ioctl(_uart_fd, TCFLSH, TCIOFLUSH) == -1) {
		printf("\033[0;31m[ protocol__splitter ]\tCould not flush terminal\033[0m\n");
		close();
		return -errno;
	}

	printf("[ protocol__splitter ]\tUART link: device: %s; baudrate: %d; flow_control: %s\n",
	       _uart_name, _baudrate, _sw_flow_control ? "SW enabled" : (_hw_flow_control ? "HW enabled" : "No"));

	return _uart_fd;
#endif
}

bool DevSerial::baudrate_to_speed(uint32_t bauds, speed_t *speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

	switch (bauds) {
	case 0:      *speed = B0;		break;

	case 50:     *speed = B50;		break;

	case 75:     *speed = B75;		break;

	case 110:    *speed = B110;		break;

	case 134:    *speed = B134;		break;

	case 150:    *speed = B150;		break;

	case 200:    *speed = B200;		break;

	case 300:    *speed = B300;		break;

	case 600:    *speed = B600;		break;

	case 1200:   *speed = B1200;		break;

	case 1800:   *speed = B1800;		break;

	case 2400:   *speed = B2400;		break;

	case 4800:   *speed = B4800;		break;

	case 9600:   *speed = B9600;		break;

	case 19200:  *speed = B19200;		break;

	case 38400:  *speed = B38400;		break;

	case 57600:  *speed = B57600;		break;

	case 115200: *speed = B115200;		break;

	case 230400: *speed = B230400;		break;

	case 460800: *speed = B460800;		break;

	case 500000: *speed = B500000;		break;

	case 921600: *speed = B921600;		break;

	case 1000000: *speed = B1000000;	break;

	case 1500000: *speed = B1500000;	break;

	case 2000000: *speed = B2000000;	break;

#ifdef B3000000

	case 3000000: *speed = B3000000;    break;
#endif

#ifdef B3500000

	case 3500000: *speed = B3500000;    break;
#endif

#ifdef B4000000

	case 4000000: *speed = B4000000;    break;
#endif

	default:
		return false;
	}

	return true;
}

int DevSerial::close()
{
	if (_uart_fd >= 0) {
		printf("\033[1;33m[ protocol__splitter ]\tUART link: Closed serial port!\033[0m\n");
		::close(_uart_fd);
		_uart_fd = -1;
	}

	fflush(stdout);
	fflush(stderr);

	return 0;
}

ssize_t DevSerial::read()
{
	int ret = 0;
	size_t i = 0;
	uint16_t packet_len, payload_len;
	Sp2Header_t *header;

	if (_buf_size == BUFFER_SIZE) {
		_buf_size = 0;
		printf("\033[0;31m[ protocol__splitter ]\tUART link: receive buffer overflow - Flushing\033[0m\n");
	}

	ret = ::read(_uart_fd, _buffer + _buf_size, BUFFER_SIZE - _buf_size);

	if (ret < 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUART link: UART receive error: %d\033[0m\n", ret);
		return ret;

	} else if (ret == 0) {
		return ret;
	}

	_buf_size += ret;

	ret = 0;

	// Enable MAVLink passthrough in case no RTPS/MAVLink packets were parsed,
	// which usually means that the protocol splitter on the FMU is not being
	// used and so no protocol splitter header is being used
	if (mavlink_passthrough.load()) {
		if (!_mavlink_passthrough_noticed) {
			printf("\033[1;33m[ protocol__splitter ]\tUART link: Changed to MAVLink passthrough as no protocol splitter headers were parsed\033[0m\n");
			_mavlink_passthrough_noticed = true;
		}

		objects->mavlink2->udp_write(_buffer, _buf_size);
		ret += _buf_size;

		// All data handled, clean up buffer
		_buf_size = 0;

		return ret;
	}

	// Search for a packet on buffer to send it
	while (_buf_size - i >= Sp2HeaderSize) {
		while (_buf_size - i >= Sp2HeaderSize &&
		       (((Sp2Header_t *) &_buffer[i])->fields.magic != Sp2HeaderMagic
			|| ((Sp2Header_t *) &_buffer[i])->fields.checksum != (_buffer[i] ^ _buffer[i + 1] ^ _buffer[i + 2])
		       )) {
			i++;
		}

		// We need at least the first <Sp2HeaderSize> bytes to get packet header
		if (i > _buf_size - Sp2HeaderSize) {
			ret = -1;
			break;
		}

		header = (Sp2Header_t *)&_buffer[i];
		payload_len = ((uint16_t)header->fields.len_h << 8) | header->fields.len_l;
		packet_len = payload_len + Sp2HeaderSize;

		// invalid packet received
		if (payload_len == 0) {
			ret = -1;
			break;
		}

		// packet is bigger than what we've read, better luck next time
		if (i + packet_len > _buf_size) {
			ret = -1;
			break;
		}

		// Write to UDP port
		if (header->fields.type == MessageType::Mavlink) {
			_protocol_splitter_header_found = true;
			objects->mavlink2->udp_write(_buffer + i + Sp2HeaderSize, payload_len);

		} else if (header->fields.type == MessageType::Rtps) {
			_protocol_splitter_header_found = true;
			objects->rtps->udp_write(_buffer + i + Sp2HeaderSize, payload_len);

		} else {
			printf("\033[0;31m[ protocol__splitter ]\tUART link: Unknown message type %u received \033[0m\n",
			       header->fields.type);
		}

		// Jump over the handled packet
		i += packet_len;
		ret += packet_len;
	}

	if (i < _buf_size) {
		// Last message not complete, save it
		memmove(_buffer, _buffer + i, _buf_size - i);
		_buf_size = _buf_size - i;

	} else {
		// All data handled, clean up buffer
		_buf_size = 0;
	}

	// Wait for X seconds without receiving any protocol splitter headers in order
	// to change to MAVLink passthrough mode. This is only applicable in the beginning,
	// when the first data is received, in order to verify if there are protocol
	// splitter headers being sent through the protocol splitter in the FMU side.
	if (!_protocol_splitter_header_found &&
	    (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _timer_start).count() >=
	     _passthrough_timeout_ms)) {
		mavlink_passthrough.store(true);
	}

	return ret;
}


DevSocket::DevSocket(const char *udp_ip, const uint16_t udp_port_recv,
		     const uint16_t udp_port_send, int uart_fd, MessageType type)
	: _uart_fd(uart_fd)
	, _udp_port_recv(udp_port_recv)
	, _udp_port_send(udp_port_send)
{
	if (nullptr != udp_ip) {
		strcpy(_udp_ip, udp_ip);
	}

	// Init the header
	_header.fields.magic		= Sp2HeaderMagic;
	_header.fields.len_h		= 0;
	_header.fields.len_l		= 0;
	_header.fields.checksum		= 0;
	_header.fields.type		= type;

	open_udp(type);
}

DevSocket::~DevSocket()
{
	// Close the sender
	if (_udp_fd >= 0) {
		close(_udp_fd);
	}

	// Close the receiver
	if (_udp_fd >= 0) {
		close(_udp_fd);
	}
}

int DevSocket::open_udp(const MessageType type)
{
	// Init receiver
	if ((_udp_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link: Create socket failed\033[0m\n");
		return -1;
	}

	memset((char *)&_inaddr, 0, sizeof(_inaddr));
	_inaddr.sin_family = AF_INET;
	_inaddr.sin_port = htons(_udp_port_recv);
	_inaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	memset((char *) &_outaddr, 0, sizeof(_outaddr));
	_outaddr.sin_family = AF_INET;
	_outaddr.sin_port = htons(_udp_port_send);
	_outaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	const std::string msg_type = type == MessageType::Mavlink ? "MAVLink" : "RTPS";

	if (bind(_udp_fd, (struct sockaddr *)&_inaddr, sizeof(_inaddr)) < 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link for %s: Bind failed\033[0m\n", msg_type.c_str());
		return -1;
	}

	printf("[ protocol__splitter ]\tUDP socket link for %s: receiving from remote through port %u...\n", msg_type.c_str(),
	       static_cast<unsigned int>(_udp_port_recv));

	if (inet_aton(_udp_ip, &_outaddr.sin_addr) == 0) {
		printf("\033[0;31m[ protocol__splitter ]\tUDP socket link for %s: inet_aton() failed\033[0m\n", msg_type.c_str());
		return -1;
	}

	printf("[ protocol__splitter ]\tUDP socket link for %s: sending to remote through port %u...\n", msg_type.c_str(),
	       static_cast<unsigned int>(_udp_port_send));

	return 0;
}

int DevSocket::close(int udp_fd)
{
	if (udp_fd >= 0) {
		printf("\033[1;33m[ protocol__splitter ]\tUDP socket link: Closed socket!\033[0m\n");
		shutdown(udp_fd, SHUT_RDWR);
		::close(udp_fd);
		udp_fd = -1;
	}

	fflush(stdout);
	fflush(stderr);

	return 0;
}

ssize_t DevSocket::udp_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !(-1 != _udp_fd)) {
		return -1;
	}

	int ret = 0;
	socklen_t addrlen = sizeof(_outaddr);

	if (ntohs(_outaddr.sin_port) == 0) {
		ret = recvfrom(_udp_fd, buffer, len, 0, (struct sockaddr *) &_outaddr, &addrlen);

	} else {
		ret = recv(_udp_fd, buffer, len, 0);
	}

	return ret;
}

ssize_t DevSocket::udp_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !(-1 != _udp_fd)) {
		return -1;
	}

	int ret = 0;
	ret = sendto(_udp_fd, buffer, len, 0, (struct sockaddr *)&_outaddr, sizeof(_outaddr));
	return ret;
}

ssize_t DevSocket::write()
{
	int i = 0;
	size_t packet_len;

	// Read from UDP port
	ssize_t payload_len = udp_read((void *)_buffer, BUFFER_SIZE);

	if (payload_len < 0) {
		return payload_len;
	}

	// Do not add the protocol splitter header if in MAVLink passthrough mode
	if (!mavlink_passthrough.load()) {
		_header.fields.len_h = ((payload_len >> 8) & 0x7f);
		_header.fields.len_l = (payload_len & 0xff);
		_header.fields.checksum = _header.bytes[0] ^ _header.bytes[1] ^ _header.bytes[2]; // Checksum
	}

	// Write to UART port
	std::unique_lock<std::mutex> write_guard(uart_mtx);

	if (!mavlink_passthrough.load()) {
		packet_len = ::write(_uart_fd, _header.bytes, Sp2HeaderSize);
	}

	packet_len += ::write(_uart_fd, _buffer, payload_len);
	write_guard.unlock();

	return packet_len;
}


void signal_handler(int signum)
{
	printf("\033[1;33m[ protocol__splitter ]\tInterrupt signal (%d) received.\033[0m\n", signum);
	running = false;
}

void serial_to_udp(pollfd *fd_uart)
{
	while (running) {
		const int ret = ::poll(fd_uart, sizeof(fd_uart) / sizeof(fd_uart[0]), 1000);

		if (ret <= 0) {
			// In case of a poll timeout or error, try to reopen the UART fd
			//
			// This is also done for timeouts as it was being verified for the case of
			// the MAVLink passthrough, on some specific platforms, the UART poll was
			// always timing out.
			//
			// @todo revisit the UART configs to understand why the timeout happens
			if (ret == 0) {
				printf("\033[1;33m[ protocol__splitter ]\tUART link: Poll timeout. ");

			} else {
				printf("\033[1;33m[ protocol__splitter ]\tUART link: Poll error (%d). ", ret);
			}

			printf("Re-opening UART link and resetting fds...\033[0m\n");

			std::unique_lock<std::mutex> uart_guard(uart_mtx);
			objects->serial->close();

			const int uart_fd = objects->serial->open_uart();
			objects->rtps->_uart_fd = uart_fd;
			objects->mavlink2->_uart_fd = uart_fd;
			uart_guard.unlock();

			fd_uart[0].fd = uart_fd;
			fd_uart[0].events = POLLIN;

			sleep(1);

		} else {
			if (fd_uart[0].revents & POLLIN) {
				// Start the timer for the pass-through activation on timeout at the first data poll
				if (objects->serial->_timer_start == std::chrono::time_point<std::chrono::system_clock>()) {
					objects->serial->_timer_start = std::chrono::system_clock::now();
				}

				objects->serial->read();
			}
		}

		fflush(stdout);
		fflush(stderr);
	}
}

void mavlink_udp_to_serial(pollfd *fds)
{
	while (running) {
		if ((::poll(fds, sizeof(fds) / sizeof(fds[0]), 100) > 0) && (fds[0].revents & POLLIN)) {
			objects->mavlink2->write();
		}

		fflush(stdout);
		fflush(stderr);
	}
}

void rtps_udp_to_serial(pollfd *fds)
{
	// If in MAVLink pass-through mode, do not poll from UDP andwrite to serial
	while (running && !mavlink_passthrough.load()) {
		if ((::poll(fds, sizeof(fds) / sizeof(fds[0]), 100) > 0) && (fds[0].revents & POLLIN)) {
			objects->rtps->write();
		}

		fflush(stdout);
		fflush(stderr);
	}
}

static void usage(const char *name)
{
	printf("usage: %s [options]\n\n"
	       "  -b <baudrate>			UART device baudrate. Default 460800\n"
	       "  -d <uart_device>		UART device. Default /dev/ttyUSB0\n"
	       "  -i <host_ip>			Host IP for UDP. Default 127.0.0.1\n"
	       "  -w <mavlink_udp_recv_port>	UDP port for receiving. Default: 5800.\n"
	       "                            	 Set 0 to autoselect.\n"
	       "  -x <mavlink_udp_send_port>	UDP port for receiving. Default: 5801.\n"
	       "                            	 Set 0 to get source port.\n"
	       "  -y <rtps_udp_recv_port>	UDP port for receiving. Default: 5900.\n"
	       "                         	 Set 0 to autoselect.\n"
	       "  -z <rtps_udp_send_port>	UDP port for receiving. Default: 5901\n"
	       "                         	 Set 0 to get source port.\n"
	       "  -t <passthrough_timeout>	Time to wait for protocol splitter headers\n"
	       "                         	 before changing to a pass-through mode\n"
	       "                         	 for MAVLink. Defaults to 3 seconds.\n"
	       "  -f <sw_flow_control>		Activates UART link SW flow control\n"
	       "  -g <hw_flow_control>		Activates UART link HW flow control\n"
	       "  -v <verbose_debug>		Add more verbosity\n\n",
	       name);
}

static int parse_options(int argc, char **argv)
{
	int ch;

	while ((ch = getopt(argc, argv, "b:d:i:t:w:x:y:z:fghv")) != EOF) {
		switch (ch) {
		case 'b': _options.baudrate			= strtoul(optarg, nullptr, 10);		break;

		case 'd': if (nullptr != optarg)		strcpy(_options.uart_device, optarg);	break;

		case 'i': if (nullptr != optarg)		strcpy(_options.host_ip, optarg);	break;

		case 'f': _options.sw_flow_control		= true;					break;

		case 'g': _options.hw_flow_control		= true;					break;

		case 'h': usage(argv[0]);			return -1;				break;

		case 't': _options.passthrough_timeout_ms	= strtoul(optarg, nullptr, 10);		break;

		case 'v': _options.verbose_debug		= true;					break;

		case 'w': _options.mavlink_udp_recv_port	= strtoul(optarg, nullptr, 10);		break;

		case 'x': _options.mavlink_udp_send_port	= strtoul(optarg, nullptr, 10);		break;

		case 'y': _options.rtps_udp_recv_port		= strtoul(optarg, nullptr, 10);		break;

		case 'z': _options.rtps_udp_send_port		= strtoul(optarg, nullptr, 10);		break;

		default:
			usage(argv[0]);
			return -1;
		}
	}

	if (_options.hw_flow_control && _options.sw_flow_control) {
		printf("\033[0;31m[ protocol__splitter ]\tHW and SW flow control set. Please set only one or another\033[0m\n");
		return -1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	if (-1 == parse_options(argc, argv)) {
		return -1;
	}

	objects = new StaticData();

	std::signal(SIGINT, signal_handler);

	// Init the serial device
	objects->serial = new DevSerial(_options.uart_device, _options.baudrate, _options.hw_flow_control,
					_options.sw_flow_control, _options.passthrough_timeout_ms);
	const int uart_fd = objects->serial->open_uart();

	// Init UDP sockets for Mavlink and RTPS
	objects->mavlink2 = new DevSocket(_options.host_ip, _options.mavlink_udp_recv_port, _options.mavlink_udp_send_port,
					  uart_fd, MessageType::Mavlink);
	objects->rtps = new DevSocket(_options.host_ip, _options.rtps_udp_recv_port, _options.rtps_udp_send_port, uart_fd,
				      MessageType::Rtps);

	// Init fd polling
	pollfd fd_uart[1] {};
	pollfd fd_udp_mavlink[1] {};
	pollfd fd_udp_rtps[1] {};

	fd_uart[0].fd = uart_fd;
	fd_uart[0].events = POLLIN;

	fd_udp_mavlink[0].fd = objects->mavlink2->_udp_fd;
	fd_udp_mavlink[0].events = POLLIN;

	fd_udp_rtps[0].fd = objects->rtps->_udp_fd;
	fd_udp_rtps[0].events = POLLIN;

	fflush(stdout);
	fflush(stderr);

	running = true;

	std::thread serial_to_udp_th(serial_to_udp, fd_uart);
	std::thread rtps_udp_to_serial_th(rtps_udp_to_serial, fd_udp_rtps);
	std::thread mavlink_udp_to_serial_th(mavlink_udp_to_serial, fd_udp_mavlink);

	serial_to_udp_th.join();
	mavlink_udp_to_serial_th.join();
	rtps_udp_to_serial_th.join();

	delete objects->serial;
	delete objects->mavlink2;
	delete objects->rtps;
	delete objects;
	objects = nullptr;

	printf("\033[1;33m[ protocol__splitter ]\tExiting...\033[0m\n");
	fflush(stdout);
	fflush(stderr);

	return 0;
}
