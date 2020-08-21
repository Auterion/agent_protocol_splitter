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

#include <arpa/inet.h>
#include <cassert>
#include <csignal>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#define BUFFER_SIZE 2048
#define DEFAULT_BAUDRATE 460800
#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"
#define DEFAULT_HOST_IP "127.0.0.1"
#define DEFAULT_MAVLINK_RECV_PORT 5800
#define DEFAULT_MAVLINK_SEND_PORT 5801
#define DEFAULT_RTPS_RECV_PORT 5900
#define DEFAULT_RTPS_SEND_PORT 5901

class DevSerial;
class Mavlink2Dev;
class RtpsDev;
class ReadBuffer;

struct StaticData {
	DevSerial *serial;
	Mavlink2Dev *mavlink2;
	RtpsDev *rtps;
	ReadBuffer *in_read_buffer;
	ReadBuffer *out_read_buffer;
};

volatile sig_atomic_t running = true;

struct options {
	uint32_t baudrate = DEFAULT_BAUDRATE;
	char uart_device[64] = DEFAULT_UART_DEVICE;
	char host_ip[16] = DEFAULT_HOST_IP;
	uint16_t mavlink_udp_recv_port = DEFAULT_MAVLINK_RECV_PORT;
	uint16_t mavlink_udp_send_port = DEFAULT_MAVLINK_SEND_PORT;
	uint16_t rtps_udp_recv_port = DEFAULT_RTPS_RECV_PORT;
	uint16_t rtps_udp_send_port = DEFAULT_RTPS_SEND_PORT;
	bool sw_flow_control = false;
	bool hw_flow_control = false;
	bool verbose_debug = false;
} _options;

namespace
{
static StaticData *objects = nullptr;
}

class ReadBuffer
{
public:
	int read(int fd);
	void move(void *dest, size_t pos, size_t n);

	uint8_t buffer[BUFFER_SIZE] = {};
	size_t buf_size = 0;

	static const size_t BUFFER_THRESHOLD = sizeof(buffer) * 0.8;
};

class DevSerial
{
public:
	DevSerial(const char *device_name, const uint32_t baudrate, const bool hw_flow_control,
		  const bool sw_flow_control);
	virtual ~DevSerial();

	int	open_uart();
	int	close();

	int _uart_fd = -1;

protected:
	uint32_t _baudrate;
	bool _hw_flow_control;
	bool _sw_flow_control;

	char _uart_name[64] = {};
	bool baudrate_to_speed(uint32_t bauds, speed_t *speed);

private:
};

class DevSocket
{
public:
	DevSocket(const char *_udp_ip, const uint16_t udp_port_recv,
		  const uint16_t udp_port_send, int uart_fd);
	virtual ~DevSocket();

	int close(int udp_fd);

	int	open_udp();
	ssize_t udp_read(void *buffer, size_t len);
	ssize_t udp_write(void *buffer, size_t len);

	int _uart_fd;
	int _udp_fd;

protected:
	char _udp_ip[16] = {};

	uint16_t _udp_port_recv;
	uint16_t _udp_port_send;
	struct sockaddr_in _outaddr;
	struct sockaddr_in _inaddr;

	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

private:
};

class Mavlink2Dev : public DevSocket
{
public:
	Mavlink2Dev(ReadBuffer *in_read_buffer, ReadBuffer *out_read_buffer,
		    const char *udp_ip, const uint16_t udp_port_recv,
		    const uint16_t udp_port_send, int uart_fd);
	virtual ~Mavlink2Dev() {}

	ssize_t	read();
	ssize_t	write();

protected:
	ReadBuffer *_in_read_buffer;
	ReadBuffer *_out_read_buffer;
	size_t _remaining_partial = 0;
	size_t _partial_start = 0;
	uint8_t _partial_buffer[BUFFER_SIZE] = {};
};

class RtpsDev : public DevSocket
{
public:
	RtpsDev(ReadBuffer *in_read_buffer, ReadBuffer *out_read_buffer,
		const char *udp_ip, const uint16_t udp_port_recv,
		const uint16_t udp_port_send, int uart_fd);
	virtual ~RtpsDev() {}

	ssize_t	read();
	ssize_t	write();

protected:
	ReadBuffer *_in_read_buffer;
	ReadBuffer *_out_read_buffer;

	static const uint8_t HEADER_SIZE = 9;
};
