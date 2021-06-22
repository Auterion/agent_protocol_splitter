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
#pragma once

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#endif

#define BUFFER_SIZE			2048
#define DEFAULT_BAUDRATE 		460800
#define DEFAULT_UART_DEVICE		"/dev/ttyUSB0"
#define DEFAULT_HOST_IP			"127.0.0.1"
#define DEFAULT_PASSTHROUGH_TIMEOUT_MS	3000
#define DEFAULT_MAVLINK_RECV_PORT	5800
#define DEFAULT_MAVLINK_SEND_PORT	5801
#define DEFAULT_RTPS_RECV_PORT		5900
#define DEFAULT_RTPS_SEND_PORT		5901

class DevSerial;
class DevSocket;

struct StaticData {
	DevSerial *serial;
	DevSocket *mavlink2;
	DevSocket *rtps;
};

/*
MessageType is in MSB of header[1]
		|
		v
	Mavlink 0000 0000b
	Rtps    1000 0000b
*/
enum MessageType {Mavlink = 0x00, Rtps = 0x01};

const char  Sp2HeaderMagic = 'S';
const int   Sp2HeaderSize  = 4;

/*
Header Structure:
     bits:   1 2 3 4 5 6 7 8
header[0] - |     Magic     |
header[1] - |T|   LenH      |
header[2] - |     LenL      |
header[3] - |   Checksum    |
*/
typedef union __attribute__((packed))
{
	uint8_t bytes[4];
	struct {
		char magic;                // 'S'
		uint8_t len_h:	7,         // Length MSB
			 type:	1;         // 0=MAVLINK, 1=RTPS
		uint8_t len_l;             // Length LSB
		uint8_t checksum;          // XOR of the three bytes above
	} fields;
} Sp2Header_t;

// Signal to stop threads
volatile sig_atomic_t running{true};

struct options {
	uint32_t baudrate = DEFAULT_BAUDRATE;
	char uart_device[64] = DEFAULT_UART_DEVICE;
	char host_ip[16] = DEFAULT_HOST_IP;
	uint16_t mavlink_udp_recv_port = DEFAULT_MAVLINK_RECV_PORT;
	uint16_t mavlink_udp_send_port = DEFAULT_MAVLINK_SEND_PORT;
	uint16_t rtps_udp_recv_port = DEFAULT_RTPS_RECV_PORT;
	uint16_t rtps_udp_send_port = DEFAULT_RTPS_SEND_PORT;
	uint64_t passthrough_timeout_ms = DEFAULT_PASSTHROUGH_TIMEOUT_MS;
	bool sw_flow_control = false;
	bool hw_flow_control = false;
	bool verbose_debug = false;
} _options;

namespace
{
static StaticData *objects = nullptr;
}

// Flag to change to MAVLink pass-through mode
std::atomic<bool> mavlink_passthrough{false};

// UART fd access mutex
std::mutex uart_mtx;

class DevSerial
{
public:
	DevSerial(const char *device_name, const uint32_t baudrate, const bool hw_flow_control,
		  const bool sw_flow_control, const double passthrough_timeout_ms);
	virtual ~DevSerial();

	ssize_t	read();
	int	open_uart();
	int	close();

	int _uart_fd{-1};

	std::chrono::time_point<std::chrono::system_clock> _timer_start;

protected:
	uint32_t _baudrate;
	bool _hw_flow_control;
	bool _sw_flow_control;

	const uint64_t _passthrough_timeout_ms;
	bool _mavlink_passthrough_noticed{false};
	bool _protocol_splitter_header_found{false};

	char _uart_name[64] {};
	bool baudrate_to_speed(uint32_t bauds, speed_t *speed);

	uint8_t _buffer[BUFFER_SIZE] {};
	size_t _buf_size{0};
private:
};

class DevSocket
{
public:
	DevSocket(const char *_udp_ip, const uint16_t udp_port_recv,
		  const uint16_t udp_port_send, int uart_fd, MessageType type);
	virtual ~DevSocket();

	int close(int udp_fd);
	ssize_t	write();
	int	open_udp(const MessageType type);
	ssize_t udp_write(void *buffer, size_t len);

	int _uart_fd;
	int _udp_fd{-1};

protected:
	char _udp_ip[16] {};

	uint16_t _udp_port_recv;
	uint16_t _udp_port_send;
	struct sockaddr_in _outaddr;
	struct sockaddr_in _inaddr;

	char _buffer[BUFFER_SIZE] {};

	Sp2Header_t _header;
private:
	ssize_t udp_read(void *buffer, size_t len);
};
