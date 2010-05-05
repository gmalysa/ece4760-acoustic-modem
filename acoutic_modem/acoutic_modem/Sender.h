#pragma once

#include "stdafx.h"
#include <string>

#define BYTE_DELAY 20
#define RESP_TIMEOUT 500
#define MAX_RETRIES 10

extern int doResend;

class Sender
{
public:
	static const unsigned char header = 0x0A;
	static const unsigned char resendHeader = 0x0F;
	static const unsigned char successResponse = 0x3E;

	Sender(wchar_t* port, int baud_rate);
	Sender(const Sender &s);
	void send(const unsigned char* data, size_t length, unsigned int delay);
	void write(const unsigned char* data, size_t length, unsigned int delay);
	unsigned char *read();
	~Sender(void);
private:
	HANDLE commPort;
	HANDLE responseEvent;
	HANDLE getResponseEvent;
	char last_crc;
};