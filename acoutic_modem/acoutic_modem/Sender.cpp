#include "StdAfx.h"
#include "Sender.h"
#include <iostream>

using namespace std;

/**
 * Initialize the COM port given, set up timeouts, etc.
 */
Sender::Sender(wchar_t* port, int baud_rate) : last_crc(0)
{
	COMMTIMEOUTS timeouts;
	commPort = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
	DCB config;
	GetCommState(commPort, &config);
	config.BaudRate = baud_rate;
	SetCommState(commPort, &config);
	timeouts.ReadIntervalTimeout = 20; 
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 100;
	SetCommTimeouts(commPort, &timeouts);

	responseEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	getResponseEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
}

/**
 * The only reason we need this is to use the winapi to duplicate the file handle
 */
Sender::Sender(const Sender &s) : last_crc(0) {
	DuplicateHandle(GetCurrentProcess(), s.commPort, GetCurrentProcess(), &this->commPort, 0, true, DUPLICATE_SAME_ACCESS);
	DuplicateHandle(GetCurrentProcess(), s.responseEvent, GetCurrentProcess(), &this->responseEvent, 0, true, DUPLICATE_SAME_ACCESS);
	DuplicateHandle(GetCurrentProcess(), s.getResponseEvent, GetCurrentProcess(), &this->getResponseEvent, 0, true, DUPLICATE_SAME_ACCESS);
}

Sender::~Sender(void)
{
	CloseHandle(commPort);
	CloseHandle(responseEvent);
	CloseHandle(getResponseEvent);
}

unsigned char *Sender::read() {
	DWORD bytes, dwRes;
	int i;
	char crc;
	OVERLAPPED osRead = {0};
	unsigned char *result = (unsigned char *)calloc(6, sizeof(unsigned char));
	unsigned char *text;
	SetCommMask(commPort, EV_RXCHAR);

	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	ReadFile(commPort, result, 1, &bytes, &osRead);
	dwRes = WaitForSingleObject(osRead.hEvent, INFINITE);
	CloseHandle(osRead.hEvent);
	if (result[0] == header) {
		for (i = 0; i < 5; ++i) {
			osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			ReadFile(commPort, result+1+i, 1, &bytes, &osRead);
			dwRes = WaitForSingleObject(osRead.hEvent, INFINITE);
			CloseHandle(osRead.hEvent);
		}
		if (result[5] != CRC8_BlockChecksum(result+1, 4)) {
			result[0] = resendHeader;
			this->write(result, 1, BYTE_DELAY);
			Sleep(BYTE_DELAY);
			free(result);
			return NULL;
		} else {
			crc = result[5];
			result[0] = successResponse;
			this->write(result, 1, BYTE_DELAY);
			if (crc != last_crc) {
				last_crc = crc;
				result[5] = '\0';
				i = 1;
				while (*(result+i++)) {;}		// Replacement for strlen for unsigned chars
				text = (unsigned char *)calloc(i, sizeof(unsigned char));
				memcpy(text, result+1, i-1);
				free(result);
				return text;
			}
			else {
				Sleep(BYTE_DELAY);
			}
			free(result);
			return NULL;
		}
	}
	else if (result[0] == resendHeader) {
		WaitForSingleObject(getResponseEvent, INFINITE);
		doResend = 1;
		SetEvent(responseEvent);
	}
	else if (result[0] == successResponse) {
		WaitForSingleObject(getResponseEvent, INFINITE);
		doResend = 0;
		SetEvent(responseEvent);
	}
	free(result);
	return NULL;
}

/**
 * Used to read actual message data from the device to print it on screen
 */
//unsigned char *Sender::readMsg() {
//}

/**
 * Sends data to another device, expects response and retransmits as appropriate
 */
void Sender::send(const unsigned char *data, size_t length, unsigned int delay) 
{
	size_t padded_length = (length & 0x3 ? (length + 4) & ~((size_t)0x3) : length);
	unsigned char block[6];
	size_t i;
	int retryCount = 0;
	DWORD response;
	unsigned char *output = (unsigned char *) calloc(padded_length, sizeof(unsigned char));
	
	memcpy(output, data, length * sizeof(unsigned char));
	block[0] = header;
	for (i = 0; i < padded_length; i += 4) {
		// Transmit block
		block[5] = CRC8_BlockChecksum(output+i, 4);
		memcpy(block+1, output+i, 4);
		this->write(block, 6, delay);

		// Get response automatically in read thread, so wait for the signal!
		ResetEvent(responseEvent);
		SetEvent(getResponseEvent);
		response = WaitForSingleObject(responseEvent, RESP_TIMEOUT);
		ResetEvent(getResponseEvent);
		if (doResend || response != WAIT_OBJECT_0) {
			i -= 4;
			if (retryCount++ > MAX_RETRIES) {
				cout << "Retry count exceeded--part of your message may be dropped." << endl;
				i += 4;
			}
		}
	}

	free(output);
}

/**
 * Writes a packet directly to serial, does not wait for a response
 */
void Sender::write(const unsigned char* data, size_t length, unsigned int delay) {
	size_t i;
	DWORD x, dwRes;
	OVERLAPPED osWrite = {0};
	for (i = 0; i < length; ++i) {
		osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		WriteFile(commPort, &data[i], 1, &x, &osWrite);
		dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
		Sleep(delay);
		CloseHandle(osWrite.hEvent);
	}
}