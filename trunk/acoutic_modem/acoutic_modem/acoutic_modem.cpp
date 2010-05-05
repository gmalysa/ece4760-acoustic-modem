// acoutic_modem.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <sstream>
#include "Sender.h"

using namespace std;

int doResend;

DWORD WINAPI readThread(LPVOID parameters) {
	Sender *s = (Sender *) parameters;
	unsigned char *data;
	while(1) {
		data = s->read();
		if (data != NULL) {
			cout << data;
			free(data);
		}
	}
	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
	DWORD tid;
	Sender sender(argv[1], 2400);
	Sender s2(sender);
	s2.ignoreDuplicates = 1;

	CreateThread(NULL, 0, &readThread, &s2, 0, &tid);
	
	if (tid == NULL) {
		cout << "Error creating thread!." << endl;
		return 1;
	}

	while(1) {
		stringbuf s;
		cin.get(s);
		sender.send((const unsigned char *)s.str().c_str(), s.str().length()+1, BYTE_DELAY);
		cin.sync();
		cin.clear();
	}
}

