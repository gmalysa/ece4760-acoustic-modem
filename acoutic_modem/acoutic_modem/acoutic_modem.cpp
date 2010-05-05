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

	CreateThread(NULL, 0, &readThread, &s2, 0, &tid);
	
	if (tid == NULL) {
		cout << "Error creating thread!." << endl;
		return 1;
	}

	while(1) {
		stringbuf s;
		cin.get(s);
		unsigned char c[5];
		while(s.sgetc() != EOF) {
			for(int i=0; i<4; ++i) {
				if(s.sgetc() == EOF) {
					c[i] = '\n';
				} else {
					c[i] = s.sbumpc();
				}
			}
			sender.send(c, 4, BYTE_DELAY);
		}
		cin.sync();
		cin.clear();
	}
}

