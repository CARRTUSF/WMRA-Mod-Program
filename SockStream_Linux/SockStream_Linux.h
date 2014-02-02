#ifndef SOCKSTREAM_LINUX_H
#define SOCKSTREAM_LINUX_H
/*

 - Untested - 

SockStream_Linux.h

Linux socket communication library. 
Wrapper for client side socket communication.

Include this file instead of 
		#include "SockStream.h"
if compiling WMRA-Mod-Program for a linux operating system.
*/

#include <cstring>
#include <stdlib.h>
#include <iostream>
#include "utility.h"

class client_tcpsocket {

public:
	client_tcpsocket();
	~client_tcpsocket();
	int open(const char * c, int port);
	int connected();
	int write( char* buffer, size_t count );
	int read( char* buffer, size_t count );
	
private:
	utility util;
	// Setting up Socket Communucation with Galil Board
	int cID, s, sID;
	
	int _connected;

};

#endif