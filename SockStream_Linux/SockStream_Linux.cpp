/*

 - Untested - 

SockStream_Linux.cpp

Linux socket communication library. 
Wrapper for client side socket communication.

Include SockStream_Linux.h file instead of SockStream.h if 
compiling WMRA-Mod-Program for a linux operating system.
*/

#include "SockStream_Linux.h"

using namespace std;

client_tcpsocket::client_tcpsocket()
{

}

client_tcpsocket::~client_tcpsocket()
{

}

int client_tcpsocket::open(const char * c, int port)
{

	// Initializing Socket Structs
	struct addrinfo host_info;
	struct addrinfo *host_info_list;

	// Setting host_info values to zeros
	memset(&host_info, 0, sizeof host_info);
	
	// Setting host type
	host_info.ai_family = AF_INET;
	host_info.ai_socktype = SOCK_STREAM;	
	
    // Getting connection info from IP
	//s = getaddrinfo("192.168.1.22", NULL, &host_info, &host_info_list);
	s = getaddrinfo(c, NULL, &host_info, &host_info_list);
	if(s != 0)
		cout << "IP error" << endl;
		
	// Setting up client(R-Pi) socket
	sID = socket(AF_INET, SOCK_STREAM, 0);
	cout << "Communication Socket ID: " << sID << endl;


	// Client(R-Pi) connecting to Server(Controller Board)
	cID = connect(sID, host_info_list->ai_addr, host_info_list->ai_addrlen);
	if(cID == 0)
	{
		cout << "WMRA Connection Success\n" << endl;
		_connected = true;
	}
	else
	{
		cout << "WMRA Connection Error\n" << endl;
		_connected = false;
	}
	
	return 1;
	
}

int client_tcpsocket::connected()
{
	return _connected;
}

int client_tcpsocket::write( char* buffer, size_t count )
{
	util.write_utility(sID,buffer, strlen(buffer)); //write the command to the controller	
}

int client_tcpsocket::read( char* buffer, size_t count )
{
	util.read_utility(sID,buffer, strlen(buffer)); //read the value from the controller
}

