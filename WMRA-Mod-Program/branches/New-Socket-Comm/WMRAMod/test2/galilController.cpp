#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include "SockStream.h"
#include "stringUtility.h"
#include "galilController.h"

using namespace std;

#define PI 3.14159265

server_tcpsocket sock;

// Public Functions

galilController::galilController(string IP)
{
	initialized = false;

	sock.open(IP.c_str());

	if(sock.is_open())
		cout << "socket open" << endl;
	else
		cout << "socket error" << endl;

	sock.connected();

	


/*

// OLD Socket For R-Pi


	// Setting up Socket Communucation with Galil Board
	int cID, s = 0;

	// Initializing Socket Structs
	struct addrinfo host_info;
	struct addrinfo *host_info_list;


	// Setting host_info values to zeros
	memset(&host_info, 0, sizeof host_info);
	

	// Setting host type
	host_info.ai_family = AF_INET;
	host_info.ai_socktype = SOCK_STREAM;
	

	// Getting connection info from IP
	s = getaddrinfo(IP, NULL, &host_info, &host_info_list);
	if(s != 0)
		cout << "IP error" << endl;
	

	// Setting up client(R-Pi) socket
	sID = socket(AF_INET, SOCK_STREAM, 0);
	cout << "Communication Socket ID: " << sID << endl;


	// Client(R-Pi) connecting to Server(Controller Board)
	cID = connect(sID, host_info_list->ai_addr, host_info_list->ai_addrlen);
	if(cID == 0)
	{
		initialized = true;
		cout << "WMRA Connection Success\n" << endl;
	}
	else
	{
		initialized = false;
		cout << "WMRA Connection Error\n" << endl;
	}

	*/
}
/*
void galilController::~MotorController()
{
	command("MO"); //turn off motors
}
*/

bool galilController::isInitialized() // return initialized
{
	return initialized;
}

string galilController::command(string Command)
{
	char com[300];
	char ret[300];
	string c = Command + "\r";
	strcpy(com, c.c_str());
	commandGalil(com, ret, sizeof(ret));
	string ret_str(ret);
	return ret_str;
}

// Private Functions

//command() sends an ASCII Command (e.g. "TPX") to the controller and retrieves a Response (e.g. "123\r\n:").
//The size of Response should be supplied as ResponseSize so that unallocated memory is not overwritten.
//If you statically allocate your response buffer (e.g. char buff[100]) use sizeof(buff).
int galilController::commandGalil(char* Command, char* Response, int ResponseSize) //returns the number of bytes read
{
   char acPartialResponse[512] = {0}; //buffer to contain partial responses (which will be concatenated together to form the final response)
   int iPartialBytesRead = 0; //number of bytes read each time through the loop
   int iTotalBytesRead = 0;   //the total number of bytes read.  Can't exceed ResponseSize.
   
   
   Response[0] = 0; //set response to null string 
   
   sock.write(Command, strlen(Command));

   //OLD WRITE FOR R-PI
   //write(sID, Command, strlen(Command)); //write the command to the controller
   //write(sID, "\r", 1);

   //keep reading until we (a) get a colon (b) get a question mark (c) fill up the callers Response buffer
   while(1)
   {
	  iPartialBytesRead = sock.read(acPartialResponse, sizeof(acPartialResponse)); //read some characters


      // OLD READ FOR R-PI
      //iPartialBytesRead = read(sID, acPartialResponse, sizeof(acPartialResponse)); //read some characters
      
      if(iPartialBytesRead <= 0)   //nothing read, keep reading until :
         continue;
      else if(iTotalBytesRead + iPartialBytesRead > ResponseSize) //get out of the loop if we will fill up the caller's buffer, iPartialBytesRead >= 1
         break;
      else 
      {
         strncat(Response, acPartialResponse, iPartialBytesRead); //add the partial response to the full response.  Response is null terminated
         iTotalBytesRead += iPartialBytesRead; //tally up the total number of bytes read
   //    printf("%s|%s|%i\n", Response, acPartialResponse, iPartialBytesRead); 
         if (acPartialResponse[iPartialBytesRead - 1] == ':' || acPartialResponse[iPartialBytesRead - 1] == '?') //got a colon, iPartialBytesRead >= 1
            break;
      }
   }
   
   return(iTotalBytesRead);
}