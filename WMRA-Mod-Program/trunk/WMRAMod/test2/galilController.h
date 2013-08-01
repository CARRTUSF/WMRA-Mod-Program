#ifndef GALILCONTROLLER_H
#define GALILCONTROLLER_H

#include <iostream>
#include <stdio.h>
//#include <sys/socket.h>
#include <cstring>
//#include <netdb.h>
#include <exception>
#include <string>
#include <sstream>
#include <vector>
#include <ctype.h>

// windows includes
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#pragma comment(lib, "Ws2_32.lib")


using namespace std;

class galilController {
public:
    galilController(string IP);
    //void ~MotorController();
    bool isInitialized(); // return initialized
    string command(string Command);
    int sID;	

private:

    bool initialized ;
    string ipAddr;
    int commandGalil(char* Command, char* Response, int ResponseSize);
};

#endif