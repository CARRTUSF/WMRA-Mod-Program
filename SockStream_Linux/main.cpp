
#include "SockStream_Linux.h"

using namespace std;

int main()
{	
	client_tcpsocket s;
	s.open("192.168.1.22", 23);

	if(s.connected())
		cout << "Connected" << endl;
	else
		cout << "Not Connected" << endl;
	
	return 1;
}