/*utility.h*/

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <exception>

class utility {
public:
	utility() {};
	int write_utility( int sID, char* buffer, size_t count ) {
		write(sID, buffer, strlen(buffer)); //write the command to the controller
	};
	int read_utility( int sID, char* buffer, size_t count ) {
		read(sID, buffer, strlen(buffer)); //write the command to the controller
	};

private:

};