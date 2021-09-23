/*
 * TCPSocket.h
 *
 *  Created on: Dec 16, 2014
 *      Author: Jack Bowling
 */

#ifndef LINCLUDES_CLSTCPSOCKET_H_
#define LINCLUDES_CLSTCPSOCKET_H_

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/types.h>
#include <vector>

#if WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#else
#include <unistd.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include "libConversion.h"
#include "libException.h"

/* BufferLength is 100 bytes */
#define REC_BUFFER_LENGTH 50000
//#define SND_BUFFER_LENGTH 500
/* Default host name of server system. Change it to your default */
/* server hostname or IP.  If the user do not supply the hostname */
/* as an argument, the_server_name_or_IP will be used as default*/
//#define SERVER "127.0.0.1"
/* Server's port number */
//#define SERVPORT 10302


class clsTCPSocket{
private:
	//char sndBuffer[SND_BUFFER_LENGTH];

public:
	const char* NameIP;
	struct hostent *HostName;

	struct sockaddr_in IP;

	int Port;
	#if WIN32
	SOCKET SocketNum;
	#else
	int SocketNum;
	#endif

	int IsBound;
	char recBuffer[REC_BUFFER_LENGTH];
	//char sndBuffer[REC_BUFFER_LENGTH];

	int Connect(void);
	bool Listen(clsTCPSocket*);
	void Close(int = 0);

	bool Configure(void);
	bool ConfigureFTP(void);
	int Write(const char*, int = 0);
	bool HasData();
	long Read(long = 0);
};

#endif /* LINCLUDES_TCPSOCKET_H_ */
