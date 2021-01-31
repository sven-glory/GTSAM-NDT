#ifndef NDTCOM_H_
#define NDTCOM_H_

#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <unistd.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/thread/mutex.hpp>

class NDTCom
{
public:	
	struct Data
	{
	    float x;
	    float y;
	    float theta;
	    int flag;
	};
	Data *pDataToSend;				//data to transmit	
	Data *pDataRecived;

private:
	char       *_ip;    	// Host IP address
	int        _cPort;          	// Host port
	struct sockaddr_in _saddr;
	int 		_sockfd;

public:
	NDTCom(char* _agv, int _nPort);

	~NDTCom();

	void start();

	void DisConnect();

	void Receive();
	
	void Send();
};


#endif