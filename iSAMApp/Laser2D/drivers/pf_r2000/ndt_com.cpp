#include <ndt_com.h>
#include <iostream>

NDTCom::NDTCom(char* _agv, int _Port):_ip(_agv),_cPort(_Port)
{
    pDataToSend = new Data();
    pDataRecived = new Data();
    _sockfd = socket(AF_INET, SOCK_STREAM, 0);  
    if(_sockfd < 0)  
    {
	perror("sockfd");
    }

    bzero(&_saddr,sizeof(_saddr)); 
    _saddr.sin_family = AF_INET; 
    _saddr.sin_port = htons(_cPort); 
    _saddr.sin_addr.s_addr = inet_addr(_ip);
    inet_pton(AF_INET, _ip, &_saddr.sin_addr);

	// while(bind(_csockfd, (struct sockaddr*)&_cAddr, sizeof(_cAddr)) < 0)
	// {
	//     std::cerr<<"connect......"<<std::endl;
	//     sleep(5);
	// }
    // std::cout<<"connected !"<<std::endl;
}


NDTCom::~NDTCom()
{
    if(!pDataToSend)
    {
	shutdown(_sockfd,SHUT_RDWR);
	delete pDataToSend;
	pDataToSend = NULL;
    delete pDataRecived;
    pDataRecived = NULL;
    }
}

void NDTCom::start()
{
    // while(listen(_csockfd, 2) < 0);   
    //     perror("listen");
    while(connect(_sockfd, (struct sockaddr*)&_saddr, sizeof(_saddr)) < 0) 
    {  
        perror("connect");  
        sleep(5);  
    }    
	return; 
}

void NDTCom::Receive()
{

    while(recv(_sockfd, pDataRecived, sizeof(Data), 0) <= 0);

    std::cout <<"x: "<< pDataRecived->x <<" y:  "<< pDataRecived->y  <<" yaw:   "<< pDataRecived ->theta << std::endl;

    return;
}

void NDTCom::Send()
{

    //socklen_t aAddr_len = sizeof(_saddr);
         
    // while((_asockfd = accept(_csockfd, (struct sockaddr*)&_saddr, &aAddr_len)) < 0) 
    // {  
    //     perror("accept");  
    //     sleep(5);  
    // }

    //inet_ntop(AF_INET, &_saddr.sin_addr, _agvip, INET_ADDRSTRLEN);

    // printf("IP is %s\n", inet_ntoa(_saddr.sin_addr));
 
    // printf("Port is %d\n", htons(_saddr.sin_port));

    send(_sockfd, pDataToSend, sizeof(Data), 0);

    return;
}
