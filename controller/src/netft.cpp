#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <stdlib.h> //exit(0);

#include "netft.h"

#define PORT "49152"

struct response_struct
{
  uint32_t rdt_sequence;
  uint32_t ft_sequence;
  uint32_t status;
  uint32_t FTData[6];
};

// socket stuff
int fd_socket;
struct addrinfo dev_addr_info, *infoptr;
struct hostent *he;

// netbox commands
unsigned char request_command[8];
//request_command[0] = htons(0x1234);
//*(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
//*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
response_struct resp;
unsigned char response[36];

netft::netft(){}

int netft::connect_netbox(std::string address)
{
    std::memset(&dev_addr_info, 0, sizeof(addrinfo));
    dev_addr_info.ai_family = AF_INET;
    dev_addr_info.ai_socktype = SOCK_DGRAM;
    dev_addr_info.ai_protocol = 0;

    if((getaddrinfo(address.c_str(), PORT, &dev_addr_info, &infoptr)) == -1)
    {
        fprintf(stderr, "getaddrinfo failed\n");
        exit(1);
    }

    if((fd_socket=socket(infoptr->ai_family, infoptr->ai_socktype, infoptr->ai_protocol)) == -1)
    {
        fprintf(stderr, "socket() failed\n");
        exit(1);
    }

    if((connect(fd_socket, infoptr->ai_addr, infoptr->ai_addrlen)) == -1)
    {
        fprintf(stderr, "connection failed, netbox is not available\n");
        exit(1);
    }

    return 0;
}

int netft::start_netbox()
{
    send(fd_socket, request_command, 8, 0);
    return 0;
}

int netft::getRDT()
{
    return 0;
}
