#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <string>
using namespace std;

std::string get_ip(std::string eth_name)
{
    int sockfd;
    struct sockaddr_in sin;
    struct ifreq ifr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        perror("socket error");
        exit(1);
    }
    strncpy(ifr.ifr_name, eth_name.c_str(), IFNAMSIZ); // Interface name

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == 0)
    { // SIOCGIFADDR 获取interface address
        memcpy(&sin, &ifr.ifr_addr, sizeof(ifr.ifr_addr));
        return inet_ntoa(sin.sin_addr);
    }
    return "127.0.0.1";
}

