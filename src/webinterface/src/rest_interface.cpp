#include "httplib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include "pubdef.h"


using namespace std;
string ip;

std::string get_ip(string eth_name)
{
    int                 sockfd;
    struct sockaddr_in  sin;
    struct ifreq        ifr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("socket error");
        exit(1);
    }
    strncpy(ifr.ifr_name, eth_name.c_str(), IFNAMSIZ);      //Interface name

    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == 0) {    //SIOCGIFADDR 获取interface address
        memcpy(&sin, &ifr.ifr_addr, sizeof(ifr.ifr_addr));
        return inet_ntoa(sin.sin_addr);
    }
    return "127.0.0.1";
}


void Web_CallBackFunc(const httplib::Request &req, httplib::Response &resp)
{
    //char lp[255];
    string rlt="";
    string action=req.get_param_value("action");
    if (action=="stand")
    {

    }

    string message=req.get_param_value("message");
    if (message=="status")
    {
        rlt = "电压:"+to_string(batt_volt);
        

    }
    /**/
    if (rlt=="")
    {
        rlt = ip+":8550/command?action=stand(go|left|back)\n";
        rlt = rlt+ip+":8550/command?meassage=battary(video|status)";
    }

    resp.set_content(rlt, "text/plain");
}

void http_thread()
{
    ip = get_ip("eno1");
    httplib::Server http_svr;
    http_svr.Get("/command", Web_CallBackFunc);

    http_svr.set_mount_point("/www/", "www");
    // 解决跨域问题
    http_svr.set_default_headers({{"Access-Control-Allow-Origin", "*"},
                                  {"Access-Control-Allow-Methods", "POST, GET, PUT, OPTIONS, DELETE"},
                                  {"Access-Control-Max-Age", "3600"},
                                  {"Access-Control-Allow-Headers", "*"},
                                  {"Content-Type", "application/json;charset=utf-8"}});
    printf("%s:8550/command?action=stand\n",ip.c_str());
    http_svr.listen("0.0.0.0", 8550);
}
