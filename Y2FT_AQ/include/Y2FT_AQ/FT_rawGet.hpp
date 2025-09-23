#pragma once

#include <iostream>
#include <iomanip>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <vector>
#include "Y2Matrix/YMatrix.hpp" // YMatrix library for matrix operations

#define MAX_RETRY 5
#define RECV_SIZE 50

/* Basic ethernet communication with AFT200-D80-EN version*/
struct FTData
{
    FTData(double fx=0.0, double fy=0.0, double fz=0.0,
    double mx=0.0, double my=0.0, double mz=0.0):
    Fx(fx),Fy(fy),Fz(fz),Mx(mx),My(my),Mz(mz)
    {}
    double Fx,Fy,Fz,Mx,My,Mz;
};


class FT_rawGet
{
    public:
        FT_rawGet(const std::string& IP, const int PORT);
        ~FT_rawGet();
        FTData FTGet();
        bool FT_init(const unsigned int init_count_num);

    private:
        std::string IP_;
        unsigned int PORT_;
        std::vector<double> init_force, init_moment;
        int s; // socket file descriptor

        bool init_flag = false;
        unsigned int init_count = 0;

        /* Base functions */
        float unpackFloat(const char *bytes);
        char *recvMsg();
};

