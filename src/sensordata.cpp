#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "Eigen/Core"
#include <Eigen/LU>
#include "ssm-log.hpp"
#include "gnss-f9p.hpp"
#include "imu.hpp"
#include "localizer.hpp"
#include "GnsstoMag.hpp"
#include <vector>

using namespace Eigen;
typedef struct{
    double time;
    double localizer[3];
    double imumag[3];
    double gnssdata[2];
}data_sec;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);
void printdata(data_sec data);

int main(void)
{
    MagEquation MF;
    SSMLog<rtk_gnss_f9p> rtk_gnss;
    SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer> local;

    //    if (!rtk_gnss.open("../../geo_yawoffset/shirane_SnowRemovedata/2022.0324.1239/rtk_gnss.log"))
    if (!rtk_gnss.open("../shiraneroad/2022.0324.1239/rtk_gnss.log"))
    {
        fprintf(stderr, "Error! Cannot open logfile");
        exit(EXIT_FAILURE);
    }

    //    if (!imu.open("../../geo_yawoffset/shirane_SnowRemovedata/2022.0324.1239/imu.log"))
    if (!imu.open("../shiraneroad/2022.0324.1239/imu.log"))
    {
        fprintf(stderr, "Error! Cannot open logfile");
        exit(EXIT_FAILURE);
    }

    //    if (!local.open("../../geo_yawoffset/shirane_SnowRemovedata/2022.0324.1239/localizer.log"))
    if (!local.open("../shiraneroad/2022.0324.1239/localizer.log"))
    {
        fprintf(stderr, "Error! Caonnot open localizer,logfile");
        exit(EXIT_FAILURE);
    }

    rtk_gnss_f9p *gnssdata;
    imu_fs *imudata;
    localizer *localdata;

    gnssdata = &rtk_gnss.data();
    imudata = &imu.data();
    localdata = &local.data();

    std::vector<data_sec>data;
    try
    {
        setSigInt();
        double time = 0.0;
        while (!gShutOff)
        {
            // double time = rtk_gnss.time();
            // printf("%f\n",time);
            if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {
                    data_sec tmp;
                    time = rtk_gnss.time();
                    tmp.time = time;
                    tmp.gnssdata[0] = gnssdata->latitude;
                    tmp.gnssdata[1] = gnssdata->longitude;
//                    printf("%f\n", time);
                    if(imu.readTime(time))
                    {
                        tmp.imumag[0] = imudata->mag[0];
                        tmp.imumag[1] = imudata->mag[1];
                        tmp.imumag[2] = imudata->mag[2];
                    }
                    if(local.readTime(time))
                    {
                        tmp.localizer[0] = localdata->estAng[0];
                        tmp.localizer[1] = localdata->estAng[1];
                        tmp.localizer[2] = localdata->estAng[2];
                    }
                    data.push_back(tmp);

                }
            }
            else
                break;
        }
        for(int i =0;i<data.size();i++)
        {
            printdata(data[i]);
        }
    }
    catch (std::runtime_error const &error)
    {
        std::cout << error.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    }
    Terminate();
    return EXIT_SUCCESS;
}
static void ctrlC(int aStatus)
{
    signal(SIGINT, NULL);
    gShutOff = 1;
}
static void setSigInt()
{
    struct sigaction sig;
    memset(&sig, 0, sizeof(sig));
    sig.sa_handler = ctrlC;
    sigaction(SIGINT, &sig, NULL);
}
static void Terminate(void)
{
    printf("\nend\n");
}
void printdata(data_sec data)
{
    printf("%f %f %f %f %f %f %f %f %f \n",data.time,data.gnssdata[0],data.gnssdata[1],data.imumag[0],data.imumag[1],data.imumag[2],data.localizer[0],data.localizer[1],data.localizer[2]);
}