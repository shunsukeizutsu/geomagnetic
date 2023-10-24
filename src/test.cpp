#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <vector>

#include "ssm-log.hpp"
#include "imu.hpp"
#include "creatfig.hpp"

static int gShutOff = 0;
static void ctrlC(int aStatus);
static void setSigInt(void);
static void Terminate(void);

int main(void)
{
    PlotData Fig(1);
	SSMLog<imu_fs,imu_property> imu;
	if (!imu.open("../2023.0402.0748/imu.log"))
    {
        fprintf(stderr, "Error! Cannot open imu.log");
        exit(EXIT_FAILURE);
    }
    imu_fs *imudata;
    imudata = &imu.data();
    try
    {
		setSigInt();
		while(!gShutOff)
		{
			if(imu.read())
			{
				Fig.SaveData2D(imu.time()-imu.getStartTime(),imudata->estAng[2]);
			}
            else
                break;
		}
        Fig.PrintFig2D();
        while(!gShutOff)
        {
            usleep(1000);
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
