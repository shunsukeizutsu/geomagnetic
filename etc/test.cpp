#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <vector>
#include <cstring>

#include "creatfig.hpp"

typedef struct
{
    double time;
    double x_coor;
    double y_coor;
} test;
typedef struct
{
    a[2];
    b[2];
} LQsolution;

static int gShutOff = 0;
static void ctrlC(int aStatus);
static void setSigInt(void);
static void Terminate(void);
static void LeastSquares(std::vector<test> &data);

int main(void)
{
    try
    {
        setSigInt();
        std::vector<test> data;
        int count = 0;
        while (!gShutOff)
        {
            if (count < 10)
            {
                test tmp;
                tmp.time = count + 1;
                tmp.x_coor = count + 2;
                tmp.y_coor = count + 3;

                data.push_back(tmp);
                count++;
            }

            else
                break;
        }
        LeastSquares(data);
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
static result LeastSquares(std::vector<data_sec> &data)
{
    double Xsum[2] = {0};
    double X2sum[2] = {0};
    double aveX[2] = {0};
    double aveX2[2] = {0};
    double Ysum[2] = {0};
    double aveY[2] = {0};
    double XYsum[2] = {0};
    double aveXY[2] = {0};

    for (int i = 0; i < data.size(); i++)
    {
        CreatMatrix MDD;
        MDD = CaluculateMatrix(data[i]);
        Xsum[0] += MDD.X(0, 0);
        Xsum[1] += MDD.X(1, 0);

        X2sum[0] += MDD.X(0, 0) * MDD.X(0, 0);
        X2sum[1] += MDD.X(1, 0) * MDD.X(1, 0);

        Ysum[0] += MDD.m(0, 0);
        Ysum[1] += MDD.m(1, 0);

        XYsum[0] += MDD.X(0, 0) * MDD.m(0, 0);
        XYsum[1] += MDD.X(1, 0) * MDD.m(1, 0);
    }

  /*  aveX[0] = Xsum[0] / data.size();
    aveX[1] = Xsum[1] / data.size();

    aveX2[0] = X2sum[0] / data.size();
    aveX2[1] = X2sum[1] / data.size();

    aveY[0] = Ysum[0] / data.size();
    aveY[1] = Ysum[1] / data.size();

    aveXY[0] = XYsum[0] / data.size();
    aveXY[1] = XYsum[1] / data.size();

    LQsolution AandB;
    double k;
    for (int i = 0; i < 2; i++)
    {

        double Molecular;   // 分子
        double Denominator; // 分母
        Molecular = aveXY[i] - aveX[i] * aveY[i];
        Denominator = aveX2[i] - aveX[i] * aveX[i];
        AandB.a[i] = Molecular / Denominator;   // 1/k
        AandB.b[i] = -a[i] * aveX[i] + aveY[i]; // offset
    }
    k = (a[0] + a[1]) / 2;
    k = 1 / k;*/
    return AandB;
}
