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
typedef struct
{
    double time;
    double localizer[3];
    double imumag[3];
    double gnssdata[3];
} data_sec;

typedef struct CalcMatrix
{
    Matrix<double, 3, 3> R;
    Matrix<double, 3, 1> m;
    Matrix<double, 3, 1> F;
    Matrix<double, 3, 1> b;
    Matrix<double, 3, 4> J;
    Matrix<double, 4, 3> JT;
    Matrix<double, 4, 1> X;
    Matrix<double, 3, 1> M;
} CreatMatrix;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);
void printdata(data_sec data);
CreatMatrix CaluculateMatrix(data_sec data, double k, double ax, double ay, double az);
std::vector<data_sec> data;