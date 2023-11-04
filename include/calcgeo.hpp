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
#include <getopt.h>

#define STRLEN 256
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
    Matrix<double, 3, 3> R;//回転行列
    Matrix<double, 3, 3> RT;//回転行列の転置
    Matrix<double, 3, 1> m;//生の地磁気センサデータ
    Matrix<double, 3, 1> X;//RT*M
    Matrix<double, 3, 1> M;//緯度経度から算出された地磁気データ
} CreatMatrix;

typedef struct
{
    double a[2];
    double b[2];
} LQsolution;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);
void printdata(data_sec data);
CreatMatrix CaluculateMatrix(data_sec data);
void printdata(double a,double b,double c);
double Caloffsetz(data_sec data, double k, double offX, double offY);
static bool setOption(int aArgc, char *aArgv[]);
static int printShortHelp(const char *programName);
static LQsolution LeastSquares(std::vector<data_sec> &data);
static void Calustatic(std::vector<double> &adata);
