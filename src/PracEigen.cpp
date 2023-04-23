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

using namespace Eigen;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);

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

    try
    {
        setSigInt();
        while (!gShutOff)
        {

            while (rtk_gnss.read())
            {
                double Mx, My, Mz;
                double ax = 17.945373;
                double ay = -19.942942;
                double az = -36.060239;
                double k = 2;

                GG gmag;

                gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);

                Mx = gmag.GmagY / 1000;
                My = gmag.GmagX / 1000;
                Mz = -gmag.GmagZ / 1000;

                if (imu.readTime(rtk_gnss.time()))
                {
                    if (local.readTime(rtk_gnss.time()))
                    {
                        double r11, r12, r13, r21, r22, r23, r31, r32, r33;
                        double Sr, Sp, Sy, Cr, Cp, Cy;
                        double k1, ax1, ay1, az1;
                        Sr = sin(localdata->estAng[0]);
                        Sp = sin(localdata->estAng[1]);
                        Sy = sin(localdata->estAng[2]);
                        Cr = cos(localdata->estAng[0]);
                        Cp = cos(localdata->estAng[1]);
                        Cy = cos(localdata->estAng[2]);
                        r11 = Cy * Cp;
                        r12 = -Sy * Cr + Cy * Sp * Sr;
                        r13 = Sr * Cy + Cy * Sp * Cr;
                        r21 = Sy * Cp;
                        r22 = Cy * Cr + Sr * Sp * Sy;
                        r23 = -Cy * Sr + Sp * Sy * Cr;
                        r31 = -Sp;
                        r32 = Cp * Sr;
                        r33 = Cp * Cr;
                        Matrix<double, 3, 1> M;
                        Matrix<double, 3, 3> R;
                        Matrix<double, 3, 1> m;
                        Matrix<double, 3, 1> F;
                        Matrix<double, 3, 1> b;
                        Matrix<double, 3, 4> J;
                        Matrix<double, 4, 3> JT;
                        Matrix<double, 4, 1> X;
                        //Matrix<double, 4, 4> A;
                        // printf("%f %f %f \n", imudata->mag[0], imudata->mag[1], imudata->mag[2]);
                        // gnss->magdata
                        M(0, 0) = Mx;
                        M(1, 0) = My;
                        M(2, 0) = Mz;
                        //std::cout << M << std::endl;
                        // rotation
                        R(0, 0) = r11;
                        R(0, 1) = r12;
                        R(0, 2) = r13;
                        R(1, 0) = r21;
                        R(1, 1) = r22;
                        R(1, 2) = r23;
                        R(2, 0) = r31;
                        R(2, 1) = r32;
                        R(2, 2) = r33;
                        // 生の地磁気データ - offset値
                        m(0, 0) = imudata->mag[0] - ax;
                        m(1, 0) = imudata->mag[1] - ay;
                        m(2, 0) = imudata->mag[2] - az;
                        F = k * R * m;
                        //std::cout << F << std::endl;
                        b = M - F;
                        //std::cout << b << std::endl;
                        // Jの行列
                        J(0, 0) = r11 * (imudata->mag[0] - ax) + r12 * (imudata->mag[1] - ay) + r13 * (imudata->mag[2] - az);
                        J(0, 1) = -k * r11;
                        J(0, 2) = -k * r12;
                        J(0, 3) = -k * r13;
                        J(1, 0) = r21 * (imudata->mag[0] - ax) + r22 * (imudata->mag[1] - ay) + r23 * (imudata->mag[2] - az);
                        J(1, 1) = -k * r21;
                        J(1, 2) = -k * r22;
                        J(1, 3) = -k * r23;
                        J(2, 0) = r31 * (imudata->mag[0] - ax) + r32 * (imudata->mag[1] - ay) + r33 * (imudata->mag[1] - az);
                        J(2, 1) = -k * r31;
                        J(2, 2) = -k * r32;
                        J(2, 3) = -k * r33;
                        // Jの転置行列
                        JT(0, 0) = J(0, 0);
                        JT(0, 1) = J(1, 0);
                        JT(0, 2) = J(2, 0);
                        JT(1, 0) = J(0, 1);
                        JT(1, 1) = J(1, 1);
                        JT(1, 2) = J(2, 1);
                        JT(2, 0) = J(0, 2);
                        JT(2, 1) = J(1, 2);
                        JT(2, 2) = J(2, 2);
                        JT(3, 0) = J(0, 3);
                        JT(3, 1) = J(1, 3);
                        JT(3, 2) = J(2, 3);

                        // delta_X
                        /*X(0, 0) = k1 - k;
                        X(1, 0) = ax1 - ax;
                        X(2, 0) = ay1 - ay;
                        X(3, 0) = az1 - az;*/


                        //A = JT * J;

                        Eigen::PartialPivLU<Eigen::MatrixXd>lu(JT*J);
                        X = lu.solve(JT*b);
                        std::cout << X << std::endl;
                        std::cout << "\n " << std::endl;
                        
                    }
                }
            }
            gShutOff = 1;
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