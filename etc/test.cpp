/************
 * 2023/04/23
 * Author : Shunsuke
 * SSMのlogfileからデータを出力するプログラム見本
 **************/

#include <stdio.h>
#include <signal.h>
#include <stdexcept>
#include <stdlib.h>
#include "ssm-log.hpp"
#include "imu.hpp"
#include "gnss-f9p.hpp"
#include "localizer.hpp"
#include "Eigen/Core"
#include <Eigen/LU>

using namespace Eigen;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);
double trans_q2( double theta );
int main(void)
{
    // SSMLog<データ構造体名,プロパティ構造体名> imu
    //"imu"の場所はなんでも良い
    // データ名とプロパティ名はheaderファイル参照
    // SSMLog<gnss_gl, gnss_property> gnss;
    SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer, localizer_property> local;

    if (!imu.open("../2023.0402.0748/imu.log"))
    {
        // logfileを開けるか否か
        fprintf(stderr, "Error! Cannot open logfile");
        exit(EXIT_FAILURE);
    }
    if (!local.open("../2023.0402.0748/localizer.log"))
    {
        // logfileを開けるか否か
        fprintf(stderr, "Error! Cannot open logfile");
        exit(EXIT_FAILURE);
    }
    // データの構造体を指すポインタを生成
    imu_fs *magdata;
    localizer *localdata;
    // データが保存されているアドレスをポインタに代入
    magdata = &imu.data();
    localdata = &local.data();

    try
    {
        setSigInt();
        double mag_offsetX = 17.0652;
        double mag_offsetY = 26.1141;
        double mag_offsetZ = -29.8457;
        double kdata = 0.232248;


        while (!gShutOff)
        {
            // bool関数であるreadを使う
            Matrix<double, 3, 3> Rrp;
            Matrix<double, 3, 1> m;
            Matrix<double, 3, 1> M;

            if (imu.read())
            {
                double tmp_mag[3];
                double Sr, Sp, Cr, Cp;
                Sr = sin(magdata->estAng[0]);
                Sp = sin(magdata->estAng[1]);
                Cr = cos(magdata->estAng[0]);
                Cp = cos(magdata->estAng[1]);
                double xx = magdata->mag[0] - mag_offsetX;
                double yy = magdata->mag[1] - mag_offsetY;
                double zz = magdata->mag[2] - mag_offsetZ;

                tmp_mag[0] = (Cp * xx) + (Sp * Sr * yy) + (Sp * Cr * zz);
                tmp_mag[1] = (Cr * yy) - (Sr * zz);
                tmp_mag[2] = -1.0 * (Sp * xx) + (Cp * Sr * yy) + (Cp * Cr * zz);
                tmp_mag[0] = tmp_mag[0] * kdata;
                tmp_mag[1] = tmp_mag[1] * kdata;
                tmp_mag[2] = tmp_mag[2] * kdata;
                double theta_mag = atan2(-1.0 * tmp_mag[1], tmp_mag[0]);
                theta_mag = trans_q2(theta_mag + (M_PI_2));
                printf("%f %f\n",imu.time(),theta_mag);
            }
            /*if(local.read())
            {
                printf("%f %f\n",local.time(),localdata->estAng[2]);
            }*/
            else
                break;
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
double trans_q2( double theta )
{
    while( theta > M_PI )
        theta -= 2.0 * M_PI;
    while( theta < -M_PI )
        theta += 2.0 * M_PI;
    return theta;
}