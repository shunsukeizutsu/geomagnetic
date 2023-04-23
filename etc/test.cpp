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

using namespace std;

static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);

int main(void)
{
    //SSMLog<データ構造体名,プロパティ構造体名> imu
    //"imu"の場所はなんでも良い
    //データ名とプロパティ名はheaderファイル参照
    SSMLog<imu_fs,imu_property> imu;
 
    if (!imu.open("/home/shunsuke/SnowRemoval_log/20230402/log_am6/2023.0402.0749/imu.log"))
    {
        //logfileを開けるか否か
        fprintf(stderr, "Error! Cannot open logfile");
        exit(EXIT_FAILURE);
    }
    //データの構造体を指すポインタを生成
    imu_fs *imudata;
    //データが保存されているアドレスをポインタに代入
    imudata=&imu.data();

    try
    {
        setSigInt();
        while(!gShutOff)
        {
            //bool関数であるreadを使う

            if(imu.read())
            {
                printf("%f %f\n",imu.time() - imu.getStartTime(),imudata->temperature);
            }
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