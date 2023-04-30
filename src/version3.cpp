#include "calcgeo.hpp"
#include "creatfig.hpp"

static char xlabel[256 / 2] = "count";
static char ylabel[256 / 2] = "offset[μT]";
static char a[256 / 2] = "offset1";
static char b[256 / 2] = "offset2";
static char c[256 / 2] = "offset3";

int main(void)
{
    MagEquation MF;
    PlotData Fig(250.0 , 0.0 ,50.0 ,-50.0,1); // gnuplot

    SSMLog<gnss_gl, gnss_property> rtk_gnss;
    SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer, localizer_property> local;
    if (!rtk_gnss.open("../2023.0402.0748/rtk_gnss.log"))
    {
        fprintf(stderr, "Error! Cannot open gnss.log");
        exit(EXIT_FAILURE);
    }
    if (!imu.open("../2023.0402.0748/imu.log"))
    {
        fprintf(stderr, "Error! Cannot open imu.log");
        exit(EXIT_FAILURE);
    }
    if (!local.open("../2023.0402.0748/localizer.log"))
    {
        fprintf(stderr, "Error! Caonnot open localizer.log");
        exit(EXIT_FAILURE);
    }

    gnss_gl *gnssdata;
    imu_fs *imudata;
    localizer *localdata;

    gnssdata = &rtk_gnss.data();
    imudata = &imu.data();
    localdata = &local.data();

    try
    {
        setSigInt();
        double time = 0.0;
        int count = 0;
        std::vector<data_sec> data;
        int count2 = 0;

        while (!gShutOff)
        {
            // １秒1つのデータを取り出し、vectorに保存
            if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {
                    count++;
                    data_sec tmp;
                    time = rtk_gnss.time();
                    tmp.time = time;
                    GG gmag;
                    gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);
                    tmp.gnssdata[0] = gmag.GmagY / 1000;
                    tmp.gnssdata[1] = gmag.GmagX / 1000;
                    tmp.gnssdata[2] = -gmag.GmagZ / 1000;
                    // printf("%f\n", time);
                    if (imu.readTime(time))
                    {
                        tmp.imumag[0] = imudata->mag[0];
                        tmp.imumag[1] = imudata->mag[1];
                        tmp.imumag[2] = imudata->mag[2];
                    }
                    if (local.readTime(time))
                    {
                        tmp.localizer[0] = localdata->estAng[0];
                        tmp.localizer[1] = localdata->estAng[1];
                        tmp.localizer[2] = localdata->estAng[2];
                    }
                    data.push_back(tmp);
                    // std::cout << count << std::endl;
                    if (count == 60)
                    {
                        count = 0; // 何個のデータを使用するか？
                        count2++;  // 1分間60個を平均したデータが何個あるか
                        double Xsum[3] = {0};
                        double X2sum[3] = {0};
                        double aveX[3] = {0};
                        double aveX2[3] = {0};
                        double Ysum[3] = {0};
                        double aveY[3] = {0};
                        double XYsum[3] = {0};
                        double aveXY[3] = {0};
                        for (int i = 0; i < data.size(); i++)
                        {
                            // printdata(data[i]);
                            CreatMatrix MDD;
                            MDD = CaluculateMatrix(data[i]);
                            Xsum[0] += MDD.X(0, 0);
                            Xsum[1] += MDD.X(1, 0);
                            Xsum[2] += MDD.X(2, 0);
                            // std::cout << Xsum[0] << " " << Xsum[1] << " " << Xsum[2] << std::endl;
                            X2sum[0] += MDD.X(0, 0) * MDD.X(0, 0);
                            X2sum[1] += MDD.X(1, 0) * MDD.X(1, 0);
                            X2sum[2] += MDD.X(2, 0) * MDD.X(2, 0);

                            Ysum[0] += MDD.m(0, 0);
                            Ysum[1] += MDD.m(1, 0);
                            Ysum[2] += MDD.m(2, 0);

                            XYsum[0] += MDD.X(0, 0) * MDD.m(0, 0);
                            XYsum[1] += MDD.X(1, 0) * MDD.m(1, 0);
                            XYsum[2] += MDD.X(2, 0) * MDD.m(2, 0);
                        }
                        // std::cout << Xsum[0] << " " << Xsum[1] << " " << Xsum[2] << std::endl;
                        aveX[0] = Xsum[0] / data.size();
                        aveX[1] = Xsum[1] / data.size();
                        aveX[2] = Xsum[2] / data.size();

                        aveX2[0] = X2sum[0] / data.size();
                        aveX2[1] = X2sum[1] / data.size();
                        aveX2[2] = X2sum[2] / data.size();

                        aveY[0] = Ysum[0] / data.size();
                        aveY[1] = Ysum[1] / data.size();
                        aveY[2] = Ysum[2] / data.size();

                        aveXY[0] = XYsum[0] / data.size();
                        aveXY[1] = XYsum[1] / data.size();
                        aveXY[2] = XYsum[2] / data.size();
                        // std::cout << data.size() << " " << data.capacity() << std::endl;
                        double a[3];
                        double b[3];
                        for (int i = 0; i < 3; i++)
                        {
                            double Denominator; // 分母
                            double Molecular;   // 分子
                            Molecular = aveXY[i] - aveX[i] * aveY[i];
                            Denominator = aveX2[i] - aveX[i] * aveX[i];
                            a[i] = Molecular / Denominator;
                            b[i] = -a[i] * aveX[i] + aveY[i];
                        }

                        // std::cout << count2 << std::endl;
                        //printdata(a[0], a[1], a[2]);
                        Fig.SaveData2Dx3(count2,a[0], count2,a[1],count2, a[2]);
                        //data.clear();
                        //data.shrink_to_fit();
                        //std::cout << data.capacity() << std::endl;
                    }
                }
            }
            else
                break;
        }

        Fig.XYlabel(xlabel ,ylabel);
        Fig.PrintFig2Dx3(a,b,c);
        while(!gShutOff)
        {
            usleep(1000);
        }
    }
    // 平均が出ているかの確認test用
    catch (std::runtime_error const &error)
    {
        std::cout << error.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    }
    // Terminate();
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
    printf("%f %f %f %f %f %f %f %f %f %f \n", data.time, data.gnssdata[0], data.gnssdata[1], data.gnssdata[2], data.imumag[0], data.imumag[1], data.imumag[2], data.localizer[0], data.localizer[1], data.localizer[2]);
}
CreatMatrix CaluculateMatrix(data_sec data)
{
    CreatMatrix CM;
    double Sr, Sp, Sy, Cr, Cp, Cy;
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    Sr = sin(data.localizer[0]); // sin(roll)
    Sp = sin(data.localizer[1]); // sin(pitch)
    Sy = sin(data.localizer[2]); // sin(yaw)
    Cr = cos(data.localizer[0]); // cos(roll)
    Cp = cos(data.localizer[1]); // cos(pitch)
    Cy = cos(data.localizer[2]); // cos(yaw)
    r11 = Cy * Cp;
    r12 = -Sy * Cr + Cy * Sp * Sr;
    r13 = Sr * Sy + Cy * Sp * Cr;
    r21 = Sy * Cp;
    r22 = Cy * Cr + Sr * Sp * Sy;
    r23 = -Cy * Sr + Sp * Sy * Cr;
    r31 = -Sp;
    r32 = Cp * Sr;
    r33 = Cp * Cr;
    // 回転行列
    CM.R(0, 0) = r11;
    CM.R(0, 1) = r12;
    CM.R(0, 2) = r13;
    CM.R(1, 0) = r21;
    CM.R(1, 1) = r22;
    CM.R(1, 2) = r23;
    CM.R(2, 0) = r31;
    CM.R(2, 1) = r32;
    CM.R(2, 2) = r33;
    // 回転行列Rの転置行列
    CM.RT = CM.R.transpose();
    // 生の地磁気データ
    CM.m(0, 0) = data.imumag[0];
    CM.m(1, 0) = data.imumag[1];
    CM.m(2, 0) = data.imumag[2];

    // gnss->magdata
    CM.M(0, 0) = data.gnssdata[0];
    CM.M(1, 0) = data.gnssdata[1];
    CM.M(2, 0) = data.gnssdata[2];

    CM.X = CM.RT * CM.M;

    return CM;
}
void printdata(double a, double b, double c)
{

    printf("%f %f %f\n", a, b, c);
}