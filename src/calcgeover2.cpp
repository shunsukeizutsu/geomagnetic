#include "calcgeo.hpp"

// gnuplo範囲
#define XLANGEMIN 0.0
//#define XLANGEMAX 224.0 // pc1のデータの数
#define XLANGEMAX 130.0
#define YLANGEMIN -2.0
#define YLANGEMAX 2.0

#define GNUPLOTVER 1

double num[200000][4];

int main(void)
{
#ifdef GNUPLOTVER
    FILE *gnuplot;                   // gnuplot
    gnuplot = popen("gnuplot", "w"); // gnuplot
#endif

    MagEquation MF;
    // SSMLog<rtk_gnss_f9p> rtk_gnss;
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

        int cnt = 0;
        /************************************************************/
        while (!gShutOff)
        {
            // １秒1つのデータを取り出し、vectorに保存
            // double time = rtk_gnss.time();
            // printf("%f\n",time);
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
                    if (count == 120)
                    {
                        count = 0;
                        count2++;
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
                            // std::cout << aveX[i] << " ";
                            // std::cout << aveX2[i] << " ";
                            // std::cout << aveY[i] << " ";
                            double Denominator; // 分母
                            double Molecular;   // 分子
                            Molecular = aveXY[i] - aveX[i] * aveY[i];
                            Denominator = aveX2[i] - aveX[i] * aveX[i];
                            a[i] = Molecular / Denominator;
                            b[i] = -a[i] * aveX[i] + aveY[i];
                        }
#ifdef GNUPLOTVER
#else
                        // printf("%d %f %f %f\n", count2, b[0], b[1], b[2]);
#endif
                        num[count2][0] = a[0];
                        num[count2][1] = a[1];
                        num[count2][2] = a[2];
                        // std::cout << count2 << std::endl;
                        data.clear();
                    }
                }
            }
            else
                break;
        }

#ifdef GNUPLOTVER
        double avex[3]={0};
        for (int k = 0; k < count2; k++)
        {
            fprintf(gnuplot, "set grid\n\n");
            fprintf(gnuplot, "set size ratio -1\n\n");
            fprintf(gnuplot, "set xlabel 'count [pcs]'\n\n");
            fprintf(gnuplot, "set ylabel 'k'\n\n");
            fprintf(gnuplot, "set key outside right\n\n");
            fprintf(gnuplot, "set size square \n\n");
            fprintf(gnuplot, "set xlabel font 'Arial,22'\n\n");
            fprintf(gnuplot, "set ylabel font 'Arial,22'\n\n");
            fprintf(gnuplot, "set xlabel offset 0,-1\n\n");
            fprintf(gnuplot, "set ylabel offset -4,0\n\n");
            fprintf(gnuplot, "set tics font 'Arial,15'\n\n");
            fprintf(gnuplot, "set grid linewidth 1.5\n\n");
            // fprintf(gnuplot, "set label '平均=%lf\n分散=%lf\n標準偏差=%lf'\n\n", ave, bunsan, SD);
            //  fprintf(gnuplot, "set grid linecolor 'dark-red'\n\n");
            fprintf(gnuplot, "set xrange[%f:%f]\nset yrange[%f:%f]\n\n", XLANGEMIN, XLANGEMAX, YLANGEMIN, YLANGEMAX); // 範囲
            fprintf(gnuplot, "p ");
            fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'red' t \"k_X\", ");  // 文法的な感じ  odm 2
            fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'blue' t \"k_Y\", "); // 文法的な感じ  odm 2
            fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'green' t \"k_Z\" "); // 文法的な感じ  odm 2
            fprintf(gnuplot, "\n");
            fprintf(gnuplot, "%d, %lf\n", k, num[k][0]); //
            avex[0] += num[k][0];
            avex[1] += num[k][1];
            avex[2] += num[k][2];
            // fprintf(gnuplot, "%d, %lf\n", k, num[k][1]); //
            //  fflush(gnuplot);                                    // 一つの文法の定義にひとつ必要 1
        }
        fprintf(gnuplot, "e\n");
        for (int k = 0; k < count2; k++)
        {
            fprintf(gnuplot, "%d, %lf\n", k, num[k][1]); //
        }
        fprintf(gnuplot, "e\n");
        for (int k = 0; k < count2; k++)
        {
            fprintf(gnuplot, "%d, %lf\n", k, num[k][2]); //
        }
        fprintf(gnuplot, "e\n");
        fflush(gnuplot); // 必須
        std::cout << avex[0]/count2 << " " << avex[1]/count2 << " " << avex[2]/count2 << std::endl;

        while (!gShutOff)
        {
            usleep(1000);
        }
        pclose(gnuplot);
#endif
        //**************************************************************************//
        // xの初期値
        // double delta_x = 10;
        /*while (abs(delta_x) > 1)
        {
            Matrix<double, 4, 1> X;
            Matrix<double, 4, 4> JTJ;
            Matrix<double, 4, 1> JTb;

            for (int i = 0; i < data.size(); i++)
            {
                CreatMatrix MDD;
                MDD = CaluculateMatrix(data[i], k, ax, ay, az);
                JTb += MDD.JT * MDD.b;
                JTJ += MDD.JT * MDD.J;
                printdata(data[i]);
            }
            Eigen::PartialPivLU<Eigen::MatrixXd> lu(JTJ);
            X = lu.solve(JTb);
            // std::cout << X << std::endl;
            std::cout << "\n " << std::endl;
            delta_x = X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0) + X(3, 0) * X(3, 0);
            k = X(0, 0);
            ax = X(1, 0) + ax;
            ay = X(2, 0) + ay;
            az = X(3, 0) + az;
            delta_x = sqrt(delta_x);
            // std::cout << delta_x << std::endl;
        }
        */
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

    // Jの行列
    /*    CM.J(0, 0) = r11 * (data.imumag[0] - ax) + r12 * (data.imumag[1] - ay) + r13 * (data.imumag[2] - az);
        CM.J(0, 1) = -k * r11;
        CM.J(0, 2) = -k * r12;
        CM.J(0, 3) = -k * r13;
        CM.J(1, 0) = r21 * (data.imumag[0] - ax) + r22 * (data.imumag[1] - ay) + r23 * (data.imumag[2] - az);
        CM.J(1, 1) = -k * r21;
        CM.J(1, 2) = -k * r22;
        CM.J(1, 3) = -k * r23;
        CM.J(2, 0) = r31 * (data.imumag[0] - ax) + r32 * (data.imumag[1] - ay) + r33 * (data.imumag[2] - az);
        CM.J(2, 1) = -k * r31;
        CM.J(2, 2) = -k * r32;
        CM.J(2, 3) = -k * r33;

        // Jの転置行列
        CM.JT = CM.J.transpose();
    */
    // gnss->magdata
    CM.M(0, 0) = data.gnssdata[0];
    CM.M(1, 0) = data.gnssdata[1];
    CM.M(2, 0) = data.gnssdata[2];

    // CM.F = k * CM.R * CM.m;
    // CM.b = CM.M - CM.F;
    CM.X = CM.RT * CM.M;
    // std::cout << CM.X << std::endl << std::endl;

    return CM;
}