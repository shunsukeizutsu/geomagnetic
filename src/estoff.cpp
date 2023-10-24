/*******１秒に１つのデータを用いて最小二乗法を用いてkとoffsetを算出するプログラム*****/
#include "calcgeo.hpp"
#include "creatfig.hpp"

static char xlabel[256 / 2] = "count[qty]";

static char ylabel[256 / 2] = "offset [μT]";
static char name[256 / 2] = "../data/offset_0_1";
static char name2[256 / 2] = "../data/offset_2";
static char a[256 / 2] = "offsetX";
static char b[256 / 2] = "offsetY";
static char c[256 / 2] = "offsetZ";
/*
static char ylabel[256 / 2] = "1/k";
static char name[256 / 2] = "../data/kdata_0_1";
static char name2[256/2] = "../data/kdata_2";
static char a[256 / 2] = "1/k0";
static char b[256 / 2] = "1/k1";
static char c[256 / 2] = "1/k2";
*/
static char path[STRLEN / 2] = "/home/haselab15/2023_Snow/SnowRemoval_log/20230401/log_am5/2023.0401.0744";
static char log_name_imu[STRLEN / 2] = "imu.log";
static char log_name_gnss[STRLEN / 2] = "rtk_gnss.log";
static char log_name_localizer[STRLEN / 2] = "localizer.log";

static char xlabel0[STRLEN / 2] = "X[0]";
static char ylabel0[STRLEN / 2] = "Y[0]";

static char xlabel1[STRLEN / 2] = "X[1]";
static char ylabel1[STRLEN / 2] = "Y[1]";

static char fname0[STRLEN / 2] = "../data/xy0";
static char fname1[STRLEN / 2] = "../data/xy1";

int main(int aArgc, char **aArgv)
{
    if (!setOption(aArgc, aArgv))
        return EXIT_FAILURE;
    MagEquation MF;
    PlotData Fig(1); // gnuplot
    PlotData Fig2(1);

    SSMLog<gnss_gl, gnss_property> rtk_gnss;
    SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer, localizer_property> local;

    char LogNamegnss[STRLEN];
    sprintf(LogNamegnss, "%s/%s", path, log_name_gnss);
    if (!rtk_gnss.open(LogNamegnss))
    {
        fprintf(stderr, "Error! Cannot open gnss.log");
        exit(EXIT_FAILURE);
    }
    char LogNameimu[STRLEN];
    sprintf(LogNameimu, "%s/%s", path, log_name_imu);
    if (!imu.open(LogNameimu))
    {
        fprintf(stderr, "Error! Cannot open imu.log");
        exit(EXIT_FAILURE);
    }
    char LogNamelocal[STRLEN];
    sprintf(LogNamelocal, "%s/%s", path, log_name_localizer);

    if (!local.open(LogNamelocal))
    {
        fprintf(stderr, "Error! Caonnot open localizer.log");
        exit(EXIT_FAILURE);
    }
    printf("open %s logfiles\n\n", path);

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
        std::vector<data_sec> data; // １秒１つのデータをvectorに構造体で保存
        int count2 = 0;

        while (!gShutOff)
        {
            if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {                           // １秒に１つのデータを出力
                    count++;                // １秒１つのデータ数
                    data_sec tmp;           // ベクターに代入する際に必要となる一時保存場所
                    time = rtk_gnss.time(); // 基準となる時間

                    GG gmag; // 緯度経度から地磁気を算出ｰ>構造体でreturn
                    gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);

                    tmp.time = time;
                    tmp.gnssdata[0] = gmag.GmagY / 1000;
                    tmp.gnssdata[1] = gmag.GmagX / 1000;
                    tmp.gnssdata[2] = -gmag.GmagZ / 1000;
                    if (imu.readTime(rtk_gnss.time()))
                    {
                        tmp.imumag[0] = imudata->mag[0];
                        tmp.imumag[1] = imudata->mag[1];
                        tmp.imumag[2] = imudata->mag[2];
                    }
                    if (local.readTime(rtk_gnss.time()))
                    {
                        tmp.localizer[0] = localdata->estAng[0];
                        tmp.localizer[1] = localdata->estAng[1];
                        tmp.localizer[2] = localdata->estAng[2];
                    }
                    data.push_back(tmp); // dataというvectorに保存

                    //*******count=60のとき（一分間）60個のデータが貯まると最小二乗法で計算をする
                    //*******この場合は蓄積されていく（60,120,180,,,,,）
                    if (count == 60)
                    {
                        count = 0; // 何個のデータを使用するか？
                        count2++;  // 1分間60個のデータ=１、２分間＝２
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
                            /*********回転行列の逆行列********/
                            Matrix<double, 3, 3> RT;
                            double Sr, Sp, Sy, Cr, Cp, Cy;
                            Sr = sin(data[i].localizer[0]); // sin(roll)
                            Sp = sin(data[i].localizer[1]); // sin(pitch)
                            Sy = sin(data[i].localizer[2]); // sin(yaw)
                            Cr = cos(data[i].localizer[0]); // cos(roll)
                            Cp = cos(data[i].localizer[1]); // cos(pitch)
                            Cy = cos(data[i].localizer[2]); // cos(yaw)
                            RT(0, 0) = Cy * Cp;
                            RT(0, 1) = Sy * Cp;
                            RT(0, 2) = -Sp;
                            RT(1, 0) = -Sy * Cr + Cy * Sp * Sr;
                            RT(1, 1) = Cy * Cr + Sp * Sy * Sr;
                            RT(1, 2) = Cp * Sr;
                            RT(2, 0) = Sr * Sy + Cy * Sp * Cr;
                            RT(2, 1) = -Cy * Sr + Sp * Sy * Cr;
                            RT(2, 2) = Cp * Cr;

                            /**********回転行列とgnssから求めた地磁気の積******/
                            Matrix<double, 3, 1> M;
                            M(0, 0) = data[i].gnssdata[0];
                            M(1, 0) = data[i].gnssdata[1];
                            M(2, 0) = data[i].gnssdata[2];

                            Matrix<double, 3, 1> X;
                            X = RT * M;

                            Xsum[0] += X(0, 0);
                            Xsum[1] += X(1, 0);
                            Xsum[2] += X(2, 0);

                            X2sum[0] += X(0, 0) * X(0, 0);
                            X2sum[1] += X(1, 0) * X(1, 0);
                            X2sum[2] += X(2, 0) * X(2, 0);

                            /********imuからの生地磁気データ*******/

                            Ysum[0] += data[i].imumag[0];
                            Ysum[1] += data[i].imumag[1];
                            Ysum[2] += data[i].imumag[2];

                            XYsum[0] += X(0, 0) * data[i].imumag[0];
                            XYsum[1] += X(1, 0) * data[i].imumag[1];
                            XYsum[2] += X(2, 0) * data[i].imumag[2];
                        }

                        /*aveX[0] = Xsum[0] / data.size();
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
                        aveXY[2] = XYsum[2] / data.size();*/
                        double a[3]; // 1/kの値
                        double b[3]; // offsetの値
                        for (int i = 0; i < 3; i++)
                        {
                            aveX[i] = Xsum[i] / data.size();
                            aveX2[i] = X2sum[i] / data.size();
                            aveY[i] = Ysum[i] / data.size();
                            aveXY[i] = XYsum[i] / data.size();

                            double Molecular;   // 分子
                            double Denominator; // 分母
                            Molecular = aveXY[i] - aveX[i] * aveY[i];
                            Denominator = aveX2[i] - aveX[i] * aveX[i];
                            a[i] = Molecular / Denominator;   // 1/k
                            b[i] = -a[i] * aveX[i] + aveY[i]; // offset
                        }
                        // printdata(count2,b[0], b[1], b[2]);

                        Fig.SaveData2Dx2(count2, b[0], count2, b[1]);
                        Fig2.SaveData2D(count2, b[2]);
                    }
                }
            }
            else
                break;
        }
        // Fig.SeTics(10.0, 10.0);
        //  Fig.XYlabel(xlabel, ylabel);
        Fig.PrintFig2Dx2(a, b);
        Fig2.PrintFig2D();
        // Fig.SaveFigure(name);
        // Fig2.SaveFigure(name2);
        while (!gShutOff)
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
void printdata(int count, double a, double b, double c)
{

    printf("%d %f %f %f\n", count, a, b, c);
}
double Caloffsetz(data_sec data, double k, double offX, double offY)
{
    double Sr, Sp, Cr, Cp;
    Sr = sin(data.localizer[0]); // sin(roll)
    Sp = sin(data.localizer[1]); // sin(pitch)

    Cr = cos(data.localizer[0]); // cos(roll)
    Cp = cos(data.localizer[1]); // cos(pitch)

    double Mol;  // 分子
    double Deno; // 分母
    Mol = (data.gnssdata[2] / k) + Sp * (data.imumag[0] - offX) - Cp * Sr * (data.imumag[1] - offY);
    Deno = Cp * Cr;
    double offsetZ = data.imumag[2] - Mol / Deno;
    return offsetZ;
}
static bool setOption(int aArgc, char *aArgv[])
{
    int opt, optIndex = 0;
    struct option longOpt[] = {
        {"log_path", 1, 0, 'p'},
        {"help", 0, 0, 'h'},
        {0, 0, 0, 0}};

    while ((opt = getopt_long(aArgc, aArgv, "p:h", longOpt, &optIndex)) != -1)
    {
        switch (opt)
        {
        case 'p':
            sprintf(path, "%s", optarg);
            break;
        case 'h':
            printShortHelp(aArgv[0]);
            return false;
            break;
        default:
            fprintf(stderr, "help : %s -h\n", aArgv[0]);
            return false;
            break;
        }
    }
    return true;
}
static int printShortHelp(const char *programName)
{
    fputs("HELP\n", stderr);
    fprintf(stderr, "\t$ %s -p %s\n", programName, path);
    fputs("OPTION\n", stderr);
    printf("\t-p | --path     PATH     : Set path for log file (default=%s)\n", path);
    return EXIT_SUCCESS;
}