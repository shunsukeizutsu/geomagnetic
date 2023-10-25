/********100個のデータを用いてoffsetとkを算出*************/
#include "calcgeo.hpp"
#include "creatfig.hpp"
#include "dssm.hpp"

static char path[STRLEN / 2] = "/home/haselab15/2023_Snow/SnowRemoval_log/20230401/log_am5/2023.0401.0744";
static char log_name_imu[STRLEN / 2] = "imu.log";
static char log_name_gnss[STRLEN / 2] = "rtk_gnss.log";
static char log_name_localizer[STRLEN / 2] = "localizer.log";

static char xlabel0[STRLEN / 2] = "X[0]";
static char ylabel0[STRLEN / 2] = "Y[0]";

static char xlabel1[STRLEN / 2] = "X[1]";
static char ylabel1[STRLEN / 2] = "Y[1]";

static char fname0[STRLEN / 2] = "../data2/xy0";
static char fname1[STRLEN / 2] = "../data2/xy1";

int main(int aArgc, char **aArgv)
{
    if (!setOption(aArgc, aArgv))
        return EXIT_FAILURE;

    PlotData Fig(0);
    PlotData Fig2(0);
    PlotData Fig3(1);
    MagEquation MF;
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
    localizer *localdata;
    gnss_gl *gnssdata;
    imu_fs *imudata;
    gnssdata = &rtk_gnss.data();
    imudata = &imu.data();
    localdata = &local.data();

    try
    {
        setSigInt();

        int count[36] = {0};

        std::vector<data_sec> datas;
        while (!gShutOff)
        {
            if (local.read())
            {
                if (local.time() - local.getStartTime() >= 1800)
                {
                    double time = local.time();
                    double yawdata = localdata->estAng[2];

                    for (int i = 0; i < 18; i++)
                    {

                        if (M_PI * i / 18 < yawdata && yawdata <= M_PI * (i + 1) / 18 && count[i] < 100)
                        {
                            data_sec tmp;
                            tmp.time = time;
                            if (imu.readTime(time))
                            {
                                tmp.imumag[0] = imudata->mag[0];
                                tmp.imumag[1] = imudata->mag[1];
                                tmp.imumag[2] = imudata->mag[2];
                            }

                            tmp.localizer[0] = localdata->estAng[0];
                            tmp.localizer[1] = localdata->estAng[1];
                            tmp.localizer[2] = localdata->estAng[2];

                            if (rtk_gnss.readTime(time))
                            {
                                GG gmag;
                                gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);
                                tmp.gnssdata[0] = gmag.GmagY / 1000;
                                tmp.gnssdata[1] = gmag.GmagX / 1000;
                                tmp.gnssdata[2] = -gmag.GmagZ / 1000;
                            }
                            datas.push_back(tmp);
                            count[i]++;
                            // printf("%d %f %f %f \n",sumcount,time,tmp.gnssdata[0],tmp.gnssdata[1]);
                            break;
                        }
                        else if (yawdata < -M_PI * i / 18 && yawdata >= -M_PI * (i + 1) / 18 && count[i + 18] < 100)
                        {
                            count[i + 18]++;
                            data_sec tmp;
                            tmp.time = time;

                            // printf("%f\n", timedata[i]);
                            if (imu.readTime(time))
                            {
                                tmp.imumag[0] = imudata->mag[0];
                                tmp.imumag[1] = imudata->mag[1];
                                tmp.imumag[2] = imudata->mag[2];
                            }

                            tmp.localizer[0] = localdata->estAng[0];
                            tmp.localizer[1] = localdata->estAng[1];
                            tmp.localizer[2] = localdata->estAng[2];

                            if (rtk_gnss.readTime(time))
                            {
                                GG gmag;
                                gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);
                                tmp.gnssdata[0] = gmag.GmagY / 1000;
                                tmp.gnssdata[1] = gmag.GmagX / 1000;
                                tmp.gnssdata[2] = -gmag.GmagZ / 1000;
                            }
                            datas.push_back(tmp);
                            break;
                        }
                    }
                }
            }
            else
                break;
        }
        LQsolution AandB;
        AandB = LeastSquares(datas); // 最小二乗法を計算
        printf("a[0]:%f a[1]:%f b[0]:%f b[1]:%f\n", AandB.a[0], AandB.a[1], AandB.b[0], AandB.b[1]);
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
void printdata(double a, double b, double c)
{

    printf("%f %f %f\n", a, b, c);
}
CreatMatrix CaluculateMatrix(data_sec data)
{
    CreatMatrix CM;
    double Sr, Sp, Sy, Cr, Cp, Cy;
    double r11,
        r12, r13, r21, r22, r23, r31, r32, r33;
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
static LQsolution LeastSquares(std::vector<data_sec> &data) // 最小二乗法を計算する関数　引数：構造体data_secのvectorデータ　返り値：最小二乗法の解aとbの構造体
{
    /**
     * グラフ化
     * */
    /*
    PlotData Fig(0);
    PlotData Fig2(0);
    */
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
        //X = RT*M Y = m
        /*
        Fig.SaveData2D(MDD.X(0, 0), MDD.m(0, 0));
        Fig2.SaveData2D(MDD.X(1, 0), MDD.m(1, 0));
        */
    }

    aveX[0] = Xsum[0] / data.size();
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
        AandB.a[i] = Molecular / Denominator;         // 1/k
        AandB.b[i] = -AandB.a[i] * aveX[i] + aveY[i]; // offset
    }
    k = (AandB.a[0] + AandB.a[1]) / 2;
    k = 1 / k;
    //X = RT*M Y = m
    /*
    Fig.SeTics(5.0, 5.0);
    Fig2.SeTics(5.0, 5.0);
    Fig.XYlabel(xlabel0, ylabel0);
    Fig2.XYlabel(xlabel1, ylabel1);

    Fig.PrintFig2D();
    Fig2.PrintFig2D();

    while (!gShutOff)
    {
        usleep(1000);
    }
    */
    return AandB;
}
