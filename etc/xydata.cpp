/*******最小二乗法をする際のXとYをプロットしたもの*******/
//X = 回転行列Rの転置*緯度経度から求めた地磁気　Y = 生の地磁気センサデータ
///１秒間１つのデータ、すべてのデータを使用
#include "calcgeo.hpp"
#include "creatfig.hpp"
#include "dssm.hpp"

static char path[STRLEN / 2] = "/home/shunsuke/2023_logdata/20230401/log_am5/2023.0401.0744";
static char log_name_imu[STRLEN / 2] = "imu.log";
static char log_name_gnss[STRLEN / 2] = "rtk_gnss.log";
static char log_name_localizer[STRLEN / 2] = "localizer.log";

static char xlabel0[STRLEN / 2] = "X[0]";
static char ylabel0[STRLEN / 2] = "Y[0]";

static char xlabel1[STRLEN / 2] = "X[1]";
static char ylabel1[STRLEN / 2] = "Y[1]";

static char fname0[STRLEN / 2] = "../data/xy0_all3";
static char fname1[STRLEN / 2] = "../data/xy1_all3";

int main(int aArgc, char **aArgv)
{
    if (!setOption(aArgc, aArgv))
        return EXIT_FAILURE;

    MagEquation MF;
    PlotData Fig(0); // gnuplot
    PlotData Fig2(0);
    PlotData Fig3(0);

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
    gnss_gl *gnssdata;
    imu_fs *imudata;
    localizer *localdata;

    gnssdata = &rtk_gnss.data();
    imudata = &imu.data();
    localdata = &local.data();

    std::vector<data_sec> data;
    try
    {
        setSigInt();
        double time = 0.0;
        int count2 = 0;

        while (!gShutOff)
        {
            // １秒1つのデータを取り出し、vectorに保存
            if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {
                    data_sec tmp;
                    time = rtk_gnss.time();

                    GG gmag;
                    gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);

                    tmp.time = time;
                    tmp.gnssdata[0] = gmag.GmagY / 1000;
                    tmp.gnssdata[1] = gmag.GmagX / 1000;
                    tmp.gnssdata[2] = -gmag.GmagZ / 1000;

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
                }
            }
            else
                break;
        }
        for (int i = data.size()*2/10; i < data.size(); i++)
        {
            CreatMatrix MDD;
            MDD = CaluculateMatrix(data[i]);
            Fig.SaveData2D(MDD.X(0, 0), MDD.m(0, 0));
            Fig2.SaveData2D(MDD.X(1, 0), MDD.m(1, 0));
            Fig3.SaveData2D(MDD.X(2,0),MDD.m(2,0));
        }
        Fig.SeTics(5.0, 5.0);
        Fig2.SeTics(5.0, 5.0);
        Fig3.SeTics(5.0,5.0);
        Fig.XYlabel(xlabel0, ylabel0);
        Fig2.XYlabel(xlabel1, ylabel1);

        Fig.PrintFig2D();
        Fig2.PrintFig2D();
        Fig3.PrintFig2D();
       // Fig.SaveFigure(fname0);
       // Fig2.SaveFigure(fname1);
        while (!gShutOff)
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
void printdata(data_sec data)
{
    printf("%f %f %f %f %f %f %f %f %f %f \n", data.time, data.gnssdata[0], data.gnssdata[1], data.gnssdata[2], data.imumag[0], data.imumag[1], data.imumag[2], data.localizer[0], data.localizer[1], data.localizer[2]);
}
CreatMatrix CaluculateMatrix(data_sec data)
{
    CreatMatrix CM;
    double Sr, Sp, Sy, Cr, Cp, Cy;
    Sr = sin(data.localizer[0]); // sin(roll)
    Sp = sin(data.localizer[1]); // sin(pitch)
    Sy = sin(data.localizer[2]); // sin(yaw)
    Cr = cos(data.localizer[0]); // cos(roll)
    Cp = cos(data.localizer[1]); // cos(pitch)
    Cy = cos(data.localizer[2]); // cos(yaw)
    CM.RT(0, 0) = Cy * Cp;
    CM.RT(0, 1) = Sy * Cp;
    CM.RT(0, 2) = -Sp;
    CM.RT(1, 0) = -Sy * Cr + Cy * Sp * Sr;
    CM.RT(1, 1) = Cy * Cr + Sp * Sy * Sr;
    CM.RT(1, 2) = Cp * Sr;
    CM.RT(2, 0) = Sr * Sy + Cy * Sp * Cr;
    CM.RT(2, 1) = -Cy * Sr + Sp * Sy * Cr;
    CM.RT(2, 2) = Cp * Cr;
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