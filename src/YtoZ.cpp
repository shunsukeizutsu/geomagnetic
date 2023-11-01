/*******offsetZを１秒***/
#include "calcgeo.hpp"
#include "creatfig.hpp"

static char xlabel[256 / 2] = "count[qty]";

static char ylabel[256 / 2] = "offset [μT]";
static char name[256 / 2] = "../data/yawdata2";
static char a[256 / 2] = "offsetX";
static char b[256 / 2] = "offsetY";
static char c[256 / 2] = "offsetZ";
/*
static char ylabel[256 / 2] = "1/k";
static char name[256 / 2] = "../data/0331_kdata_pm1";
static char a[256 / 2] = "1/k0";
static char b[256 / 2] = "1/k1";
static char c[256 / 2] = "1/k2";
*/
static char path[STRLEN / 2] = "/home/haselab15/2023_logdata/20230401/log_am5/2023.0401.0744";
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
    PlotData Fig(8000.0,0.0,1000.0,-1000.0,1); // gnuplot

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

    //１号車
    double k = 1/0.434656;
    double offX = -9.99637;
    double offY = 9.69863;
    //５号車
//    double k = 1/0.499748;
//    double offX = 19.73009;
//    double offY = 22.57176;

    double time = 0.0;
    int count = 0;

    double sum = 0.0;

    try
    {
        setSigInt();
        while (!gShutOff)
        {
            if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {
                    count++;
                    time = rtk_gnss.time();
                    GG gmag;
                    gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);
                    double gnssdataX = gmag.GmagY / 1000;
                    double gnssdataY = gmag.GmagX / 1000;
                    double gnssdataZ = -gmag.GmagZ / 1000;

                    double Sr, Sp, Sy, Cr, Cp, Cy;

                    if (local.readTime(time))
                    {
                        Sr = sin(localdata->estAng[0]);
                        Sp = sin(localdata->estAng[1]);
                        Sy = sin(localdata->estAng[2]);
                        Cr = cos(localdata->estAng[0]);
                        Cp = cos(localdata->estAng[1]);
                        Cy = cos(localdata->estAng[2]);
                        //printf("%f %f %f %f %f %f\n", Sr,Sp,Sy,Cr,Cp,Cy);
                    }
                    if (imu.readTime(time))
                    {//回転行列１列目
                        double Mol;
                        double Deno;
                        Mol = (gnssdataY / k) - Sy * Cp * (imudata->mag[0] - offX) - (Cy * Cr + Sy * Sp * Sr) * (imudata->mag[1] - offY);
                        Deno = - Cy * Sr + Sp * Sy * Cr;
                        double offsetZ = imudata->mag[2] - (Mol / Deno);
                        Fig.SaveData2D(count, offsetZ);
                        sum += offsetZ;

                    }
                }
            }
            else
                break;
        }
        // std::cout << k << std::endl;
        // std::cout << sum / count << std::endl;
        Fig.PrintFig2D();
        while (!gShutOff)
        {
            usleep(10000);
        }
        
        //Fig.SaveFigure(name);
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