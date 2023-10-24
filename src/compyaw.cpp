#include "calcgeo.hpp"
#include "creatfig.hpp"

static char a[256 / 2] = "imumag";
static char b[256 / 2] = "localizer";

static char path[STRLEN / 2] = "/home/shunsuke/2023_logdata/20230401/log_am5/2023.0401.0744";
static char log_name_imu[STRLEN / 2] = "imu.log";
static char log_name_gnss[STRLEN / 2] = "rtk_gnss.log";
static char log_name_localizer[STRLEN / 2] = "localizer.log";

static char xlabel0[STRLEN / 2] = "X[0]";
static char ylabel0[STRLEN / 2] = "Y[0]";

static char xlabel1[STRLEN / 2] = "X[1]";
static char ylabel1[STRLEN / 2] = "Y[1]";

static char fname0[STRLEN / 2] = "../data2/comyaw";
static char fname1[STRLEN / 2] = "../data2/xy1";

double trans_q2(double theta);
int main(int aArgc, char **aArgv)
{
    if (!setOption(aArgc, aArgv))
        return EXIT_FAILURE;

    PlotData Fig(1);

    SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer, localizer_property> local;

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
    imu_fs *imudata;

    imudata = &imu.data();
    localdata = &local.data();

    double k = 2.658937;
    double mag_offsetX = 26.239983;
    double mag_offsetY = 14.299279;
    double mag_offsetZ = 31.728186 ;

    try
    {
        setSigInt();
        int count = 1;
        while (!gShutOff)
        {
            // bool関数であるreadを使う
            Matrix<double, 3, 3> Rrp;
            Matrix<double, 3, 1> m;
            Matrix<double, 3, 1> M;

            if (local.read())
            {
                double tmp_mag[3];

                double Sr, Sp, Cr, Cp;
                Sr = sin(localdata->estAng[0]);
                Sp = sin(localdata->estAng[1]);
                Cr = cos(localdata->estAng[0]);
                Cp = cos(localdata->estAng[1]);

                // printf("%d %f %f\n", count, local.time(), localdata->estAng[2]);

                // count++;
                if (imu.readTime(local.time()))
                {
                    double xx = imudata->mag[0] - mag_offsetX;
                    double yy = imudata->mag[1] - mag_offsetY;
                    double zz = imudata->mag[2] - mag_offsetZ;
                    tmp_mag[0] = (Cp * xx) + (Sp * Sr * yy) + (Sp * Cr * zz);
                    tmp_mag[1] = (Cr * yy) - (Sr * zz);
                    tmp_mag[2] = -1.0 * (Sp * xx) + (Cp * Sr * yy) + (Cp * Cr * zz);

                    tmp_mag[0] = tmp_mag[0] * k;
                    tmp_mag[1] = tmp_mag[1] * k;
                    tmp_mag[2] = tmp_mag[2] * k;
                    double theta_mag = atan2(-1.0 * tmp_mag[1], tmp_mag[0]);
                    theta_mag = trans_q2(theta_mag + (M_PI_2));
                    Fig.SaveData2Dx2(local.time() - local.getStartTime(), theta_mag, local.time() - local.getStartTime(), localdata->estAng[2]);
                }
            }
            else
                break;
        }
        Fig.PrintFig2Dx2(a, b);
        Fig.SaveFigure(fname0);
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
double trans_q2(double theta)
{
    while (theta > M_PI)
        theta -= 2.0 * M_PI;
    while (theta < -M_PI)
        theta += 2.0 * M_PI;
    return theta;
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
