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
static char path[STRLEN / 2] = "/home/shunsuke/2023_logdata/20230401/log_am5/2023.0401.0744";
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
    
    SSMLog<localizer, localizer_property> local;

    char LogNamelocal[STRLEN];
    sprintf(LogNamelocal, "%s/%s", path, log_name_localizer);

    if (!local.open(LogNamelocal))
    {
        fprintf(stderr, "Error! Caonnot open localizer.log");
        exit(EXIT_FAILURE);
    }
    printf("open %s logfiles\n\n", path);

    localizer *localdata;
    localdata = &local.data();

    try
    {
        setSigInt();
        double time = 0.0;
        int count = 0;
        int count2 = 0;
        PlotData Fig(1);
        while (!gShutOff)
        {
            if (local.read())
            {
               // printf("%f %f\n",local.time(),localdata->estAng[2]*180/M_PI);
                Fig.SaveData2D(local.time(),localdata->estAng[2]);
            }
            else
                break;
        }
        Fig.PrintFig2D();
        while(!gShutOff)
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