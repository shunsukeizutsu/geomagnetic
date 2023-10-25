/**datファイルからデータを読み込み平均を出力*****/
/*file内の構造（count data1 data2 data3）*/
/*スペース区切り*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <stdexcept>
#include <unistd.h>
#include <string.h>
#include <vector>

#define DATPATH "data.dat";

typedef struct
{
    double count;
    double data[3];
} Data;

using namespace std;
static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);

int main(void)
{
    try
    {
        setSigInt();
        FILE *fp;
        vector<Data> alldata;

        char fname[] = "../data/0331.0803_3/a2data.dat";
        fp = fopen(fname, "r");

        if (fp == NULL)
        {
            printf("do not open %s error\n", fname);
            exit(EXIT_FAILURE);
        }
        double data1, data2, data3;
        int count;
        while (fscanf(fp, "%d %lf %lf %lf", &count, &data1, &data2, &data3) != EOF)
        {
            Data tmp;
            tmp.count = count;
            tmp.data[0] = data1;
            tmp.data[1] = data2;
            tmp.data[2] = data3;

            alldata.push_back(tmp);
        }
        fclose(fp);

        double sum[3] = {0};
        double ave[3] = {0};
        for (int i = 0; i < alldata.size(); i++)
        {
            sum[0] += alldata[i].data[0];
            sum[1] += alldata[i].data[1];
            sum[2] += alldata[i].data[2];
        }
        ave[0]= sum[0]/alldata.size();
        ave[1]= sum[1]/alldata.size();
        ave[2]= sum[2]/alldata.size();
        printf("%f %f %f\n",ave[0],ave[1],ave[2]);
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
