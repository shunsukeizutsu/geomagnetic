////*****ファイルからデータを読み取り平均と分散、標準偏差を出力****////
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

using namespace std;
static int gShutOff = 0;
static void setSigInt(void);
static void Terminate(void);
static void Calustatic(std::vector<double> &adata);

int main(void)
{
    try
    {
        setSigInt();
        FILE *fp;
        vector<double> adata;

        char fname[] = "../data3/offsetz_5.dat";
        fp = fopen(fname, "r");

        if (fp == NULL)
        {
            printf("do not open %s error\n", fname);
            exit(EXIT_FAILURE);
        }
        double data;
        while (fscanf(fp, "%lf", &data) != EOF)
        {
            // std::cout << data << std::endl;
            adata.push_back(data);
        }
        fclose(fp);

        Calustatic(adata);
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
static void Calustatic(std::vector<double> &adata)
{
    double sum = 0;
    double ave;
    for (int i = 0; i < adata.size(); i++)
    {
        sum += adata[i];
    }
    ave = sum / adata.size();
    std::cout << "平均：" << ave << std::endl;
    double sum2 = 0;
    for (int i = 0; i < adata.size(); i++)
    {
        sum2 += (adata[i] - ave) * (adata[i] - ave);
        // std::cout << adata[i]-ave << std::endl;;
    }
    double ave2 = sum2 / adata.size();
    std::cout << "分散：" << ave2 << std::endl;
    std::cout << "標準：" << sqrt(ave2) << std::endl;
}