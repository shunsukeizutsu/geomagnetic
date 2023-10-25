/*傾きがある値以下になると、そこの数値を出力するプログラム*/
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
    double data; 
}Data;

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

        char fname[]="../data/a2data.dat";
        fp = fopen(fname,"r");

        if (fp == NULL)
        {
            printf("do not open %s error\n",fname);
            exit(EXIT_FAILURE);
        }
        double data;
        int count;
        while (fscanf(fp, "%d %lf", &count,&data) != EOF)
        {
            Data tmp;
            tmp.count = count;
            tmp.data = data;
            alldata.push_back(tmp);
        }
        fclose(fp);

        int begintime = alldata[0].count;
        double begindata = alldata[0].data;

        for(int i=1;i<alldata.size();i++)
        {
            if(abs(alldata[i].data - begindata) < 0.0001)
            {
                cout << alldata[i].count << " " << alldata[i].data << endl;
                cout << "stop" << endl;
                break;
            }
            begindata = alldata[i].data;
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
