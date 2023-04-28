#ifndef CREATFIG_HPP_
#define CREATFIG_HPP_

#include <stdio.h>
#include <iostream>
#include <vector>

typedef struct Figuredata{
    double xdata;
    double ydata;
    double zdata;
}plot_data;

class PlotData
{
    protected:
        FILE *gp;
        std::vector<plot_data> Vdata;
    public:
        PlotData();
        PlotData(double xmax,double xmin,double ymax,double ymin);
        ~PlotData();
        void saveData2D(double Xdata,double Ydata);
        void PrintFig2D(void);
        void PrintFig3D(void);
};
#endif //CREATFIG_HPP