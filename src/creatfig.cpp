/*for (int k = 0; k < count2; k++)
{
    fprintf(gnuplot, "set grid\n\n");
    fprintf(gnuplot, "set size ratio -1\n\n");
    fprintf(gnuplot, "set xlabel 'count [pcs]'\n\n");
    fprintf(gnuplot, "set ylabel 'k'\n\n");
    fprintf(gnuplot, "set key outside right\n\n");
    fprintf(gnuplot, "set size square \n\n");
    fprintf(gnuplot, "set xlabel font 'Arial,22'\n\n");
    fprintf(gnuplot, "set ylabel font 'Arial,22'\n\n");
    fprintf(gnuplot, "set xlabel offset 0,-1\n\n");
    fprintf(gnuplot, "set ylabel offset -4,0\n\n");
    fprintf(gnuplot, "set tics font 'Arial,15'\n\n");
    fprintf(gnuplot, "set grid linewidth 1.5\n\n");
    // fprintf(gnuplot, "set label '平均=%lf\n分散=%lf\n標準偏差=%lf'\n\n", ave, bunsan, SD);
    //  fprintf(gnuplot, "set grid linecolor 'dark-red'\n\n");
    fprintf(gnuplot, "set xrange[%f:%f]\nset yrange[%f:%f]\n\n", XLANGEMIN, XLANGEMAX, YLANGEMIN, YLANGEMAX); // 範囲
    fprintf(gnuplot, "p ");
    fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'red' t \"k_X\", ");  // 文法的な感じ  odm 2
    fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'blue' t \"k_Y\", "); // 文法的な感じ  odm 2
    fprintf(gnuplot, " '-' pt 7 ps 1 lc rgb 'green' t \"k_Z\" "); // 文法的な感じ  odm 2
    fprintf(gnuplot, "\n");
    fprintf(gnuplot, "%d, %lf\n", k, num[k][0]); //
    avex[0] += num[k][0];
    avex[1] += num[k][1];
    avex[2] += num[k][2];
    // fprintf(gnuplot, "%d, %lf\n", k, num[k][1]); //
    //  fflush(gnuplot);                                    // 一つの文法の定義にひとつ必要 1
}*/
#include "creatfig.hpp"
PlotData::PlotData()
{
    //gp = popen("gnuplot","w");//シェル起動
    //fprintf(gp,"set grid\n");
}
PlotData::PlotData(double xmax,double xmin,double yamx,double ymin)
{
    //FILE *gp;
    //gp = popen("gnuplot","w");//シェル起動
    //fprintf(gp,"set grid\n");
}
PlotData::~PlotData()
{
    //pclose(gp);
}
void PlotData::saveData2D(double Xdata,double Ydata)
{
    plot_data tmp;
    tmp.xdata = Xdata;
    tmp.ydata = Ydata;
    Vdata.push_back(tmp);
}
void PlotData::PrintFig2D(void)
{
    for(int i=0;i<Vdata.size();i++)
    {
        printf("%f %f\n",Vdata[i].xdata,Vdata[i].ydata);    
    }
}
void PlotData::PrintFig3D(void)
{

}