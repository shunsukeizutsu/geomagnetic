/****************
GNSSから地磁気を計算するプログラム
*****************/
#include "GnsstoMag.hpp"

GG MagEquation::MagXYZ(double latitude,double longitude)
{//緯度経度から地磁気XYZを出力する
    double delta_phi = latitude - 37;
    double delta_lamda = longitude - 138;
    double Declin = Declination(delta_phi, delta_lamda) * M_PI / 180;

    //dip(delta_phi,delta_lamda);
    GG gmag;
    gmag.GmagX = Horizonforce(delta_phi, delta_lamda) * cos(Declin);
    gmag.GmagY = Horizonforce(delta_phi, delta_lamda) * -sin(Declin);
    gmag.GmagZ = Verticalforce(delta_phi,delta_lamda);
    //printf("%f %f %f \n",gmag.GmagX/1000,gmag.GmagY/1000,gmag.GmagZ/1000);
    return gmag;
}
double MagEquation::Declination(double del_phi, double del_lamda)
{//偏角:地図の北と方位磁石が指す磁北とのズレの角度
    double Declin;
    double num1, num2, num3, num4, num5, num6;
    num1 = 8 + 15.822 / 60;
    num2 = 18.462 / 60;
    num3 = 7.726 / 60;
    num4 = 0.007 / 60;
    num5 = 0.655 / 60;
//8度15.822分 + 18.462分*delta_phi - 7.726分*delta_lamda + 0.007分*delta_phi^2 - 0.007分*delta_phi*delta_lamda - 0.655分*delta_lamda^2
    Declin = num1 + num2 * del_phi - num3 * del_lamda + num4 * del_phi * del_phi - num4 * del_lamda * del_phi - num5 * del_lamda * del_lamda;
//    printf("偏角D:%f[度]\t",Declin);
    return Declin; // 単位は度
}

double MagEquation::dip(double del_phi, double del_lamda)
{//伏角：磁針が水平面に対して傾く角度
    double dip;
    double dnum1, dnum2, dnum3, dnum4, dnum5, dnum6;
    dnum1 = 51 + 26.559 / 60;
    dnum2 = 72.683 / 60;
    dnum3 = 8.642 / 60;
    dnum4 = 0.943 / 60;
    dnum5 = 0.142 / 60;
    dnum6 = 0.585 / 60;

    dip = dnum1 + dnum2 * del_phi - dnum3 * del_lamda - dnum4 * del_phi * del_phi - dnum5 * del_phi * del_lamda + dnum6 * del_lamda * del_lamda;
//    printf("伏角:%f[度]\t",dip);
    return dip;
}

double MagEquation::Magforce(double del_phi, double del_lamda)
{
    double force;
    force = 47881.463 + 547.650 * del_phi - 256.043 * del_lamda - 2.388 * del_phi * del_phi - 2.750 * del_phi * del_lamda + 5.199 * del_lamda * del_lamda;
//    printf("全磁力:%f[nT]\t",force);
    return force;
}

double MagEquation::Horizonforce(double del_phi, double del_lamda)
{
    double Hforce;
    Hforce = 29855.926 - 439.613 * del_phi - 74.293 * del_lamda - 6.703 * del_phi * del_phi + 7.987 * del_phi * del_lamda - 5.094 * del_lamda * del_lamda;
//    printf("水平分力:%f[nT]\t",Hforce);
    return Hforce;
}

double MagEquation::Verticalforce(double del_phi, double del_lamda)
{
    double Vforce;
    Vforce = 37441.791 + 1058.480 * del_phi - 274.371 * del_lamda - 10.853 * del_phi * del_phi - 6.454 * del_phi * del_lamda + 9.899 * del_lamda * del_lamda;
//    printf("鉛直分力:%f[nT]\n",Vforce);
    return Vforce;
}
