#include <stdio.h>
#include <math.h>


typedef struct GnssGeomagnetism{
    double GmagX;
    double GmagY;
    double GmagZ;
}GG;

class MagEquation
{
public:
    GG MagXYZ(double latitude, double longitude);
    double Declination(double del_phi, double del_lamda);
    double dip(double del_phi, double del_lamda);
    double Magforce(double del_phi, double del_lamda);
    double Horizonforce(double del_phi, double del_lamda);
    double Verticalforce(double del_phi, double del_lamda);
};