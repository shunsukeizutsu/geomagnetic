#include <stdio.h>
#include <math.h>
#include <iostream>
#include "GnsstoMag.hpp"

int main(void)
{
    MagEquation mag;

    GG magnet = mag.MagXYZ(37,138);
    std::cout << magnet.GmagX << " " << magnet.GmagY << " " << magnet.GmagZ << std::endl;
    
    return 0;
}