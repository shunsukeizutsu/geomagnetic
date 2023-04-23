#include "calcgeo.hpp"

int main(void)
{
    MagEquation MF;
    // SSMLog<rtk_gnss_f9p> rtk_gnss;
    SSMLog<gnss_gl> rtk_gnss;
    // SSMLog<imu_fs, imu_property> imu;
    SSMLog<localizer> local;
    SSMLog<imu_fs, imu_property> imu;

    if (!rtk_gnss.open("../2023.0402.0748/rtk_gnss.log"))
    {
        fprintf(stderr, "Error! Cannot open gnss.log");
        exit(EXIT_FAILURE);
    }
/*    if (!imu.open("../2023.0402.0748/imu.log"))
    {
        fprintf(stderr, "Error! Cannot open imu.log");
        exit(EXIT_FAILURE);
    }
    if (!local.open("../2023.0402.0748/localizer.log"))
    {
        fprintf(stderr, "Error! Caonnot open localizer.log");
        exit(EXIT_FAILURE);
    }*/

    gnss_gl *gnssdata;
//    imu_fs *imudata;
//    localizer *localdata;

    gnssdata = &rtk_gnss.data();
//    imudata = &imu.data();
//    localdata = &local.data();

    try
    {
        setSigInt();
        double time = 0.0;
        int count = 0;
        /************************************************************/
        while (!gShutOff)
        {
            // １秒1つのデータを取り出し、vectorに保存
            //  double time = rtk_gnss.time();
            //  printf("%f\n",time);
            /*if (rtk_gnss.read())
            {
                if (rtk_gnss.time() - time >= 1.0)
                {
                    // printf("%d\t",count);
                    count++;

                    if (count > 60)
                    {
                        break;
                    }
                    data_sec tmp;
                    time = rtk_gnss.time();
                    tmp.time = time;
                    GG gmag;
                    gmag = MF.MagXYZ(gnssdata->latitude, gnssdata->longitude);
                    tmp.gnssdata[0] = gmag.GmagY / 1000;
                    tmp.gnssdata[1] = gmag.GmagX / 1000;
                    tmp.gnssdata[2] = -gmag.GmagZ / 1000;
                    // printf("%f\n", time);
                    if (imu.readTime(time))
                    {
                        tmp.imumag[0] = imudata->mag[0];
                        tmp.imumag[1] = imudata->mag[1];
                        tmp.imumag[2] = imudata->mag[2];
                    }
                    if (local.readTime(time))
                    {
                        tmp.localizer[0] = localdata->estAng[0];
                        tmp.localizer[1] = localdata->estAng[1];
                        tmp.localizer[2] = localdata->estAng[2];
                    }
                    data.push_back(tmp);
                }
            }
            else
                break;
            */
        }
        //**************************************************************************//
        // xの初期値
        /*double k = 2.0;
        double ax = 19.899967;
        double ay = 19.996183;
        double az = -26.814685;
        double delta_x = 10;
        while (abs(delta_x) > 1)
        {
            Matrix<double, 4, 1> X;
            Matrix<double, 4, 4> JTJ;
            Matrix<double, 4, 1> JTb;

            for (int i = 0; i < data.size(); i++)
            {
                CreatMatrix MDD;
                MDD = CaluculateMatrix(data[i], k, ax, ay, az);
                JTb += MDD.JT * MDD.b;
                JTJ += MDD.JT * MDD.J;
                // printdata(data[i]);
            }
            Eigen::PartialPivLU<Eigen::MatrixXd> lu(JTJ);
            X = lu.solve(JTb);
            // std::cout << X << std::endl;
            std::cout << "\n " << std::endl;
            delta_x = X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0) + X(3, 0) * X(3, 0);
            k = X(0, 0);
            ax = X(1, 0) + ax;
            ay = X(2, 0) + ay;
            az = X(3, 0) + az;
            delta_x = sqrt(delta_x);
            std::cout << delta_x << std::endl;
        }*/
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
void printdata(data_sec data)
{
    printf("%f %f %f %f %f %f %f %f %f \n", data.time, data.gnssdata[0], data.gnssdata[1], data.imumag[0], data.imumag[1], data.imumag[2], data.localizer[0], data.localizer[1], data.localizer[2]);
}
CreatMatrix CaluculateMatrix(data_sec data, double k, double ax, double ay, double az)
{
    CreatMatrix CM;
    double Sr, Sp, Sy, Cr, Cp, Cy;
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    Sr = sin(data.localizer[0]);
    Sp = sin(data.localizer[1]);
    Sy = sin(data.localizer[2]);
    Cr = cos(data.localizer[0]);
    Cp = cos(data.localizer[1]);
    Cy = cos(data.localizer[2]);
    r11 = Cy * Cp;
    r12 = -Sy * Cr + Cy * Sp * Sr;
    r13 = Sr * Cy + Cy * Sp * Cr;
    r21 = Sy * Cp;
    r22 = Cy * Cr + Sr * Sp * Sy;
    r23 = -Cy * Sr + Sp * Sy * Cr;
    r31 = -Sp;
    r32 = Cp * Sr;
    r33 = Cp * Cr;
    // 回転行列
    CM.R(0, 0) = r11;
    CM.R(0, 1) = r12;
    CM.R(0, 2) = r13;
    CM.R(1, 0) = r21;
    CM.R(1, 1) = r22;
    CM.R(1, 2) = r23;
    CM.R(2, 0) = r31;
    CM.R(2, 1) = r32;
    CM.R(2, 2) = r33;

    CM.m(0, 0) = data.imumag[0] - ax;
    CM.m(1, 0) = data.imumag[1] - ay;
    CM.m(2, 0) = data.imumag[2] - az;

    // Jの行列
    CM.J(0, 0) = r11 * (data.imumag[0] - ax) + r12 * (data.imumag[1] - ay) + r13 * (data.imumag[2] - az);
    CM.J(0, 1) = -k * r11;
    CM.J(0, 2) = -k * r12;
    CM.J(0, 3) = -k * r13;
    CM.J(1, 0) = r21 * (data.imumag[0] - ax) + r22 * (data.imumag[1] - ay) + r23 * (data.imumag[2] - az);
    CM.J(1, 1) = -k * r21;
    CM.J(1, 2) = -k * r22;
    CM.J(1, 3) = -k * r23;
    CM.J(2, 0) = r31 * (data.imumag[0] - ax) + r32 * (data.imumag[1] - ay) + r33 * (data.imumag[2] - az);
    CM.J(2, 1) = -k * r31;
    CM.J(2, 2) = -k * r32;
    CM.J(2, 3) = -k * r33;

    // Jの転置行列
    CM.JT(0, 0) = CM.J(0, 0);
    CM.JT(0, 1) = CM.J(1, 0);
    CM.JT(0, 2) = CM.J(2, 0);
    CM.JT(1, 0) = CM.J(0, 1);
    CM.JT(1, 1) = CM.J(1, 1);
    CM.JT(1, 2) = CM.J(2, 1);
    CM.JT(2, 0) = CM.J(0, 2);
    CM.JT(2, 1) = CM.J(1, 2);
    CM.JT(2, 2) = CM.J(2, 2);
    CM.JT(3, 0) = CM.J(0, 3);
    CM.JT(3, 1) = CM.J(1, 3);
    CM.JT(3, 2) = CM.J(2, 3);

    // gnss->magdata
    CM.M(0, 0) = data.gnssdata[0];
    CM.M(1, 0) = data.gnssdata[1];
    CM.M(2, 0) = data.gnssdata[2];

    CM.F = k * CM.R * CM.m;
    CM.b = CM.M - CM.F;
    return CM;
}