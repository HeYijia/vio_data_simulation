#include <sys/time.h>
#include <fstream>
#include <iomanip>
#include "imu.h"

double now()
{
    struct timeval ts;
    gettimeofday(&ts, NULL);
    return (double)ts.tv_sec + ts.tv_usec * 1e-6;
}

int main(int argc, char** argv)
{
    const std::string home_path = getenv("HOME");
    const std::string bag_path = home_path + "/imu.txt";
    std::ofstream ofs(bag_path);

    double begin = now();
    std::cout << "Start generate data, please waiting..." << std::endl;

    // IMU model
    Param params;
    params.t_end = 3600 * 2; // 2 hours
    IMU imuGen(params);

    const char symbol[4] = {'|', '/', '-', '\\'};
    for (double t = params.t_start; t < params.t_end;)
    {
        if( (int)t % params.imu_frequency == 0)
        {
            int i = (int)( (t - params.t_start) / (params.t_end - params.t_start) * 100);
            printf("[#][%d%%][%c]\r", i, symbol[i % 4]);
            fflush(stdout);
        }

        // create imu data && add imu noise
        MotionData data = imuGen.MotionModel(0);
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);

        ofs << std::fixed << std::setprecision(6)
            << begin + t << " "
            << std::setprecision(7)
            << data_noise.imu_acc(0) << " "
            << data_noise.imu_acc(1) << " "
            << data_noise.imu_acc(2) << " "
            << data_noise.imu_gyro(0) << " "
            << data_noise.imu_gyro(1) << " "
            << data_noise.imu_gyro(2) << "\n";

        t += 1.0 / params.imu_frequency;
    }
    fflush(stdout);
    std::cout << "Done, save to " << bag_path << std::endl;

    return 0;
}
