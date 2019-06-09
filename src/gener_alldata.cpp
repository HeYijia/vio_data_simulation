#include <ros/ros.h> 
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

#include <fstream>

#include "imu.h"
#include "utilities.h"

int main(int argc, char** argv)
{
    // IMU model
 
    Param params;
    IMU imuGen(params);

    rosbag::Bag bag;
    bag.open("/home/hyj/all_ros_ws/vio_sim_data_ws/src/data/imu.bag", rosbag::bagmode::Write);

    ros::Time::init();
    double begin =ros::Time::now().toSec();
    std::cout << " start generate data, please waiting..."<<std::endl;

    char bar[102]={0};
    const char symbol[4] = {'|','/','-','\\'};
	
    for (double t = params.t_start; t<params.t_end;) {

	if( (int)t % params.imu_frequency == 0)
	{
	    int i = (int)( (t - params.t_start) / (params.t_end - params.t_start) * 100);
	    bar[i] = '#';
	    printf("[%s][%d%%][%c]\r", bar, i, symbol[i%4]);
	    fflush(stdout);
	}	
	
        sensor_msgs::Imu imu_data;
        
        ros::Time time_now(begin+t);

        imu_data.header.stamp = time_now;
        imu_data.header.frame_id = "base_link";

        // create imu data && add imu noise
        MotionData data = imuGen.MotionModel(0);
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        // to qua
        Eigen::Quaterniond q(data_noise.Rwb);


        //四元数位姿
        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation.w = q.w();
        //线加速度
        imu_data.linear_acceleration.x = data_noise.imu_acc(0); 
        imu_data.linear_acceleration.y = data_noise.imu_acc(1);
        imu_data.linear_acceleration.z = data_noise.imu_acc(2);
        //角速度
        imu_data.angular_velocity.x = data_noise.imu_gyro(0); 
        imu_data.angular_velocity.y = data_noise.imu_gyro(1); 
        imu_data.angular_velocity.z = data_noise.imu_gyro(2);

        bag.write("imu", time_now, imu_data);

        t += 1.0/params.imu_frequency;
    }

    bag.close();
    std::cout << " END "<<std::endl;


/*    
    ros::init(argc, argv, "imu");     
    ros::NodeHandle n;

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu", 20);
    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";

        // IMU model
        Param params;
        IMU imuGen(params);

        // create imu data
        // imu pose gyro acc
        MotionData data = imuGen.MotionModel(0);
        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        // to qua
        Eigen::Quaterniond q(data_noise.Rwb);

        //四元数位姿
        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation.w = q.w();
        //线加速度
        imu_data.linear_acceleration.x = data_noise.imu_acc(0); 
        imu_data.linear_acceleration.y = data_noise.imu_acc(1);
        imu_data.linear_acceleration.z = data_noise.imu_acc(2);
        //角速度
        imu_data.angular_velocity.x = data_noise.imu_gyro(0); 
        imu_data.angular_velocity.y = data_noise.imu_gyro(1); 
        imu_data.angular_velocity.z = data_noise.imu_gyro(2);

        IMU_pub.publish(imu_data);

        ros::spinOnce();  
        loop_rate.sleep();  
    }
*/
    return 0;
}


