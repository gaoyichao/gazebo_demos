#include <iostream>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

void cb(ConstLaserScanStampedPtr & _msg)
{
    std::cout << _msg->DebugString() << std::endl;
}

int main(int argc, char **argv)
{
    std::cerr << "GAZEBO_MAJOR_VERSION:" << GAZEBO_MAJOR_VERSION << std::endl;
    gazebo::client::setup(argc, argv);

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", cb);

    pub->WaitForConnection();

    gazebo::msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(argv[1]), 0, 0));
  
    pub->Publish(msg);

    for (int i = 0; i < 10; i++)
        gazebo::common::Time::MSleep(10);

    gazebo::client::shutdown();
}
