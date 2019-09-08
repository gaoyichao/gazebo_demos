#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

int main(int _argc, char **_argv)
{
    std::cerr << "GAZEBO_MAJOR_VERSION:" << GAZEBO_MAJOR_VERSION << std::endl;
    gazebo::client::setup(_argc, _argv);

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

    pub->WaitForConnection();

    gazebo::msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
  
    pub->Publish(msg);

    gazebo::client::shutdown();
}
