#include <iostream>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <SetPIDParamRequest.pb.h>

int main(int argc, char **argv)
{
    std::cerr << "GAZEBO_MAJOR_VERSION:" << GAZEBO_MAJOR_VERSION << std::endl;

    gazebo::client::setup(argc, argv);

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo_demos_proto::msgs::SetPIDParamRequest>("~/my_velodyne/set_pid_param");

    pub->WaitForConnection();

    gazebo_demos_proto::msgs::SetPIDParamRequest request;
    request.set_pgain(0.01);
    pub->Publish(request);

    gazebo::client::shutdown();
}
