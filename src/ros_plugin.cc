#ifndef ROS_PLUGIN_CC
#define ROS_PLUGIN_CC

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace gazebo {
    class ROSPlugin : public SystemPlugin {
        public: ROSPlugin() {}
                ~ROSPlugin() {}

        public: void Load(int argc, char **argv)
        {
            std::cout << "---------------------------------------" << std::endl;
            std::cout << "douniwan" << std::endl;
            std::cout << "argc = " << argc << ", " << argv[0] << std::endl;
            std::cout << "---------------------------------------" << std::endl;
            if (!ros::isInitialized()) {
                ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler);
            }


            this->mRosNode.reset(new ros::NodeHandle("~"));
            this->mRosClockPub = this->mRosNode->advertise<rosgraph_msgs::Clock>("/clock", 10);

            mWorldCreatedConnection = gazebo::event::Events::ConnectWorldCreated(boost::bind(&ROSPlugin::OnWorldCreated, this, _1));
            mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSPlugin::OnWorldUpdateBegin, this)); 
        }

        private: event::ConnectionPtr mUpdateConnection;
        private: event::ConnectionPtr mWorldCreatedConnection;

        private: void OnWorldCreated(std::string world_name)
        {
            std::cout << "world [" << world_name << "] created" << std::endl;
            mWorld = gazebo::physics::get_world(world_name);
        }

        private: std::unique_ptr<ros::NodeHandle> mRosNode;
        private: ros::Publisher mRosClockPub;
        private: void OnWorldUpdateBegin()
        {
            common::Time curTime = mWorld->SimTime();
            rosgraph_msgs::Clock rosTime;
            rosTime.clock.fromSec(curTime.Double());
            mRosClockPub.publish(rosTime);
        }

        private: gazebo::physics::WorldPtr mWorld;
    };

    GZ_REGISTER_SYSTEM_PLUGIN(ROSPlugin)
};

#endif


