#include <thread>
#include <deque>
#include <cmath>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/math/Angle.hh>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_service_options.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


namespace gazebo
{
    class DiffCartPlugin : public ModelPlugin
    {
        public: DiffCartPlugin()
        {
        }

        public: ~DiffCartPlugin()
        {
        }

        /*
         * Load - 在插入本插件的时候, Gazebo就会调用该函数, 进行一些初始化的操作.
         * 
         * @model: 指向与本插件相关联的模型的指针
         * @sdf: 指向本插件SDF元素的指针
         */
        public: virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            std::cout << "douniwan" << std::endl;

            mWheelDiff = 0.26;
            mWheelRadius = 0.1;
            mLeftWheelCmd = 0;
            mRightWheelCmd = 0;

            mWorld = model->GetWorld();
            mModel = model;
            mLeftWheel = model->GetJoint("diffcart::base_2_left_wheel");
            mRightWheel = model->GetJoint("diffcart::base_2_right_wheel");

            mSimTime = mWorld->SimTime();
            mLastCmdTime = mSimTime;
            mCmdTimeout = 0.1;

            // ROS 配置
            this->mRosNode.reset(new ros::NodeHandle("~"));
            mRosCmdVelSub = mRosNode->subscribe("/cmd_vel", 100, &DiffCartPlugin::OnCmdVelFromRos, this);

            mUpdateEndCnt = event::Events::ConnectWorldUpdateEnd(boost::bind(&DiffCartPlugin::OnWorldUpdateEnd, this));
        }

        /*
         * OnWorldUpdateEnd - 每次Gazebo完成仿真更新之后的回调函数
         */
        private: void OnWorldUpdateEnd()
        {
            mSimTime = mWorld->SimTime();

            ros::spinOnce();

            AdjustVelocity();
        }
 
        private:
            common::Time mSimTime;
            common::Time mLastCmdTime;
            double mCmdTimeout;         // 单位s

        /*
         * OnCmdVelFromRos - 订阅来自ROS的控制消息
         */
        private: void OnCmdVelFromRos(const geometry_msgs::TwistConstPtr & msg)
        {
            mLastCmdTime = mWorld->SimTime();

            mLeftWheelCmd = msg->linear.x - msg->angular.z * mWheelDiff * 0.5;
            mRightWheelCmd = msg->linear.x + msg->angular.z * mWheelDiff * 0.5;
        }

        private: void AdjustVelocity()
        {
            common::Time td = mSimTime - mLastCmdTime;
            if (td.Double() > mCmdTimeout) {
                mLeftWheelCmd = 0;
                mRightWheelCmd = 0;
            }
            mLeftWheel->SetVelocity(0, mLeftWheelCmd / mWheelRadius);
            mRightWheel->SetVelocity(0, mRightWheelCmd / mWheelRadius);
        }

        private:
            std::unique_ptr<ros::NodeHandle> mRosNode;
			ros::Subscriber mRosCmdVelSub;

            physics::WorldPtr mWorld;
            physics::ModelPtr mModel;
            physics::JointPtr mLeftWheel;
            physics::JointPtr mRightWheel;

            event::ConnectionPtr mUpdateEndCnt;

            double mWheelDiff;
            double mWheelRadius;
            double mLeftWheelCmd;
            double mRightWheelCmd;
    };

    GZ_REGISTER_MODEL_PLUGIN(DiffCartPlugin)
}

