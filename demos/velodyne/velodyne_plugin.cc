#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
    class VelodynePlugin : public ModelPlugin
    {
        public: VelodynePlugin() {}

        /*
         * Load - 在插入本插件的时候, Gazebo就会调用该函数, 进行一些初始化的操作.
         * 
         * @model: 指向与本插件相关联的模型的指针
         * @sdf: 指向本插件SDF元素的指针
         */
        public: virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            if (model->GetJointCount() == 0) {
                std::cerr << "没有正确装载velodyne模型\n";
                return;
            }
            this->mModel = model;
            this->mJoint = model->GetJoints()[0];
            this->mPid = common::PID(0.1, 0, 0);
            this->mModel->GetJointController()->SetVelocityPID(this->mJoint->GetScopedName(), this->mPid);

            // ROS 配置
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_demos", ros::init_options::NoSigintHandler);
            }
            this->mRosNode.reset(new ros::NodeHandle("gazebo_demos"));

            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->mModel->GetName() + "/vel_cmd",
                    1,
                    boost::bind(&VelodynePlugin::OnVelCmdMsgFromROS, this, _1),
                    ros::VoidPtr(), &this->mRosQueue);
            this->mRosSub = this->mRosNode->subscribe(so);
            this->mRosQueueThread = std::thread(std::bind(&VelodynePlugin::RosQueueThread, this));
        }

        /*
         * SetVelocity - 设定Velodyne的转速
         * 
         * @vel: 激光雷达的扫描转速, rad/s
         */
        public: void SetVelocity(const double &vel)
        {
            this->mModel->GetJointController()->SetVelocityTarget(this->mJoint->GetScopedName(), vel);
        }

        /*
         * OnVelCmdMsgFromROS - 接收到来自ROS系统的调速消息的回调函数
         * 
         * @msg: 调速消息
         */
        private: void OnVelCmdMsgFromROS(const std_msgs::Float32ConstPtr & msg)
        {
            this->SetVelocity(msg->data);
        }

        /*
         * RosQueueThread - 处理ROS消息队列的线程
         */
        private: void RosQueueThread()
        {
            static const double timeout = 0.01;
            while (this->mRosNode->ok())
                this->mRosQueue.callAvailable(ros::WallDuration(timeout));
        }


        private:
            std::unique_ptr<ros::NodeHandle> mRosNode;
            ros::Subscriber mRosSub;
            ros::CallbackQueue mRosQueue;
            std::thread mRosQueueThread;

            physics::ModelPtr mModel;
            physics::JointPtr mJoint;
            common::PID mPid;

            gazebo::transport::NodePtr mGazeboNode;
            gazebo::transport::SubscriberPtr mLaserScanSub;
    };

    // 向Gazebo注册本插件
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
