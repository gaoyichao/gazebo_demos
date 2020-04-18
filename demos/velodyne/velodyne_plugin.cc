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
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Angle.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_service_options.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo_demos/SetPIDParams.h>
#include <SetPIDParamRequest.pb.h>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace gazebo
{
    typedef const boost::shared_ptr<
        const gazebo_demos_proto::msgs::SetPIDParamRequest>
        SetPIDParamRequestPtr;

    class VelodynePlugin : public ModelPlugin
    {
        public: VelodynePlugin() {}
                ~VelodynePlugin() {
                    delete mTfBr;
                }

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
            std::cout << "douniwan" << std::endl;

            this->mWorld = model->GetWorld();
            this->mModel = model;
            this->mJoint = model->GetJoints()[0];
            this->mPid = common::PID(0.1, 0, 0);
            this->mModel->GetJointController()->SetVelocityPID(this->mJoint->GetScopedName(), this->mPid);

            // ROS 配置
            this->mRosNode.reset(new ros::NodeHandle("gazebo_demos"));

            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->mModel->GetName() + "/vel_cmd",
                    1,
                    boost::bind(&VelodynePlugin::OnVelCmdMsgFromROS, this, _1),
                    ros::VoidPtr(), &this->mRosQueue);
            this->mRosSub = this->mRosNode->subscribe(so);
            ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<gazebo_demos::SetPIDParams>(
                    "/" + this->mModel->GetName() + "/SetPIDParams",
                    boost::bind(&VelodynePlugin::OnSetPIDParamSrvFromROS, this, _1, _2),
                    ros::VoidPtr(), &this->mRosQueue);
            this->mRosSetPIDParamSrv = this->mRosNode->advertiseService(aso);

            // Laser Scan
            this->mGazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->mGazeboNode->Init();
            mLaserScanSub = this->mGazeboNode->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", &VelodynePlugin::OnLaserScanMsg, this);
            this->mRosPub = this->mRosNode->advertise<sensor_msgs::LaserScan>("/laserscan", 10);
            this->mTfBr = new tf2_ros::TransformBroadcaster();

            mSetPIDParamSub = this->mGazeboNode->Subscribe("~/my_velodyne/set_pid_param", &VelodynePlugin::OnSetPIDParamMsg, this);

            mPlotFlag = false;
            mSimTime = mWorld->SimTime();
            mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VelodynePlugin::OnWorldUpdateBegin, this)); 

            this->mRosQueueThread = std::thread(std::bind(&VelodynePlugin::RosQueueThread, this));
        }


        private:
            event::ConnectionPtr mUpdateConnection;
            common::Time mSimTime;
            double mVelCmd;
            std::vector<double> mPlotTimes;
            std::vector<double> mPlotVels;
            std::vector<double> mPlotCmds;
            bool mPlotFlag;

        private: void OnWorldUpdateBegin()
        {
            common::Time curTime = mWorld->SimTime();
            mSimTime = curTime;
            
            if (mPlotFlag) {
                if (mPlotTimes.size() > 100) {
                    mPlotFlag = false;
                }
    
                mPlotTimes.push_back(curTime.Double());
                mPlotVels.push_back(this->mJoint->GetVelocity(0));
                mPlotCmds.push_back(mVelCmd);
                plt::clf();
                plt::plot(mPlotTimes, mPlotVels);
                plt::plot(mPlotTimes, mPlotCmds);
                plt::xlim(mPlotTimes.front(), mPlotTimes.back());
                plt::pause(0.001);
            }
        }

        /*
         * SetVelocity - 设定Velodyne的转速
         * 
         * @vel: 激光雷达的扫描转速, rad/s
         */
        public: void SetVelocity(const double &vel)
        {
            mPlotFlag = true;
            mPlotTimes.clear();
            mPlotVels.clear();
            mPlotCmds.clear();
            mVelCmd = vel;
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

        private: bool OnSetPIDParamSrvFromROS(gazebo_demos::SetPIDParams::Request & req, gazebo_demos::SetPIDParams::Response & res)
        {
            mPid.SetPGain(req.P);
            mPid.SetIGain(req.I);
            mPid.SetDGain(req.D);
            this->mModel->GetJointController()->SetVelocityPID(this->mJoint->GetScopedName(), this->mPid);
            res.result = true;
            return true;
        }

        /*
         * RosQueueThread - 处理ROS消息队列的线程
         */
        private: void RosQueueThread()
        {
            static const double timeout = 0.01;
            while (this->mRosNode->ok()) {
                while (!this->mLaserScanQueue.empty()) {
                    sensor_msgs::LaserScan &laser_msg = mLaserScanQueue.front();
                    this->mRosPub.publish(laser_msg);
                    mLaserScanQueue.pop_front();
                }
                this->mRosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private: void OnSetPIDParamMsg(SetPIDParamRequestPtr & msg)
        {
            gazebo_demos_proto::msgs::SetPIDParamRequest request;
            mPid.SetPGain(msg->pgain());
            mPid.SetIGain(msg->igain());
            mPid.SetDGain(msg->dgain());
            this->mModel->GetJointController()->SetVelocityPID(this->mJoint->GetScopedName(), this->mPid);
        }

        private: void OnLaserScanMsg(ConstLaserScanStampedPtr & msg)
        {
            sensor_msgs::LaserScan laser_msg;
            laser_msg.header.stamp = ros::Time(msg->time().sec(), msg->time().nsec());
            laser_msg.header.frame_id = "top";
			laser_msg.angle_min = msg->scan().angle_min();
  			laser_msg.angle_max = msg->scan().angle_max();
  			laser_msg.angle_increment = msg->scan().angle_step();
  			laser_msg.time_increment = 0;  // instantaneous simulator scan
  			laser_msg.scan_time = 0;  // not sure whether this is correct
  			laser_msg.range_min = msg->scan().range_min();
  			laser_msg.range_max = msg->scan().range_max();
  			laser_msg.ranges.resize(msg->scan().ranges_size());
  			std::copy(msg->scan().ranges().begin(), msg->scan().ranges().end(), laser_msg.ranges.begin());
  			laser_msg.intensities.resize(msg->scan().intensities_size());
  			std::copy(msg->scan().intensities().begin(), msg->scan().intensities().end(), laser_msg.intensities.begin());
            mLaserScanQueue.push_back(laser_msg);

            gazebo::common::Time now = this->mModel->GetWorld()->SimTime();
            double position = mJoint->Position();
            double c = std::cos(position);
            double s = std::sin(position);
            tf2::Matrix3x3 rot(c,  0,  s,
                               s,  0, -c,
                               0,  1,  0);
            tf2::Vector3 ori(0, 0, 0);
            tf2::Transform trans(rot, ori);
            geometry_msgs::TransformStamped stampedTrans;
            stampedTrans.transform = tf2::toMsg(trans);
            stampedTrans.header.frame_id = "base";
            stampedTrans.header.stamp = ros::Time(now.sec, now.nsec);
            stampedTrans.child_frame_id = "top";
            mTfBr->sendTransform(stampedTrans);
        }

        private:
            std::unique_ptr<ros::NodeHandle> mRosNode;
            ros::Subscriber mRosSub;
            ros::Publisher mRosPub;
            ros::ServiceServer mRosSetPIDParamSrv;
            ros::CallbackQueue mRosQueue;
            tf2_ros::TransformBroadcaster *mTfBr;
            std::thread mRosQueueThread;

            physics::WorldPtr mWorld;
            physics::ModelPtr mModel;
            physics::JointPtr mJoint;
            common::PID mPid;

            gazebo::transport::NodePtr mGazeboNode;
            gazebo::transport::SubscriberPtr mLaserScanSub;
            gazebo::transport::SubscriberPtr mSetPIDParamSub;
            std::deque<sensor_msgs::LaserScan> mLaserScanQueue;
    };

    // 向Gazebo注册本插件
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
