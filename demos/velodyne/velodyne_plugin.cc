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
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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
        public: VelodynePlugin()
        {
            mPCCount = 0;
            mPointCloud.clear();
        }

        public: ~VelodynePlugin() {
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
            this->mModel->GetJointController()->SetVelocityTarget(this->mJoint->GetScopedName(), 20);

            // ROS 配置
            this->mRosNode.reset(new ros::NodeHandle("gazebo_demos"));

            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->mModel->GetName() + "/vel_cmd",
                    1,
                    boost::bind(&VelodynePlugin::OnVelCmdMsgFromROS, this, _1),
                    ros::VoidPtr(), &this->mRosQueue);
            this->mRosVelSub = this->mRosNode->subscribe(so);
            ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<gazebo_demos::SetPIDParams>(
                    "/" + this->mModel->GetName() + "/SetPIDParams",
                    boost::bind(&VelodynePlugin::OnSetPIDParamSrvFromROS, this, _1, _2),
                    ros::VoidPtr(), &this->mRosQueue);
            this->mRosSetPIDParamSrv = this->mRosNode->advertiseService(aso);
            this->mRosPointCloudPub = this->mRosNode->advertise<sensor_msgs::PointCloud2>("/point_cloud_3d", 10);

            // Laser Scan
            this->mGazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->mGazeboNode->Init();
            mLaserScanSub = this->mGazeboNode->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", &VelodynePlugin::OnLaserScanMsg, this);

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
         * OnSetPIDParamSrvFromROS - 接收来自ROS系统的PID调参服务请求的回调函数
         * 
         * @req: 请求数据
         * @res: 响应结果
         */
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
                this->mRosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        /*
         * OnLaserScanMsg - 接收到Gazebo中雷达扫描数据消息的回调函数
         * 
         * @msg: 雷达扫描数据
         */
        private: void OnLaserScanMsg(ConstLaserScanStampedPtr & msg)
        {
            gazebo::msgs::Pose const & pose_msg = msg->scan().world_pose();
            ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msg);
            Eigen::Quaterniond qua(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

            int count = msg->scan().count();
            double angle_min = msg->scan().angle_min();
            double angle_step = msg->scan().angle_step();
            double range_min = msg->scan().range_min();
            double range_max = msg->scan().range_max();

            for (int i = 0; i < count; i++) {
                double range = msg->scan().ranges(i);
                if (std::isinf(range) || range > range_max || range < range_min)
                    continue;

                double angle = angle_min + angle_step * i;
				Eigen::Vector3d p(range * cos(angle), range * sin(angle), 0);
                p = qua.matrix() * p;
                pcl::PointXYZ point(p[0] + pose.Pos().X(), p[1] + pose.Pos().Y(), p[2] + pose.Pos().Z());
        		mPointCloud.push_back(point);  
            }
            mPCCount++;

            if (mPCCount > 100) {
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(mPointCloud, pc_msg);
                pc_msg.header.stamp = ros::Time(msg->time().sec(), msg->time().nsec());
                pc_msg.header.frame_id = "base";
                this->mRosPointCloudPub.publish(pc_msg);

                mPCCount = 0;
                mPointCloud.clear();
            }
        }

        private:
            std::unique_ptr<ros::NodeHandle> mRosNode;
            ros::Subscriber mRosVelSub;
            ros::Publisher mRosPointCloudPub;
            ros::ServiceServer mRosSetPIDParamSrv;
            ros::CallbackQueue mRosQueue;
            std::thread mRosQueueThread;

            physics::WorldPtr mWorld;
            physics::ModelPtr mModel;
            physics::JointPtr mJoint;
            common::PID mPid;

            gazebo::transport::NodePtr mGazeboNode;
            gazebo::transport::SubscriberPtr mLaserScanSub;

            private: pcl::PointCloud<pcl::PointXYZ> mPointCloud;
            private: int mPCCount;

    };

    // 向Gazebo注册本插件
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
