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
    class VelodynePlugin : public ModelPlugin
    {
        public: VelodynePlugin()
        {
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

            // ROS 配置
            this->mRosNode.reset(new ros::NodeHandle("~"));
            this->mRosPointCloudPub = this->mRosNode->advertise<sensor_msgs::PointCloud2>("/point_cloud_3d", 10);

            // Laser Scan
            this->mGazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->mGazeboNode->Init();
            mLaserScanSub = this->mGazeboNode->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", &VelodynePlugin::OnLaserScanMsg, this);
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
            int vertical_count = msg->scan().vertical_count();
            double angle_min = msg->scan().angle_min();
            double angle_step = msg->scan().angle_step();
            double range_min = msg->scan().range_min();
            double range_max = msg->scan().range_max();
            double vertical_angle_min = msg->scan().vertical_angle_min();
            double vertical_angle_max = msg->scan().vertical_angle_max();
            double vertical_step = msg->scan().vertical_angle_step();

            pcl::PointCloud<pcl::PointXYZ> pc;
            for (int i = 0; i < count; i++) {
                for (int j = 0; j < vertical_count; j++) {
                    double range = msg->scan().ranges(i + j * count);
                    if (std::isinf(range) || range > range_max || range < range_min)
                        continue;
                    double pitch = j * vertical_step + vertical_angle_min;
                    double yaw = i * angle_step + angle_min;
                    double cp = cos(pitch);
                    double sp = sin(pitch);
                    double cy = cos(yaw);
                    double sy = sin(yaw);
                    Eigen::Vector3d point(range * cp * cy, range * cp * sy, range * sp);
                    point = qua.matrix() * point;
                    pc.push_back(pcl::PointXYZ(point[0] + pose.Pos().X(), point[1] + pose.Pos().Y(), point[2] + pose.Pos().Z()));
                }
            }

            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(pc, pc_msg);
            pc_msg.header.stamp = ros::Time(msg->time().sec(), msg->time().nsec());
            pc_msg.header.frame_id = "base";
            this->mRosPointCloudPub.publish(pc_msg);
        }

        private:
            std::unique_ptr<ros::NodeHandle> mRosNode;
            ros::Publisher mRosPointCloudPub;

            physics::WorldPtr mWorld;
            physics::ModelPtr mModel;

            gazebo::transport::NodePtr mGazeboNode;
            gazebo::transport::SubscriberPtr mLaserScanSub;
    };

    // 向Gazebo注册本插件
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
