#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    // 一个控制Velodyne传感器的插件，它是一个Model类型的插件，所以继承自ModelPlugin
    class VelodynePlugin : public ModelPlugin
    {
        // 构造函数
        public: VelodynePlugin() {}

        // 在插入本插件的时候，Gazebo就会调用该函数。
        // 该函数是一个虚函数，我们可以通过重载该函数来对插件进行一些初始化的操作。
        // 输入参数_model是一个指向与本插件相关联的模型的指针。
        // 输入参数_sdf则是一个指向本插件SDF元素的指针。
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
           if (_model->GetJointCount() == 0) {
               std::cerr << "没有正确装在velodyne模型\n";
               return;
           }
           this->model = _model;
           this->joint = _model->GetJoints()[0];
           // 配置PID控制器, 比例增益为0.1.
           this->pid = common::PID(0.1, 0, 0);
           // 应用PID控制器
           this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->GetName());
            // 订阅主题名字
            std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
            // 订阅主题，并注册回调函数OnMsg
            this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);
        }

        // 设定Velodyne的转速
        // 输入参数_vel定义了控制的目标转速
        void SetVelocity(const double &_vel)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
        }

        // 处理接收的消息 
        // 输入参数_msg记录了接收到的消息内容，是一个Vector3的消息，这里只用到其中的x分量。
        private: void OnMsg(ConstVector3dPtr &_msg)
        {
            this->SetVelocity(_msg->x());
        }

        private:
            physics::ModelPtr model;
            physics::JointPtr joint;
            common::PID pid;

            transport::NodePtr node;
            transport::SubscriberPtr sub;
    };

    // 向Gazebo注册本插件
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
