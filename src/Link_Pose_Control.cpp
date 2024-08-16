#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ml_project/GetPose.h>

// namespace gazebo
// {
  class Link_Pose_Control : public gazebo::ModelPlugin
  {
    private: gazebo::physics::ModelPtr model;
    private: ros::NodeHandle nh;
    private: ros::Subscriber poseSub;
    private: ros::ServiceServer poseService;

    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cout << "Loading Link Pose Control Plugin\n" << std::endl;
      this->model = _model;

      // ROS Node and Subscriber
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      ROS_INFO("ROS node initialized within Gazebo Plugin");
      poseSub = nh.subscribe("link/pose", 10, &Link_Pose_Control::OnPoseReceived, this);
      poseService = nh.advertiseService("get_link_pose", &Link_Pose_Control::GetLinkPose, this);
    }

    // Callback function to set the model pose
    public: void OnPoseReceived(const geometry_msgs::PoseConstPtr &msg)
    {
      ignition::math::Pose3d newPose(msg->position.x, msg->position.y, msg->position.z,
                                     msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      this->model->SetWorldPose(newPose);
    }

    // Service function to get the camera pose
    private: bool GetLinkPose(ml_project::GetPose::Request &req, ml_project::GetPose::Response &res)
    {
      ignition::math::Pose3d currentPose = this->model->WorldPose();
      
      res.pose.position.x = currentPose.Pos().X();
      res.pose.position.y = currentPose.Pos().Y();
      res.pose.position.z = currentPose.Pos().Z();
      res.pose.orientation.w = currentPose.Rot().W();
      res.pose.orientation.x = currentPose.Rot().X();
      res.pose.orientation.y = currentPose.Rot().Y();
      res.pose.orientation.z = currentPose.Rot().Z();
      
      return true;
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(Link_Pose_Control)
// }
