#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "example_interfaces/msg/bool.hpp"

using Bool = example_interfaces::msg::Bool;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_=node;
            arm_ = std::make_shared<MoveGroup>(node_,"arm");
            gripper_ = std::make_shared<MoveGroup>(node_,"gripper");
            open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper",10
                ,std::bind(&Commander::OpenGripperCb,this,_1));
            arm_->setMaxAccelerationScalingFactor(1.0);
            arm_->setMaxVelocityScalingFactor(1.0);
        }

        void NamedGoal(const std::string& pose_name)
        {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(pose_name);
            plan_and_exec(arm_);
        }

        void JointGoal(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);
            plan_and_exec(arm_);
        }

        void PoseGoal(double x,double y ,double z , double roll, double pitch ,double yaw,
             bool cartesian=false)
        {
            tf2::Quaternion q;
            q.setRPY(roll,pitch,yaw);

            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = "base_link";
            target.pose.position.x = x;
            target.pose.position.y = y;
            target.pose.position.z = z;
            target.pose.orientation.w =q.getW();
            target.pose.orientation.x =q.getX();
            target.pose.orientation.y =q.getY();
            target.pose.orientation.z =q.getZ();

            arm_->setStartStateToCurrentState();

            if(!cartesian)
            {
                arm_->setPoseTarget(target);
                plan_and_exec(arm_);
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target.pose);
                moveit_msgs::msg::RobotTrajectory traj;

                double fraction =arm_->computeCartesianPath(waypoints,0.01,traj);
                if(fraction > 0.99)
                {
                    arm_->execute(traj);
                }

            }
        }

        void OpenGrip()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_open");
            plan_and_exec(gripper_);
        }

         void CloseGrip()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_close");
            plan_and_exec(gripper_);
        }



    private:

        void plan_and_exec(const std::shared_ptr<MoveGroup> &interface)
        {

            MoveGroup::Plan plan;
            bool flag = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(flag) interface->execute(plan);
        }

        void OpenGripperCb(const Bool &msg)
        {
            if(msg.data == true) OpenGrip();
            else CloseGrip();
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroup> arm_;
        std::shared_ptr<MoveGroup> gripper_;
        std::shared_ptr<rclcpp::Subscription<Bool>> open_gripper_sub_;
};

int main(int argc,char** argv)
{   
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = std::make_shared<Commander>(node);
    rclcpp::spin(node);
    return 0;
}