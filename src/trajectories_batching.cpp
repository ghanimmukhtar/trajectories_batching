#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <baxter_mover_utils/baxter_mover.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_state/conversions.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

using namespace baxter_mover;

//double _half_time, _half_time_2;

class test_mover{
    public:
        test_mover(){
                init();
            }

        void init(){
                ROS_INFO("INITIALIZING ...");
                _joint_states_sub = _nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &test_mover::joint_state_callback, this);
                //_left_feedback_sub = _nh.subscribe<control_msgs::FollowJointTrajectoryFeedback>("/robot/limb/left/follow_joint_trajectory/feedback", 10, &test_mover::left_feedback_callback, this);
                //_right_feedback_sub = _nh.subscribe<control_msgs::FollowJointTrajectoryFeedback>("/robot/limb/right/follow_joint_trajectory/feedback", 10, &test_mover::right_feedback_callback, this);
                //_ac_l.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/robot/limb/left/follow_joint_trajectory", true));
                //_ac_r.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/robot/limb/right/follow_joint_trajectory", true));
                _baxter_mover.reset(new BAXTER_Mover(_nh));
                _right_joint_states = std::vector<double>(7, 0);
                _left_joint_states = std::vector<double>(7, 0);

                _my_spinner.reset(new ros::AsyncSpinner(1));
                _my_spinner->start();
                ROS_WARN_STREAM("baxter mover right joint names size is : " << _baxter_mover->global_parameters.get_baxter_right_arm_joints_names().size());
                ROS_WARN_STREAM("baxter mover left joint names size is : " << _baxter_mover->global_parameters.get_baxter_left_arm_joints_names().size());
                ROS_INFO("INITIALIZED!!!");
                _initialized = true;
            }
        void joint_state_callback(const sensor_msgs::JointStateConstPtr& jo_state){
                if(!_initialized)
                    return;
                //ROS_WARN_STREAM("baxter joint names size is : " << jo_state->name.size());
                //right arm joints values
                _right_joint_states[0] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[0]))];
                _right_joint_states[1] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[1]))];
                _right_joint_states[2] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[2]))];
                _right_joint_states[3] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[3]))];
                _right_joint_states[4] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[4]))];
                _right_joint_states[5] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[5]))];
                _right_joint_states[6] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                  jo_state->name.end(),
                                                                                                  _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[6]))];

                //left arm joints values
                _left_joint_states[0] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[0]))];
                _left_joint_states[1] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[1]))];
                _left_joint_states[2] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[2]))];
                _left_joint_states[3] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[3]))];
                _left_joint_states[4] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[4]))];
                _left_joint_states[5] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[5]))];
                _left_joint_states[6] = jo_state->position[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                 jo_state->name.end(),
                                                                                                 _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[6]))];
            }


        void left_done_callback(const actionlib::SimpleClientGoalState& state){
                if(state.isDone()){
                        _trajectory_done = true;
                        ROS_WARN("DONE!!!");
                    }
                else{
                        ROS_WARN("NOT DONE YET :(");
                        _trajectory_done = false;
                    }
            }

        void left_active_callback(){

            }

        void left_feedback_callback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
                ROS_WARN_STREAM("AT THE FEEDBACK, LEFT TIME FROM START : " << feedback->actual.time_from_start.toSec());
                ROS_INFO_STREAM("The Half time for the trajectory Is : " << _half_time);
                if(feedback->actual.time_from_start.toSec() > _half_time)
                    _half_done = true;
                else
                    _half_done = false;
            }

        void right_feedback_callback(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr& feedback){
                ROS_WARN("AT THE FEEDBACK, RIGHT :) :) :)");
            }

        bool bigger_half_time(trajectory_msgs::JointTrajectoryPoint point, double time){
                ROS_INFO_STREAM("This point time from start is : " << point.time_from_start.toSec());
                return point.time_from_start.toSec() > time;
            }

        void plan_and_execute(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac){
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
                _trajectory_done = false;
                _half_done = false;
                _trajectory_size = 0;

                _x = (_max_x - _min_x) * ((double)rand() / (double)RAND_MAX) + _min_x;
                _y = (_max_y - _min_y) * ((double)rand() / (double)RAND_MAX) + _min_y;
                _z = (_max_z - _min_z) * ((double)rand() / (double)RAND_MAX) + _min_z;


                _baxter_mover->group->setPositionTarget(_x, _y, _z);
                _baxter_mover->group->plan(_the_plan);

                if(!ac.waitForServer(ros::Duration(2.0)))
                    {
                        ROS_ERROR("Could not connect to action server");
                        return ;
                    }

                _trajectory_size = _the_plan.trajectory_.joint_trajectory.points.size();
                _goal.trajectory = _the_plan.trajectory_.joint_trajectory;
                _half_time = _goal.trajectory.points.back().time_from_start.toSec()/2.0;
                ROS_WARN_STREAM("Half time for first trajectory is : " << _half_time);

                auto it = std::find_if(_goal.trajectory.points.begin(), _goal.trajectory.points.end(), std::bind(&test_mover::bigger_half_time, this, std::placeholders::_1, _half_time));
                //auto it = std::find_if(_goal.trajectory.points.begin(), _goal.trajectory.points.end(), this->bigger_half_time);
                _the_index = distance(_goal.trajectory.points.begin(), it);

                ROS_WARN_STREAM("INDEX OF FIRST TRAJECTORY HALF TIME IS : " << _the_index);

                //find second plan
                _start_state_second_trajectory = _baxter_mover->group->getCurrentState();
                moveit::core::jointTrajPointToRobotState(_goal.trajectory, _the_index, *_start_state_second_trajectory);
                _baxter_mover->group->setStartState(*_start_state_second_trajectory);
                bool second_plan = false;
                while(!second_plan){
                        _baxter_mover->group->setRandomTarget();
                        second_plan = _baxter_mover->group->plan(_plan_2);
                    }
                _goal_2.trajectory = _plan_2.trajectory_.joint_trajectory;
                _half_time_2 = _goal_2.trajectory.points.back().time_from_start.toSec()/2.0;
                ROS_WARN_STREAM("Half time for second trajectory is : " << _half_time_2);

                it = std::find_if(_goal_2.trajectory.points.begin(), _goal_2.trajectory.points.end(), std::bind(&test_mover::bigger_half_time, this, std::placeholders::_1, _half_time_2));
                ROS_WARN_STREAM("LENGTH OF SECOND TRAJECTORY IS : " << _goal_2.trajectory.points.size());
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());


                ac.sendGoal(_goal,
                            boost::bind(&test_mover::left_done_callback, this, _1),
                            boost::bind(&test_mover::left_active_callback, this),
                            boost::bind(&test_mover::left_feedback_callback, this, _1));

                //test
                while(!_half_done && !ac.getState().isDone());

                ROS_WARN("I AM IN THE WHILE IN THE IF");

                ac.cancelAllGoals();
                ac.sendGoal(_goal_2);
                ROS_WARN("**************************************************************************");
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
            }
    private:
        ros::NodeHandle _nh;
        std::unique_ptr<ros::AsyncSpinner> _my_spinner;
        BAXTER_Mover::Ptr _baxter_mover;
        //std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> _ac_l, _ac_r;
        ros::Subscriber _joint_states_sub, _left_feedback_sub, _right_feedback_sub;
        std::vector<double> _right_joint_states, _left_joint_states;
        control_msgs::FollowJointTrajectoryGoal _goal, _goal_2, _goal_3;
        moveit::planning_interface::MoveGroup::Plan _the_plan, _plan_2, _plan_3;
        robot_state::RobotStatePtr _start_state_second_trajectory;
        double _x, _y, _z, _min_x = 0.4, _max_x = 1.0, _min_y = 0.0, _max_y = 1.0, _min_z = 0.0, _max_z = 0.4;
        double _half_time, _half_time_2;
        bool _initialized = false, _trajectory_done = false, _half_done = false;
        int _trajectory_size = 0;
        size_t _the_index;
    };

int main(int argc, char **argv)
    {
        ros::init (argc, argv, "trajectories_batching_node");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::NodeHandle nh;

        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);

        test_mover my_tester;

        while(ros::ok()){
                std::cin.ignore();
                my_tester.plan_and_execute(ac_l);
            }
        return 0;
    }