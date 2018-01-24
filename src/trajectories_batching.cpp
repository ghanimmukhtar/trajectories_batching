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

class test_mover{
    public:
        test_mover(){
                init();
            }

        void init(){
                ROS_INFO("INITIALIZING ...");
                _joint_states_sub = _nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &test_mover::joint_state_callback, this);
                _baxter_mover.reset(new BAXTER_Mover(_nh));
                _right_joint_states = std::vector<double>(7, 0);
                _left_joint_states = std::vector<double>(7, 0);
                _zero_vector = std::vector<double>(7, 0);
                _right_joint_velocity = std::vector<double> (7, 0);
                _left_joint_velocity = std::vector<double> (7, 0);

                _my_spinner.reset(new ros::AsyncSpinner(1));
                _my_spinner->start();
                ROS_INFO("INITIALIZED!!!");
                _initialized = true;
            }
        void joint_state_callback(const sensor_msgs::JointStateConstPtr& jo_state){
                if(!_initialized)
                    return;
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

                //right arm joints velocities
                _right_joint_velocity[0] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[0]))];
                _right_joint_velocity[1] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[1]))];
                _right_joint_velocity[2] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[2]))];
                _right_joint_velocity[3] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[3]))];
                _right_joint_velocity[4] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[4]))];
                _right_joint_velocity[5] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_right_arm_joints_names()[5]))];
                _right_joint_velocity[6] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
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
                //left arm joints velocities
                _left_joint_velocity[0] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[0]))];
                _left_joint_velocity[1] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[1]))];
                _left_joint_velocity[2] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[2]))];
                _left_joint_velocity[3] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[3]))];
                _left_joint_velocity[4] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[4]))];
                _left_joint_velocity[5] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[5]))];
                _left_joint_velocity[6] = jo_state->velocity[distance(jo_state->name.begin(), find(jo_state->name.begin(),
                                                                                                    jo_state->name.end(),
                                                                                                    _baxter_mover->global_parameters.get_baxter_left_arm_joints_names()[6]))];
                if(_flag_for_joint_states_cb)
                    if(largest_difference(_left_joint_velocity, _zero_vector) < 0.01)
                        ROS_ERROR("ACTUAL VELOCITY OF ZEROS");
            }

        //get largest difference between elements of two vectors
        double largest_difference(std::vector<double> &first, std::vector<double> &second){
                Eigen::VectorXd difference(first.size());
                double my_max = 0;
                for(size_t j = 0; j < first.size(); ++j)
                    difference(j) = fabs(first[j] - second[j]);
                for(size_t j = 0; j < first.size(); ++j){
                        if(difference(j) > my_max)
                            my_max = difference(j);
                    }
                return my_max;
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

        void left_feedback_callback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback, std::vector<double> target_joint_values){
                //ROS_WARN_STREAM("AT THE FEEDBACK, LEFT TIME FROM START : " << feedback->actual.time_from_start.toSec());
                //ROS_INFO_STREAM("The Half time for the trajectory Is : " << time);
                _current_left_feedback_velocities = feedback->desired.velocities;
                _current_left_feedback_accelerations = feedback->desired.accelerations;

                std::vector<double> feedback_joint_values = feedback->actual.positions;
                double diff = largest_difference(target_joint_values, feedback_joint_values);
                if(diff < 0.04){
                        ROS_INFO("TRAJECTORY HALF DONE YEAH !!!");
                        _half_done = true;
                        _flag_for_joint_states_cb = true;
                    }
                else{
                        //ROS_WARN_STREAM("TRAJECTORY HALF NOT DONE DONE, largest difference is : " << diff);
                        _half_done = false;
                        _flag_for_joint_states_cb = false;
                    }
                if(largest_difference(_current_left_feedback_velocities, _zero_vector) < 0.01)
                    ROS_WARN("DESIRED VELOCITY OF ZEROS");
                if(largest_difference(_left_joint_velocity, _zero_vector) < 0.01)
                    ROS_ERROR("ACTUAL VELOCITY OF ZEROS");
            }

        void right_feedback_callback(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr& feedback){
                ROS_WARN("AT THE FEEDBACK, RIGHT :) :) :)");
            }

        bool bigger_half_time(trajectory_msgs::JointTrajectoryPoint point, double time){
                //ROS_INFO_STREAM("This point time from start is : " << point.time_from_start.toSec());
                return point.time_from_start.toSec() > time;
            }

        void modify_trajectory(control_msgs::FollowJointTrajectoryGoal& trajectory){
                if(trajectory.trajectory.points.size() < 2){
                        ROS_ERROR("THE TRAJECTORY HAS ONLY ONE POINT, CAN'T BE PROCESSED");
                        return;
                    }
                std::vector<double> current_feedback_velocity = _current_left_feedback_velocities;
                std::vector<double> current_feedback_acceleration = _current_left_feedback_accelerations;
                std::vector<double> a_0 = trajectory.trajectory.points[0].positions;
                std::vector<double> a_1 = current_feedback_velocity;
                std::vector<double> a_2 = current_feedback_acceleration;
                std::vector<double> a_3(7, 0);

                //ROS_INFO_STREAM("INITIALIZING COEFFECIENTS, A_0 size = " << a_0.size() << " A_1 size = " << a_1.size() << " A_2 size = " << a_2.size());
                //get a_3 vector of coeffecients
                for(int i = 0; i < 7; i++){
                        a_3[i] = (trajectory.trajectory.points[1].positions[i] - a_0[i] - a_1[i] * trajectory.trajectory.points[1].time_from_start.toSec() -
                                a_2[i] * pow(trajectory.trajectory.points[1].time_from_start.toSec(), 2.0)) / pow(trajectory.trajectory.points[1].time_from_start.toSec(), 3.0);
                        //ROS_INFO_STREAM("ELEMENT : " << i << " of A_3 is : " << a_3[i]);
                    }



                for(size_t i = 0; i < trajectory.trajectory.points.size(); i++){
                        for(size_t j = 0; j < trajectory.trajectory.points[i].velocities.size(); j++){
                                //modify velocities
                                trajectory.trajectory.points[i].velocities[j] = a_1[j] +
                                        2 * a_2[j] * trajectory.trajectory.points[i].time_from_start.toSec() +
                                        3 * a_3[j] * pow(trajectory.trajectory.points[i].time_from_start.toSec() , 2.0);

                                //modify accelerations
                                trajectory.trajectory.points[i].accelerations[j] = 2 * a_2[j] +
                                        6 * a_3[j] * trajectory.trajectory.points[i].time_from_start.toSec();
                            }
                        //ROS_INFO_STREAM("MODIFYING ELEMENT : " << i << " of trajectory with : " << trajectory.trajectory.points.size() << " POINTS");
                    }
            }

        void plan_and_execute(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac){
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());
                _trajectory_done = false;
                _half_done = false;
                _trajectory_size = 0;

//                _x = (_max_x - _min_x) * ((double)rand() / (double)RAND_MAX) + _min_x;
//                _y = (_max_y - _min_y) * ((double)rand() / (double)RAND_MAX) + _min_y;
//                _z = (_max_z - _min_z) * ((double)rand() / (double)RAND_MAX) + _min_z;

                geometry_msgs::Pose the_point;
                the_point.position.x = 0.65;
                the_point.position.y = 0.6;
                the_point.position.z = 0.25;
                the_point.orientation.w = 0.0;
                the_point.orientation.x = 0.0;
                the_point.orientation.y = 1.0;
                the_point.orientation.z = 0.0;

                _baxter_mover->group->setPoseTarget(the_point);
//                _baxter_mover->group->setPositionTarget(0.65, 0.6, 0.25);
                _baxter_mover->group->plan(_the_plan);
                _baxter_mover->group->execute(_the_plan);

                _waypoints.clear();
                _waypoints.push_back(the_point);
                //geometry_msgs::Pose the_point;
                the_point.position.y = 0.4;

                _waypoints.push_back(the_point);

                double fraction_0 = _baxter_mover->group->computeCartesianPath(_waypoints, 0.01, 0.0, _robot_trajectory_0);
                ROS_WARN_STREAM("fraction solved of desired path 0 in this trial is: " <<
                                fraction_0);

                if(!ac.waitForServer(ros::Duration(2.0)))
                    {
                        ROS_ERROR("Could not connect to action server");
                        return ;
                    }

                //_trajectory_size = _the_plan.trajectory_.joint_trajectory.points.size();
                //_goal.trajectory = _the_plan.trajectory_.joint_trajectory;
                _trajectory_size = _robot_trajectory_0.joint_trajectory.points.size();
                _goal.trajectory = _robot_trajectory_0.joint_trajectory;
                _half_time = _goal.trajectory.points.back().time_from_start.toSec()/2.0;
                //ROS_WARN_STREAM("Half time for first trajectory is : " << _half_time);

                auto it = std::find_if(_goal.trajectory.points.begin(), _goal.trajectory.points.end(), std::bind(&test_mover::bigger_half_time, this, std::placeholders::_1, _half_time));
                _the_index = distance(_goal.trajectory.points.begin(), it);

                //ROS_WARN_STREAM("INDEX OF FIRST TRAJECTORY HALF TIME IS : " << _the_index);

                _left_target_joint_values = _goal.trajectory.points[_the_index].positions;
                //find second plan
                _start_state_second_trajectory = _baxter_mover->group->getCurrentState();
                moveit::core::jointTrajPointToRobotState(_goal.trajectory, _the_index, *_start_state_second_trajectory);
                _baxter_mover->group->setStartState(*_start_state_second_trajectory);
                bool second_plan = false;
                while(!second_plan){
                        //_baxter_mover->group->setRandomTarget();
                        //second_plan = _baxter_mover->group->plan(_plan_2);
                        _waypoints.clear();
                        the_point.position.y -= 0.2;
                        _waypoints.push_back(the_point);
                        double fraction_1 = _baxter_mover->group->computeCartesianPath(_waypoints, 0.01, 0.0, _robot_trajectory_1);
                        ROS_WARN_STREAM("fraction solved of desired path 1 in this trial is: " <<
                                        fraction_1);
                        if(fraction_1 == 1)
                            second_plan = true;
                    }
                //_goal_2.trajectory = _plan_2.trajectory_.joint_trajectory;
                _goal_2.trajectory = _robot_trajectory_1.joint_trajectory;
                //_half_time_2 = _goal_2.trajectory.points.back().time_from_start.toSec()/2.0;
                //ROS_WARN_STREAM("Half time for second trajectory is : " << _half_time_2);

                it = std::find_if(_goal_2.trajectory.points.begin(), _goal_2.trajectory.points.end(), std::bind(&test_mover::bigger_half_time, this, std::placeholders::_1, _half_time_2));
                _index_2 = distance(_goal_2.trajectory.points.begin(), it);

                //ROS_WARN_STREAM("INDEX OF SECOND TRAJECTORY HALF TIME IS : " << _index_2);

                //find third plan
                _start_state_third_trajectory = _baxter_mover->group->getCurrentState();
                moveit::core::jointTrajPointToRobotState(_goal_2.trajectory, _index_2, *_start_state_third_trajectory);
                _baxter_mover->group->setStartState(*_start_state_third_trajectory);
                bool third_plan = false;
                while(!third_plan){
                        //_baxter_mover->group->setRandomTarget();
                        //third_plan = _baxter_mover->group->plan(_plan_3);
                        _waypoints.clear();
                        the_point.position.y -= 0.2;
                        _waypoints.push_back(the_point);
                        double fraction_2 = _baxter_mover->group->computeCartesianPath(_waypoints, 0.01, 0.0, _robot_trajectory_2);
                        ROS_WARN_STREAM("fraction solved of desired path 2 in this trial is: " <<
                                        fraction_2);
                        if(fraction_2 == 1)
                            third_plan = true;
                    }
                //_goal_3.trajectory = _plan_3.trajectory_.joint_trajectory;
                _goal_3.trajectory = _robot_trajectory_2.joint_trajectory;

                //execute those trajectories, half of first and second ones and the whole third one
                _baxter_mover->group->setStartState(*_baxter_mover->group->getCurrentState());

                ac.sendGoal(_goal,
                            boost::bind(&test_mover::left_done_callback, this, _1),
                            boost::bind(&test_mover::left_active_callback, this),
                            boost::bind(&test_mover::left_feedback_callback, this, _1, _left_target_joint_values));


                while(!_half_done && !ac.getState().isDone());
                ROS_WARN("I AM IN THE WHILE IN THE IF 111111111111111");
                modify_trajectory(_goal_2);
                _half_done = false;

                _left_target_joint_values = _goal_2.trajectory.points[_index_2].positions;
                //ac.cancelAllGoals();
                ac.sendGoal(_goal_2,
                            boost::bind(&test_mover::left_done_callback, this, _1),
                            boost::bind(&test_mover::left_active_callback, this),
                            boost::bind(&test_mover::left_feedback_callback, this, _1, _left_target_joint_values));

                while(!_half_done && !ac.getState().isDone());
                ROS_WARN("I AM IN THE WHILE IN THE IF 222222222222222");

                modify_trajectory(_goal_3);
                //ac.cancelAllGoals();
                ac.sendGoalAndWait(_goal_3);

                _flag_for_joint_states_cb = false;
                ROS_WARN("I AM IN THE WHILE IN THE IF 333333333333");
                ROS_WARN("**************************************************************************");
            }
    private:
        ros::NodeHandle _nh;
        std::unique_ptr<ros::AsyncSpinner> _my_spinner;
        BAXTER_Mover::Ptr _baxter_mover;
        ros::Subscriber _joint_states_sub, _left_feedback_sub, _right_feedback_sub;
        std::vector<double> _right_joint_states, _left_joint_states, _left_target_joint_values, _right_target_joint_values, _right_joint_velocity, _left_joint_velocity;
        control_msgs::FollowJointTrajectoryGoal _goal, _goal_2, _goal_3;
        std::vector<double> _current_left_feedback_velocities, _current_left_feedback_accelerations, _zero_vector;
        moveit::planning_interface::MoveGroup::Plan _the_plan, _plan_2, _plan_3;
        moveit_msgs::RobotTrajectory _robot_trajectory_0, _robot_trajectory_1, _robot_trajectory_2;
        robot_state::RobotStatePtr _start_state_second_trajectory, _start_state_third_trajectory;
        double _x, _y, _z, _min_x = 0.4, _max_x = 1.0, _min_y = 0.0, _max_y = 1.0, _min_z = 0.0, _max_z = 0.4;
        double _half_time, _half_time_2;
        bool _initialized = false, _trajectory_done = false, _half_done = false, _flag_for_joint_states_cb = false;
        int _trajectory_size = 0;
        size_t _the_index, _index_2;
        std::vector<geometry_msgs::Pose> _waypoints;
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
