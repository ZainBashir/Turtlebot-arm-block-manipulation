/*
 * Copyright (c) 2011, Willow Garage, Inc.
 *
 * Modified by Yamid Espinel, Zain Bashir, Solène Guillaume - 2018
 *
 * All rights reserved.
 *
 * Distribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Distributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Distributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



/*
*   WARNING  **************
*
*    1) Follow safety guidelines at http://www.usfirst.org/sites/default/files/uploadedFiles/Robotics_Programs/FRC/Resources/2015%20FRC%20Team%20Safety%20Manual-%20FINAL%202.6.15.pdf
*    2) Follow manufacturer guidelines 
*    3) Set the arm to a very slow speed
*    4) Always be ready to emergency stop the arm
*    5) MOUNT THE ARM USING break-away mounts such as small spring based clips 
*    6) Review and understand the code
*
*  REQUIREMENTS ************
*
*  0. ROS Indigo
*  1. Kinect and PhantomX Pincher Arm
*  2. Turtlebot arm version https://github.com/answer17/turtlebot_arm
*  3. Arbotix ROS version https://github.com/answer17/arbotix_ros
*  4. Calibration as outlined in block_detection_action_server.cpp
*
*  USAGE ***************
*
*  Launch block_manipulation_demo.launch
*  When D is pressed, it will detect blocks at table height with size
*  specified.  The blocks will be shown in Rviz
*  When M is pressed, those blocks will be sorted according to color and moved to 
*   destination
*
*  SETUP ****************
* 
*     Kinect position: Y is appx 450mm from arm base and appx 450mm above table
*                      X is appx 170 from arm base
*                       
*      This gives Kinect best view of blocks without arm obstruction and is in
*      sweet spot of Kinect accuracy
*
*      blocks X position is from 12cm from arm base to 24cm
*
*
*  DIAGRAM (from above) *****************
*
*              /KINECT\     
*
*             [blocks]
    Y
*   ^   [base]-----<grip
*   |   
*   |         [destination (wrong color)]
    |         [destination (target color)]
*   ----->X
*
*
* When you have read the above statements
* remove the space between the asterisk and slash below:
*/



#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <tf/tf.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>
#include <turtlebot_arm_block_manipulation/PickAndPlaceAction.h>
#include <turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction.h>
#include <arbotix_msgs/Relax.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>

#include <string>
#include <sstream>

const std::string gripper_param = "/gripper_controller";
const std::string pick_and_place_topic = "/pick_and_place";

// Flag to indicate block matched our target color:
#define COLOR_MATCH 0.002

namespace turtlebot_arm_block_manipulation
{

class BlockManipulationAction
{
private:
  // Object to communicate with the ROS system:
  ros::NodeHandle nh_;
 
  geometry_msgs::PoseArray poseMsg;
  geometry_msgs::Pose start_pose_bumped, end_pose_bumped;
  
  // Actions and services
  actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;
  actionlib::SimpleActionClient<PickAndPlaceAction> pick_and_place_action_;

  BlockDetectionGoal block_detection_goal_;
  InteractiveBlockManipulationGoal interactive_manipulation_goal_;
  PickAndPlaceGoal pick_and_place_goal_;

  // Parameters
  std::string arm_link;
  double gripper_open, gripper_tighten, gripper_closed, z_up, z_down, block_size, target_x, target_y;
  bool once;

  
  int blockIndex;  // block we are working on
  int blockCount;  // number of blocks found

public:

  bool ppcalled = false;
      geometry_msgs::Pose blockList[20];  // List of the positions of blocks we;ve found
      geometry_msgs::Pose armUpPose; // Pose of the robot in vertical way (used as initial pose)

      ros::Subscriber vsdone_subscriber; // Subscriber used to listen to the status message sent by the Visual Servoing module
      ros::Publisher  ppdone_publisher; // Published for sending completion signal to the Navigation module

    std_msgs::String robot_msg; // Variable used to store the status message sent by the robot
    std::string armMode; // Current status of the arm (initial position, moving, final position)
    
  // Class constructor:
  BlockManipulationAction() : nh_("~"),
    block_detection_action_("block_detection", true),
    interactive_manipulation_action_("interactive_manipulation", true),
    pick_and_place_action_("pick_and_place", true)
  {

    // Subscribe to the topic through which the robot sends the status message
    vsdone_subscriber = nh_.subscribe("/relay/robot_status", 1, &BlockManipulationAction::robotMessage, this);

    // Publish to the topic through which the robot receives pick and place completion message
    ppdone_publisher = nh_.advertise< std_msgs::String >("/arm_status", 1, true);

    // Load parameters
    nh_.param<std::string>("arm_link", arm_link, "/arm_link"); // Reference frame for object localization
    nh_.param<double>(gripper_param + "/max_opening", gripper_open, 0.042); // Gripper open position
    nh_.param<double>("grip_tighten", gripper_tighten, -0.0015); // Gripper closed position
    nh_.param<double>("z_up", z_up, 0.12);   // Arm up position
    nh_.param<double>("table_height", z_down, 0.01); // Table height
    nh_.param<double>("block_size", block_size, 0.03);  // Block size to detect
    nh_.param<bool>("once", once, false);
    nh_.param<double>("target_x", target_x, .26);    // X target for first block
    nh_.param<double>("target_y", target_y, -.06);   // Y target for first block 

    // Initialize goals
    block_detection_goal_.frame = arm_link; // Set the block detection reference frame to be the arm's base
    block_detection_goal_.table_height = z_down; // Set table height
    block_detection_goal_.block_size = block_size; // Set block's size
    
    pick_and_place_goal_.frame = arm_link; // Set the pick and place algorithm reference frame to be the arm's base
    pick_and_place_goal_.z_up = z_up; // Set arm's up position
    pick_and_place_goal_.gripper_open = gripper_open; // Set gripper opened's position in meters
    //pick_and_place_goal_.gripper_closed = block_size - 0.01 - gripper_tighten;
    pick_and_place_goal_.gripper_closed = block_size  - gripper_tighten; // Set gripper closed's position in meters
    pick_and_place_goal_.topic = pick_and_place_topic; // Set pick and place algorithm's topic
   
    
    ROS_INFO("Gripper settings: closed=%.4f block size=%.4f tighten=%.4f", (float) pick_and_place_goal_.gripper_closed, (float) block_size, (float) gripper_tighten );
    
    interactive_manipulation_goal_.block_size = block_size; // Send size of the block to the interactive manipulation algorithm
    interactive_manipulation_goal_.frame = arm_link; // Set the interactive manipulation algorithm to have as reference frame the arm's base
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    block_detection_action_.waitForServer();
    ROS_INFO(" 1. Found block_detection server.");
    
    interactive_manipulation_action_.waitForServer();
    ROS_INFO(" 2. Found interactive_manipulation server.");
    
    pick_and_place_action_.waitForServer();
    ROS_INFO(" 3. Found pick_and_place server.");
    
  }

    // Callback function that stores the message received from the navigation algorithm:
    void robotMessage(const std_msgs::String& msg){
        robot_msg.data = msg.data;
        }

  // Callback function in charge of executing the block detection algorithm:
  void detectBlocks()
  {
  // Have Block Detection Server detect blocks and callback "addBlocks" when done
    block_detection_action_.sendGoal(block_detection_goal_, boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
  }

  // Function in charge of gathering the detected blocks from the detection algorithm and sending them to the interactive manipulation algorithm
  void addBlocks(const actionlib::SimpleClientGoalState& state, const BlockDetectionResultConstPtr& result)
  {
    //ROS_INFO(" Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    
    //Check if the block detection algorithm has failed or not:
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    // Save blocks for later use during sorting
       for (unsigned int i=0; i < result->blocks.poses.size(); i++)
    {    
      blockList[i] = result->blocks.poses[i];

      // TODO Fix kluge - Z POSITION WAS OVERWRITTEN TO PASS COLOR!!
      // TODO table height is hardcoded, and restored here
      blockList[i].position.z = 0.125; //* blockList[i].position.x -0.085;
      
      //ROS_INFO("   Saving block %d x=%f", i, blockList[i].position.x);
    }
    
    blockCount = result->blocks.poses.size();
    blockIndex = 0;
    
    // Add blocks to Interactive Manipulation Server for Rviz visualization and perform pick and place of the detected block:
    //ROS_INFO("Adding blocks...");
    interactive_manipulation_action_.sendGoal(interactive_manipulation_goal_, boost::bind( &BlockManipulationAction::pickAndPlace, this, _1, _2));

    ppcalled = true;

  }
  
  // Callback function for performing pick and place of the block that has been detected
  void pickAndPlace(const actionlib::SimpleClientGoalState& state, const InteractiveBlockManipulationResultConstPtr& result)
  {
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("  Select Marker Failed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    
    //ROS_INFO(" Got interactive marker callback.");
    ROS_INFO("Picking and placing...");

    // Set the arm state to moving:
    armMode="m";

    //blockList = [];
    //memset(blockList, 0x00, 20*sizeof(int));
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
  }
  
  // Callback function activated after the arm has finished putting the cube on the robot
  void finish(const actionlib::SimpleClientGoalState& state, const PickAndPlaceResultConstPtr& result)
  {
    //Check status of the pick and place algorithm:
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO(" Pick and place - Succeeded!");
    else
      ROS_INFO(" Pick and place - Failed! %s",  state.toString().c_str());

    // If the "once" parameter is set, stop ROS:
    if (once)
      ros::shutdown();
            
   //moveBlock(armUpPose,armUpPose);
      
   // Set arm state to finished mode:
   armMode="i";

  }
  
  // Function to move the arm to a desired pose in the world frame:
  void moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
  // Amount (in meters) to bump the result by in the z dimension.
  double bump_size = 0.005;
  
    // Set arm status to moving
    armMode="m";
      
    // Return pickup and place poses as the result of the action
    start_pose_bumped = start_pose;
    start_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    end_pose_bumped = end_pose;
    end_pose_bumped.position.z -= block_size/2.0 - bump_size;
    
    // Publish pickup and place poses for visualizing on RViz
    
    /*poseMsg.header.frame_id = arm_link;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.poses.push_back(start_pose_bumped);
    poseMsg.poses.push_back(end_pose_bumped);*/
    //ROS_INFO("Demo publishing to PickNPlace.  PoseX=%.4f", (float) poseMsg.poses[0].position.x);
    
    pick_and_place_goal_.pickup_pose = start_pose_bumped;
    pick_and_place_goal_.place_pose = end_pose_bumped;
    pick_and_place_goal_.topic = "";
    
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
    
    pick_and_place_goal_.topic = pick_and_place_topic;  // restore topic
   }


 };
};


int main(int argc, char** argv)
{

  // Initialize pick and place completion message:
  std_msgs::String ppdoneMsg;
  ppdoneMsg.data = "ppdone";
  // initialize node
  ros::init(argc, argv, "block_manipulation");
  turtlebot_arm_block_manipulation::BlockManipulationAction manip;

  // everything is done in cloud callback, just spin
  ros::AsyncSpinner spinner(2);
  spinner.start();

   // manip.armMode="i"

    ros::Duration(2).sleep(); // sleep for 2 second

    // Make the arm move to a straight up position:
    manip.armUpPose.position.x = 0.00771822780371;
    manip.armUpPose.position.y = 0.00868746638298;
    manip.armUpPose.position.z = 0.354724377394;
    manip.armUpPose.orientation.w = 1;
    manip.armUpPose.orientation.x = 0;
    manip.armUpPose.orientation.y = 1;
    manip.armUpPose.orientation.z = 0;
    manip.moveBlock(manip.armUpPose,manip.armUpPose);
    //manip.blockList = [];
    
    // Reset vector of blocks:
    memset(manip.blockList, 0x00, 20*sizeof(int));

  // Run loop at a frequency of 4 Hz:
  ros::Rate r(0.25);

  while (ros::ok())
  {
    // If the arm is not moving already and there hasn't been any pick and place:
    if (manip.armMode!="m" && manip.ppcalled==false) {

        std::cout << "  WAITING FOR ROBOT MESSAGE!!!!!!" << std::endl;
        /*std::string instr;
        getline (std::cin, instr );

        if (instr == "d") {manip.detectBlocks(); manip.armMode="d";}*/
    
        // Wait for Visual Servoing completion message
        if (manip.robot_msg.data == "vsdone") {
            //If message is received, perform block detection and pick and place:
            std::cout << " MESSAGE RECEIVED XDDDDDDDDDDDDDDDDD" << std::endl;
            manip.robot_msg.data = "";
            manip.detectBlocks(); manip.armMode="d";
        }

    } else if (manip.armMode=="i" && manip.ppcalled==true) {
        // If pick and place has been finished, send completion mesage back to the robot:
        std::cout << " Sending pick and place finished message..." << std::endl;
        manip.ppdone_publisher.publish(ppdoneMsg);
    }
   r.sleep();
  }

  spinner.stop();
}
