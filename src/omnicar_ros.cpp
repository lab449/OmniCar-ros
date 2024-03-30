#include "omnicar_ros.h"

OmnicarRos::OmnicarRos()
{

    ros::param::param<std::string>("ns", robotNamespace, "");
    ros::param::param<std::string>("serial_port", serialPort, "/dev/ttyUSB0");
    ros::param::param<int>("control_freq_update", controlFreqUpdate, 50);
    ros::param::param<std::string>("cmd_vel_topic", cmdVelTopic, "cmd_vel");
    ros::param::param<std::string>("odometry_topic", cmdVelTopic, "odom");
    ros::param::param<std::vector<std::string>>("robot_joint_names", robotJointNames, {"fl_caster_wheel", "fr_caster_wheel", "rl_caster_wheel", "rr_caster_wheel"});
    // if (ros::param::has("robot_joint_names")){

    // }

    jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 2);
    odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 2);
    cmdVelSubscriber = n.subscribe(cmdVelTopic, 2, &OmnicarRos::controlLinearVelocityCallback, this);
    loopRatePtr = std::make_shared<ros::Rate>(controlFreqUpdate);

    serialCommunicationPtr = std::make_shared<SerialCommunication>(serialPort);
}
OmnicarRos::~OmnicarRos()
{
    delete jointStatesPublisher;
    delete odomPublisher;
    delete cmdVelSubscriber;
    loopRatePtr.reset();
    serialCommunicationPtr->stop();
    serialCommunicationPtr.reset();

    ROS_INFO("Communication deactivated!");
}

void OmnicarRos::publishRobotState(){
    rosJointState.header.stamp = currentTime;
    rosJointState.name = robotJointNames;
    // Convert types
    rosJointState.position.assign(
        serialCommunicationPtr->robotState.jointsPosition.data(), 
        serialCommunicationPtr->robotState.jointsPosition.data()+serialCommunicationPtr->robotState.jointsPosition.rows()
    );
    rosJointState.velocity.assign(
        serialCommunicationPtr->robotState.jointsVelocity.data(),
        serialCommunicationPtr->robotState.jointsVelocity.data()+serialCommunicationPtr->robotState.jointsVelocity.rows()
    );
    
    jointStatesPublisher.publish(rosJointState);
    jointStatesPublisher.publish(rosJointState);
}

void OmnicarRos::publishOdom()
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    Eigen::Vector3f receivedOdom = serialCommunicationPtr->robotState.odomPose;
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(receivedOdom[0]);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = currentTime;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base_link";

    odomTrans.transform.translation.x = receivedOdom[1];
    odomTrans.transform.translation.y = receivedOdom[2];
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;

    //send the transform
    odomBroadcaster.sendTransform(odomTrans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = receivedOdom[1];
    odom.pose.pose.position.y = receivedOdom[2];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    //publish the message
    odomPublisher.publish(odom);
}
void OmnicarRos::controlLinearVelocityCallback(const geometry_msgs::Twist& msg){
    serialCommunicationPtr->linearVelocityControl<<msg.angular.z, msg.linear.x, msg.linear.y;
}


void OmnicarRos::initialize(){
    ROS_INFO("Trying to communicate...");
    serialCommunicationPtr->run();
    while(!isConnectionEnstablished()){
        loop();
    }
    ROS_INFO("Communication established!");
}

void OmnicarRos::loop(){
    ros::spinOnce();
    currentTime = ros::Time::now();
    publishRobotState();
    publishOdom();
    loopRatePtr->sleep();
}

bool OmnicarRos::isConnectionEnstablished(){
    return serialCommunicationPtr->isConnectionEnstablished();
}


