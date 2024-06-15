#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

// Function to read goals from the CSV file
std::vector<geometry_msgs::PoseStamped> readGoals(const std::string& file_path) {
    std::vector<geometry_msgs::PoseStamped> goals;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return goals;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";

        // Read position x
        std::getline(ss, token, ',');
        goal.pose.position.x = std::stof(token);

        // Read position y
        std::getline(ss, token, ',');
        goal.pose.position.y = std::stof(token);

        goal.pose.position.z = 0;

        

        // Read orientation theta
        std::getline(ss, token, ',');
        float theta = std::stof(token);

        // orientation x
        goal.pose.orientation.x = 0;

        // orientation w
        goal.pose.orientation.w = cos(theta / 2);

        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = sin(theta / 2);

        goals.push_back(goal);
    }

    file.close();
    return goals;
}

void sendGoal(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac, const geometry_msgs::PoseStamped& goal) {
    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose = goal;

    ROS_INFO("Sending goal: [position: (%f, %f, %f), orientation: (%f, %f, %f, %f)]",
             goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
             goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

    ac.sendGoal(move_base_goal);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_goal_publisher");
    ros::NodeHandle nh;

    // Path to the CSV file (change this to your file path)
    std::string file_path = std::string(ros::package::getPath("second_project")) + "/csv/waypoints.csv";
    std::vector<geometry_msgs::PoseStamped> goals = readGoals(file_path);

    if (goals.empty()) {
        ROS_ERROR("No goals to process.");
        return 1;
    }

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for the move_base action server to come up");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    for (const auto& goal : goals) {
        sendGoal(ac, goal);

        // Wait for the result
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached.");
        } else {
            ROS_WARN("The base failed to reach the goal for some reason.");
        }
    }

    return 0;
}