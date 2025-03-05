#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <jsoncpp/json/json.h>

class TrajectoryReaderPublisher {
public:
    // Constructor: latched publisher (the third argument "true" latches the last published message)
    TrajectoryReaderPublisher()
        : markerPub(nh.advertise<visualization_msgs::MarkerArray>("saved_trajectory_markers", 10, true))
    {
    }

    void run(const std::string &file_name) {
        // Load the JSON data
        Json::Value jsonTrajectory;
        std::ifstream file(file_name);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file: %s", file_name.c_str());
            return;
        }
        try {
            file >> jsonTrajectory;
        } catch (std::exception &e) {
            ROS_ERROR("Failed to parse JSON: %s", e.what());
            return;
        }

        // Build the MarkerArray
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // or "odom", "base_link", etc., depending on your setup
        marker.ns = "saved_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Convert all JSON points to a Marker strip
        for (const auto &jsonPoint : jsonTrajectory) {
            geometry_msgs::Point point;
            point.x = jsonPoint["x"].asDouble();
            point.y = jsonPoint["y"].asDouble();
            point.z = jsonPoint["z"].asDouble();
            marker.points.push_back(point);
        }

        markerArray.markers.push_back(marker);

        // Publish in a loop, so new subscribers can see it anytime
        ros::Rate rate(1.0); // 1 Hz
        while (ros::ok()) {
            // Update the timestamp each time
            marker.header.stamp = ros::Time::now();

            markerPub.publish(markerArray);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher markerPub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_reader_publisher");

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun <package> trajectory_reader_publisher <path/to/trajectory.json>");
        return 1;
    }

    TrajectoryReaderPublisher node;
    node.run(argv[1]);
    return 0;
}
