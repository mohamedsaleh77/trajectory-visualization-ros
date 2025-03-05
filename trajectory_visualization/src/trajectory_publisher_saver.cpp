#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_visualization/SaveTrajectory.h>
#include <fstream>
#include <jsoncpp/json/json.h>

class TrajectoryPublisherSaver {
public:
    TrajectoryPublisherSaver() : tfListener(tfBuffer) {
        markerPub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);
        saveService = nh.advertiseService("save_trajectory", &TrajectoryPublisherSaver::saveTrajectoryCallback, this);
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            updateTrajectory();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher markerPub;
    ros::ServiceServer saveService;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::vector<geometry_msgs::Point> trajectoryPoints;

    void updateTrajectory() {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            geometry_msgs::Point point;
            point.x = transformStamped.transform.translation.x;
            point.y = transformStamped.transform.translation.y;
            point.z = transformStamped.transform.translation.z;
            trajectoryPoints.push_back(point);

            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.points = trajectoryPoints;
            markerArray.markers.push_back(marker);
            markerPub.publish(markerArray);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    bool saveTrajectoryCallback(trajectory_visualization::SaveTrajectory::Request &req,
                                trajectory_visualization::SaveTrajectory::Response &res) {
        Json::Value jsonTrajectory;
        for (const auto &point : trajectoryPoints) {
            Json::Value jsonPoint;
            jsonPoint["x"] = point.x;
            jsonPoint["y"] = point.y;
            jsonPoint["z"] = point.z;
            jsonTrajectory.append(jsonPoint);
        }

        std::ofstream file(req.file_name);
        if (file.is_open()) {
            file << jsonTrajectory;
            file.close();
            res.success = true;
        } else {
            res.success = false;
        }
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_publisher_saver");
    TrajectoryPublisherSaver trajectoryPublisherSaver;
    trajectoryPublisherSaver.run();
    return 0;
}	
