#include "nuslam/slammer.hpp"
#include "nuslam/ekf.hpp"
#include "turtlelib/se2d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

arma::vec sample_gaussian_noise(const arma::mat& Sigma, arma::arma_rng::seed_type seed = 0)
{
    if (Sigma.n_rows != Sigma.n_cols) {
        throw std::invalid_argument("Sigma must be square.");
    }
    if (!Sigma.is_sympd()) { // symmetric positive definite check
        throw std::invalid_argument("Sigma must be symmetric positive definite (use nearest SPD if needed).");
    }

    if (seed != 0) arma::arma_rng::set_seed(seed);

    const arma::uword n = Sigma.n_rows;

    // Standard normal z ~ N(0, I)
    arma::vec z = arma::randn<arma::vec>(n);

    // Cholesky factor: Sigma = L * L^T (lower-triangular)
    arma::mat L = arma::chol(Sigma, "lower");

    // x = L z ~ N(0, Sigma)
    return L * z;
}

class DiffDriveProcessModel : public EKFProcessModel {
    public:
    arma::vec g(const arma::vec& x, const arma::vec& u) override{
      arma::vec update_pred = arma::zeros(x.n_rows);
      arma::vec update_noise = arma::zeros(x.n_rows);
      update_noise.subvec(0, 2) = sample_gaussian_noise(Q(3));

      if(std::abs(u.at(0)) < 1e-5){
        //make an arma vec that's the same size as x but all zeros
        update_pred.at(1) = u.at(1) * std::cos(x.at(0));
        update_pred.at(2) = u.at(1) * std::sin(x.at(0));
      }else{
        update_pred.at(0) = u.at(0);
        update_pred.at(1) = u.at(1)/u.at(0) * (std::sin(x.at(0) + u.at(0)) - std::sin(x.at(0)));
        update_pred.at(2) = u.at(1)/u.at(0) * (-std::cos(x.at(0) + u.at(0)) + std::cos(x.at(0)));
      }

      return x + update_pred + update_noise;
    }
    arma::mat A(const arma::vec& x, const arma::vec& u) override{
      arma::mat A = arma::eye(x.n_rows, x.n_rows);

      if(std::abs(u.at(0)) < 1e-5){
        A.at(1, 0) = -u.at(1) * std::sin(x.at(0));
        A.at(2, 0) = u.at(1) * std::cos(x.at(0));
      }else{
        A.at(1, 0) = u.at(1)/u.at(0) * (std::cos(x.at(0) + u.at(0)) - std::cos(x.at(0)));
        A.at(2, 0) = u.at(1)/u.at(0) * (std::sin(x.at(0) + u.at(0)) - std::sin(x.at(0)));
      }

      return A;
    }

    arma::mat Q(size_t size) override{
      arma::mat Q = arma::zeros(size, size);
      Q.submat(0, 0, 2, 2) = arma::eye(3, 3) * 1/100;
      return Q;
    }
};

class CylinderMeasureModel : public EKFMeasurementModel {
  public:
    int index = 0;
    arma::vec h(const arma::vec& x) override{
      arma::vec measurement_pred = arma::zeros(2);
      double landmark_x = x.at(indexToLandmark(index));
      double landmark_y = x.at(indexToLandmark(index) + 1);
      double robot_x = x.at(0);
      double robot_y = x.at(1);
      measurement_pred.at(0) = std::sqrt(std::pow(landmark_x - robot_x, 2) + std::pow(landmark_y - robot_y, 2));
      measurement_pred.at(1) = std::atan2(landmark_y - robot_y, landmark_x - robot_x) - x.at(0);

      arma::vec full_measure = arma::zeros(x.n_rows);

      int idx = indexToLandmark(index);

      full_measure.subvec(idx, idx + 1) = measurement_pred;

      return full_measure;
    }

    arma::mat H(const arma::vec& x) override{
      arma::mat H = arma::zeros(2, x.n_rows);
      double landmark_x = x.at(indexToLandmark(index));
      double landmark_y = x.at(indexToLandmark(index) + 1);
      double robot_x = x.at(0);
      double robot_y = x.at(1);
      double dx = landmark_x - robot_x;
      double dy = landmark_y - robot_y;
      double q = dx*dx + dy*dy;

      H.at(0, 0) = -dx / std::sqrt(q);
      H.at(0, 1) = -dy / std::sqrt(q);
      H.at(0, indexToLandmark(index)) = dx / std::sqrt(q);
      H.at(0, indexToLandmark(index) + 1) = dy / std::sqrt(q);

      H.at(1, 0) = dy / q;
      H.at(1, 1) = -dx / q;
      H.at(1, indexToLandmark(index)) = -dy / q;
      H.at(1, indexToLandmark(index) + 1) = dx / q;

      return H;
    }

    arma::mat R(size_t size) override{
      int idx = indexToLandmark(index);

      arma::mat R = arma::zeros(size, size);
      R.submat(idx, idx, idx + 1, idx + 1) = arma::eye(2, 2) * 1/100;
      return R;
    }

    private:
    int indexToLandmark(int index) {
      return 3 + 2 * index;
    }
};

Slammer::Slammer() : 
Node("nuslam"),
ekf(DiffDriveEKF(std::make_unique<DiffDriveProcessModel>(), arma::vec({0, 0, 0}), arma::zeros(3, 3)))
{
    odomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Slammer::odomCallback, this, std::placeholders::_1));
    fakeInputSub = this->create_subscription<visualization_msgs::msg::MarkerArray>("fake_input", 10, std::bind(&Slammer::fakeInputCallback, this, std::placeholders::_1));
    slamMarkerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("slam_landmarks", 10);

    declare_parameter("obstacle_radius", 0.5);

    tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Slammer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert odometry message to control input
    double linear_vel = msg->twist.twist.linear.x;
    double angular_vel = msg->twist.twist.angular.z;

    arma::vec u = {angular_vel, linear_vel, 0};

    // EKF prediction step
    ekf.predict(u);

    // Compute the map->odom transform and broadcast it
    arma::vec robotPose = ekf.getRobotPose();

    turtlelib::Transform2D T_map_robot(
        turtlelib::Vector2D{robotPose(1), robotPose(2)}, robotPose(0));

    const auto& odom_pos = msg->pose.pose.position;
    tf2::Quaternion odom_q;
    tf2::fromMsg(msg->pose.pose.orientation, odom_q);
    double odom_roll, odom_pitch, odom_yaw;
    tf2::Matrix3x3(odom_q).getRPY(odom_roll, odom_pitch, odom_yaw);
    turtlelib::Transform2D T_odom_robot(
        turtlelib::Vector2D{odom_pos.x, odom_pos.y}, odom_yaw);

    turtlelib::Transform2D T_map_odom = T_map_robot * T_odom_robot.inv();

    tf2::Quaternion map_odom_q;
    map_odom_q.setRPY(0, 0, T_map_odom.rotation());

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";
    tf_msg.transform.translation.x = T_map_odom.translation().x;
    tf_msg.transform.translation.y = T_map_odom.translation().y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(map_odom_q);
    tf_broadcaster->sendTransform(tf_msg);
}

void Slammer::fakeInputCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    // Initialize LandmarkIDtoIndex if needed
    if (LandmarkIDtoIndex.size() != msg->markers.size()) {
        LandmarkIDtoIndex.assign(msg->markers.size(), -1);
    }
    
    // Iterate through markers
    for (size_t i = 0; i < msg->markers.size(); ++i) {
        const auto& marker = msg->markers[i];
        
        // Check if marker action is add/modify (action = 0)
        if (marker.action == 0) {
            // If this landmark hasn't been assigned an index yet
            if (LandmarkIDtoIndex[i] == -1) {
                // Find the lowest unoccupied integer
                int lowest_unoccupied = 0;
                while (true) {
                    bool found = false;
                    for (int val : LandmarkIDtoIndex) {
                        if (val == lowest_unoccupied) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        break;
                    }
                    lowest_unoccupied++;
                }
                
                LandmarkIDtoIndex[i] = lowest_unoccupied;
            }
            
            int landmark_index = LandmarkIDtoIndex[i];
            
            // Check if we need to extend the state
            if (static_cast<size_t>(landmark_index) + 1 > static_cast<size_t>(ekf.getNumObstacles())) {
                // Create high initial covariance for the new landmark
                arma::vec new_landmark_state = {marker.pose.position.x, marker.pose.position.y};
                arma::mat high_cov = arma::eye(2, 2) * 1e6;
                ekf.extendState(new_landmark_state, high_cov);
            }
            
            // Create measurement update with the proper index
            CylinderMeasureModel measurement_model;
            measurement_model.index = landmark_index;
            
            arma::vec measurement = {marker.scale.x, marker.scale.y};
            
            // Run EKF update with that measurement model
            ekf.update(measurement, measurement_model);

            arma::vec robotPose = ekf.getRobotPose();
        }
    }

    visualization_msgs::msg::MarkerArray slamdmarks;

    for(int i = 0; i < static_cast<int>(LandmarkIDtoIndex.size()); i++) {
        if(LandmarkIDtoIndex.at(i) != -1) {
            int obs_index = LandmarkIDtoIndex.at(i);
            arma::vec pos = ekf.getObstaclePosition(obs_index);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pos(0);
            marker.pose.position.y = pos(1);
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            double r = this->get_parameter("obstacle_radius").as_double();
            marker.scale.x = 2.0 * r;
            marker.scale.y = 2.0 * r;
            marker.scale.z = 2.0 * r;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            slamdmarks.markers.push_back(marker);
        }
    }
    slamMarkerPub->publish(slamdmarks);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}