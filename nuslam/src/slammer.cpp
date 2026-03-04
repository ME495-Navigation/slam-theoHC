#include "nuslam/slammer.hpp"
#include "nuslam/ekf.hpp"

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
      update_noise.subvec(0, 2) = sample_gaussian_noise(Q());

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
    arma::mat Q() override{
      return arma::eye(3, 3) * 1/100;
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

    private:
    int indexToLandmark(int index) {
      return 3 + 2 * index;
    }
};

Slammer::Slammer() : 
Node("nuslam")
{
    odomSub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Slammer::odomCallback, this, std::placeholders::_1));
    fakeInputSub = this->create_subscription<visualization_msgs::msg::MarkerArray>("fake_input", 10, std::bind(&Slammer::fakeInputCallback, this, std::placeholders::_1));

    ekf = ExtendedKalmanFilter(std::make_unique<DiffDriveProcessModel>(), arma::vec({0, 0, 0}), arma::eye(3, 3));

    tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Slammer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received odometry message: position=(%f, %f), orientation=(%f, %f, %f, %f)", 
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void Slammer::fakeInputCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received fake input message with %zu markers", msg->markers.size());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}