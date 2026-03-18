#include "nuslam/ekf.hpp"
#include "turtlelib/angle.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(std::unique_ptr<EKFProcessModel> process_model, arma::vec state_estimate, arma::mat estimate_covariance):
    process_model(std::move(process_model)),
    state_estimate(state_estimate),
    estimate_covariance(estimate_covariance)
{}

void ExtendedKalmanFilter::predict(const arma::vec& control_input) {
    arma::mat A = process_model->A(state_estimate, control_input);
    //Predict the next state using the process model
    state_estimate = process_model->g(state_estimate, control_input);
    //Predict the next covariance using the linearized process model and the process noise covariance
    estimate_covariance = A * estimate_covariance * A.t() + 
        process_model->Q(state_estimate.n_rows);
}

void ExtendedKalmanFilter::update(const arma::vec& measurement, EKFMeasurementModel& measurement_model) {
    //Compute the Kalman gain
    arma::mat H = measurement_model.H(state_estimate);
    arma::mat K = estimate_covariance * H.t() * 
        arma::inv(H * estimate_covariance * H.t() + 
        measurement_model.R());
    //Update the state estimate using the measurement and the Kalman gain
    arma::vec innovation = measurement - measurement_model.h(state_estimate);
    innovation = normalizeInnovation(innovation);
    state_estimate = state_estimate + K * innovation;
    state_estimate = normalizeState(state_estimate);
    //Update the estimate covariance using the Kalman gain and the linearized measurement model
    estimate_covariance = (arma::eye(estimate_covariance.n_rows, estimate_covariance.n_cols) - K * H) * estimate_covariance;
}

void ExtendedKalmanFilter::extendState(const arma::vec& new_state, const arma::mat& new_covariance) {
    //Extend the state estimate and covariance to include the new state
    state_estimate = arma::join_vert(state_estimate, new_state);
    estimate_covariance = arma::join_vert(arma::join_horiz(estimate_covariance, arma::zeros(estimate_covariance.n_rows, new_covariance.n_cols)),
        arma::join_horiz(arma::zeros(new_covariance.n_rows, estimate_covariance.n_cols), new_covariance));
}

DiffDriveEKF::DiffDriveEKF(std::unique_ptr<EKFProcessModel> process_model, arma::vec state_estimate, arma::mat estimate_covariance) :
    ExtendedKalmanFilter(std::move(process_model), state_estimate, estimate_covariance)
{}

void DiffDriveEKF::extendStateWithObstacle(const arma::vec& new_state, const arma::mat& new_covariance) {
    ExtendedKalmanFilter::extendState(new_state, new_covariance);
    num_obstacles++;
}

arma::vec DiffDriveEKF::normalizeInnovation(arma::vec innovaton) {
    //Wrap the angle in the innovation to [-pi, pi]
    innovaton.at(1) = turtlelib::normalize_angle(innovaton.at(1));
    return innovaton;
}

arma::vec DiffDriveEKF::normalizeState(arma::vec state){
    //Wrap the angle in the robot pose to [-pi, pi]
    state.at(0) = turtlelib::normalize_angle(state.at(0));
    return state;
}