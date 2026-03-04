#include "nuslam/ekf.hpp"


ExtendedKalmanFilter::ExtendedKalmanFilter(std::unique_ptr<EKFProcessModel> process_model, arma::vec state_estimate, arma::mat estimate_covariance):
    process_model(std::move(process_model)),
    state_estimate(state_estimate),
    estimate_covariance(estimate_covariance)
{}

void ExtendedKalmanFilter::predict(const arma::vec& control_input) {
    //Predict the next state using the process model
    state_estimate = process_model->g(state_estimate, control_input);
    //Predict the next covariance using the linearized process model and the process noise covariance
    estimate_covariance = process_model->A(state_estimate, control_input) * estimate_covariance * process_model->A(state_estimate, control_input).t() + 
        process_model->Q(state_estimate.n_rows);
}

void ExtendedKalmanFilter::update(const arma::vec& measurement, EKFMeasurementModel& measurement_model) {
    //Compute the Kalman gain
    arma::mat K = estimate_covariance * measurement_model.H(state_estimate).t() * 
        arma::inv(measurement_model.H(state_estimate) * estimate_covariance * measurement_model.H(state_estimate).t() + 
        measurement_model.R(state_estimate.n_rows));
    //Update the state estimate using the measurement and the Kalman gain
    state_estimate = state_estimate + K * (measurement - measurement_model.h(state_estimate));
    //Update the estimate covariance using the Kalman gain and the linearized measurement model
    estimate_covariance = (arma::eye(estimate_covariance.n_rows, estimate_covariance.n_cols) - K * measurement_model.H(state_estimate)) * estimate_covariance;
}

void ExtendedKalmanFilter::extendState(const arma::vec& new_state, const arma::mat& new_covariance) {
    //Extend the state estimate and covariance to include the new state
    state_estimate = arma::join_vert(state_estimate, new_state);
    estimate_covariance = arma::join_vert(arma::join_horiz(estimate_covariance, arma::zeros(estimate_covariance.n_rows, new_covariance.n_cols)),
        arma::join_horiz(arma::zeros(new_covariance.n_rows, estimate_covariance.n_cols), new_covariance));
}