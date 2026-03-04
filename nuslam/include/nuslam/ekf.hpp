#ifndef NUSLAM_EKF_HPP
#define NUSLAM_EKF_HPP

#include "armadillo"
#include <functional>
#include <memory>

class EKFProcessModel {
    /// \brief Process model for the EKF, containing the process model, its linearization, and the process noise covariance
    public:
    /// \brief The state transition function, which takes in the current state and control input and outputs the next state
    /// \param x The current state
    /// \param u The control input
    /// \return The predicted next state
    virtual arma::vec g(const arma::vec& x, const arma::vec& u) = 0;
    /// \brief The linearized state transition function, which takes in the current state and control input and outputs the Jacobian of the state transition function with respect to the state
    /// \param x The current state
    /// \return The Jacobian of the state transition function with respect to the state
    virtual arma::mat A(const arma::vec& x, const arma::vec& u) = 0; //Linearized update model
    /// \brief The covariance of the process noise.
    /// \param x The current state
    /// \param u The control input
    /// \return The covariance of the process noise
    virtual arma::mat Q(size_t size) = 0;
    virtual ~EKFProcessModel() = default;
};

class EKFMeasurementModel {
    public:
    /// \brief The measurement function, which takes in the current state and outputs the expected measurement
    /// \param x The current state
    /// \return The expected measurement
    virtual arma::vec h(const arma::vec& x) = 0; //Measurement model
    /// \brief The linearized measurement function, which takes in the current state and outputs the Jacobian of the measurement function with respect to the state
    /// \param x The current state
    /// \return The Jacobian of the measurement function with respect to the state
    virtual arma::mat H(const arma::vec& x) = 0; //Linearized measurement model
    /// \brief The covariance of the measurement noise, which can in theory depend on the current state
    /// \param x The current state
    /// \return The covariance of the measurement noise
    virtual arma::mat R(size_t size) = 0; //Measurement noise covariance
    virtual ~EKFMeasurementModel() = default;
};

class ExtendedKalmanFilter {
    public:
        ExtendedKalmanFilter() = default;
        ExtendedKalmanFilter(std::unique_ptr<EKFProcessModel> process_model, arma::vec state_estimate, arma::mat estimate_covariance);

        void predict(const arma::vec& control_input);
        void update(const arma::vec& measurement, EKFMeasurementModel& measurement_model);

        void extendState(const arma::vec& new_state, const arma::mat& new_covariance);

        arma::vec getStateEstimate() const { return state_estimate; }
        arma::mat getEstimateCovariance() const { return estimate_covariance; }

    private:
        std::unique_ptr<EKFProcessModel> process_model;

        arma::vec state_estimate;
        arma::mat estimate_covariance;

};

#endif // NUSLAM_EKF_HPP