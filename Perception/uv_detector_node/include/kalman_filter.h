#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

/**
 * @brief Implementation of a Kalman Filter
 * 
 * This class implements a standard Kalman Filter for state estimation.
 * It supports:
 * - State prediction and update
 * - Process and measurement noise handling
 * - Dynamic system modeling
 * - State observation and control input
 */
class KalmanFilter
{
    private:
    // members
    bool isInitialized;     ///< Flag indicating if the filter is initialized
    MatrixXd states;        ///< Current state vector
    MatrixXd stateMatrix;   ///< State transition matrix (A)
    MatrixXd inputMatrix;   ///< Control input matrix (B)
    MatrixXd obsMatrix;     ///< Observation matrix (H)
    MatrixXd uncertainty;   ///< State uncertainty covariance (P)
    MatrixXd processNoise;  ///< Process noise covariance (Q)
    MatrixXd obsNoise;      ///< Observation noise covariance (R)

    public:
    /**
     * @brief Default constructor
     */
    KalmanFilter();

    /**
     * @brief Set up the Kalman Filter with initial parameters
     * @param states Initial state vector
     * @param stateMatrix State transition matrix
     * @param inputMatrix Control input matrix
     * @param obsMatrix Observation matrix
     * @param uncertainty Initial state uncertainty
     * @param processNoise Process noise covariance
     * @param obsNoise Observation noise covariance
     */
    void setup(MatrixXd states,
                MatrixXd stateMatrix,
                MatrixXd inputMatrix,
                MatrixXd obsMatrix,
                MatrixXd uncertainty,
                MatrixXd processNoise,
                MatrixXd obsNoise);

    /**
     * @brief Update the state transition matrix
     * @param stateMatrix New state transition matrix
     */
    void setStateMatrix(MatrixXd stateMatrix);

    /**
     * @brief Perform state estimation step
     * @param measurement Current measurement vector
     * @param controlInput Current control input vector
     */
    void estimate(MatrixXd measurement, MatrixXd controlInput);

    /**
     * @brief Get the current value of a specific state
     * @param stateIndex Index of the state to retrieve
     * @return Current value of the specified state
     */
    double getState(int stateIndex);
};

#endif