/**
 * @file kalman_filter.cpp
 * @brief Implementation of the Kalman Filter class
 * 
 * This file contains the implementation of the Kalman Filter class,
 * which provides state estimation capabilities for tracking objects.
 * The filter is used for tracking objects in the UV detection system.
 */

#include "kalman_filter.h"
#include <iostream>

/**
 * @brief Default constructor
 * 
 * Initializes the Kalman Filter with default values and sets
 * the initialization flag to false.
 */
KalmanFilter::KalmanFilter() : isInitialized(false)
{
    // Initialize all matrices to zero size
    states = MatrixXd::Zero(0, 0);
    stateMatrix = MatrixXd::Zero(0, 0);
    inputMatrix = MatrixXd::Zero(0, 0);
    obsMatrix = MatrixXd::Zero(0, 0);
    uncertainty = MatrixXd::Zero(0, 0);
    processNoise = MatrixXd::Zero(0, 0);
    obsNoise = MatrixXd::Zero(0, 0);
}

/**
 * @brief Set up the Kalman Filter with initial parameters
 * 
 * Initializes the filter with the provided matrices and sets
 * the initialization flag to true.
 * 
 * @param states Initial state vector
 * @param stateMatrix State transition matrix
 * @param inputMatrix Control input matrix
 * @param obsMatrix Observation matrix
 * @param uncertainty Initial state uncertainty
 * @param processNoise Process noise covariance
 * @param obsNoise Observation noise covariance
 */
void KalmanFilter::setup(MatrixXd states,
                        MatrixXd stateMatrix,
                        MatrixXd inputMatrix,
                        MatrixXd obsMatrix,
                        MatrixXd uncertainty,
                        MatrixXd processNoise,
                        MatrixXd obsNoise)
{
    this->states = states;
    this->stateMatrix = stateMatrix;
    this->inputMatrix = inputMatrix;
    this->obsMatrix = obsMatrix;
    this->uncertainty = uncertainty;
    this->processNoise = processNoise;
    this->obsNoise = obsNoise;
    
    isInitialized = true;
}

/**
 * @brief Update the state transition matrix
 * 
 * Updates the state transition matrix, typically used when the
 * sampling time changes.
 * 
 * @param stateMatrix New state transition matrix
 */
void KalmanFilter::setStateMatrix(MatrixXd stateMatrix)
{
    this->stateMatrix = stateMatrix;
}

/**
 * @brief Perform state estimation step
 * 
 * Implements the Kalman Filter prediction and update steps:
 * 1. Predict the next state
 * 2. Update the state based on measurements
 * 
 * @param measurement Current measurement vector
 * @param controlInput Current control input vector
 */
void KalmanFilter::estimate(MatrixXd measurement, MatrixXd controlInput)
{
    if (!isInitialized)
    {
        std::cout << "Kalman Filter not initialized!" << std::endl;
        return;
    }

    // Prediction step
    MatrixXd predictedStates = stateMatrix * states + inputMatrix * controlInput;
    MatrixXd predictedUncertainty = stateMatrix * uncertainty * stateMatrix.transpose() + processNoise;

    // Update step
    MatrixXd innovation = measurement - obsMatrix * predictedStates;
    MatrixXd innovationCovariance = obsMatrix * predictedUncertainty * obsMatrix.transpose() + obsNoise;
    MatrixXd kalmanGain = predictedUncertainty * obsMatrix.transpose() * innovationCovariance.inverse();

    // Update state and uncertainty
    states = predictedStates + kalmanGain * innovation;
    uncertainty = (MatrixXd::Identity(states.rows(), states.rows()) - kalmanGain * obsMatrix) * predictedUncertainty;
}

/**
 * @brief Get the current value of a specific state
 * 
 * @param stateIndex Index of the state to retrieve
 * @return Current value of the specified state
 */
double KalmanFilter::output(int stateIndex)
{
    if (!isInitialized)
    {
        std::cout << "Kalman Filter not initialized!" << std::endl;
        return 0.0;
    }

    if (stateIndex < 0 || stateIndex >= states.rows())
    {
        std::cout << "Invalid state index!" << std::endl;
        return 0.0;
    }

    return states(stateIndex, 0);
} 