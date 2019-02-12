
#ifndef LEGGED_VIO_H
#define LEGGED_VIO_H

/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

namespace legged_vio {
/*
 * @brief MsckfVio Implements the algorithm in
 *    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
 *    "A Multi-State Constraint Kalman Filter for Vision-aided
 *    Inertial Navigation",
 *    http://www.ee.ucr.edu/~mourikis/tech_reports/TR_MSCKF.pdf
 */
class LeggedVio {
  public:

    // Constructor
    LeggedVio(ros::NodeHandle& pnh);

    // Disable copy and assign constructor
    LeggedVio(const LeggedVio&) = delete;
    LeggedVio operator=(const LeggedVio&) = delete;

    // Destructor
    ~LeggedVio() {}

    /*
     * @brief initialize Initialize the VIO.
     */
    bool initialize();

    /*
     * @brief reset Resets the VIO to initial status.
     */
    void reset();

    typedef boost::shared_ptr<LeggedVio> Ptr;
    typedef boost::shared_ptr<const LeggedVio> ConstPtr;

  private:

    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();

    /*
     * @brief createRosIO
     *    Create ros publisher and subscirbers.
     */
    bool createRosIO();

    /*
     * @brief imuCallback
     *    Callback function for the imu message.
     * @param msg IMU msg.
     */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    /*
     * @brief featureCallback
     *    Callback function for feature measurements.
     * @param msg Stereo feature measurements.
     */
    void featureCallback(const CameraMeasurementConstPtr& msg);

    /*
     * @brief publish Publish the results of VIO.
     * @param time The time stamp of output msgs.
     */
    void publish(const ros::Time& time);

    // Ros node handle
    ros::NodeHandle nh;

};

typedef LeggedVio::Ptr LeggedVioPtr;
typedef LeggedVio::ConstPtr LeggedVioConstPtr;

} // namespace legged_vio

#endif
