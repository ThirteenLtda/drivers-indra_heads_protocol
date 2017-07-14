#ifndef INDRA_HEADS_REQUESTED_CONFIGURATION_HPP
#define INDRA_HEADS_REQUESTED_CONFIGURATION_HPP

#include <indra_heads_protocol/Protocol.hpp>

namespace indra_heads_protocol
{
    /** Structure that holds the current system configuration
     */
    struct RequestedConfiguration
    {
        enum ControlModes
        {
            STOP,
            SELF_TEST,
            ANGLES_RELATIVE,
            ANGLES_GEO,
            ANGULAR_VELOCITY,
            STABILIZED
        };

        /** Time of last update */
        base::Time time;

        /** The refresh rate of the PT status message */
        Rates rate_status_pt;

        /** The refresh rate of the IMU status message */
        Rates rate_status_imu;

        /** Discriminates between ship-relative and geo-relative heading modes
         * in pose control
         */
        ControlModes control_mode;

        /** Expected Yaw/Pitch/Roll pose or velocity when in angular pose or
         * velocity control modes
         */
        Eigen::Vector3d rpy;

        /** Expected lat/lon/altitude when in stabilized mode
         */
        GeoTarget lat_lon_alt;

        RequestedConfiguration()
            : rate_status_pt(RATE_DISABLED)
            , rate_status_imu(RATE_DISABLED)
            , control_mode(STOP) {}
    };
}

#endif

