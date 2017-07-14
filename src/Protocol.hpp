#ifndef INDRA_HEADS_PROTOCOL_HPP
#define INDRA_HEADS_PROTOCOL_HPP

#include <cstdint>
#include <Eigen/Core>
#include <arpa/inet.h>
#include <base/Float.hpp>

namespace indra_heads_protocol
{
    enum CommandIDs {
        ID_DEPLOY = 0,
        ID_BITE   = 1,
        ID_STATUS_REFRESH_RATE_PT = 2,
        ID_STATUS_REFRESH_RATE_IMU = 3,
        ID_ANGLES_RELATIVE = 4,
        ID_ANGLES_GEO      = 5,
        ID_ANGULAR_VELOCITIES = 6,
        ID_ENABLE_STABILIZATION = 7,
        ID_STABILIZATION_TARGET = 8,
        ID_LAST = 8
    };

    typedef std::int16_t crc_t;
    static const int MIN_PACKET_SIZE = 2 + sizeof(crc_t);

    enum MessageTypes {
        MSG_REQUEST = 0,
        MSG_RESPONSE = 1,
        MSG_LAST_TYPE = 1
    };

    enum ResponseStatus {
        STATUS_OK = 0,
        STATUS_FAILED = 1,
        STATUS_UNSUPPORTED = 2
    };

    enum Rates {
        RATE_DISABLED = 0,
        RATE_10HZ = 1,
        RATE_20HZ = 2,
        RATE_50HZ = 3
    };

    struct GeoTarget {
        double latitude;
        double longitude;
        double altitude;

        GeoTarget()
            : latitude(base::unknown<double>())
            , longitude(base::unknown<double>())
            , altitude(base::unknown<double>()) {}
        GeoTarget(double latitude, double longitude, double altitude)
            : latitude(latitude)
            , longitude(longitude)
            , altitude(altitude) {}
    };

    namespace details {
        crc_t    compute_crc(uint8_t const* buffer, uint32_t size);

        uint16_t encode_angle(double angle);
        uint8_t  encode_angular_velocity(double velocity);
        uint32_t encode_latlon(double angle);
        uint32_t encode_altitude(double altitude);

        double decode_angle(uint16_t angle);
        double decode_angular_velocity(uint8_t velocity);
        double decode_latlon(uint32_t angle);
        double decode_altitude(uint16_t altitude);
    }

    namespace packets {
        int getPacketSize(CommandIDs command_id, MessageTypes message_type);

        struct SimpleMessage
        {
            uint8_t command_id;
            uint8_t message_type = MSG_REQUEST;

            SimpleMessage(CommandIDs command)
                : command_id(command) {}
        } __attribute__((packed));

        struct StatusRefreshRate
        {
            uint8_t command_id;
            uint8_t message_type = MSG_REQUEST;
            uint16_t rate;

            StatusRefreshRate(CommandIDs command, Rates rate)
                : command_id(command)
                , rate(htons(rate)) {}

        } __attribute__((packed));

        struct Angles
        {
            uint8_t command_id;
            uint8_t message_type = MSG_REQUEST;
            uint16_t yaw;
            uint16_t roll;
            uint16_t pitch;

            Angles(CommandIDs command, float yaw, float pitch, float roll)
                : command_id(command)
                , yaw(details::encode_angle(yaw))
                , roll(details::encode_angle(roll))
                , pitch(details::encode_angle(pitch)) {}
        } __attribute__((packed));

        struct AngularVelocities
        {
            uint8_t command_id = ID_ANGULAR_VELOCITIES;
            uint8_t message_type = MSG_REQUEST;
            uint8_t yaw;
            uint8_t pitch;
            uint8_t padding = 0;
            uint8_t roll;

            AngularVelocities(float yaw, float pitch, float roll)
                : yaw(details::encode_angular_velocity(yaw))
                , pitch(details::encode_angular_velocity(pitch))
                , roll(details::encode_angular_velocity(roll)) {}
        } __attribute__((packed));

        struct EnableStabilization
        {
            uint8_t command_id = ID_ENABLE_STABILIZATION;
            uint8_t message_type = MSG_REQUEST;
            uint8_t yaw;
            uint8_t pitch;
            uint8_t padding = 0;
            uint8_t roll;

            EnableStabilization(bool yaw, bool pitch, bool roll)
                : yaw(yaw ? 1 : 0)
                , pitch(pitch ? 1 : 0)
                , roll(roll ? 1 : 0) {}
        } __attribute__((packed));

        struct StabilizationTarget
        {
            uint8_t command_id = ID_STABILIZATION_TARGET;
            uint8_t message_type = MSG_REQUEST;
            uint8_t  latitude_sign;
            uint8_t  padding0 = 0;
            uint32_t latitude;
            uint8_t  longitude_sign;
            uint8_t  padding1 = 0;
            uint32_t longitude;
            uint16_t altitude;

            StabilizationTarget(float latitude, float longitude, float altitude)
                : latitude_sign(latitude > 0 ? 1 : 0)
                , latitude(details::encode_latlon(latitude))
                , longitude_sign(longitude > 0 ? 1 : 0)
                , longitude(details::encode_latlon(longitude))
                , altitude(details::encode_altitude(altitude)) {}
        } __attribute__((packed));

        struct Response
        {
            uint8_t command_id;
            uint8_t message_type = MSG_RESPONSE;
            uint8_t padding;
            uint8_t status;

            Response(CommandIDs command_id, ResponseStatus status)
                : command_id(command_id)
                , padding(0)
                , status(status) {}
        } __attribute__((packed));
    }

    static const int MAX_PACKET_SIZE = 16;

    /** Creation of the request messages
     *
     * One should use one of the functions to create a message and then
     * write that message (the CRC is done by the write)
     *
     * <code>
     * const auto msg = requests::Deploy();
     * driver.writeMessage(msg);
     * </code>
     */
    namespace requests {

        inline packets::SimpleMessage Deploy()
        {
            return packets::SimpleMessage(ID_DEPLOY);
        }

        inline packets::SimpleMessage BITE()
        {
            return packets::SimpleMessage(ID_BITE);
        }

        inline packets::StatusRefreshRate StatusRefreshRatePT(Rates rate)
        {
            return packets::StatusRefreshRate(ID_STATUS_REFRESH_RATE_PT, rate);
        }

        inline packets::StatusRefreshRate StatusRefreshRateIMU(Rates rate)
        {
            return packets::StatusRefreshRate(ID_STATUS_REFRESH_RATE_IMU, rate);
        }

        inline packets::Angles AnglesRelative(float yaw, float pitch, float roll)
        {
            return packets::Angles(ID_ANGLES_RELATIVE, yaw, pitch, roll);
        }

        inline packets::Angles AnglesGeo(float yaw, float pitch, float roll)
        {
            return packets::Angles(ID_ANGLES_GEO, yaw, pitch, roll);
        }

        inline packets::AngularVelocities AngularVelocities(float yaw, float pitch, float roll)
        {
            return packets::AngularVelocities(yaw, pitch, roll);
        }

        inline packets::EnableStabilization EnableStabilization(bool yaw, bool pitch, bool roll)
        {
            return packets::EnableStabilization(yaw, pitch, roll);
        }

        inline packets::StabilizationTarget StabilizationTarget(float latitude, float longitude, float altitude)
        {
            return packets::StabilizationTarget(latitude, longitude, altitude);
        }

        template<typename T> void packetize(uint8_t* buffer, T const& packet) {
            std::memcpy(buffer, &packet, sizeof(packet));
            *reinterpret_cast<crc_t*>(buffer + sizeof(packet)) =
                details::compute_crc(buffer, sizeof(packet));
        }

        template<typename T> std::vector<uint8_t> packetize(T const& packet) {
            std::vector<uint8_t> buffer;
            buffer.resize(sizeof(packet) + sizeof(crc_t));
            packetize(buffer.data(), packet);
            return buffer;
        }

        Rates decode(packets::StatusRefreshRate const& angle);
        Eigen::Vector3d decode(packets::Angles const& angle);
        Eigen::Vector3d decode(packets::AngularVelocities const& angle);
        GeoTarget decode(packets::StabilizationTarget const& angle);
    }

    /** Representation of the reply messages
     */
    namespace reply {
        inline packets::Response Response(CommandIDs command_id, ResponseStatus status)
        {
            return packets::Response(command_id, status);
        }
        ResponseStatus parse(packets::Response const& message);
    }
}

#endif
