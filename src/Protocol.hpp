#ifndef INDRA_HEADS_PROTOCOL_HPP
#define INDRA_HEADS_PROTOCOL_HPP

#include <cstdint>
#include <Eigen/Core>
#include <arpa/inet.h>
#include <base/Float.hpp>

namespace indra_heads_protocol
{
    enum CommandIDs {
        ID_STOP = 0,
        ID_BITE   = 1,
        ID_STATUS_REFRESH_RATE_PT = 2,
        ID_STATUS_REFRESH_RATE_IMU = 3,
        ID_ANGLES_RELATIVE = 4,
        ID_ANGLES_GEO      = 5,
        ID_ANGULAR_VELOCITY = 6,
        ID_ENABLE_STABILIZATION = 7,
        ID_STABILIZATION_TARGET = 8
    };

    static const int ID_LAST = ID_STABILIZATION_TARGET;

    typedef std::int8_t crc_t;
    static const int MIN_PACKET_SIZE = 2 + sizeof(crc_t);

    enum MessageTypes {
        MSG_REQUEST = 0,
        MSG_RESPONSE = 1
    };
    static const int MSG_LAST_TYPE = 1;

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
        crc_t compute_crc(uint8_t const* buffer, uint32_t size);
        void  encode_crc(uint8_t* encoded, crc_t crc);

        // Encode an angle (2 bytes)
        void encode_angle(uint8_t* encoded, double angle);
        // Encode an angular veloticy (2 bytes)
        void encode_angular_velocity(uint8_t* encoded, double velocity);
        // Encode either a latitude or a longitude (5 bytes)
        void encode_latlon(uint8_t* encoded, double angle);
        // Encode an altitude (3 bytes)
        void encode_altitude(uint8_t* encoded, double altitude);

        double decode_angle(uint8_t const* encoded);
        double decode_angular_velocity(uint8_t const* encoded);
        double decode_latlon(uint8_t const* encoded);
        double decode_altitude(uint8_t const* encoded);
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
            uint8_t rate;

            StatusRefreshRate(CommandIDs command, Rates rate)
                : command_id(command)
                , rate(rate) {}

        } __attribute__((packed));

        struct Angles
        {
            uint8_t command_id;
            uint8_t message_type = MSG_REQUEST;
            uint8_t yaw[2];
            uint8_t pitch[2];
            uint8_t roll[2];

            Angles(CommandIDs command, double yaw, double pitch, double roll)
                : command_id(command)
            {
                details::encode_angle(this->yaw, yaw);
                details::encode_angle(this->pitch, pitch);
                details::encode_angle(this->roll, roll);
            }
        } __attribute__((packed));

        struct AngularVelocities
        {
            uint8_t command_id = ID_ANGULAR_VELOCITY;
            uint8_t message_type = MSG_REQUEST;
            uint8_t yaw[2];
            uint8_t pitch[2];
            uint8_t roll[2];

            AngularVelocities(double yaw, double pitch, double roll)
            {
                details::encode_angular_velocity(this->yaw, yaw);
                details::encode_angular_velocity(this->pitch, pitch);
                details::encode_angular_velocity(this->roll, roll);
            }
        } __attribute__((packed));

        struct EnableStabilization
        {
            uint8_t command_id = ID_ENABLE_STABILIZATION;
            uint8_t message_type = MSG_REQUEST;
            uint8_t yaw;
            uint8_t pitch;
            uint8_t roll;

            EnableStabilization(bool yaw, bool pitch, bool roll)
                : yaw(yaw ? 1 : 0)
                , pitch(pitch ? 1 : 0)
                , roll(roll ? 1 : 0) {}
        } __attribute__((packed));

        struct PositionGeo
        {
            uint8_t command_id = ID_STABILIZATION_TARGET;
            uint8_t message_type = MSG_REQUEST;
            uint8_t latitude[5];
            uint8_t longitude[5];
            uint8_t altitude[3];

            PositionGeo(double latitude, double longitude, double altitude)
            {
                details::encode_latlon(this->latitude, latitude);
                details::encode_latlon(this->longitude, longitude);
                details::encode_altitude(this->altitude, altitude);
            }
        } __attribute__((packed));

        struct Response
        {
            uint8_t command_id;
            uint8_t message_type = MSG_RESPONSE;
            uint8_t status;

            Response(CommandIDs command_id, ResponseStatus status)
                : command_id(command_id)
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
     * const auto msg = requests::Stop();
     * driver.writeMessage(msg);
     * </code>
     */
    namespace requests {

        inline packets::SimpleMessage Stop()
        {
            return packets::SimpleMessage(ID_STOP);
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

        inline packets::Angles AnglesRelative(double yaw, double pitch, double roll)
        {
            return packets::Angles(ID_ANGLES_RELATIVE, yaw, pitch, roll);
        }

        inline packets::Angles AnglesGeo(double yaw, double pitch, double roll)
        {
            return packets::Angles(ID_ANGLES_GEO, yaw, pitch, roll);
        }

        inline packets::AngularVelocities AngularVelocities(
            double yaw, double pitch, double roll)
        {
            return packets::AngularVelocities(yaw, pitch, roll);
        }

        inline packets::EnableStabilization EnableStabilization(
            bool yaw, bool pitch, bool roll)
        {
            return packets::EnableStabilization(yaw, pitch, roll);
        }

        inline packets::PositionGeo PositionGeo(
            double latitude, double longitude, double altitude)
        {
            return packets::PositionGeo(latitude, longitude, altitude);
        }

        template<typename T> void packetize(uint8_t* buffer, T const& packet) {
            std::memcpy(buffer, &packet, sizeof(packet));
            crc_t crc = details::compute_crc(buffer, sizeof(packet));
            details::encode_crc(buffer + sizeof(packet), crc);
        }

        template<typename T> std::vector<uint8_t> packetize(T const& packet) {
            std::vector<uint8_t> buffer;
            buffer.resize(sizeof(packet) + sizeof(crc_t));
            packetize(buffer.data(), packet);
            return std::move(buffer);
        }

        Rates decode(packets::StatusRefreshRate const& angle);
        Eigen::Vector3d decode(packets::Angles const& angle);
        Eigen::Vector3d decode(packets::AngularVelocities const& angle);
        bool decode(packets::EnableStabilization const& angle);
        GeoTarget decode(packets::PositionGeo const& angle);
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
