#include <indra_heads_protocol/Protocol.hpp>
#include <stdexcept>
#include <cmath>
#include <arpa/inet.h>
#include <boost/crc.hpp>

using namespace indra_heads_protocol;

int packets::getPacketSize(CommandIDs command_id, MessageTypes message_type)
{
    if (message_type == MSG_RESPONSE)
        return sizeof(packets::Response);

    switch(command_id)
    {
        case ID_STOP:
            return sizeof(packets::SimpleMessage);
        case ID_BITE:
            return sizeof(packets::SimpleMessage);
        case ID_STATUS_REFRESH_RATE_PT:
            return sizeof(packets::StatusRefreshRate);
        case ID_STATUS_REFRESH_RATE_IMU:
            return sizeof(packets::StatusRefreshRate);
        case ID_ANGLES_RELATIVE:
        case ID_ANGLES_GEO:
            return sizeof(packets::Angles);
        case ID_ANGULAR_VELOCITY_RELATIVE:
        case ID_ANGULAR_VELOCITY_GEO:
            return sizeof(packets::AngularVelocities);
        case ID_STABILIZATION_TARGET:
            return sizeof(packets::PositionGeo);
        default:
            throw std::invalid_argument("getPacketSize called with an invalid CommandIDs");
    };
}

crc_t details::compute_crc(uint8_t const* buffer, uint32_t size)
{
    return boost::crc<8, 7, 0, 0, false, false>(buffer, size);
}

void details::encode_crc(uint8_t* encoded, crc_t crc)
{
    encoded[0] = crc;
}

void details::encode_angle(uint8_t* encoded, double angle)
{
    double normalized = std::fmod(angle, 2 * M_PI);
    if (normalized < 0)
        normalized += 2 * M_PI;
    uint16_t integral  = std::floor(normalized * 360 / M_PI);
    encoded[0] = (integral & 0xFF00) >> 8;
    encoded[1] = (integral & 0x00FF);
}

double details::decode_angle(uint8_t const* angle)
{
    uint16_t integral =
        static_cast<uint16_t>(angle[0]) << 8 |
        static_cast<uint16_t>(angle[1]) << 0;
    return static_cast<double>(integral) / 360 * M_PI;
}

void details::encode_angular_velocity(uint8_t* encoded, double velocity)
{
    encoded[0] = velocity > 0 ? 0 : 1;
    encoded[1] = std::round(std::abs(velocity) * 1800 / M_PI);
}

double details::decode_angular_velocity(uint8_t const* encoded)
{
    int sign = encoded[0] ? -1 : 1;
    double velocity = static_cast<double>(encoded[1]) * M_PI / 1800;
    return sign * velocity;
}

void details::encode_latlon(uint8_t* encoded, double angle)
{
    uint32_t integral = std::round(std::abs(angle) * 1e6);
    encoded[0] = angle > 0 ? 0 : 1;
    encoded[1] = (integral & 0xFF000000) >> 24;
    encoded[2] = (integral & 0x00FF0000) >> 16;
    encoded[3] = (integral & 0x0000FF00) >> 8;
    encoded[4] = (integral & 0x000000FF) >> 0;
}

double details::decode_latlon(uint8_t const* encoded)
{
    int sign = encoded[0] > 0 ? -1 : 1;
    uint32_t integral =
        encoded[1] << 24 |
        encoded[2] << 16 |
        encoded[3] << 8 |
        encoded[4];
    double decoded = static_cast<double>(integral) * 1e-6;
    return sign * decoded;
}

void details::encode_altitude(uint8_t* encoded, double altitude)
{
    uint16_t integral = static_cast<uint16_t>(std::round(std::abs(altitude) * 10));
    encoded[0] = altitude > 0 ? 0 : 1;
    encoded[1] = (integral & 0xFF00) >> 8;
    encoded[2] = (integral & 0x00FF) >> 0;
}

double details::decode_altitude(uint8_t const* encoded)
{
    int sign = encoded[0] > 0 ? -1 : 1;
    uint16_t integral =
        static_cast<uint16_t>(encoded[1]) << 8 |
        static_cast<uint16_t>(encoded[2]) << 0;
    double altitude = static_cast<double>(integral) * 0.1;
    return sign * altitude;
}

Rates requests::decode(packets::StatusRefreshRate const& status)
{
    return static_cast<Rates>(status.rate);
}
Eigen::Vector3d requests::decode(packets::Angles const& angles)
{
    return Eigen::Vector3d(
            details::decode_angle(angles.roll),
            details::decode_angle(angles.pitch),
            details::decode_angle(angles.yaw));
}
Eigen::Vector3d requests::decode(packets::AngularVelocities const& velocities)
{
    return Eigen::Vector3d(
            details::decode_angular_velocity(velocities.roll),
            details::decode_angular_velocity(velocities.pitch),
            details::decode_angular_velocity(velocities.yaw));
}
GeoTarget requests::decode(packets::PositionGeo const& target)
{
    return GeoTarget(
            details::decode_latlon(target.latitude),
            details::decode_latlon(target.longitude),
            details::decode_altitude(target.altitude));
}
ResponseStatus reply::parse(packets::Response const& message)
{
    return static_cast<ResponseStatus>(message.status);
}
