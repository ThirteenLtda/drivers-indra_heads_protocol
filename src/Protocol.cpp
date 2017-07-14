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
        case ID_DEPLOY:
            return sizeof(packets::SimpleMessage);
        case ID_BITE:
            return sizeof(packets::SimpleMessage);
        case ID_STATUS_REFRESH_RATE_PT:
            return sizeof(packets::StatusRefreshRate);
        case ID_STATUS_REFRESH_RATE_IMU:
            return sizeof(packets::StatusRefreshRate);
        case ID_ANGLES_RELATIVE:
            return sizeof(packets::Angles);
        case ID_ANGLES_GEO:
            return sizeof(packets::Angles);
        case ID_ANGULAR_VELOCITIES:
            return sizeof(packets::AngularVelocities);
        case ID_ENABLE_STABILIZATION:
            return sizeof(packets::EnableStabilization);
        case ID_STABILIZATION_TARGET:
            return sizeof(packets::StabilizationTarget);
        default:
            throw std::invalid_argument("getPacketSize called with an invalid CommandIDs");
    };
}

crc_t details::compute_crc(uint8_t const* buffer, uint32_t size)
{
    boost::crc_16_type crc;
    crc.process_bytes(buffer, size);
    return htons(crc.checksum());
}

uint16_t details::encode_angle(double angle)
{
    double normalized = std::fmod(angle, 2 * M_PI);
    if (normalized < 0)
        normalized += 2 * M_PI;
    uint16_t encoded  = std::floor(normalized * 360 / M_PI);
    return htons(encoded);
}

double details::decode_angle(uint16_t angle)
{
    angle = ntohs(angle);
    return static_cast<double>(angle) / 360 * M_PI;
}

uint8_t details::encode_angular_velocity(double velocity)
{
    return std::round(velocity * 1800 / M_PI);
}

double details::decode_angular_velocity(uint8_t velocity)
{
    return static_cast<double>(velocity) * M_PI / 1800;
}

uint32_t details::encode_latlon(double angle)
{
    uint32_t encoded = std::round(std::abs(angle) * 1e6); 
    return htonl(encoded);
}

double details::decode_latlon(uint32_t angle)
{
    return static_cast<double>(ntohl(angle)) * 1e-6; 
}

uint32_t details::encode_altitude(double altitude)
{
    throw std::runtime_error("do not know how to do that yet");
}

double details::decode_altitude(uint16_t altitude)
{
    throw std::runtime_error("do not know how to do that yet");
}

Rates requests::decode(packets::StatusRefreshRate const& status)
{
    return static_cast<Rates>(ntohs(status.rate));
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
GeoTarget requests::decode(packets::StabilizationTarget const& target)
{
    return GeoTarget(
            (target.latitude_sign == 1 ? 1 : -1) * details::decode_latlon(target.latitude),
            (target.longitude_sign == 1 ? 1 : -1) * details::decode_latlon(target.longitude),
            details::decode_altitude(target.altitude));
}

ResponseStatus reply::parse(packets::Response const& message)
{
    return static_cast<ResponseStatus>(message.status);
}

