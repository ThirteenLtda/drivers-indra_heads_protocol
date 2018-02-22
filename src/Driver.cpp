#include <indra_heads_protocol/Driver.hpp>
#include <indra_heads_protocol/Protocol.hpp>
#include <indra_heads_protocol/Response.hpp>
#include <iostream>

using namespace std;
using namespace indra_heads_protocol;

Driver::Driver()
    : iodrivers_base::Driver(indra_heads_protocol::MAX_PACKET_SIZE * 10) 
{
    // NOTE: MAX_PACKET_SIZE here is this->MAX_PACKET_SIZE which is initialized
    // using the value passed to the constructor above. The confusion here stems
    // from the fact that MAX_PACKET_SIZE in iodrivers_base::Driver is actually
    // the size of the internal buffer.
    mReadBuffer.resize(MAX_PACKET_SIZE);
    mWriteBuffer.resize(MAX_PACKET_SIZE);
}

int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const
{
    if (buffer_size == 0)
        return 0;
    else if (buffer[0] > ID_LAST)
        return -1;
    else if (buffer_size < 2)
        return 0;
    else if (buffer[1] > MSG_LAST_TYPE)
        return -1;

    size_t packet_size   = packets::getPacketSize(
            static_cast<CommandIDs>(buffer[0]),
            static_cast<MessageTypes>(buffer[1]));
    size_t expected_size = packet_size + sizeof(crc_t);
    if (buffer_size < expected_size)
        return 0;

    crc_t expected_crc = *reinterpret_cast<crc_t const*>(buffer + packet_size);
    crc_t actual_crc   = details::compute_crc(buffer, packet_size);
    if (actual_crc != expected_crc)
        return -1;
    return expected_size;
}

CommandIDs Driver::readRequest()
{
    readPacket(mReadBuffer.data(), MAX_PACKET_SIZE);
    if (mReadBuffer[1] == MSG_RESPONSE)
        throw std::runtime_error("expected a command packet but got a response");

    mRequestedConfiguration.time = base::Time::now();

    switch(mReadBuffer[0])
    {
        case ID_DEPLOY:
            mRequestedConfiguration.command_id = ID_DEPLOY;
            mRequestedConfiguration.control_mode = RequestedConfiguration::STOP;
            return ID_DEPLOY;
        case ID_BITE:
            mRequestedConfiguration.command_id = ID_BITE;
            mRequestedConfiguration.control_mode = RequestedConfiguration::SELF_TEST;
            return ID_BITE;
        case ID_STATUS_REFRESH_RATE_PT:
            mRequestedConfiguration.command_id = ID_STATUS_REFRESH_RATE_PT;
            mRequestedConfiguration.rate_status_pt =
                requests::decode(reinterpret_cast<packets::StatusRefreshRate&>(mReadBuffer[0]));
            return ID_STATUS_REFRESH_RATE_PT;
        case ID_STATUS_REFRESH_RATE_IMU:
            mRequestedConfiguration.command_id = ID_STATUS_REFRESH_RATE_IMU;
            mRequestedConfiguration.rate_status_imu =
                requests::decode(reinterpret_cast<packets::StatusRefreshRate&>(mReadBuffer[0]));
            return ID_STATUS_REFRESH_RATE_IMU;
        case ID_ANGLES_RELATIVE:
            mRequestedConfiguration.command_id = ID_ANGLES_RELATIVE;
            mRequestedConfiguration.control_mode = RequestedConfiguration::ANGLES_RELATIVE;
            mRequestedConfiguration.rpy =
                requests::decode(reinterpret_cast<packets::Angles&>(mReadBuffer[0]));
            return ID_ANGLES_RELATIVE;
        case ID_ANGLES_GEO:
            mRequestedConfiguration.command_id = ID_ANGLES_GEO;
            mRequestedConfiguration.control_mode = RequestedConfiguration::ANGLES_GEO;
            mRequestedConfiguration.rpy =
                requests::decode(reinterpret_cast<packets::Angles&>(mReadBuffer[0]));
            return ID_ANGLES_GEO;
        case ID_ANGULAR_VELOCITIES:
            mRequestedConfiguration.command_id = ID_ANGULAR_VELOCITIES;
            mRequestedConfiguration.control_mode = RequestedConfiguration::ANGULAR_VELOCITY;
            mRequestedConfiguration.rpy =
                requests::decode(reinterpret_cast<packets::AngularVelocities&>(mReadBuffer[0]));
            return ID_ANGULAR_VELOCITIES;
        case ID_STABILIZATION_TARGET:
            mRequestedConfiguration.command_id = ID_STABILIZATION_TARGET;
            mRequestedConfiguration.control_mode = RequestedConfiguration::STABILIZED;
            mRequestedConfiguration.lat_lon_alt =
                requests::decode(reinterpret_cast<packets::StabilizationTarget&>(mReadBuffer[0]));
            return ID_STABILIZATION_TARGET;
        default:
            throw std::logic_error("should never have reached this");
            // Never reached;
    }

    throw std::logic_error("should never have reached this");
    // Never reached;
}

void Driver::writeResponse(Response response)
{
    auto packet = reply::Response(response.command_id, response.status);
    sendRequest(packet);
}

Response Driver::readResponse()
{
    readPacket(mReadBuffer.data(), MAX_PACKET_SIZE);
    if (mReadBuffer[1] == MSG_REQUEST)
        throw std::runtime_error("expected a response packet but got a request");

    return Response {
        static_cast<CommandIDs>(mReadBuffer[0]),
        reply::parse(reinterpret_cast<packets::Response const&>(mReadBuffer[0]))
    };
}

RequestedConfiguration Driver::getRequestedConfiguration() const
{
    return mRequestedConfiguration;
}
