#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <indra_heads_protocol/Driver.hpp>
#include <iodrivers_base/Fixture.hpp>

using namespace indra_heads_protocol;

struct DriverTest : public ::testing::Test, public iodrivers_base::Fixture<Driver>
{
    DriverTest()
    {
        driver.openURI("test://");
    }

    CommandIDs readRequest()
    {
        CommandIDs id = driver.readRequest();
        requestedConfiguration = driver.getRequestedConfiguration();
        return id;
    }

    Response readResponse()
    {
        return driver.readResponse();
    }

    RequestedConfiguration requestedConfiguration;
};

TEST_F(DriverTest, it_keeps_a_single_byte_that_is_a_valid_command_ID) {
    uint8_t msg[] = { 0x0 };
    pushDataToDriver(msg, msg + 1);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(1, getQueuedBytes());
}

TEST_F(DriverTest, it_rejects_a_first_byte_that_is_not_a_valid_command_ID) {
    uint8_t msg[] = { 0xF0 };
    pushDataToDriver(msg, msg + 1);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(0, getQueuedBytes());
}

TEST_F(DriverTest, it_keeps_a_header_of_a_valid_request) {
    uint8_t msg[] = { 0x00, 0x00 };
    pushDataToDriver(msg, msg + 2);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(2, getQueuedBytes());
}

TEST_F(DriverTest, it_keeps_a_header_of_a_valid_reply) {
    uint8_t msg[] = { 0x00, 0x01 };
    pushDataToDriver(msg, msg + 2);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(2, getQueuedBytes());
}

TEST_F(DriverTest, it_rejects_a_header_if_the_command_type_is_invalid) {
    uint8_t msg[] = { 0x00, 0x02 };
    pushDataToDriver(msg, msg + 2);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(1, getQueuedBytes());
}

TEST_F(DriverTest, it_waits_for_the_packet_to_match_the_expected_command_length) {
    uint8_t msg[] = { 0x02, 0x00, 0x00, 0x02 };
    pushDataToDriver(msg, msg + 3);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(3, getQueuedBytes());
}

TEST_F(DriverTest, it_returns_the_packet_if_it_has_the_expected_length_and_CRC) {
    uint8_t msg[] = { 0x02, 0x00, 0x02, 0xD8 };
    pushDataToDriver(msg, msg + 5);
    ASSERT_EQ( 4, readPacket().size() );
}

TEST_F(DriverTest, it_rejects_an_invalid_request_CRC) {
    uint8_t msg[] = { 0x02, 0x00, 0x00, 0x02, 0x21 };
    pushDataToDriver(msg, msg + 6);
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(0, getQueuedBytes());
}

TEST_F(DriverTest, it_returns_a_response) {
    uint8_t msg[] = {0x05, 0x01, 0x01, 0xD2 };
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ( 4, readPacket().size() );
}

TEST_F(DriverTest, it_rejects_an_invalid_response_CRC) {
    uint8_t msg[] = {0x05, 0x01, 0x01, 0x21};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_ANY_THROW( readPacket() );
    ASSERT_EQ(3, getQueuedBytes());
}

TEST_F(DriverTest, it_interprets_a_STOP_command) {
    uint8_t msg[] = {0x00, 0x00, 0x00};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_STOP, readRequest());
    ASSERT_EQ(RequestedConfiguration::STOP, requestedConfiguration.control_mode);
}

TEST_F(DriverTest, it_interprets_a_BITE_command) {
    uint8_t msg[] = {0x01, 0x00, 0x15};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_BITE, readRequest());
    ASSERT_EQ(RequestedConfiguration::SELF_TEST, requestedConfiguration.control_mode);
}

TEST_F(DriverTest, it_interprets_a_STATUS_REFRESH_RATE_PT_command) {
    uint8_t msg[] = {0x02, 0x00, 0x02, 0xD8};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_STATUS_REFRESH_RATE_PT, readRequest());
    ASSERT_EQ(RATE_20HZ, requestedConfiguration.rate_status_pt);
    ASSERT_EQ(RequestedConfiguration::STOP, requestedConfiguration.control_mode);
}

TEST_F(DriverTest, it_interprets_a_STATUS_REFRESH_RATE_IMU_command) {
    uint8_t msg[] = {0x03, 0x00, 0x01, 0xBA};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_STATUS_REFRESH_RATE_IMU, readRequest());
    ASSERT_EQ(RATE_10HZ, requestedConfiguration.rate_status_imu);
    ASSERT_EQ(RequestedConfiguration::STOP, requestedConfiguration.control_mode);
}

TEST_F(DriverTest, it_interprets_a_ANGLES_RELATIVE_command) {
    uint8_t msg[] = {0x04, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0x04};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_ANGLES_RELATIVE, readRequest());
    ASSERT_EQ(RequestedConfiguration::ANGLES_RELATIVE, requestedConfiguration.control_mode);
    ASSERT_TRUE(Eigen::Vector3d(0.19199, 0.29671, 0.09599).isApprox(requestedConfiguration.rpy, 1e-4));
}

TEST_F(DriverTest, it_interprets_a_ANGLES_GEO_command) {
    uint8_t msg[] = {0x05, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0x17};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_ANGLES_GEO, readRequest());
    ASSERT_EQ(RequestedConfiguration::ANGLES_GEO, requestedConfiguration.control_mode);
    ASSERT_TRUE(Eigen::Vector3d(0.19199, 0.29671, 0.09599).isApprox(requestedConfiguration.rpy, 1e-4));
}

TEST_F(DriverTest, it_interprets_a_ANGULAR_VELOCITIES_command_as_GEO_by_default) {
    uint8_t msg[] = {0x06, 0x00, 0x00, 0x39, 0x01, 0x73, 0x00, 0xAC, 0xC6};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_ANGULAR_VELOCITIES, readRequest());
    ASSERT_EQ(RequestedConfiguration::ANGULAR_VELOCITY_GEO, requestedConfiguration.control_mode);
    ASSERT_TRUE(Eigen::Vector3d(0.300197, -0.200713, 0.09948).isApprox(requestedConfiguration.rpy, 1e-4));
}

TEST_F(DriverTest, it_interprets_a_ANGULAR_VELOCITIES_command_as_RELATIVE_if_stabilization_is_disabled) {
    { uint8_t msg[] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x29};
      pushDataToDriver(msg, msg + sizeof(msg));
      ASSERT_EQ(ID_ENABLE_STABILIZATION, readRequest()); }

    uint8_t msg[] = {0x06, 0x00, 0x00, 0x39, 0x01, 0x73, 0x00, 0xAC, 0xC6};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_ANGULAR_VELOCITIES, readRequest());
    ASSERT_EQ(RequestedConfiguration::ANGULAR_VELOCITY_RELATIVE,
        requestedConfiguration.control_mode);
    ASSERT_TRUE(Eigen::Vector3d(0.300197, -0.200713, 0.09948).
        isApprox(requestedConfiguration.rpy, 1e-4));
}

TEST_F(DriverTest, it_interprets_a_STABILIZATION_TARGET_command) {
    uint8_t msg[] = {
        0x08, 0x00,
        0x01, 0x00, 0x01, 0x86, 0xA0,
        0x00, 0x00, 0x03, 0x0D, 0x40,
        0x01, 0x00, 0x03,
        0x8C };
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_EQ(ID_STABILIZATION_TARGET, readRequest());
    ASSERT_EQ(RequestedConfiguration::POSITION_GEO, requestedConfiguration.control_mode);

    ASSERT_NEAR(-0.1, requestedConfiguration.lat_lon_alt.latitude, 1e-4);
    ASSERT_NEAR(0.2, requestedConfiguration.lat_lon_alt.longitude, 1e-4);
    ASSERT_NEAR(-0.3, requestedConfiguration.lat_lon_alt.altitude, 1e-4);
}

TEST_F(DriverTest, it_throws_if_a_response_is_received_while_expecting_a_command) {
    uint8_t msg[] = {0x05, 0x01, 0x00, 0x01, 0x22};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_THROW(readRequest(), std::runtime_error);
}

TEST_F(DriverTest, it_interprets_a_response) {
    uint8_t msg[] = {0x05, 0x01, 0x01, 0xD2};
    pushDataToDriver(msg, msg + sizeof(msg));
    auto response = readResponse();
    ASSERT_EQ(ID_ANGLES_GEO, response.command_id);
    ASSERT_EQ(STATUS_FAILED, response.status);
}

TEST_F(DriverTest, it_throws_if_a_request_is_received_while_expecting_a_response) {
    uint8_t msg[] = {0x06, 0x00, 0x39, 0x73, 0x00, 0xAC, 0x6E};
    pushDataToDriver(msg, msg + sizeof(msg));
    ASSERT_THROW(readResponse(), std::runtime_error);
}
