#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <indra_heads_protocol/Protocol.hpp>

using namespace std;
using namespace indra_heads_protocol;
using ::testing::ElementsAre;

// CRC generation site: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html

TEST(Protocol, StopMessage) {
    ASSERT_THAT(requests::packetize(requests::Stop()),
            ElementsAre(0x00, 0x00, 0x00));
}

TEST(Protocol, BITE) {
    ASSERT_THAT(requests::packetize(requests::BITE()),
            ElementsAre(0x01, 0x00, 0x15));
}

TEST(Protocol, StatusRefreshRatePT) {
    ASSERT_THAT(requests::packetize(requests::StatusRefreshRatePT(RATE_20HZ)),
            ElementsAre(0x02, 0x00, 0x02, 0xD8));
}

TEST(Protocol, StatusRefreshRateIMU) {
    ASSERT_THAT(requests::packetize(requests::StatusRefreshRateIMU(RATE_10HZ)),
            ElementsAre(0x03, 0x00, 0x01, 0xBA));
}

TEST(Protocol, encode_angle_normalizes_its_input) {
    uint8_t encoded[2];
    details::encode_angle(encoded, -150.1 * M_PI / 180);
    // This is 420 - 0x1A4
    ASSERT_EQ(0x01, encoded[0]);
    ASSERT_EQ(0xA3, encoded[1]);
}

TEST(Protocol, encode_angle_handles_upper_limit_properly) {
    uint8_t encoded[2];
    details::encode_angle(encoded, 2 * M_PI - 1e-9);
    // This is 719 - 0x1A4
    ASSERT_EQ(encoded[0], 0x02);
    ASSERT_EQ(encoded[1], 0xCF);
}

TEST(Protocol, AnglesRelative) {
    ASSERT_THAT(requests::packetize(requests::AnglesRelative(0.1, 0.3, 0.2)),
            ElementsAre(0x04, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0x04));
}

TEST(Protocol, AnglesGeo) {
    ASSERT_THAT(requests::packetize(requests::AnglesGeo(0.1, 0.3, 0.2)),
            ElementsAre(0x05, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0x17));
}

TEST(Protocol, AngularVelocityRelative) {
    ASSERT_THAT(requests::packetize(requests::AngularVelocityRelative(0.1, -0.2, 0.3)),
            ElementsAre(0x06, 0x00, 0x0, 0x39, 0x1, 0x73, 0x0, 0xAC, 0xC6));
}

TEST(Protocol, AngularVelocityGeo) {
    ASSERT_THAT(requests::packetize(requests::AngularVelocityGeo(0.1, -0.2, 0.3)),
            ElementsAre(0x07, 0x00, 0x0, 0x39, 0x1, 0x73, 0x0, 0xAC, 0xD5));
}

TEST(Protocol, StabilizationTarget) {
    vector<uint8_t> packet = requests::packetize(requests::PositionGeo(-0.1, 0.2, -0.3));
    // ElementsAre supports 10 elements max
    ASSERT_THAT(vector<uint8_t>(packet.begin(), packet.begin() + 7),
            ElementsAre(0x08, 0x00,
                        0x01, 0x00, 0x01, 0x86, 0xA0));
    ASSERT_THAT(vector<uint8_t>(packet.begin() + 7, packet.end()),
            ElementsAre(0x00, 0x00, 0x03, 0x0D, 0x40,
                        0x01, 0x00, 0x03,
                        0x8C));
}

TEST(Protocol, Response) {
    ASSERT_THAT(requests::packetize(reply::Response(ID_ANGLES_GEO, STATUS_FAILED)),
            ElementsAre(0x05, 0x01, 0x01, 0xD2));
}
