#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <indra_heads_protocol/Protocol.hpp>

using namespace std;
using namespace indra_heads_protocol;
using ::testing::ElementsAre;

TEST(Protocol, DeployMessage) {
    ASSERT_THAT(requests::packetize(requests::Deploy()),
            ElementsAre(0x00, 0x00, 0x00, 0x00));
}

TEST(Protocol, BITE) {
    ASSERT_THAT(requests::packetize(requests::BITE()),
            ElementsAre(0x01, 0x00, 0x90, 0x01));
}

TEST(Protocol, StatusRefreshRatePT) {
    ASSERT_THAT(requests::packetize(requests::StatusRefreshRatePT(RATE_20HZ)),
            ElementsAre(0x02, 0x00, 0x00, 0x02, 0x79, 0x80));
}

TEST(Protocol, StatusRefreshRateIMU) {
    ASSERT_THAT(requests::packetize(requests::StatusRefreshRateIMU(RATE_10HZ)),
            ElementsAre(0x03, 0x00, 0x00, 0x01, 0x84, 0xC1));
}

TEST(Protocol, encode_angle_normalizes_its_input) {
    uint16_t encoded = details::encode_angle(-150.1 * M_PI / 180);
    // This is 420 - 0x1A4
    uint8_t const* bytes = reinterpret_cast<uint8_t*>(&encoded);
    ASSERT_EQ(bytes[0], 0x01);
    ASSERT_EQ(bytes[1], 0xA3);
}

TEST(Protocol, encode_angle_handles_upper_limit_properly) {
    uint16_t encoded = details::encode_angle(2 * M_PI - 1e-9);
    // This is 719 - 0x1A4
    uint8_t const* bytes = reinterpret_cast<uint8_t*>(&encoded);
    ASSERT_EQ(bytes[0], 0x02);
    ASSERT_EQ(bytes[1], 0xCF);
}

TEST(Protocol, AnglesRelative) {
    ASSERT_THAT(requests::packetize(requests::AnglesRelative(0.1, 0.2, 0.3)),
            ElementsAre(0x04, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0xF6, 0x85));
}

TEST(Protocol, AnglesGeo) {
    ASSERT_THAT(requests::packetize(requests::AnglesGeo(0.1, 0.2, 0.3)),
            ElementsAre(0x05, 0x00, 0x00, 0xB, 0x00, 0x22, 0x00, 0x16, 0x3A, 0x44));
}

TEST(Protocol, AngularVelocities) {
    ASSERT_THAT(requests::packetize(requests::AngularVelocities(0.1, 0.2, 0.3)),
            ElementsAre(0x06, 0x00, 0x39, 0x73, 0x00, 0xAC, 0x5C, 0xFD));
}

TEST(Protocol, EnableStabilization) {
    ASSERT_THAT(requests::packetize(requests::EnableStabilization(true, false, true)),
            ElementsAre(0x07, 0x00, 0x01, 0x00, 0x00, 0x01, 0x8B, 0xC1));
}

TEST(Protocol, StabilizationTarget) {
    vector<uint8_t> packet = requests::packetize(requests::StabilizationTarget(-0.1, 0.2, 0.3));
    // ElementsAre supports 10 elements max
    ASSERT_THAT(vector<uint8_t>(packet.begin(), packet.begin() + 8),
            ElementsAre(0x08, 0x00, 0x01, 0x00, 0x00, 0x01, 0x86, 0xA0));
    ASSERT_THAT(vector<uint8_t>(packet.begin() + 8, packet.end()),
            ElementsAre(0x00, 0x00, 0x00, 0x03, 0x0D, 0x40,
                        0x00, 0x00));
}

TEST(Protocol, Response) {
    ASSERT_THAT(requests::packetize(reply::Response(ID_ANGLES_GEO, STATUS_FAILED)),
            ElementsAre(0x05, 0x01, 0x00, 0x01, 0xCC, 0x90));
}

