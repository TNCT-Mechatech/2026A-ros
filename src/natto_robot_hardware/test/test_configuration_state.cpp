#include <natto_robot_hardware/serial_frames.hpp>
#include <gtest/gtest.h>
#include <deque>
#include <vector>

using namespace natto_serial;
using namespace std::chrono_literals;
using Time = ConfigurationState::Time;

namespace {
struct Port : SerialDev {
    std::deque<uint8_t> rx;
    std::vector<std::vector<uint8_t>> tx;
    int readable_len() override { return rx.size(); }
    int read() override { const auto value=rx.front(); rx.pop_front(); return value; }
    int write(unsigned char *p, unsigned int n) override { tx.emplace_back(p,p+n); return n; }
    void receive(const std::vector<uint8_t> &p) { rx.insert(rx.end(),p.begin(),p.end()); }
};
initialization_ack_packet_t reply(const SerialFrames &f, size_t i, InitializationResult result=InitializationResult::Applied) {
    const auto &p=f.configuration.requested(i);
    return {p.session_id,p.config_id,p.session_id,p.config_id,result,{0,0,0}};
}
void confirm_all(SerialFrames &f, Time now) {
    for (size_t i=0;i<MOTORS;++i) f.configuration.observe_ack(i,reply(f,i),now);
}
}

TEST(Configuration, MotorZeroLossRetriesOnlyPendingWithFrozenIdentityAndPayload) {
    Port port;
    SerialBridge serial(&port);
    SerialFrames f;
    f.register_all(serial);
    f.init[0].data.vel_kp=.12f;
    f.init[0].data.reset_encoder_origin=true;
    f.initialize(serial,Time{});
    ASSERT_EQ(port.tx.size(),4u);
    EXPECT_NE(f.init[0].data.session_id,0u);
    EXPECT_NE(f.init[0].data.config_id,0u);
    for (size_t i=1;i<MOTORS;++i) f.configuration.observe_ack(i,reply(f,i),Time{});
    EXPECT_FALSE(f.configuration.ready());
    f.init[0].data.vel_kp=99; // Retry must not accidentally change the payload.
    f.service_initialization(serial,Time{}+99ms);
    EXPECT_EQ(port.tx.size(),4u);
    f.service_initialization(serial,Time{}+100ms);
    ASSERT_EQ(port.tx.size(),5u);
    EXPECT_EQ(port.tx.front(),port.tx.back());
    EXPECT_EQ(f.configuration.attempts(0),2u);
    for (size_t i=1;i<MOTORS;++i) EXPECT_EQ(f.configuration.attempts(i),1u);
    f.configuration.observe_ack(0,reply(f,0,InitializationResult::AlreadyApplied),Time{}+101ms);
    EXPECT_TRUE(f.configuration.ready());
}

TEST(Configuration, LostAckCanBeConfirmedByPeriodicIdentityButOtherSessionsCannot) {
    Port port; SerialBridge serial(&port); SerialFrames f; f.register_all(serial);
    f.initialize(serial,Time{});
    for (size_t i=0;i<MOTORS;++i) {
        motor_feedback_packet_t feedback{};
        feedback.applied_session_id=f.configuration.session_id()^1u;
        feedback.applied_config_id=f.configuration.config_id(i);
        f.configuration.observe_feedback(i,feedback,Time{});
        EXPECT_FALSE(f.configuration.confirmed(i));
        feedback.applied_session_id=f.configuration.session_id();
        f.configuration.observe_feedback(i,feedback,Time{});
    }
    EXPECT_TRUE(f.configuration.ready());
}

TEST(Configuration, AckMustMatchBothRequestedAndAppliedIdentity) {
    Port port; SerialBridge serial(&port); SerialFrames f; f.register_all(serial); f.initialize(serial,Time{});
    auto ack=reply(f,0); ++ack.config_id;
    f.configuration.observe_ack(0,ack,Time{});
    EXPECT_FALSE(f.configuration.confirmed(0));
    ack=reply(f,0); ++ack.applied_session_id;
    f.configuration.observe_ack(0,ack,Time{});
    EXPECT_FALSE(f.configuration.confirmed(0));
    ack=reply(f,0); ++ack.applied_config_id;
    f.configuration.observe_ack(0,ack,Time{});
    EXPECT_FALSE(f.configuration.confirmed(0));
}

TEST(Configuration, NegativeOrUnknownResultsLatchFailureUntilNewConfiguration) {
    for (auto result : {InitializationResult::Invalid,InitializationResult::Stale,InitializationResult::IdConflict,
                        static_cast<InitializationResult>(255)}) {
        Port port; SerialBridge serial(&port); SerialFrames f; f.register_all(serial); f.initialize(serial,Time{});
        f.configuration.observe_ack(0,reply(f,0,result),Time{});
        confirm_all(f,Time{});
        EXPECT_TRUE(f.configuration.failed());
        EXPECT_FALSE(f.configuration.ready());
        EXPECT_THROW(f.service_initialization(serial,Time{}+100ms),std::runtime_error);
        f.initialize(serial,Time{}+200ms);
        EXPECT_FALSE(f.configuration.failed());
        EXPECT_EQ(f.configuration.config_id(0),2u);
    }
}

TEST(Configuration, DeadlineDoesNotTurnHealthyFeedbackWithoutIdentityIntoSuccess) {
    Port port; SerialBridge serial(&port); SerialFrames f; f.register_all(serial); f.initialize(serial,Time{});
    for (size_t i=0;i<MOTORS;++i) f.configuration.observe_feedback(i,motor_feedback_packet_t{},Time{}+1999ms);
    EXPECT_THROW(f.service_initialization(serial,Time{}+2000ms),std::runtime_error);
    EXPECT_TRUE(f.configuration.failed());
    EXPECT_FALSE(f.configuration.ready());
}

TEST(Configuration, ResetFeedbackRevokesPermissionAndIsSeenEvenInMixedDrainBatch) {
    Port host_port,device_port;
    SerialBridge host(&host_port),device(&device_port);
    SerialFrames f,board; f.register_all(host); board.register_all(device);
    f.initialize(host); confirm_all(f,ConfigurationState::Clock::now());
    ASSERT_TRUE(f.configuration.ready());
    device.write(FEEDBACK_FRAME_ID_BASE); host_port.receive(device_port.tx.back());
    board.ack[0].data=reply(f,0);
    device.write(INIT_ACK_FRAME_ID_BASE); host_port.receive(device_port.tx.back());
    const auto received=f.drain(host);
    EXPECT_TRUE(received.configuration_lost);
    EXPECT_TRUE(received.motor_updated[0]);
    EXPECT_TRUE(received.ack_updated[0]);
    EXPECT_TRUE(f.configuration.ready()); // Caller must still discard old motion.
}

TEST(Configuration, FirmwareResetStartsANewConfirmationDeadlineWithSameConfig) {
    Port port; SerialBridge serial(&port); SerialFrames f; f.register_all(serial);
    f.initialize(serial,Time{}); confirm_all(f,Time{});
    const auto requested=f.configuration.requested(0);
    f.configuration.observe_feedback(0,motor_feedback_packet_t{},Time{}+30s);
    EXPECT_FALSE(f.configuration.ready());
    EXPECT_NO_THROW(f.service_initialization(serial,Time{}+30s));
    EXPECT_EQ(f.init[0].data.session_id,requested.session_id);
    EXPECT_EQ(f.init[0].data.config_id,requested.config_id);
    EXPECT_EQ(f.configuration.attempts(0),2u);
    EXPECT_THROW(f.service_initialization(serial,Time{}+32s),std::runtime_error);
}

TEST(Configuration, LegacySizedFeedbackCannotConfirmConfiguration) {
    Port host_port,device_port;
    SerialBridge host(&host_port),device(&device_port);
    SerialFrames f; f.register_all(host); f.initialize(host);
    struct LegacyFeedback { uint8_t bytes[32]; };
    sb::Message<LegacyFeedback> legacy; legacy.data={};
    device.add_frame(FEEDBACK_FRAME_ID_BASE,&legacy);
    device.write(FEEDBACK_FRAME_ID_BASE); host_port.receive(device_port.tx.back());
    const auto received=f.drain(host);
    EXPECT_EQ(received.invalid,1u);
    EXPECT_FALSE(f.configuration.ready());
    EXPECT_FALSE(received.motor_updated[0]);
}
