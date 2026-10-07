#include <natto_robot_hardware/serial_frames.hpp>
#include <gtest/gtest.h>
#include <deque>
#include <vector>

using namespace natto_serial;
class MemorySerial : public SerialDev {
public:
    std::deque<uint8_t> input;
    std::vector<std::vector<uint8_t>> writes;
    int readable_len() override { return input.size(); }
    int read() override { auto value=input.front(); input.pop_front(); return value; }
    int write(unsigned char *data, unsigned int len) override {
        writes.emplace_back(data, data+len);
        return len;
    }
    void receive(const std::vector<uint8_t> &bytes) { input.insert(input.end(), bytes.begin(), bytes.end()); }
};

TEST(SerialProtocol, NativeABI) {
    EXPECT_EQ(sizeof(InitializationPackets), 44u);
    EXPECT_EQ(sizeof(MotorCommandVel), 4u);
    EXPECT_EQ(sizeof(MotorCommandPos), 8u);
    EXPECT_EQ(sizeof(MotorCommandDuty), 4u);
    EXPECT_EQ(sizeof(MotorFeedback), 32u);
    EXPECT_EQ(sizeof(SwitchFeedback), 4u);
    EXPECT_EQ(offsetof(MotorFeedback, encoder_raw_count), 16u);
}

TEST(SerialProtocol, AllTwentyOneFramesRegisterAndRoundTrip) {
    MemorySerial tx, rx;
    SerialBridge sender(&tx, 1024), receiver(&rx, 1024);
    SerialFrames outgoing, incoming;
    ASSERT_NO_THROW(outgoing.register_all(sender));
    ASSERT_NO_THROW(incoming.register_all(receiver));
    for (size_t i=0; i<MOTORS; ++i) {
        outgoing.init[i].data.encoder_ppr = 2048+i;
        outgoing.velocity[i].data.target_motor_rps = -2.5f+i;
        outgoing.position[i].data.target_pos = -(INT64_C(1)<<40)+i;
        outgoing.duty[i].data.duty = -.5f;
        outgoing.feedback[i].data.encoder_count = -123;
        outgoing.feedback[i].data.encoder_raw_count = (INT64_C(1)<<40)+i;
    }
    outgoing.switches.data.state[2] = true;
    for (uint8_t id=0; id<=20; ++id) {
        ASSERT_GT(sender.write(id), 0) << int(id);
        rx.receive(tx.writes.back());
        ASSERT_EQ(receiver.update(), 0) << int(id);
    }
    for (size_t i=0; i<MOTORS; ++i) {
        EXPECT_EQ(incoming.init[i].data.encoder_ppr, 2048+i);
        EXPECT_FLOAT_EQ(incoming.velocity[i].data.target_motor_rps, -2.5f+i);
        EXPECT_EQ(incoming.position[i].data.target_pos, -(INT64_C(1)<<40)+i);
        EXPECT_FLOAT_EQ(incoming.duty[i].data.duty, -.5f);
        EXPECT_EQ(incoming.feedback[i].data.encoder_count, -123);
        EXPECT_EQ(incoming.feedback[i].data.encoder_raw_count, (INT64_C(1)<<40)+i);
    }
    EXPECT_TRUE(incoming.switches.data.state[2]);
    EXPECT_EQ(receiver.update(), -1);
}

TEST(SerialProtocol, DrainKeepsLatestOfFiveFrameBurstsAndConsumesErrors) {
    MemorySerial tx, rx;
    SerialBridge sender(&tx, 1024), receiver(&rx, 1024);
    SerialFrames outgoing, incoming;
    outgoing.register_all(sender); incoming.register_all(receiver);
    // A registered ID with a wrong payload length, then an unknown ID.
    MotorCommandVelPacket wrong;
    wrong.data.target_motor_rps = 0;
    sender.add_frame(16, &wrong);
    sender.write(16); rx.receive(tx.writes.back());
    sender.add_frame(99, &wrong);
    sender.write(99); rx.receive(tx.writes.back());
    outgoing.register_all(sender);
    for (int tick=0; tick<20; ++tick) {
        for (size_t i=0; i<MOTORS; ++i) {
            outgoing.feedback[i].data.encoder_raw_count = tick*100+i;
            sender.write(16+i); rx.receive(tx.writes.back());
        }
        sender.write(20); rx.receive(tx.writes.back());
    }
    const auto result=incoming.drain(receiver);
    EXPECT_EQ(result.frames, 102u);
    EXPECT_EQ(result.invalid, 1u);
    EXPECT_EQ(result.unregistered, 1u);
    EXPECT_EQ(result.status, -3);
    EXPECT_FALSE(result.budget_exhausted);
    EXPECT_TRUE(rx.input.empty());
    EXPECT_TRUE(result.switches_updated);
    for (size_t i=0; i<MOTORS; ++i) {
        EXPECT_TRUE(result.motor_updated[i]);
        EXPECT_EQ(incoming.feedback[i].data.encoder_raw_count, 1900+i);
    }
    EXPECT_EQ(incoming.drain(receiver).status, -1);
}

TEST(SerialProtocol, DrainIsBoundedUnderSustainedInput) {
    MemorySerial tx, rx;
    SerialBridge sender(&tx, 1024), receiver(&rx, 1024);
    SerialFrames outgoing, incoming;
    outgoing.register_all(sender); incoming.register_all(receiver);
    sender.write(20);
    for (int i=0; i<300; ++i) rx.receive(tx.writes.back());
    const auto first=incoming.drain(receiver);
    EXPECT_EQ(first.frames, 256u);
    EXPECT_TRUE(first.budget_exhausted);
    EXPECT_EQ(first.status, 0);
    const auto second=incoming.drain(receiver);
    EXPECT_EQ(second.frames, 44u);
    EXPECT_FALSE(second.budget_exhausted);
    EXPECT_TRUE(rx.input.empty());
}
