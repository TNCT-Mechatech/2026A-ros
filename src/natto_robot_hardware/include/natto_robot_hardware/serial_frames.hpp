#pragma once

#include "configuration_state.hpp"
#include "serial_protocol.hpp"

#include <SerialBridge.hpp>

#include <array>
#include <cstring>
#include <stdexcept>
#include <string>

namespace natto_serial {
struct DrainResult {
    int                      status           = -1;
    size_t                   frames           = 0;
    size_t                   invalid          = 0;
    size_t                   unregistered     = 0;
    bool                     budget_exhausted = false;
    std::array<bool, MOTORS> motor_updated{};
    bool                     switches_updated = false;
    std::array<bool, MOTORS> ack_updated{};
    bool                     configuration_lost = false;
};

// Packet storage owns all Message objects for the lifetime of SerialBridge.
struct SerialFrames {
    std::array<InitializationPacket, MOTORS>    init;
    std::array<MotorCommandVelPacket, MOTORS>   velocity;
    std::array<MotorCommandPosPacket, MOTORS>   position;
    std::array<MotorCommandDutyPacket, MOTORS>  duty;
    std::array<MotorFeedbackPacket, MOTORS>     feedback;
    std::array<InitializationAckPacket, MOTORS> ack;
    SwitchFeedbackPacket                        switches;
    ConfigurationState                          configuration;

    SerialFrames () {
        reset ();
    }
    void reset () {
        for (size_t i = 0; i < MOTORS; ++i) {
            clear (init[i]);
            clear (velocity[i]);
            clear (position[i]);
            clear (duty[i]);
            clear (feedback[i]);
            clear (ack[i]);
        }
        clear (switches);
    }
    void reset_received () {
        for (auto &packet : feedback) clear (packet);
        for (auto &packet : ack) clear (packet);
        clear (switches);
    }
    void register_all (SerialBridge &serial) {
        for (size_t i = 0; i < MOTORS; ++i) {
            add (serial, INIT_FRAME_ID_BASE + i, init[i]);
            add (serial, CMD_VEL_FRAME_ID_BASE + i, velocity[i]);
            add (serial, CMD_POS_FRAME_ID_BASE + i, position[i]);
            add (serial, CMD_DUTY_FRAME_ID_BASE + i, duty[i]);
            add (serial, FEEDBACK_FRAME_ID_BASE + i, feedback[i]);
            add (serial, INIT_ACK_FRAME_ID_BASE + i, ack[i]);
        }
        add (serial, SWITCH_FEEDBACK_FRAME_ID, switches);
    }
    static int write (SerialBridge &serial, uint8_t id, sb::MessageInterface &packet) {
        const int result = serial.write (id);
        if (result != packet.size () + 3) throw std::runtime_error ("failed/short serial write for frame " + std::to_string (id));
        return result;
    }
    void initialize (SerialBridge &serial, ConfigurationState::Time now = ConfigurationState::Clock::now ()) {
        configuration.begin (init, now);
        service_initialization (serial, now);
    }
    void service_initialization (SerialBridge &serial, ConfigurationState::Time now = ConfigurationState::Clock::now ()) {
        configuration.check_timeout (now);
        if (configuration.failed ()) throw std::runtime_error ("STM " + configuration.error ());
        for (size_t i = 0; i < MOTORS; ++i)
            if (configuration.due (i, now)) {
                init[i].data = configuration.requested (i);
                write (serial, INIT_FRAME_ID_BASE + i, init[i]);
                configuration.sent (i, now);
            }
    }
    DrainResult drain (SerialBridge &serial) {
        DrainResult result;
        // Each update consumes at most one frame. Bound work even under continuous
        // traffic; 256 frames is well above 10ms worth at 230400 baud.
        for (size_t n = 0; n < 256; ++n) {
            const int status = serial.update ();
            if (status == -1) return result;
            if (status < -3 || status > 0) throw std::runtime_error ("unexpected serial.update result");
            ++result.frames;
            result.invalid += status == -2;
            result.unregistered += status == -3;
            // Preserve an error in a mixed batch; the terminating -1 is normal.
            if (result.status >= -1 || status < 0) result.status = status;
            for (size_t i = 0; i < MOTORS; ++i) {
                const bool ready = configuration.ready ();
                if (ack[i].was_updated ()) {
                    result.ack_updated[i] = true;
                    configuration.observe_ack (i, ack[i].data, ConfigurationState::Clock::now ());
                }
                if (feedback[i].was_updated ()) {
                    result.motor_updated[i] = true;
                    configuration.observe_feedback (i, feedback[i].data, ConfigurationState::Clock::now ());
                }
                result.configuration_lost |= ready && !configuration.ready ();
            }
            result.switches_updated = switches.was_updated () || result.switches_updated;
        }
        result.budget_exhausted = true;
        return result;
    }

   private:
    template <class Packet>
    static void clear (Packet &packet) {
        std::memset (&packet.data, 0, sizeof (packet.data));
        packet.was_updated ();
    }
    static void add (SerialBridge &serial, uint8_t id, sb::MessageInterface &packet) {
        if (serial.add_frame (id, &packet) != 0) throw std::runtime_error ("cannot register STM frame " + std::to_string (id));
    }
};
}  // namespace natto_serial
