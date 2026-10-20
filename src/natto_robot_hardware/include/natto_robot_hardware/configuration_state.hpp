#pragma once
#include "serial_protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace natto_serial {
// One sender per serial port. All access belongs to the node's mutually exclusive
// callback group. Retries preserve both identity and the frozen configuration.
class ConfigurationState {
   public:
    using Clock        = std::chrono::steady_clock;
    using Time         = Clock::time_point;
    using Milliseconds = std::chrono::milliseconds;

    explicit ConfigurationState (uint32_t session = fresh_session ()) : session_ (session) {
        if (!session) throw std::invalid_argument ("configuration session_id must be nonzero");
        results_.fill (-1);
    }
    void set_timeouts (Milliseconds retry, Milliseconds timeout) {
        if (retry.count () <= 0 || timeout <= retry) throw std::invalid_argument ("initialization_timeout_ms must exceed positive initialization_retry_ms");
        retry_   = retry;
        timeout_ = timeout;
    }
    void begin (std::array<InitializationPacket, MOTORS> &packets, Time now) {
        if (sequence_ == std::numeric_limits<uint32_t>::max ()) {
            const auto old = session_;
            do {
                session_ = fresh_session ();
            } while (session_ == old);
            sequence_ = 0;
        }
        ++sequence_;
        confirmed_.fill (false);
        attempts_.fill (0);
        results_.fill (-1);
        applied_session_.fill (0);
        applied_config_.fill (0);
        error_.clear ();
        active_  = true;
        started_ = now;
        for (size_t i = 0; i < MOTORS; ++i) {
            packets[i].data.session_id = session_;
            packets[i].data.config_id  = sequence_;
            requested_[i]              = packets[i].data;
        }
    }
    bool ready () const {
        return active_ && error_.empty () && std::all_of (confirmed_.begin (), confirmed_.end (), [] (bool value) { return value; });
    }
    bool active () const {
        return active_;
    }
    bool failed () const {
        return !error_.empty ();
    }
    const std::string &error () const {
        return error_;
    }
    uint32_t session_id () const {
        return session_;
    }
    uint32_t config_id (size_t i) const {
        return requested_.at (i).config_id;
    }
    bool confirmed (size_t i) const {
        return confirmed_.at (i);
    }
    unsigned attempts (size_t i) const {
        return attempts_.at (i);
    }
    int result (size_t i) const {
        return results_.at (i);
    }
    uint32_t applied_session_id (size_t i) const {
        return applied_session_.at (i);
    }
    uint32_t applied_config_id (size_t i) const {
        return applied_config_.at (i);
    }
    const initialization_packet_t &requested (size_t i) const {
        return requested_.at (i);
    }
    const char *state () const {
        return failed () ? "Configuration failed" : ready () ? "Configuration applied" : "Waiting for configuration";
    }
    void observe_ack (size_t i, const initialization_ack_packet_t &ack, Time now) {
        if (!active_ || failed () || ack.session_id != session_ || ack.config_id != config_id (i)) return;
        results_[i]         = static_cast<uint8_t> (ack.result);
        applied_session_[i] = ack.applied_session_id;
        applied_config_[i]  = ack.applied_config_id;
        if (results_[i] > 1 || ack.reserved[0] || ack.reserved[1] || ack.reserved[2]) {
            error_ = "motor " + std::to_string (i) + " initialization rejected (result=" + std::to_string (results_[i]) + ")";
            confirmed_.fill (false);
            return;
        }
        observe_identity (i, ack.applied_session_id, ack.applied_config_id, now);
    }
    void observe_feedback (size_t i, const motor_feedback_packet_t &feedback, Time now) {
        if (active_ && !failed ()) observe_identity (i, feedback.applied_session_id, feedback.applied_config_id, now);
    }
    void check_timeout (Time now) {
        if (active_ && !ready () && !failed () && now - started_ >= timeout_) {
            error_ = "configuration confirmation timed out; pending motors:";
            for (size_t i = 0; i < MOTORS; ++i)
                if (!confirmed_[i]) error_ += " " + std::to_string (i);
        }
    }
    bool due (size_t i, Time now) const {
        return active_ && !failed () && !confirmed_[i] && (attempts_[i] == 0 || now - sent_[i] >= retry_);
    }
    void sent (size_t i, Time now) {
        sent_[i] = now;
        ++attempts_[i];
    }

   private:
    static uint32_t fresh_session () {
        std::random_device random;
        uint32_t           value;
        do {
            value = static_cast<uint32_t> (random ());
        } while (!value);
        return value;
    }
    void observe_identity (size_t i, uint32_t session, uint32_t config, Time now) {
        const bool was_ready = ready ();
        applied_session_[i]  = session;
        applied_config_[i]   = config;
        confirmed_[i]        = session == session_ && config == config_id (i);
        if (was_ready && !confirmed_[i]) {
            started_ = now;
            // The board may have reset without closing the USB device. Retry the
            // same request; STM deduplicates it if the setting is still present.
            for (auto &stamp : sent_) stamp = now - retry_;
        }
    }
    uint32_t                                    session_, sequence_{0};
    bool                                        active_{false};
    Milliseconds                                retry_{100}, timeout_{2000};
    Time                                        started_{};
    std::array<Time, MOTORS>                    sent_{};
    std::array<bool, MOTORS>                    confirmed_{};
    std::array<unsigned, MOTORS>                attempts_{};
    std::array<int, MOTORS>                     results_{};
    std::array<uint32_t, MOTORS>                applied_session_{}, applied_config_{};
    std::array<initialization_packet_t, MOTORS> requested_{};
    std::string                                 error_;
};
}  // namespace natto_serial
