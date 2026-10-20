#include <gtest/gtest.h>
#include <LinuxHardwareSerial.hpp>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <deque>
#include <thread>
#include <string>
#include <vector>

extern "C" int fdwrite(int fd, unsigned char *data, unsigned int len);
extern "C" ssize_t __real_write(int fd, const void *data, size_t len);
extern "C" int __real_poll(pollfd *fds, nfds_t count, int timeout);

namespace {
constexpr int TEST_FD = 123456;
struct WriteResult {
    ssize_t count;
    int error = 0;
};
std::deque<WriteResult> results;
std::vector<unsigned char> transmitted;
int write_calls = 0;
int poll_calls = 0;
int persistent_write_error = 0;
bool interrupt_poll = false;
short poll_events = POLLOUT;

class LinuxSerialWrite : public testing::Test {
protected:
    void SetUp() override {
        results.clear();
        transmitted.clear();
        write_calls = poll_calls = persistent_write_error = 0;
        interrupt_poll = false;
        poll_events = POLLOUT;
    }
    unsigned char packet[9] = {0, 7, 13, 1, 2, 3, 4, 23, 0};
    void expect_complete() {
        EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), static_cast<int>(sizeof(packet)));
        EXPECT_EQ(transmitted, std::vector<unsigned char>(packet, packet + sizeof(packet)));
    }
};
}  // namespace

// Linker wrappers affect only the synthetic fd; other I/O keeps using the OS.
extern "C" ssize_t __wrap_write(int fd, const void *data, size_t len) {
    if (fd != TEST_FD) return __real_write(fd, data, len);
    ++write_calls;
    WriteResult result{static_cast<ssize_t>(len), persistent_write_error};
    if (persistent_write_error) result.count = -1;
    if (!results.empty()) {
        result = results.front();
        results.pop_front();
    }
    if (result.count < 0) {
        errno = result.error;
        return -1;
    }
    const auto count = std::min(len, static_cast<size_t>(result.count));
    const auto *bytes = static_cast<const unsigned char *>(data);
    transmitted.insert(transmitted.end(), bytes, bytes + count);
    return static_cast<ssize_t>(count);
}

extern "C" int __wrap_poll(pollfd *fds, nfds_t count, int timeout) {
    if (count != 1 || fds[0].fd != TEST_FD) return __real_poll(fds, count, timeout);
    ++poll_calls;
    EXPECT_GT(timeout, 0);
    EXPECT_LE(timeout, 10);
    EXPECT_TRUE(fds[0].events & POLLOUT);
    if (interrupt_poll) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        errno = EINTR;
        return -1;
    }
    fds[0].revents = poll_events;
    return 1;
}

TEST_F(LinuxSerialWrite, PartialWritesPreserveTheEntireFrameWithoutDuplication) {
    results = {{2}, {3}, {4}};
    expect_complete();
    EXPECT_EQ(write_calls, 3);
}

TEST_F(LinuxSerialWrite, TemporaryBackpressureAfterAPartialWritePreservesTheFrame) {
    results = {{2}, {-1, EAGAIN}, {3}, {-1, EWOULDBLOCK}, {4}};
    expect_complete();
    EXPECT_EQ(poll_calls, 2);
}

TEST_F(LinuxSerialWrite, InterruptedWriteRetriesTheRemainingBytes) {
    results = {{2}, {-1, EINTR}, {7}};
    expect_complete();
}

TEST_F(LinuxSerialWrite, InterruptedPollDoesNotExtendTheDeadline) {
    persistent_write_error = EAGAIN;
    interrupt_poll = true;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, ETIMEDOUT);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, std::chrono::milliseconds(10));
    EXPECT_LT(elapsed, std::chrono::milliseconds(200));
    EXPECT_GT(poll_calls, 0);
}

TEST_F(LinuxSerialWrite, PersistentInterruptedWritesAlsoHaveABoundedDeadline) {
    persistent_write_error = EINTR;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, ETIMEDOUT);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(200));
}

TEST_F(LinuxSerialWrite, PermanentWriteFailureKeepsTheOriginalError) {
    results = {{2}, {-1, EIO}};
    EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, EIO);
    EXPECT_EQ(write_calls, 2);
    EXPECT_EQ(poll_calls, 0);
}

TEST_F(LinuxSerialWrite, DisconnectDuringBackpressureFailsImmediately) {
    persistent_write_error = EAGAIN;
    poll_events = POLLOUT | POLLHUP;
    EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, EIO);
    EXPECT_EQ(poll_calls, 1);
}

TEST_F(LinuxSerialWrite, InvalidDescriptorDuringBackpressureKeepsBadFdError) {
    persistent_write_error = EAGAIN;
    poll_events = POLLNVAL;
    EXPECT_EQ(fdwrite(TEST_FD, packet, sizeof(packet)), -1);
    EXPECT_EQ(errno, EBADF);
}

TEST_F(LinuxSerialWrite, EmptyWriteDoesNotTouchTheDevice) {
    EXPECT_EQ(fdwrite(TEST_FD, packet, 0), 0);
    EXPECT_EQ(write_calls, 0);
}

TEST_F(LinuxSerialWrite, RealPseudoTerminalBackpressureRecoversWithoutLosingBytes) {
    const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(master, 0);
    struct CloseFd {
        int fd;
        ~CloseFd() { close(fd); }
    } close_master{master};
    ASSERT_EQ(grantpt(master), 0);
    ASSERT_EQ(unlockpt(master), 0);
    ASSERT_NE(ptsname(master), nullptr);
    LinuxHardwareSerial serial(ptsname(master), B230400);

    // Fill the real kernel transmit queue before allowing the peer to read.
    unsigned char filler[1024];
    std::fill(std::begin(filler), std::end(filler), 0x55);
    size_t queued = 0;
    ssize_t result = 0;
    while (queued < 1024 * 1024) {
        result = __real_write(serial.diagnostic_fd(), filler, sizeof(filler));
        if (result < 0) break;
        queued += static_cast<size_t>(result);
    }
    ASSERT_EQ(result, -1);
    ASSERT_EQ(errno, EAGAIN);
    std::vector<unsigned char> received;
    std::thread reader([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        unsigned char buffer[4096];
        while (received.size() < queued + sizeof(packet) && std::chrono::steady_clock::now() < end) {
            const auto count = read(master, buffer, sizeof(buffer));
            if (count > 0) {
                received.insert(received.end(), buffer, buffer + count);
            } else {
                pollfd descriptor{master, POLLIN, 0};
                __real_poll(&descriptor, 1, 1);
            }
        }
    });
    const int sent = serial.write(packet, sizeof(packet));
    reader.join();
    EXPECT_EQ(sent, static_cast<int>(sizeof(packet)));
    std::vector<unsigned char> expected(queued, 0x55);
    expected.insert(expected.end(), packet, packet + sizeof(packet));
    EXPECT_EQ(received, expected);
}
