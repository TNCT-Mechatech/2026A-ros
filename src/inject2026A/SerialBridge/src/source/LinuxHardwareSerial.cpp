/**
 * @file LinuxHardwareSerial.cpp
 * @brief Define the class required for serial communication using SerialBridge in Linux environment.
 * @author Taiyou Komazawa
 * @date 2021/8/18
 */

#if defined(__linux__) || defined(__APPLE__)

#include "../LinuxHardwareSerial.hpp"

#include <system_error>

extern "C" {
int fdwrite (int fd, unsigned char *data, unsigned int len) {
    return write (fd, data, len);
}

int fdread (int fd) {
    unsigned char c[1];
    if (read (fd, c, 1) == 1)
        return *c;
    else
        return 0;
}

long fdtell (int fd) {
    int len = 0;
    if (ioctl (fd, FIONREAD, &len) < 0) {
        throw std::system_error (errno, std::generic_category (), "serial FIONREAD");
    }
    return len;
}
}

/**
 * @brief LinuxHardwareSerial class constructor.
 * @param[in] port Serial device virtual file path.(ex : "/dev/ttyUSB0")
 * @param[in] baud_rate (speed_t) Macro related to baud rate by c_cflag.
 *  (ex : B9600(9600 baudrate), B115200(115200 baudrate))
 */
LinuxHardwareSerial::LinuxHardwareSerial (const char port[], speed_t baud_rate) {
    const int fd = open (port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw std::system_error (errno, std::generic_category (), "open serial port");
    }
    // On every setup failure, release the fd before allowing the node to retry.
    const auto fail = [fd] (const char *operation) {
        const int error = errno;
        close (fd);
        throw std::system_error (error, std::generic_category (), operation);
    };
    if (tcgetattr (fd, &_oldtio) < 0) fail ("tcgetattr");
    struct termios tio = _oldtio;
    cfmakeraw (&tio);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag |= CREAD | CLOCAL | CS8;
    tio.c_iflag |= IGNPAR | IGNBRK;
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN]  = 0;
    if (cfsetispeed (&tio, baud_rate) < 0) fail ("cfsetispeed");
    if (cfsetospeed (&tio, baud_rate) < 0) fail ("cfsetospeed");
    if (tcsetattr (fd, TCSANOW, &tio) < 0) fail ("tcsetattr");
    if (tcflush (fd, TCIOFLUSH) < 0) fail ("tcflush");
    _fd = fd;
}

/**
 * @brief LinuxHardwareSerial class destructor.
 */
LinuxHardwareSerial::~LinuxHardwareSerial () {
    tcsetattr (_fd, TCSANOW, &_oldtio);
    close (_fd);
}

/**
 * @brief Return a received character
 * @return unsigned char
 */
int LinuxHardwareSerial::read () {
    return fdread (_fd);
}

/**
 * @brief Return number of available characters
 * @return int of available chracter.
 */
int LinuxHardwareSerial::readable_len () {
    return (int)fdtell (_fd);
}

/**
 * @brief A function of write array of charaters
 * @param[in] data (characters pointer) characters
 * @param[in] len (int) length of characters
 * @return int length of characters
 */
int LinuxHardwareSerial::write (unsigned char *data, unsigned int len) {
    return fdwrite (_fd, data, len);
}

#endif  // #ifdef __linux__
