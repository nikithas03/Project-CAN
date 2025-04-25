#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <sstream>

class CANReceiver {
public:
    CANReceiver(const std::string& device, int baudrate = 1000000)
        : device_(device), baudrate_(baudrate), fd_(-1) {}

    bool open() {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            perror("Failed to open device");
            return false;
        }

        struct termios tio;
        if (tcgetattr(fd_, &tio) == -1) {
            perror("Failed to get terminal settings");
            close();
            return false;
        }

        cfsetispeed(&tio, B1000000);
        cfsetospeed(&tio, B1000000);
        tio.c_cflag = CS8 | CSTOPB | CREAD | CLOCAL;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;

        if (tcsetattr(fd_, TCSANOW, &tio) == -1) {
            perror("Failed to set terminal settings");
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void receive() {
        if (fd_ == -1) {
            std::cerr << "Device not open" << std::endl;
            return;
        }

        std::vector<uint8_t> buffer;
        while (true) {
            uint8_t byte;
            ssize_t n = read(fd_, &byte, 1);
            if (n <= 0) {
                usleep(1000); // Brief sleep to avoid busy-waiting
                continue;
            }

            if (byte == 0xAA) { // Start of frame
                buffer.clear();
                buffer.push_back(byte);
            } else if (!buffer.empty()) {
                buffer.push_back(byte);
                if (byte == 0x55 && buffer.size() >= 5) { // End of frame
                    processFrame(buffer);
                    buffer.clear();
                }
            }
        }
    }

private:
    void processFrame(const std::vector<uint8_t>& frame) {
        if (frame.size() < 5 || frame[0] != 0xAA || frame.back() != 0x55) {
            std::cerr << "Invalid frame format" << std::endl;
            return;
        }

        uint8_t frame_info = frame[1];
        uint8_t dlc = frame_info & 0x0F;
        bool extended = (frame_info & 0x20) != 0;

        if (frame.size() != (size_t)(dlc + (extended ? 7 : 5))) {
            std::cerr << "Frame length mismatch" << std::endl;
            return;
        }

        uint32_t id;
        if (extended) {
            id = (frame[5] << 24) | (frame[4] << 16) | (frame[3] << 8) | frame[2];
        } else {
            id = (frame[3] << 8) | frame[2];
        }

        std::vector<uint8_t> data(frame.begin() + (extended ? 6 : 4), frame.begin() + (extended ? 6 : 4) + dlc);

        // Get timestamp
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        double timestamp = tv.tv_sec + tv.tv_usec / 1000000.0;

        // Print frame
        std::cout << std::fixed << std::setprecision(6) << timestamp
                  << " Frame ID: " << std::hex << std::setw(4) << std::setfill('0') << id
                  << ", Data: ";
        for (auto byte : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;
    }

    std::string device_;
    int baudrate_;
    int fd_;
};

int main() {
    // Hardcoded parameters
    std::string device = "/dev/ttyUSB0";
    int baudrate = 1000000; // 1 Mbit/s

    // Initialize receiver
    CANReceiver can(device, baudrate);
    if (!can.open()) {
        std::cerr << "Error: Failed to initialize CAN receiver" << std::endl;
        return 1;
    }

    std::cout << "Receiving CAN frames on " << device << "..." << std::endl;

    // Receive frames indefinitely
    can.receive();

    // Cleanup (unreachable unless interrupted)
    can.close();
    return 0;
}
