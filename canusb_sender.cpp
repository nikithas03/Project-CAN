#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <sstream>

class CANSender {
public:
    CANSender(const std::string& device, int baudrate = 1000000) 
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

    bool sendCANFrame(uint32_t id, const std::vector<uint8_t>& data, bool extended = false) {
        if (fd_ == -1) {
            std::cerr << "Device not open" << std::endl;
            return false;
        }

        if (data.size() > 8) {
            std::cerr << "Data too long (max 8 bytes)" << std::endl;
            return false;
        }

        std::vector<uint8_t> frame;
        frame.push_back(0xAA); // Start byte
        frame.push_back(0xC0 | (data.size() & 0xF)); // Frame info
        if (extended) frame.back() |= 0x20; // Extended frame flag
        if (extended) {
            frame.push_back(id & 0xFF);
            frame.push_back((id >> 8) & 0xFF);
            frame.push_back((id >> 16) & 0xFF);
            frame.push_back((id >> 24) & 0xFF);
        } else {
            frame.push_back(id & 0xFF);
            frame.push_back((id >> 8) & 0xFF);
        }
        frame.insert(frame.end(), data.begin(), data.end());
        frame.push_back(0x55); // End byte

        ssize_t written = write(fd_, frame.data(), frame.size());
        if (written != static_cast<ssize_t>(frame.size())) {
            perror("Failed to write frame");
            return false;
        }

        std::cout << "Sent CAN frame - ID: " << std::hex << std::setw(8) << id 
                  << " (" << (extended ? "EXT" : "STD") << "), Data: ";
        for (auto byte : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;

        return true;
    }

private:
    std::string device_;
    int baudrate_;
    int fd_;
};

std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(strtoul(byteString.c_str(), nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

int main() {
    // Hardcoded parameters
    std::string device = "/dev/ttyUSB1";
    int baudrate = 1000000; // 1 Mbit/s
    uint32_t can_id = 0x701;
    std::string data_hex = "7F";
    bool extended = false;

    // Convert hex data to bytes
    std::vector<uint8_t> data = hexStringToBytes(data_hex);
    if (data.size() > 8) {
        std::cerr << "Error: Data too long, truncated to 8 bytes" << std::endl;
        data.resize(8);
    }

    // Initialize sender
    CANSender can(device, baudrate);
    if (!can.open()) {
        std::cerr << "Error: Failed to initialize CAN sender" << std::endl;
        return 1;
    }

    std::cout << "Sending CAN frames every 1 second on " << device << "..." << std::endl;

    // Send frames every 1 second
    while (true) {
        if (!can.sendCANFrame(can_id, data, extended)) {
            std::cerr << "Error: Failed to send CAN frame" << std::endl;
            can.close();
            return 1;
        }
        sleep(1); // Wait 1 second
    }

    // Cleanup (unreachable unless interrupted)
    can.close();
    return 0;
}
