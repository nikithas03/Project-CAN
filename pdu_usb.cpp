#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <poll.h>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <random>
#include <chrono>

// Constants for PDU
const uint32_t PDU_COMMAND_ID = 0xC77E00F1;
const uint32_t PDU_RESPONSE_ID = 0x18EFF160;
const uint32_t PDU_ALERT_ID = 0x18FFF160;

// PDU Enums
enum PduOpCode {
    CHANNEL_CONTROL_1_CMD = 0x01,
    CHANNEL_CONTROL_1_RSP = 0x02,
    CHANNEL_CONTROL_2_CMD = 0x03,
    CHANNEL_CONTROL_2_RSP = 0x04,
    RESET_CMD = 0x07,
    RESET_RSP = 0x08,
    CHANNEL_STATUS_1_CMD = 0x21,
    CHANNEL_STATUS_1_RSP = 0x22,
    CHANNEL_STATUS_2_CMD = 0x25,
    CHANNEL_STATUS_2_RSP = 0x26,
    INPUT_STATUS_CMD = 0x27,
    INPUT_STATUS_RSP = 0x28,
    TEMPERATURE_CMD = 0x33,
    TEMPERATURE_RSP = 0x34,
    GROUPED_CHANNELS_CMD = 0x35,
    GROUPED_CHANNELS_RSP = 0x36,
    TRIP_ALERT = 0x50
};

enum PduStatusCode {
    PDU_SUCCESS = 0x00,
    GENERAL_RW_ERROR = 0x01,
    READ_NOT_SUPPORTED = 0x02,
    WRITE_NOT_SUPPORTED = 0x03,
    ERROR_WRITING_FLASH = 0x05,
    WRONG_ELEMENT = 0x07,
    CHANNEL_DOESNT_EXIST = 0x08,
    GROUP_DOESNT_EXIST = 0x09,
    SENSOR_DOESNT_EXIST = 0x0A,
    BOARD_DOESNT_EXIST = 0x0B,
    WRONG_ADDRESS = 0x0C,
    WRONG_PAGE = 0x0D,
    WRONG_STATE = 0x14,
    WRONG_FLASH_INDEX = 0x17,
    WRONG_FLASH_KEY = 0x18,
    GROUP_IS_EMPTY = 0x20,
    CURRENT_OUT_OF_BOUNDS = 0x30,
    OPCODE_DOESNT_EXIST = 0x3E,
    GENERAL_ERROR = 0x3F
};

enum PduRWFlag {
    READ = 0,
    WRITE = 1,
    MACRO = 2
};

// Simulated CAN frame structure
struct CanFrame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

// Channel state structure
struct ChannelState {
    bool isOn;
    bool isBattle;
    bool isTripped;
    float current;
    float voltage;
    uint8_t groupId;
};

// PDU Simulator class
class PduSimulator {
public:
    PduSimulator(const std::string& device = "/dev/ttyUSB1", int baudrate = 1000000)
        : serial_fd_(-1), device_(device), baudrate_(baudrate), running_(true),
          rd_(), gen_(rd_()), currentAddress_(0) {
        initialize_serial();
        initialize_channels();
        start_threads();
    }

    ~PduSimulator() {
        running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (send_thread_.joinable()) send_thread_.join();
        close_serial();
    }

private:
    int serial_fd_;
    std::string device_;
    int baudrate_;
    std::atomic<bool> running_;
    std::thread receive_thread_;
    std::thread send_thread_;
    std::mutex send_mutex_;
    std::vector<ChannelState> channels_;
    float input_current_;
    float input_voltage_;
    float temperature_;
    std::vector<uint16_t> group_memberships_;
    std::random_device rd_;
    std::mt19937 gen_;
    uint8_t currentAddress_;

    bool initialize_serial() {
        serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ == -1) {
            std::cerr << "Failed to open serial device " << device_ << ": " << strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Serial device opened: " << device_ << std::endl;

        struct termios tio;
        if (tcgetattr(serial_fd_, &tio) == -1) {
            std::cerr << "Failed to get terminal attributes: " << strerror(errno) << std::endl;
            ::close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        std::cout << "Got terminal attributes" << std::endl;

        speed_t baud = (baudrate_ == 500000) ? B500000 : B1000000;
        cfsetispeed(&tio, baud);
        cfsetospeed(&tio, baud);
        tio.c_cflag = CS8 | CREAD | CLOCAL;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 1;

        if (tcsetattr(serial_fd_, TCSANOW, &tio) == -1) {
            std::cerr << "Failed to set terminal attributes: " << strerror(errno) << std::endl;
            ::close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }
        std::cout << "Set terminal attributes" << std::endl;

        std::vector<std::string> init_cmds = {
            "\r\nZ1\r\n", // Normal mode
            "\r\nO\r\n"   // Open CAN channel
            // Removed "\r\nL1\r\n" to disable loopback mode
        };
        for (const auto& cmd : init_cmds) {
            std::cout << "Sending init command: " << cmd.substr(2, cmd.size() - 4) << std::endl;
            ssize_t written = write(serial_fd_, cmd.c_str(), cmd.size());
            if (written < 0) {
                std::cerr << "Failed to send init command '" << cmd.substr(2, cmd.size() - 4)
                          << "': " << strerror(errno) << std::endl;
                ::close(serial_fd_);
                serial_fd_ = -1;
                return false;
            }
            if (written != static_cast<ssize_t>(cmd.size())) {
                std::cerr << "Incomplete write for init command '" << cmd.substr(2, cmd.size() - 4)
                          << "': wrote " << written << " of " << cmd.size() << " bytes" << std::endl;
                ::close(serial_fd_);
                serial_fd_ = -1;
                return false;
            }
            std::cout << "Wrote " << written << " bytes for init command" << std::endl;
            usleep(100000);
        }

        tcflush(serial_fd_, TCIOFLUSH);
        std::cout << "Serial port opened on " << device_ << " at " << baudrate_ << " bps" << std::endl;
        return true;
    }

    void close_serial() {
        if (serial_fd_ != -1) {
            ::close(serial_fd_);
            serial_fd_ = -1;
        }
    }

    void initialize_channels() {
        channels_.resize(8, {false, false, false, 0.0f, 0.0f, 251});
        group_memberships_.resize(50, 0);
        input_current_ = 0.0f;
        input_voltage_ = 24.0f;
        temperature_ = 25.0f;

        // Assign channels 0-3 to group 0, channels 4-7 to group 1
        for (int i = 0; i < 4; ++i) {
            channels_[i].groupId = 100;
            group_memberships_[0] |= (1 << (i * 2));
        }
        for (int i = 4; i < 8; ++i) {
            channels_[i].groupId = 101;
            group_memberships_[1] |= (1 << (i * 2));
        }
    }

    bool send_frame(const CanFrame& frame) {
        if (serial_fd_ == -1) {
            std::cerr << "Serial device not open" << std::endl;
            return false;
        }

        if (frame.can_dlc > 8) {
            std::cerr << "Data too long (max 8 bytes)" << std::endl;
            return false;
        }

        std::vector<uint8_t> serial_frame;
        serial_frame.push_back(0xAA); // Start byte
        serial_frame.push_back(0xC0 | (frame.can_dlc & 0x0F) | 0x20); // Frame info (extended ID)
        serial_frame.push_back(frame.can_id & 0xFF);
        serial_frame.push_back((frame.can_id >> 8) & 0xFF);
        serial_frame.push_back((frame.can_id >> 16) & 0xFF);
        serial_frame.push_back((frame.can_id >> 24) & 0xFF);
        for (uint8_t i = 0; i < frame.can_dlc; ++i) {
            serial_frame.push_back(frame.data[i]);
        }
        serial_frame.push_back(0x55); // End byte

        std::cout << "Sending frame: ";
        for (const auto& byte : serial_frame) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
        }
        std::cout << std::dec << std::endl;

        std::lock_guard<std::mutex> lock(send_mutex_);
        ssize_t written = write(serial_fd_, serial_frame.data(), serial_frame.size());
        if (written != static_cast<ssize_t>(serial_frame.size())) {
            std::cerr << "Failed to write frame: " << strerror(errno)
                      << " (wrote " << written << " of " << serial_frame.size() << " bytes)" << std::endl;
            return false;
        }

        log_frame("TX", frame.can_id, std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc));
        return true;
    }

    void log_frame(const std::string& direction, uint32_t id, const std::vector<uint8_t>& data) {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        auto time = std::chrono::system_clock::to_time_t(now);
        std::cout << std::put_time(std::localtime(&time), "%F %T") << "."
                  << std::setw(3) << std::setfill('0') << ms.count() << " "
                  << direction << " - ID: 0x" << std::hex << std::setw(8) << std::setfill('0')
                  << id << " (EXT), Data: ";
        for (const auto& byte : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
        }
        std::cout << std::dec << std::endl;
    }

    void start_threads() {
        receive_thread_ = std::thread([this]() {
            std::vector<uint8_t> buffer;
            struct pollfd fds[1];
            fds[0].fd = serial_fd_;
            fds[0].events = POLLIN;

            while (running_) {
                int ret = poll(fds, 1, 100);
                if (ret < 0) {
                    std::cerr << "Poll error: " << strerror(errno) << std::endl;
                    continue;
                }
                if (ret == 0 || !(fds[0].revents & POLLIN)) {
                    continue;
                }

                uint8_t bytes[128];
                ssize_t n = read(serial_fd_, bytes, sizeof(bytes));
                if (n < 0) {
                    std::cerr << "Read error: " << strerror(errno) << std::endl;
                    continue;
                }
                if (n == 0) continue;

                // Debug: Log raw bytes before parsing
                std::cout << "Raw received " << n << " bytes: ";
                for (ssize_t i = 0; i < n; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)bytes[i] << " ";
                }
                std::cout << std::dec << std::endl;

                for (ssize_t i = 0; i < n; ++i) {
                    uint8_t byte = bytes[i];
                    if (byte == 0xAA) {
                        buffer.clear();
                        buffer.push_back(byte);
                    } else if (!buffer.empty()) {
                        buffer.push_back(byte);
                        if (byte == 0x55 && buffer.size() >= 7) {
                            if (buffer[0] != 0xAA || buffer.back() != 0x55) {
                                std::cerr << "Invalid frame format (start/end bytes)" << std::endl;
                                buffer.clear();
                                continue;
                            }

                            uint8_t frame_info = buffer[1];
                            uint8_t dlc = frame_info & 0x0F;
                            bool extended = (frame_info & 0x20) != 0;

                            size_t expected_size = dlc + (extended ? 7 : 5);
                            if (buffer.size() != expected_size) {
                                std::cerr << "Frame length mismatch: expected " << expected_size
                                          << ", got " << buffer.size() << std::endl;
                                buffer.clear();
                                continue;
                            }

                            CanFrame frame;
                            if (extended) {
                                frame.can_id = (buffer[5] << 24) | (buffer[4] << 16) | (buffer[3] << 8) | buffer[2];
                                frame.can_dlc = dlc;
                                for (uint8_t j = 0; j < dlc; ++j) {
                                    frame.data[j] = buffer[6 + j];
                                }
                            } else {
                                frame.can_id = (buffer[3] << 8) | buffer[2];
                                frame.can_dlc = dlc;
                                for (uint8_t j = 0; j < dlc; ++j) {
                                    frame.data[j] = buffer[4 + j];
                                }
                            }

                            log_frame("RX", frame.can_id, std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc));
                            process_command(frame);
                            buffer.clear();
                        }
                    }
                }
            }
        });

        send_thread_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    void process_command(const CanFrame& frame) {
        // Relax CAN ID check to match lower 29 bits
        if ((frame.can_id & 0x1FFFFFFF) != (PDU_COMMAND_ID & 0x1FFFFFFF) || frame.can_dlc < 3) {
            std::cerr << "Invalid command frame: ID=0x" << std::hex << frame.can_id
                      << ", DLC=" << std::dec << static_cast<int>(frame.can_dlc) << std::endl;
            return;
        }

        uint8_t opCode = frame.data[0];
        uint8_t rwFlag = frame.data[1] & 0x03;
        uint8_t element = frame.data[2];

        // Extract address from CAN ID
        uint8_t address = (frame.can_id >> 8) & 0x07;
        if (address != currentAddress_) {
            std::cerr << "Address mismatch: Frame address=" << static_cast<int>(address)
                      << ", PDU address=" << static_cast<int>(currentAddress_) << std::endl;
            sendResponse(opCode + 1, WRONG_ADDRESS, element, {});
            return;
        }

        std::cout << "Received command - OpCode: 0x" << std::hex << static_cast<int>(opCode)
                  << ", RWFlag: " << static_cast<int>(rwFlag) << ", Element: " << static_cast<int>(element)
                  << ", Address: " << static_cast<int>(address) << std::dec << std::endl;

        switch (opCode) {
            case CHANNEL_CONTROL_1_CMD:
                processChannelControl1(rwFlag, element, frame.data[3]);
                break;
            case CHANNEL_CONTROL_2_CMD:
                processChannelControl2(rwFlag, element, frame.data[3]);
                break;
            case RESET_CMD:
                processReset(rwFlag);
                break;
            case CHANNEL_STATUS_1_CMD:
                processChannelStatus1(rwFlag, element);
                break;
            case CHANNEL_STATUS_2_CMD:
                processChannelStatus2(rwFlag, element);
                break;
            case INPUT_STATUS_CMD:
                processInputStatus(rwFlag, element);
                break;
            case TEMPERATURE_CMD:
                processTemperature(rwFlag, element);
                break;
            case GROUPED_CHANNELS_CMD:
                processGroupedChannels(rwFlag, element);
                break;
            default:
                sendResponse(opCode + 1, OPCODE_DOESNT_EXIST, element, {});
                std::cerr << "Unsupported OpCode: 0x" << std::hex << static_cast<int>(opCode) << std::dec << std::endl;
                break;
        }
    }

    void sendResponse(uint8_t opCode, uint8_t status, uint8_t element, const std::vector<uint8_t>& data) {
        CanFrame response;
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        memset(response.data, 0, 8);
        response.data[0] = opCode;
        response.data[1] = (status << 2); // Match VCU's extractStatus shift
        response.data[2] = element;
        for (size_t i = 0; i < data.size() && i < 5; ++i) {
            response.data[3 + i] = data[i];
        }
        send_frame(response);
    }

    void sendAlert(uint8_t opCode, uint8_t channelId) {
        CanFrame alert;
        alert.can_id = PDU_ALERT_ID;
        alert.can_dlc = 8;
        memset(alert.data, 0, 8);
        alert.data[0] = opCode;
        alert.data[2] = channelId;
        send_frame(alert);
        std::cout << "Sent Overload Alert: Channel " << static_cast<int>(channelId)
                  << " tripped (Current=" << channels_[channelId].current << "A)" << std::endl;
    }

    void processChannelControl1(uint8_t rwFlag, uint8_t element, uint8_t state) {
        if (rwFlag != WRITE && rwFlag != MACRO) {
            sendResponse(CHANNEL_CONTROL_1_RSP, WRITE_NOT_SUPPORTED, element, {});
            return;
        }

        bool isGroup = element >= 100;
        uint8_t status = PDU_SUCCESS;

        if (isGroup) {
            uint8_t groupId = element - 100;
            if (groupId >= 50) {
                sendResponse(CHANNEL_CONTROL_1_RSP, GROUP_DOESNT_EXIST, element, {});
                return;
            }
            if (group_memberships_[groupId] == 0) {
                sendResponse(CHANNEL_CONTROL_1_RSP, GROUP_IS_EMPTY, element, {});
                return;
            }
            for (int i = 0; i < 8; ++i) {
                if ((group_memberships_[groupId] >> (i * 2)) & 0x03) {
                    channels_[i].isOn = (state != 0);
                    channels_[i].isTripped = false;
                    updateChannelValues(i);
                }
            }
        } else {
            if (element >= 8) {
                sendResponse(CHANNEL_CONTROL_1_RSP, CHANNEL_DOESNT_EXIST, element, {});
                return;
            }
            channels_[element].isOn = (state != 0);
            channels_[element].isTripped = false;
            updateChannelValues(element);
        }

        sendResponse(CHANNEL_CONTROL_1_RSP, status, element, {state});
        std::cout << "Channel Control 1: Element=" << static_cast<int>(element)
                  << ", State=" << (state ? "ON" : "OFF") << ", Status=" << static_cast<int>(status) << std::endl;
    }

    void processChannelControl2(uint8_t rwFlag, uint8_t element, uint8_t battleState) {
        if (rwFlag != WRITE && rwFlag != MACRO) {
            sendResponse(CHANNEL_CONTROL_2_RSP, WRITE_NOT_SUPPORTED, element, {});
            return;
        }

        bool isGroup = element >= 100;
        uint8_t status = PDU_SUCCESS;
        bool enableBattle = (battleState == 0x0D);

        if (isGroup) {
            uint8_t groupId = element - 100;
            if (groupId >= 50) {
                sendResponse(CHANNEL_CONTROL_2_RSP, GROUP_DOESNT_EXIST, element, {});
                return;
            }
            if (group_memberships_[groupId] == 0) {
                sendResponse(CHANNEL_CONTROL_2_RSP, GROUP_IS_EMPTY, element, {});
                return;
            }
            for (int i = 0; i < 8; ++i) {
                if ((group_memberships_[groupId] >> (i * 2)) & 0x03) {
                    channels_[i].isBattle = enableBattle;
                }
            }
        } else {
            if (element >= 8) {
                sendResponse(CHANNEL_CONTROL_2_RSP, CHANNEL_DOESNT_EXIST, element, {});
                return;
            }
            channels_[element].isBattle = enableBattle;
        }

        sendResponse(CHANNEL_CONTROL_2_RSP, status, element, {battleState});
        std::cout << "Channel Control 2: Element=" << static_cast<int>(element)
                  << ", Battle=" << (enableBattle ? "ENABLED" : "DISABLED")
                  << ", Status=" << static_cast<int>(status) << std::endl;
    }

    void processReset(uint8_t rwFlag) {
        if (rwFlag != WRITE) {
            sendResponse(RESET_RSP, WRITE_NOT_SUPPORTED, 0, {});
            return;
        }

        for (auto& channel : channels_) {
            channel.isOn = false;
            channel.isBattle = false;
            channel.isTripped = false;
            channel.current = 0.0f;
            channel.voltage = 0.0f;
        }
        input_current_ = 0.0f;
        input_voltage_ = 24.0f;
        temperature_ = 25.0f;

        sendResponse(RESET_RSP, PDU_SUCCESS, 0, {});
        std::cout << "PDU Reset: Status=0" << std::endl;
    }

    void processChannelStatus1(uint8_t rwFlag, uint8_t element) {
        if (rwFlag != READ) {
            sendResponse(CHANNEL_STATUS_1_RSP, READ_NOT_SUPPORTED, element, {});
            return;
        }

        bool isGroup = element >= 100;
        uint8_t status = PDU_SUCCESS;

        if (isGroup) {
            uint8_t groupId = element - 100;
            if (groupId >= 50) {
                sendResponse(CHANNEL_STATUS_1_RSP, GROUP_DOESNT_EXIST, element, {});
                return;
            }
            if (group_memberships_[groupId] == 0) {
                sendResponse(CHANNEL_STATUS_1_RSP, GROUP_IS_EMPTY, element, {});
                return;
            }
            bool anyOn = false;
            float totalCurrent = 0.0f;
            float avgVoltage = 0.0f;
            int activeChannels = 0;
            for (int i = 0; i < 8; ++i) {
                if ((group_memberships_[groupId] >> (i * 2)) & 0x03) {
                    updateChannelValues(i);
                    if (channels_[i].isOn) {
                        anyOn = true;
                        totalCurrent += channels_[i].current;
                        avgVoltage += channels_[i].voltage;
                        activeChannels++;
                    }
                }
            }
            avgVoltage = activeChannels > 0 ? avgVoltage / activeChannels : 0.0f;
            uint16_t currentRaw = static_cast<uint16_t>(totalCurrent / 0.001f);
            uint16_t voltageRaw = static_cast<uint16_t>((avgVoltage + 1606.0f) / 0.05f);
            std::vector<uint8_t> data = {
                static_cast<uint8_t>(anyOn ? 1 : 0),
                static_cast<uint8_t>(currentRaw >> 8),
                static_cast<uint8_t>(currentRaw & 0xFF),
                static_cast<uint8_t>(voltageRaw >> 8),
                static_cast<uint8_t>(voltageRaw & 0xFF)
            };
            sendResponse(CHANNEL_STATUS_1_RSP, status, element, data);
            std::cout << "Channel Status 1: Element=" << static_cast<int>(element)
                      << ", On=" << anyOn << ", Current=" << totalCurrent << "A, Voltage=" << avgVoltage
                      << "V, Status=" << static_cast<int>(status) << std::endl;
        } else {
            if (element >= 8) {
                sendResponse(CHANNEL_STATUS_1_RSP, CHANNEL_DOESNT_EXIST, element, {});
                return;
            }
            updateChannelValues(element);
            float current = channels_[element].isOn ? channels_[element].current : 0.0f;
            float voltage = channels_[element].isOn ? channels_[element].voltage : 0.0f;
            if (current > 25.0f && !channels_[element].isBattle && !channels_[element].isTripped) {
                channels_[element].isTripped = true;
                channels_[element].isOn = false;
                channels_[element].current = 0.0f;
                channels_[element].voltage = 0.0f;
                sendAlert(TRIP_ALERT, element);
                current = 0.0f;
                voltage = 0.0f;
            }
            uint16_t currentRaw = static_cast<uint16_t>(current / 0.001f);
            uint16_t voltageRaw = static_cast<uint16_t>((voltage + 1606.0f) / 0.05f);
            std::vector<uint8_t> data = {
                static_cast<uint8_t>(channels_[element].isOn ? 1 : 0),
                static_cast<uint8_t>(currentRaw >> 8),
                static_cast<uint8_t>(currentRaw & 0xFF),
                static_cast<uint8_t>(voltageRaw >> 8),
                static_cast<uint8_t>(voltageRaw & 0xFF)
            };
            sendResponse(CHANNEL_STATUS_1_RSP, status, element, data);
            std::cout << "Channel Status 1: Element=" << static_cast<int>(element)
                      << ", On=" << channels_[element].isOn << ", Current=" << current
                      << "A, Voltage=" << voltage << "V, Status=" << static_cast<int>(status) << std::endl;
        }
    }

    void processChannelStatus2(uint8_t rwFlag, uint8_t element) {
        if (rwFlag != READ) {
            sendResponse(CHANNEL_STATUS_2_RSP, READ_NOT_SUPPORTED, element, {});
            return;
        }

        bool isGroup = element >= 100;
        uint8_t status = PDU_SUCCESS;

        if (isGroup) {
            uint8_t groupId = element - 100;
            if (groupId >= 50) {
                sendResponse(CHANNEL_STATUS_2_RSP, GROUP_DOESNT_EXIST, element, {});
                return;
            }
            if (group_memberships_[groupId] == 0) {
                sendResponse(CHANNEL_STATUS_2_RSP, GROUP_IS_EMPTY, element, {});
                return;
            }
            bool anyOn = false;
            bool anyTripped = false;
            bool anyBattle = false;
            for (int i = 0; i < 8; ++i) {
                if ((group_memberships_[groupId] >> (i * 2)) & 0x03) {
                    updateChannelValues(i);
                    anyOn |= channels_[i].isOn;
                    anyTripped |= channels_[i].isTripped;
                    anyBattle |= channels_[i].isBattle;
                }
            }
            uint16_t stateBits = (anyOn ? 0x01 : 0) | (anyTripped ? (1 << 8) : 0) | (anyBattle ? (1 << 12) : 0);
            std::vector<uint8_t> data = {
                static_cast<uint8_t>(stateBits >> 8),
                static_cast<uint8_t>(stateBits & 0xFF),
                static_cast<uint8_t>(element)
            };
            sendResponse(CHANNEL_STATUS_2_RSP, status, element, data);
            std::cout << "Channel Status 2: Element=" << static_cast<int>(element)
                      << ", On=" << anyOn << ", Tripped=" << anyTripped << ", Battle=" << anyBattle
                      << ", Group=" << static_cast<int>(element) << ", Status=" << static_cast<int>(status) << std::endl;
        } else {
            if (element >= 8) {
                sendResponse(CHANNEL_STATUS_2_RSP, CHANNEL_DOESNT_EXIST, element, {});
                return;
            }
            updateChannelValues(element);
            if (channels_[element].current > 25.0f && !channels_[element].isBattle && !channels_[element].isTripped) {
                channels_[element].isTripped = true;
                channels_[element].isOn = false;
                channels_[element].current = 0.0f;
                channels_[element].voltage = 0.0f;
                sendAlert(TRIP_ALERT, element);
            }
            uint16_t stateBits = (channels_[element].isOn ? 0x01 : 0) |
                                 (channels_[element].isTripped ? (1 << 8) : 0) |
                                 (channels_[element].isBattle ? (1 << 12) : 0);
            std::vector<uint8_t> data = {
                static_cast<uint8_t>(stateBits >> 8),
                static_cast<uint8_t>(stateBits & 0xFF),
                channels_[element].groupId
            };
            sendResponse(CHANNEL_STATUS_2_RSP, status, element, data);
            std::cout << "Channel Status 2: Element=" << static_cast<int>(element)
                      << ", On=" << channels_[element].isOn << ", Tripped=" << channels_[element].isTripped
                      << ", Battle=" << channels_[element].isBattle << ", Group=" << static_cast<int>(channels_[element].groupId)
                      << ", Status=" << static_cast<int>(status) << std::endl;
        }
    }

    void processInputStatus(uint8_t rwFlag, uint8_t element) {
        if (rwFlag != READ || element != 190) {
            sendResponse(INPUT_STATUS_RSP, READ_NOT_SUPPORTED, element, {});
            return;
        }

        updateInputValues();
        uint32_t currentRaw = static_cast<uint32_t>((input_current_ + 80000.0f) / 0.01f);
        uint16_t voltageRaw = static_cast<uint16_t>((input_voltage_ + 1606.0f) / 0.05f);
        std::vector<uint8_t> data = {
            static_cast<uint8_t>(currentRaw >> 16),
            static_cast<uint8_t>(currentRaw >> 8),
            static_cast<uint8_t>(currentRaw & 0xFF),
            static_cast<uint8_t>(voltageRaw >> 8),
            static_cast<uint8_t>(voltageRaw & 0xFF)
        };
        sendResponse(INPUT_STATUS_RSP, PDU_SUCCESS, element, data);
        std::cout << "Input Status: Current=" << input_current_ << "A, Voltage=" << input_voltage_
                  << "V, Status=0" << std::endl;
    }

    void processTemperature(uint8_t rwFlag, uint8_t element) {
        if (rwFlag != READ || element != 152) {
            sendResponse(TEMPERATURE_RSP, READ_NOT_SUPPORTED, element, {});
            return;
        }

        updateTemperature();
        uint16_t tempRaw = static_cast<uint16_t>((temperature_ + 273.0f) / 0.03125f);
        std::vector<uint8_t> data = {
            static_cast<uint8_t>(tempRaw >> 8),
            static_cast<uint8_t>(tempRaw & 0xFF)
        };
        sendResponse(TEMPERATURE_RSP, PDU_SUCCESS, element, data);
        std::cout << "Temperature: " << temperature_ << "°C, Status=0" << std::endl;
    }

    void processGroupedChannels(uint8_t rwFlag, uint8_t element) {
        if (rwFlag != READ) {
            sendResponse(GROUPED_CHANNELS_RSP, READ_NOT_SUPPORTED, element, {});
            return;
        }

        if (element < 100 || element >= 150) {
            sendResponse(GROUPED_CHANNELS_RSP, GROUP_DOESNT_EXIST, element, {});
            return;
        }

        uint8_t groupId = element - 100;
        uint16_t membership = group_memberships_[groupId];
        if (membership == 0) {
            sendResponse(GROUPED_CHANNELS_RSP, GROUP_IS_EMPTY, element, {});
            return;
        }

        std::vector<uint8_t> data = {
            static_cast<uint8_t>(membership >> 8),
            static_cast<uint8_t>(membership & 0xFF)
        };
        sendResponse(GROUPED_CHANNELS_RSP, PDU_SUCCESS, element, data);
        std::cout << "Grouped Channels: Group=" << static_cast<int>(groupId)
                  << ", Membership=0x" << std::hex << membership << std::dec << ", Status=0" << std::endl;
    }

    void updateChannelValues(uint8_t channelId) {
        if (!channels_[channelId].isOn) {
            channels_[channelId].current = 0.0f;
            channels_[channelId].voltage = 0.0f;
            return;
        }

        std::uniform_real_distribution<float> currentDist(0.0f, channels_[channelId].isBattle ? 50.0f : 30.0f);
        std::uniform_real_distribution<float> voltageDist(9.0f, 32.0f);
        channels_[channelId].current = currentDist(gen_);
        channels_[channelId].voltage = voltageDist(gen_);
    }

    void updateInputValues() {
        std::uniform_real_distribution<float> currentDist(0.0f, 100.0f);
        std::uniform_real_distribution<float> voltageDist(9.0f, 32.0f);
        input_current_ = currentDist(gen_);
        input_voltage_ = voltageDist(gen_);
    }

    void updateTemperature() {
        std::uniform_real_distribution<float> tempDist(-40.0f, 125.0f);
        temperature_ = tempDist(gen_);
    }
};

// int main(int argc, char* argv[]) {
//     std::string device = "/dev/ttyUSB0";
//     int baudrate = 1000000;

//     if (argc > 1) {
//         device = argv[1];
//     }
//     if (argc > 2) {
//         try {
//             baudrate = std::stoi(argv[2]);
//             if (baudrate != 500000 && baudrate != 1000000) {
//                 std::cerr << "Unsupported baud rate: " << baudrate << ". Using 1000000." << std::endl;
//                 baudrate = 1000000;
//             }
//         } catch (const std::exception& e) {
//             std::cerr << "Invalid baud rate argument: " << argv[2] << ". Using 1000000." << std::endl;
//             baudrate = 1000000;
//         }
//     }

//     PduSimulator pdu(device, baudrate);
//     while (true) {
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//     return 0;
// }

int main(int argc, char* argv[]) {
    std::string device = "/dev/ttyUSB1";
    int baudrate = 1000000;

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        try {
            baudrate = std::stoi(argv[2]);
            if (baudrate != 500000 && baudrate != 1000000) {
                std::cerr << "Warning: Unsupported baud rate, using 1000000" << std::endl;
                baudrate = 1000000;
            }
        } catch (...) {
            std::cerr << "Warning: Invalid baud rate, using 1000000" << std::endl;
            baudrate = 1000000;
        }
    }

    try {
        std::cout << "Starting PDU Simulator on " << device << " at " << baudrate << " bps..." << std::endl;
        PduSimulator pdu(device, baudrate);
        std::cout << "PDU Simulator started" << std::endl;
        std::cout << "PDU Simulator running. Press Enter to exit." << std::endl;
        std::string input;
        std::getline(std::cin, input);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "PDU Simulator stopped" << std::endl;
    return 0;
}
