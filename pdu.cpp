#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <poll.h>
#include <iomanip>
#include <sstream>

// Constants for PDU
const canid_t PDU_COMMAND_ID = 0xC77E00F1;
const canid_t PDU_RESPONSE_ID = 0x18EFF1600;
const canid_t PDU_ALERT_ID = 0x18FFF1600;

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
    GROUPED_CHANNELS_RSP = 0x36
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

// VCU Constants and Enums
struct CanMessage {
    uint8_t priority;
    can_frame frame;
    
    bool operator<(const CanMessage& other) const {
        return priority > other.priority;
    }
};

class IntegratedController {
public:
    static constexpr uint8_t VCU_NODE_ID = 0x02;
    static constexpr uint8_t MCU_NODE_ID = 0x01;
    
    // VCU CAN IDs
    static constexpr uint32_t NMT_CMD_ID     = 0x000;
    static constexpr uint32_t SYNC_ID        = 0x080;
    static constexpr uint32_t RPD01_ID       = 0x200 + VCU_NODE_ID;
    static constexpr uint32_t RPD02_ID       = 0x300 + VCU_NODE_ID;
    static constexpr uint32_t SDO_TX_ID      = 0x580 + MCU_NODE_ID;
    static constexpr uint32_t SDO_RX_ID      = 0x600 + VCU_NODE_ID;
    static constexpr uint32_t TPD01_ID       = 0x180 + MCU_NODE_ID;
    static constexpr uint32_t TPD02_ID       = 0x280 + MCU_NODE_ID;
    static constexpr uint32_t EMCY_ID        = 0x080 + MCU_NODE_ID;
    static constexpr uint32_t HEARTBEAT_ID   = 0x700 + MCU_NODE_ID;

    enum class NMT_State {
        INITIALIZING,
        PRE_OPERATIONAL,
        OPERATIONAL,
        STOPPED
    };

    enum class SDO_Command {
        UPLOAD = 0x40,
        DOWNLOAD = 0x20,
        ABORT = 0x80
    };

    IntegratedController(const std::string& interface = "vcan0")
        : interface_(interface), running_(true), nmt_state_(NMT_State::INITIALIZING),
          last_sync_time_(0), rpdo1_timer_(0), currentAddress_(0) {
        initialize_socket();
        start_threads();
        registerDefaultAlertCallbacks();
    }

    ~IntegratedController() {
        running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (send_thread_.joinable()) send_thread_.join();
        close(sockfd_);
    }

    // PDU Methods
    void setAddress(uint8_t addr) {
        if (addr > 7) {
            std::cerr << "Warning: Address must be between 0-7, using 0" << std::endl;
            currentAddress_ = 0;
        } else {
            currentAddress_ = addr;
        }
    }

    uint8_t getAddress() const { return currentAddress_; }

    bool sendPduCommand(const struct can_frame& frame, PduOpCode responseOpCode, 
                       struct can_frame* response, int timeoutMs = 1000) {
        CanMessage msg{
            .priority = 5,
            .frame = frame
        };
        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            send_queue_.push(msg);
            send_cv_.notify_one();
        }

        if (responseOpCode != static_cast<PduOpCode>(0) && response != nullptr) {
            std::unique_lock<std::mutex> lock(responseMutex_);
            responses_.clear();
            bool received = responseCV_.wait_for(lock, std::chrono::milliseconds(timeoutMs), 
                [this, responseOpCode]() { 
                    return responses_.find(static_cast<PduOpCode>(responseOpCode)) != responses_.end(); 
                });
            if (received) {
                *response = responses_[static_cast<PduOpCode>(responseOpCode)];
                responses_.erase(static_cast<PduOpCode>(responseOpCode));
                return true;
            }
            return false;
        }
        return true;
    }

    bool channelControl1(uint8_t element, bool turnOn, bool isMacro = false) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_CONTROL_1_CMD;
        frame.data[1] = (isMacro ? MACRO : WRITE) & 0x03;
        frame.data[2] = element;
        frame.data[3] = turnOn ? 1 : 0;
        bool result = sendPduCommand(frame, CHANNEL_CONTROL_1_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
        }
        return result;
    }

    bool channelControl2(uint8_t element, bool enableBattle, bool isMacro = false) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_CONTROL_2_CMD;
        frame.data[1] = (isMacro ? MACRO : WRITE) & 0x03;
        frame.data[2] = element;
        frame.data[3] = enableBattle ? 0x0D : 0x00;
        bool result = sendPduCommand(frame, CHANNEL_CONTROL_2_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
        }
        return result;
    }

    bool resetDevice() {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = RESET_CMD;
        frame.data[1] = WRITE & 0x03;
        bool result = sendPduCommand(frame, RESET_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
        }
        return result;
    }

    bool getChannelStatus1(uint8_t element, float* current, float* voltage) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_STATUS_1_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = element;
        bool result = sendPduCommand(frame, CHANNEL_STATUS_1_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
            bool isOn;
            if (element < 100) {
                if (element >= 8) {
                    std::cerr << "Invalid channel number" << std::endl;
                    return false;
                }
                isOn = (response.data[3] & 0x01);
            } else {
                isOn = (response.data[3] & 0x01);
            }
            if (isOn) {
                uint16_t currentRaw = (response.data[4] << 8) | response.data[5];
                *current = currentRaw * 0.001f;
                uint16_t voltageRaw = (response.data[6] << 8) | response.data[7];
                *voltage = (voltageRaw * 0.05f) - 1606.0f;
            } else {
                *current = 0.0f;
                *voltage = 0.0f;
            }
        }
        return result;
    }

    bool getChannelStatus2(uint8_t element, bool* isOn, bool* isTripped, bool* isBattle, uint8_t* groupId) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_STATUS_2_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = element;
        bool result = sendPduCommand(frame, CHANNEL_STATUS_2_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
            uint16_t stateBits = (response.data[3] << 8) | response.data[4];
            *isOn = (stateBits & 0x01) == 0x01;
            *isTripped = ((stateBits >> 8) & 0x01) == 0x01;
            *isBattle = ((stateBits >> 12) & 0x01) == 0x01;
            *groupId = response.data[5];
        }
        return result;
    }

    bool getInputStatus(float* current, float* voltage) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = INPUT_STATUS_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 190;
        bool result = sendPduCommand(frame, INPUT_STATUS_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
            uint32_t currentRaw = (response.data[3] << 16) | (response.data[4] << 8) | response.data[5];
            *current = (currentRaw * 0.01f) - 80000.0f;
            uint16_t voltageRaw = (response.data[6] << 8) | response.data[7];
            *voltage = (voltageRaw * 0.05f) - 1606.0f;
        }
        return result;
    }

    bool getTemperature(float* temperature) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = TEMPERATURE_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 152;
        bool result = sendPduCommand(frame, TEMPERATURE_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
            uint16_t tempRaw = (response.data[3] << 8) | response.data[4];
            *temperature = (tempRaw * 0.03125f) - 273.0f;
        }
        return result;
    }

    bool getGroupedChannels(uint8_t groupId, std::vector<uint8_t>* channelIds) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = GROUPED_CHANNELS_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 100 + groupId;
        bool result = sendPduCommand(frame, GROUPED_CHANNELS_RSP, &response);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status == GROUP_DOESNT_EXIST) {
                std::cerr << "Group " << static_cast<int>(groupId) << " doesn't exist" << std::endl;
                return false;
            }
            if (status != PDU_SUCCESS) {
                std::cerr << "Command failed: " << getStatusDescription(status) << std::endl;
                return false;
            }
            uint16_t membership = (response.data[3] << 8) | response.data[4];
            channelIds->clear();
            for (int i = 0; i < 8; i++) {
                if ((membership >> (i*2)) & 0x03) {
                    channelIds->push_back(i);
                }
            }
        }
        return result;
    }

    // VCU Methods
    void send_nmt_command(uint8_t command, uint8_t node_id = MCU_NODE_ID) {
        CanMessage msg{
            .priority = 2,
            .frame = {
                .can_id = NMT_CMD_ID,
                .can_dlc = 2,
                .data = {command, node_id}
            }
        };
        queue_message(msg);
        
        if (node_id == VCU_NODE_ID) {
            switch (command) {
                case 0x01: nmt_state_ = NMT_State::OPERATIONAL; break;
                case 0x02: nmt_state_ = NMT_State::STOPPED; break;
                case 0x80: nmt_state_ = NMT_State::PRE_OPERATIONAL; break;
                case 0x81: case 0x82: nmt_state_ = NMT_State::INITIALIZING; break;
            }
        }
    }

    void send_sync() {
        if (nmt_state_ != NMT_State::OPERATIONAL) {
            std::cerr << "Cannot send SYNC in current NMT state\n";
            return;
        }

        CanMessage sync_msg{
            .priority = 3,
            .frame = {
                .can_id = SYNC_ID,
                .can_dlc = 0,
                .data = {}
            }
        };
        queue_message(sync_msg);
        last_sync_time_ = get_current_time_ms();
        rpdo1_timer_ = last_sync_time_;
        send_rpdo1(1500, 100, 1);
    }

    void send_motor_command(int16_t speed, int16_t torque, bool enable) {
        if (nmt_state_ != NMT_State::OPERATIONAL) {
            std::cerr << "Cannot send motor commands in current NMT state\n";
            return;
        }

        CanMessage msg{
            .priority = 5,
            .frame = {
                .can_id = RPD01_ID,
                .can_dlc = 5,
                .data = {
                    static_cast<uint8_t>(speed & 0xFF),
                    static_cast<uint8_t>(speed >> 8),
                    static_cast<uint8_t>(torque & 0xFF),
                    static_cast<uint8_t>(torque >> 8),
                    static_cast<uint8_t>(enable)
                }
            }
        };
        queue_message(msg);
        std::cout << "RPDO1 sent: Speed = " << speed << ", Torque = " << torque
                  << ", Enable = " << (int)enable << "\n";
    }

    void send_sdo_command(SDO_Command cmd, uint16_t index, uint8_t subindex, uint32_t data = 0) {
        if (nmt_state_ == NMT_State::STOPPED) {
            std::cerr << "Cannot send SDO commands in STOPPED state\n";
            return;
        }

        CanMessage msg{
            .priority = 4,
            .frame = {
                .can_id = SDO_RX_ID,
                .can_dlc = 8,
                .data = {
                    static_cast<uint8_t>(cmd),
                    static_cast<uint8_t>(index & 0xFF),
                    static_cast<uint8_t>(index >> 8),
                    subindex,
                    static_cast<uint8_t>(data & 0xFF),
                    static_cast<uint8_t>((data >> 8) & 0xFF),
                    static_cast<uint8_t>((data >> 16) & 0xFF),
                    static_cast<uint8_t>((data >> 24) & 0xFF)
                }
            }
        };
        queue_message(msg);
    }

    void send_rpdo1(uint16_t target_speed, uint16_t torque_limit, uint8_t motor_enable) {
        if (nmt_state_ != NMT_State::OPERATIONAL) {
            return;
        }

        CanMessage msg{
            .priority = 6,
            .frame = {
                .can_id = RPD01_ID,
                .can_dlc = 5,
                .data = {
                    static_cast<uint8_t>(target_speed & 0xFF),
                    static_cast<uint8_t>(target_speed >> 8),
                    static_cast<uint8_t>(torque_limit & 0xFF),
                    static_cast<uint8_t>(torque_limit >> 8),
                    motor_enable
                }
            }
        };
        queue_message(msg);
        std::cout << "RPDO1 sent: Speed = " << target_speed << ", Torque = " << torque_limit
                  << ", Enable = " << (int)motor_enable << "\n";
    }

    void send_rpdo2(uint16_t max_speed, uint8_t regen_level, uint8_t assist_level) {
        if (nmt_state_ != NMT_State::OPERATIONAL) {
            return;
        }

        CanMessage msg{
            .priority = 6,
            .frame = {
                .can_id = RPD02_ID,
                .can_dlc = 4,
                .data = {
                    static_cast<uint8_t>(max_speed & 0xFF),
                    static_cast<uint8_t>(max_speed >> 8),
                    regen_level,
                    assist_level
                }
            }
        };
        queue_message(msg);
        std::cout << "RPDO2 sent: Max Speed = " << max_speed << ", Regen = " << (int)regen_level
                  << ", Assist = " << (int)assist_level << "\n";
    }

    NMT_State get_nmt_state() const { return nmt_state_; }

private:
    int sockfd_;
    std::string interface_;
    std::atomic<bool> running_;
    std::atomic<NMT_State> nmt_state_;
    std::priority_queue<CanMessage> send_queue_;
    std::mutex send_mutex_;
    std::condition_variable send_cv_;
    std::thread send_thread_;
    std::thread receive_thread_;
    uint64_t last_sync_time_;
    uint64_t rpdo1_timer_;
    std::mutex responseMutex_;
    std::condition_variable responseCV_;
    std::map<PduOpCode, struct can_frame> responses_;
    std::map<PduOpCode, std::vector<std::function<void(const struct can_frame&)>>> alertCallbacks_;
    uint8_t currentAddress_;

    uint64_t get_current_time_ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }

    void initialize_socket() {
        if ((sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            throw std::runtime_error("Socket creation failed");
        }

        ifreq ifr;
        strcpy(ifr.ifr_name, interface_.c_str());
        if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
            close(sockfd_);
            throw std::runtime_error("Interface ioctl failed");
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sockfd_);
            throw std::runtime_error("Socket bind failed");
        }
    }

    void start_threads() {
        send_thread_ = std::thread([this]() {
            while (running_) {
                std::unique_lock<std::mutex> lock(send_mutex_);
                send_cv_.wait(lock, [this]() {
                    return !send_queue_.empty() || !running_;
                });
    
                while (!send_queue_.empty()) {
                    auto msg = send_queue_.top();
                    send_queue_.pop();
                    lock.unlock();
    
                    if (write(sockfd_, &msg.frame, sizeof(can_frame)) != sizeof(can_frame)) {
                        std::cerr << "Failed to send CAN frame\n";
                    }
    
                    lock.lock();
                }
            }
        });

        receive_thread_ = std::thread([this]() {
            struct pollfd fds[1];
            fds[0].fd = sockfd_;
            fds[0].events = POLLIN;
    
            while (running_) {
                if (poll(fds, 1, 100) > 0 && fds[0].revents & POLLIN) {
                    can_frame frame;
                    if (read(sockfd_, &frame, sizeof(can_frame)) == sizeof(can_frame)) {
                        // Check if extended ID (29-bit) or standard ID (11-bit)
                        if (frame.can_id & CAN_EFF_FLAG) {
                            process_pdu_frame(frame);
                        } else {
                            process_vcu_frame(frame);
                        }
                    }
                }
                if (nmt_state_ == NMT_State::OPERATIONAL && last_sync_time_ > 0) {
                    uint64_t now = get_current_time_ms();
                    if (now - rpdo1_timer_ >= 200) {
                        send_rpdo1(1500, 100, 1);
                        rpdo1_timer_ = now;
                    }
                }
            }
        });
    }

    void queue_message(const CanMessage& msg) {
        std::lock_guard<std::mutex> lock(send_mutex_);
        send_queue_.push(msg);
        send_cv_.notify_one();
    }

    std::string getStatusDescription(uint8_t status) {
        switch (status) {
            case PDU_SUCCESS: return "Success";
            case GENERAL_RW_ERROR: return "General R/W error";
            case READ_NOT_SUPPORTED: return "Read not supported for opcode";
            case WRITE_NOT_SUPPORTED: return "Write not supported for opcode";
            case ERROR_WRITING_FLASH: return "Error writing to flash";
            case WRONG_ELEMENT: return "Wrong element selected";
            case CHANNEL_DOESNT_EXIST: return "Channel number doesn't exist";
            case GROUP_DOESNT_EXIST: return "Group number doesn't exist";
            case SENSOR_DOESNT_EXIST: return "Sensor doesn't exist";
            case BOARD_DOESNT_EXIST: return "Board doesn't exist";
            case WRONG_ADDRESS: return "Wrong address";
            case WRONG_PAGE: return "Wrong page";
            case WRONG_STATE: return "Wrong state";
            case WRONG_FLASH_INDEX: return "Wrong flash index";
            case WRONG_FLASH_KEY: return "Wrong flash key";
            case GROUP_IS_EMPTY: return "Group is empty";
            case CURRENT_OUT_OF_BOUNDS: return "Current limit or overload threshold is out of bounds";
            case OPCODE_DOESNT_EXIST: return "Opcode doesn't exist";
            case GENERAL_ERROR: return "General error";
            default: return "Unknown status code";
        }
    }

    uint8_t extractStatus(uint8_t statusByte) {
        return (statusByte >> 2);
    }

    void registerDefaultAlertCallbacks() {
        registerAlertCallback(static_cast<PduOpCode>(0x50), [this](const struct can_frame& frame) {
            uint8_t channelId = frame.data[2];
            std::cout << "Trip Alert received for channel " << static_cast<int>(channelId) << std::endl;

            bool isOn, isTripped, isBattle;
            uint8_t groupId;
            if (getChannelStatus2(channelId, &isOn, &isTripped, &isBattle, &groupId)) {
                std::cout << "Confirmed Channel " << static_cast<int>(channelId) << " status:" 
                          << " On=" << isOn << ", Tripped=" << isTripped << ", Battle=" << isBattle 
                          << ", Group=" << (groupId == 251 ? "None" : std::to_string(groupId - 100)) << std::endl;

                if (isTripped) {
                    std::cout << "Attempting to reset channel " << static_cast<int>(channelId) << std::endl;
                    if (channelControl1(channelId, false)) {
                        std::cout << "Channel " << static_cast<int>(channelId) << " turned OFF" << std::endl;
                    } else {
                        std::cerr << "Failed to turn OFF channel " << static_cast<int>(channelId) << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    if (channelControl1(channelId, true)) {
                        std::cout << "Channel " << static_cast<int>(channelId) << " turned ON - Reset complete" << std::endl;
                    } else {
                        std::cerr << "Failed to turn ON channel " << static_cast<int>(channelId) << std::endl;
                    }
                } else {
                    std::cout << "Channel " << static_cast<int>(channelId) << " not tripped, no reset needed" << std::endl;
                }
            } else {
                std::cerr << "Failed to confirm status for channel " << static_cast<int>(channelId) << std::endl;
            }
        });
    }

    void registerAlertCallback(PduOpCode alertOpCode, std::function<void(const struct can_frame&)> callback) {
        alertCallbacks_[alertOpCode].push_back(callback);
    }

    void process_pdu_frame(const can_frame& frame) {
        if (frame.can_id == PDU_RESPONSE_ID) {
            uint8_t opCode = frame.data[0];
            {
                std::lock_guard<std::mutex> lock(responseMutex_);
                responses_[static_cast<PduOpCode>(opCode)] = frame;
            }
            responseCV_.notify_all();
            std::cout << "PDU Response - OpCode: 0x" << std::hex << static_cast<int>(opCode) 
                      << ", Status: " << getStatusDescription(extractStatus(frame.data[1])) << std::dec << std::endl;
        } else if (frame.can_id == PDU_ALERT_ID) {
            std::cout << "PDU Alert - Data: ";
            for (int i = 0; i < frame.can_dlc; i++) {
                std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
            }
            std::cout << std::dec << std::endl;
            uint8_t opCode = frame.data[0];
            if (alertCallbacks_.find(static_cast<PduOpCode>(opCode)) != alertCallbacks_.end()) {
                for (auto& callback : alertCallbacks_[static_cast<PduOpCode>(opCode)]) {
                    callback(frame);
                }
            }
        }
    }

    void process_vcu_frame(const can_frame& frame) {
        uint8_t priority = get_vcu_receive_priority(frame.can_id);
        if (frame.can_id != RPD01_ID) {
            std::cout << "VCU Received CAN ID 0x" << std::hex << frame.can_id 
                      << " (Priority: " << std::dec << static_cast<int>(priority) << ")\n";
        }

        switch (frame.can_id) {
            case EMCY_ID: handle_emergency(frame); break;
            case TPD01_ID: handle_tpdo1(frame); break;
            case TPD02_ID: handle_tpdo2(frame); break;
            case HEARTBEAT_ID: handle_heartbeat(frame); break;
            case SDO_TX_ID: handle_sdo_response(frame); break;
        }
    }

    uint8_t get_vcu_receive_priority(uint32_t can_id) const {
        static const std::map<uint32_t, uint8_t> priority_map = {
            {EMCY_ID, 4}, {TPD01_ID, 6}, {TPD02_ID, 8}, {SDO_TX_ID, 7},
            {HEARTBEAT_ID, 11}
        };
        auto it = priority_map.find(can_id);
        return (it != priority_map.end()) ? it->second : 12;
    }

    void handle_emergency(const can_frame& frame) {
        std::cerr << "Emergency! Code: " << static_cast<int>(frame.data[0])
                  << " | Error Reg: " << static_cast<int>(frame.data[1])
                  << " | Vendor: " << static_cast<int>(frame.data[2]) << "\n";
    }

    void handle_tpdo1(const can_frame& frame) {
        uint16_t rpm = (frame.data[1] << 8) | frame.data[0];
        uint8_t current = frame.data[2];
        uint8_t temp = frame.data[3];
        uint16_t speed = (frame.data[5] << 8) | frame.data[4];
        
        std::cout << "Motor Data - RPM: " << rpm
                  << " | Current: " << static_cast<int>(current)
                  << "A | Temp: " << static_cast<int>(temp)
                  << "°C | Speed: " << speed << "\n";
    }

    void handle_tpdo2(const can_frame& frame) {
        uint16_t voltage = (frame.data[1] << 8) | frame.data[0];
        uint16_t current = (frame.data[3] << 8) | frame.data[2];
        uint8_t soc = frame.data[4];
        uint8_t temp = frame.data[5];
        
        std::cout << "Battery Data - Voltage: " << voltage
                  << "V | Current: " << current
                  << "A | SOC: " << static_cast<int>(soc)
                  << "% | Controller Temp: " << static_cast<int>(temp) << "°C\n";
    }

    void handle_heartbeat(const can_frame& frame) {
        std::cout << "MCU Heartbeat - State: ";
        switch (frame.data[0]) {
            case 0x00: std::cout << "Initializing"; break;
            case 0x04: std::cout << "Stopped"; break;
            case 0x05: std::cout << "Operational"; break;
            case 0x7F: std::cout << "Pre-operational"; break;
            default: std::cout << "Unknown";
        }
        std::cout << "\n";
    }

    void handle_sdo_response(const can_frame& frame) {
        uint8_t command = frame.data[0] & 0xE0;
        uint16_t index = (frame.data[2] << 8) | frame.data[1];
        uint8_t subindex = frame.data[3];

        if (command == static_cast<uint8_t>(SDO_Command::ABORT)) {
            uint32_t abort_code = (frame.data[7] << 24) | (frame.data[6] << 16) | 
                                 (frame.data[5] << 8) | frame.data[4];
            std::cerr << "SDO Abort - Index: 0x" << std::hex << index
                      << ", Subindex: 0x" << static_cast<int>(subindex)
                      << ", Code: 0x" << abort_code << std::dec << "\n";
            return;
        }

        if (command == static_cast<uint8_t>(SDO_Command::UPLOAD)) {
            uint32_t data = (frame.data[7] << 24) | (frame.data[6] << 16) | 
                           (frame.data[5] << 8) | frame.data[4];
            std::cout << "SDO Read Response - Index: 0x" << std::hex << index
                      << ", Subindex: 0x" << static_cast<int>(subindex)
                      << ", Data: 0x" << data << " (" << std::dec << data << ")\n";
        } else {
            std::cout << "SDO Write Acknowledged - Index: 0x" << std::hex << index
                      << ", Subindex: 0x" << static_cast<int>(subindex) << std::dec << "\n";
        }
    }
};

uint32_t parse_hex(const std::string& s) {
    uint32_t value;
    std::stringstream ss;
    ss << std::hex << s;
    ss >> value;
    return value;
}

void display_help() {
    std::cout << "Integrated VCU/PDU Controller\n"
              << "Commands:\n"
              << "VCU Commands:\n"
              << "  n [cmd][node] - NMT command (e.g., n 0101 for start node 1)\n"
              << "  y            - Send SYNC message\n"
              << "  m            - Send motor command\n"
              << "  s            - Show NMT state\n"
              << "  r [idx] [sub] - SDO Read\n"
              << "  w [idx] [sub] [data] - SDO Write\n"
              << "  p [n] [data...] - Send RPDO (n=1|2)\n"
              << "PDU Commands:\n"
              << "  on [ch/gr]   - Turn channel/group ON\n"
              << "  off [ch/gr]  - Turn channel/group OFF\n"
              << "  battle [ch/gr] - Enable battle mode\n"
              << "  normal [ch/gr] - Disable battle mode\n"
              << "  status1 [ch/gr] - Get current/voltage\n"
              << "  status2 [ch/gr] - Get state info\n"
              << "  input        - Get input voltage/current\n"
              << "  temp         - Get board temperature\n"
              << "  group [id]   - Get channels in group\n"
              << "  reset        - Reset PDU\n"
              << "  addr [0-7]   - Set PDU address\n"
              << "General:\n"
              << "  y         - Show this help\n"
              << "  q            - Quit\n";
}

int main() {
    try {
        IntegratedController controller("vcan0");
        std::cout << "Integrated VCU/PDU Controller initialized\n";
        display_help();

        std::string input;
        while (true) {
            std::cout << "> ";
            std::getline(std::cin, input);
            if (input.empty()) continue;

            std::istringstream iss(input);
            std::vector<std::string> tokens;
            std::string token;
            while (iss >> token) tokens.push_back(token);

            if (tokens[0] == "q") break;
            else if (tokens[0] == "help") {
                display_help();
            }
            // VCU Commands
            else if (tokens[0] == "n" && tokens.size() > 1) {
                try {
                    if (tokens[1].length() == 4) {
                        uint8_t nmt_cmd = static_cast<uint8_t>(parse_hex(tokens[1].substr(0, 2)));
                        uint8_t node_id = static_cast<uint8_t>(parse_hex(tokens[1].substr(2, 2)));
                        controller.send_nmt_command(nmt_cmd, node_id);
                        std::cout << "Sent NMT command: 0x" << std::hex << static_cast<int>(nmt_cmd) 
                                  << " to node: 0x" << static_cast<int>(node_id) << std::dec << "\n";
                    } else {
                        std::cout << "Invalid NMT command format\n";
                    }
                } catch (...) {
                    std::cout << "Invalid NMT command\n";
                }
            }
            else if (tokens[0] == "y") {
                controller.send_sync();
                std::cout << "Sent SYNC message\n";
            }
            else if (tokens[0] == "m") {
                if (controller.get_nmt_state() == IntegratedController::NMT_State::OPERATIONAL) {
                    controller.send_motor_command(1500, 100, true);
                    std::cout << "Sent motor command\n";
                } else {
                    std::cout << "Cannot send motor commands in current NMT state\n";
                }
            }
            else if (tokens[0] == "s") {
                auto state = controller.get_nmt_state();
                std::cout << "NMT state: ";
                switch (state) {
                    case IntegratedController::NMT_State::INITIALIZING: std::cout << "Initializing"; break;
                    case IntegratedController::NMT_State::PRE_OPERATIONAL: std::cout << "Pre-operational"; break;
                    case IntegratedController::NMT_State::OPERATIONAL: std::cout << "Operational"; break;
                    case IntegratedController::NMT_State::STOPPED: std::cout << "Stopped"; break;
                }
                std::cout << "\n";
            }
            else if (tokens[0] == "r" && tokens.size() >= 3) {
                try {
                    uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                    uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                    controller.send_sdo_command(IntegratedController::SDO_Command::UPLOAD, index, subindex);
                    std::cout << "Sent SDO Read - Index: 0x" << std::hex << index
                              << ", Subindex: 0x" << static_cast<int>(subindex) << std::dec << "\n";
                } catch (...) {
                    std::cout << "Invalid SDO read format\n";
                }
            }
            else if (tokens[0] == "w" && tokens.size() >= 4) {
                try {
                    uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                    uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                    uint32_t data = parse_hex(tokens[3]);
                    controller.send_sdo_command(IntegratedController::SDO_Command::DOWNLOAD, index, subindex, data);
                    std::cout << "Sent SDO Write - Index: 0x" << std::hex << index
                              << ", Subindex: 0x" << static_cast<int>(subindex)
                              << ", Data: 0x" << data << std::dec << "\n";
                } catch (...) {
                    std::cout << "Invalid SDO write format\n";
                }
            }
            else if (tokens[0] == "p" && tokens.size() >= 2) {
                try {
                    int pdo_num = std::stoi(tokens[1]);
                    if (pdo_num == 1 && tokens.size() >= 5) {
                        uint16_t speed = static_cast<uint16_t>(parse_hex(tokens[2]));
                        uint16_t torque = static_cast<uint16_t>(parse_hex(tokens[3]));
                        uint8_t enable = static_cast<uint8_t>(parse_hex(tokens[4]));
                        controller.send_rpdo1(speed, torque, enable);
                    } else if (pdo_num == 2 && tokens.size() >= 5) {
                        uint16_t max_speed = static_cast<uint16_t>(parse_hex(tokens[2]));
                        uint8_t regen = static_cast<uint8_t>(parse_hex(tokens[3]));
                        uint8_t assist = static_cast<uint8_t>(parse_hex(tokens[4]));
                        controller.send_rpdo2(max_speed, regen, assist);
                    } else {
                        std::cout << "Invalid RPDO format\n";
                    }
                } catch (...) {
                    std::cout << "Invalid RPDO parameters\n";
                }
            }
            // PDU Commands
            else if (tokens[0] == "addr" && tokens.size() > 1) {
                try {
                    uint8_t addr = std::stoi(tokens[1]);
                    controller.setAddress(addr);
                    std::cout << "Address set to " << static_cast<int>(addr) << std::endl;
                } catch (...) {
                    std::cerr << "Invalid address\n";
                }
            }
            else if (tokens[0] == "on" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    if (controller.channelControl1(element, true)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " turned ON\n";
                    } else {
                        std::cerr << "Failed to turn ON channel/group\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID\n";
                }
            }
            else if (tokens[0] == "off" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    if (controller.channelControl1(element, false)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " turned OFF\n";
                    } else {
                        std::cerr << "Failed to turn OFF channel/group\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID\n";
                }
            }
            else if (tokens[0] == "battle" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    if (controller.channelControl2(element, true)) {
                        std::cout << "Battle mode ENABLED for channel/group " << static_cast<int>(element) << "\n";
                    } else {
                        std::cerr << "Failed to enable battle mode\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID\n";
                }
            }
            else if (tokens[0] == "normal" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    if (controller.channelControl2(element, false)) {
                        std::cout << "Battle mode DISABLED for channel/group " << static_cast<int>(element) << "\n";
                    } else {
                        std::cerr << "Failed to disable battle mode\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID\n";
                }
            }
            else if (tokens[0] == "status1" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    float current, voltage;
                    if (controller.getChannelStatus1(element, &current, &voltage)) {
                        std::cout << (element < 100 ? "Channel " : "Group ") << static_cast<int>(element % 100) << " status:\n";
                        bool isOn = (current > 0.0f || voltage > 0.0f);
                        if (isOn) {
                            std::cout << "  Current: " << current << " A\n";
                            std::cout << "  Voltage: " << voltage << " V\n";
                        } else {
                            std::cout << "  State: OFF\n";
                        }
                    } else {
                        std::cerr << "Failed to get status\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid element ID\n";
                }
            }
            else if (tokens[0] == "status2" && tokens.size() > 1) {
                try {
                    uint8_t element = std::stoi(tokens[1]);
                    bool isOn, isTripped, isBattle;
                    uint8_t groupId;
                    if (controller.getChannelStatus2(element, &isOn, &isTripped, &isBattle, &groupId)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " state:\n"
                                  << "  Power: " << (isOn ? "ON" : "OFF") << "\n"
                                  << "  Trip: " << (isTripped ? "TRIPPED" : "NORMAL") << "\n"
                                  << "  Battle: " << (isBattle ? "ENABLED" : "DISABLED") << "\n"
                                  << "  Group ID: " << (groupId == 251 ? "NONE" : std::to_string(groupId - 100)) << "\n";
                    } else {
                        std::cerr << "Failed to get state\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID\n";
                }
            }
            else if (tokens[0] == "input") {
                float current, voltage;
                if (controller.getInputStatus(&current, &voltage)) {
                    std::cout << "Input status:\n"
                              << "  Current: " << current << " A\n"
                              << "  Voltage: " << voltage << " V\n";
                } else {
                    std::cerr << "Failed to get input status\n";
                }
            }
            else if (tokens[0] == "temp") {
                float temperature;
                if (controller.getTemperature(&temperature)) {
                    std::cout << "Board temperature: " << temperature << " °C\n";
                } else {
                    std::cerr << "Failed to get temperature\n";
                }
            }
            else if (tokens[0] == "group" && tokens.size() > 1) {
                try {
                    uint8_t groupId = std::stoi(tokens[1]);
                    if (groupId > 49) {
                        std::cerr << "Group ID must be 0-49\n";
                        continue;
                    }
                    std::vector<uint8_t> channelIds;
                    if (controller.getGroupedChannels(groupId, &channelIds)) {
                        std::cout << "Group " << static_cast<int>(groupId) << " channels: ";
                        for (size_t i = 0; i < channelIds.size(); i++) {
                            std::cout << static_cast<int>(channelIds[i]) << (i < channelIds.size() - 1 ? ", " : "");
                        }
                        std::cout << "\n";
                    } else {
                        std::cerr << "Failed to get group info\n";
                    }
                } catch (...) {
                    std::cerr << "Invalid group ID\n";
                }
            }
            else if (tokens[0] == "reset") {
                if (controller.resetDevice()) {
                    std::cout << "PDU reset successfully\n";
                } else {
                    std::cerr << "Failed to reset PDU\n";
                }
            }
            else {
                std::cout << "Unknown command. Type 'help' for commands.\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Program terminated.\n";
    return 0;
}
