#include <iostream>
#include <cstdint>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <unordered_map>
#include <poll.h>
#include <iomanip>
#include <sstream>
#include <array>
#include <vector>
#include <map>
#include <functional>
#include <string>

struct CanMessage {
    uint8_t priority;
    can_frame frame;
    bool expects_response;
    uint32_t response_id;
    uint8_t response_opcode;
    
    bool operator<(const CanMessage& other) const {
        return priority > other.priority; // Lower priority value first
    }
};

class VCUController {
public:
    // VCU and MCU Node IDs
    static constexpr uint8_t VCU_NODE_ID = 0x02;
    static constexpr uint8_t MCU_NODE_ID = 0x01;

    // MCU CAN IDs
    static constexpr uint32_t NMT_CMD_ID     = 0x000;
    static constexpr uint32_t SYNC_ID        = 0x080;
    static constexpr uint32_t RPD01_ID       = 0x200 + VCU_NODE_ID; // 0x202
    static constexpr uint32_t RPD02_ID       = 0x300 + VCU_NODE_ID; // 0x302
    static constexpr uint32_t SDO_TX_ID      = 0x580 + MCU_NODE_ID; // 0x581
    static constexpr uint32_t SDO_RX_ID      = 0x600 + VCU_NODE_ID; // 0x602
    static constexpr uint32_t TPD01_ID       = 0x180 + MCU_NODE_ID; // 0x181
    static constexpr uint32_t TPD02_ID       = 0x280 + MCU_NODE_ID; // 0x281
    static constexpr uint32_t EMCY_ID        = 0x080 + MCU_NODE_ID; // 0x081
    static constexpr uint32_t HEARTBEAT_ID   = 0x700 + MCU_NODE_ID; // 0x701

    // PDU CAN IDs
    static constexpr canid_t PDU_COMMAND_ID  = 0xC77E00F1;
    static constexpr canid_t PDU_RESPONSE_ID = 0x18EFF1600;
    static constexpr canid_t PDU_ALERT_ID    = 0x18FFF1600;

    // NMT States
    enum class NMT_State {
        INITIALIZING,
        PRE_OPERATIONAL,
        OPERATIONAL,
        STOPPED
    };

    // SDO Commands
    enum class SDO_Command {
        UPLOAD = 0x40,
        DOWNLOAD = 0x20,
        ABORT = 0x80
    };

    // PDU OpCodes
    enum OpCode {
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

    // PDU Status Codes
    enum StatusCode {
        SUCCESS = 0x00,
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

    // PDU RW Flags
    enum RWFlag {
        READ = 0,
        WRITE = 1,
        MACRO = 2
    };

    VCUController(const std::string& interface = "vcan0")
        : interface_(interface), running_(true), nmt_state_(NMT_State::INITIALIZING),
          last_sync_time_(0), rpdo1_timer_(0), current_address_(0) {
        initialize_socket();
        register_default_alert_callbacks();
        start_threads();
    }

    ~VCUController() {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        if (receive_thread_.joinable()) receive_thread_.join();
        close(sockfd_);
    }

    // MCU Command Functions
    void send_nmt_command(uint8_t command, uint8_t node_id = MCU_NODE_ID) {
        CanMessage msg{
            .priority = 2,
            .frame = {
                .can_id = NMT_CMD_ID,
                .can_dlc = 2,
                .data = {command, node_id}
            },
            .expects_response = false,
            .response_id = 0,
            .response_opcode = 0
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
            },
            .expects_response = false,
            .response_id = 0,
            .response_opcode = 0
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
            },
            .expects_response = false,
            .response_id = 0,
            .response_opcode = 0
        };
        queue_message(msg);
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
            },
            .expects_response = true,
            .response_id = SDO_TX_ID,
            .response_opcode = 0
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
            },
            .expects_response = false,
            .response_id = 0,
            .response_opcode = 0
        };
        queue_message(msg);
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
            },
            .expects_response = false,
            .response_id = 0,
            .response_opcode = 0
        };
        queue_message(msg);
    }

    NMT_State get_nmt_state() const { return nmt_state_; }

    // PDU Command Functions
    void set_address(uint8_t addr) {
        if (addr > 7) {
            std::cerr << "Warning: Address must be between 0-7, using 0\n";
            current_address_ = 0;
        } else {
            current_address_ = addr;
        }
    }

    uint8_t get_address() const {
        return current_address_;
    }

    bool send_pdu_command(const struct can_frame& frame, OpCode response_opcode, 
                         struct can_frame* response, int timeout_ms = 1000) {
        CanMessage msg{
            .priority = 7,
            .frame = frame,
            .expects_response = (response_opcode != static_cast<OpCode>(0)),
            .response_id = PDU_RESPONSE_ID,
            .response_opcode = static_cast<uint8_t>(response_opcode)
        };
        {
            std::lock_guard<std::mutex> lock(response_mutex_);
            responses_.clear();
        }
        queue_message(msg);

        if (msg.expects_response && response != nullptr) {
            std::unique_lock<std::mutex> lock(response_mutex_);
            bool received = response_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                [this, response_opcode]() {
                    return responses_.find(response_opcode) != responses_.end();
                });
            if (received) {
                *response = responses_[response_opcode];
                responses_.erase(response_opcode);
                return true;
            }
            return false;
        }
        return true;
    }

    void register_alert_callback(OpCode alert_opcode, std::function<void(const struct can_frame&)> callback) {
        alert_callbacks_[alert_opcode].push_back(callback);
    }

    bool channel_control_1(uint8_t element, bool turn_on, bool is_macro = false) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_CONTROL_1_CMD;
        frame.data[1] = (is_macro ? MACRO : WRITE) & 0x03;
        frame.data[2] = element;
        frame.data[3] = turn_on ? 1 : 0;
        bool result = send_pdu_command(frame, CHANNEL_CONTROL_1_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
        }
        return result;
    }

    bool channel_control_2(uint8_t element, bool enable_battle, bool is_macro = false) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_CONTROL_2_CMD;
        frame.data[1] = (is_macro ? MACRO : WRITE) & 0x03;
        frame.data[2] = element;
        frame.data[3] = enable_battle ? 0x0D : 0x00;
        bool result = send_pdu_command(frame, CHANNEL_CONTROL_2_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
        }
        return result;
    }

    bool reset_device() {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = RESET_CMD;
        frame.data[1] = WRITE & 0x03;
        bool result = send_pdu_command(frame, RESET_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
        }
        return result;
    }

    bool get_channel_status_1(uint8_t element, float* current, float* voltage) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_STATUS_1_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = element;
        bool result = send_pdu_command(frame, CHANNEL_STATUS_1_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
            bool is_on;
            if (element < 100) {
                if (element >= 8) {
                    std::cerr << "Invalid channel number\n";
                    return false;
                }
                is_on = (response.data[3] & 0x01);
            } else {
                is_on = (response.data[3] & 0x01);
            }
            if (is_on) {
                uint16_t current_raw = (response.data[4] << 8) | response.data[5];
                *current = current_raw * 0.001f;
                uint16_t voltage_raw = (response.data[6] << 8) | response.data[7];
                *voltage = (voltage_raw * 0.05f) - 1606.0f;
            } else {
                *current = 0.0f;
                *voltage = 0.0f;
            }
        }
        return result;
    }

    bool get_channel_status_2(uint8_t element, bool* is_on, bool* is_tripped, bool* is_battle, uint8_t* group_id) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = CHANNEL_STATUS_2_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = element;
        bool result = send_pdu_command(frame, CHANNEL_STATUS_2_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
            uint16_t state_bits = (response.data[3] << 8) | response.data[4];
            *is_on = (state_bits & 0x01) == 0x01;
            *is_tripped = ((state_bits >> 8) & 0x01) == 0x01;
            *is_battle = ((state_bits >> 12) & 0x01) == 0x01;
            *group_id = response.data[5];
        }
        return result;
    }

    bool get_input_status(float* current, float* voltage) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = INPUT_STATUS_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 190;
        bool result = send_pdu_command(frame, INPUT_STATUS_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
            uint32_t current_raw = (response.data[3] << 16) | (response.data[4] << 8) | response.data[5];
            *current = (current_raw * 0.01f) - 80000.0f;
            uint16_t voltage_raw = (response.data[6] << 8) | response.data[7];
            *voltage = (voltage_raw * 0.05f) - 1606.0f;
        }
        return result;
    }

    bool get_temperature(float* temperature) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = TEMPERATURE_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 152;
        bool result = send_pdu_command(frame, TEMPERATURE_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
            uint16_t temp_raw = (response.data[3] << 8) | response.data[4];
            *temperature = (temp_raw * 0.03125f) - 273.0f;
        }
        return result;
    }

    bool get_grouped_channels(uint8_t group_id, std::vector<uint8_t>* channel_ids) {
        struct can_frame frame;
        struct can_frame response;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = PDU_COMMAND_ID;
        frame.can_dlc = 8;
        frame.data[0] = GROUPED_CHANNELS_CMD;
        frame.data[1] = READ & 0x03;
        frame.data[2] = 100 + group_id;
        bool result = send_pdu_command(frame, GROUPED_CHANNELS_RSP, &response, 1000);
        if (result) {
            uint8_t status = extract_status(response.data[1]);
            if (status == GROUP_DOESNT_EXIST) {
                std::cerr << "Group " << static_cast<int>(group_id) << " doesn't exist\n";
                return false;
            }
            if (status != SUCCESS) {
                std::cerr << "Command failed: " << get_status_description(status) << "\n";
                return false;
            }
            uint16_t membership = (response.data[3] << 8) | response.data[4];
            channel_ids->clear();
            for (int i = 0; i < 8; i++) {
                if ((membership >> (i*2)) & 0x03) {
                    channel_ids->push_back(i);
                }
            }
        }
        return result;
    }

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
    uint8_t current_address_;
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    std::map<OpCode, struct can_frame> responses_;
    std::map<OpCode, std::vector<std::function<void(const struct can_frame&)>>> alert_callbacks_;

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

                    if (write(sockfd_, &msg.frame, sizeof(can_frame)) == sizeof(can_frame)) {
                        std::cerr << "Sent CAN ID 0x" << std::hex << msg.frame.can_id 
                                  << " (Priority: " << std::dec << int(msg.priority) << ")\n";
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
                        process_incoming_frame(frame);
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

    void process_incoming_frame(const can_frame& frame) {
        const auto priority = get_receive_priority(frame.can_id);
        std::cout << "Received CAN ID 0x" << std::hex << frame.can_id 
                  << " (Priority: " << std::dec << int(priority) << ")\n";

        if (frame.can_id == PDU_RESPONSE_ID) {
            handle_pdu_response(frame);
        } else if (frame.can_id == PDU_ALERT_ID) {
            handle_pdu_alert(frame);
        } else {
            handle_mcu_frame(frame);
        }
    }

    uint8_t get_receive_priority(uint32_t can_id) const {
        static const std::unordered_map<uint32_t, uint8_t> priority_map = {
            {EMCY_ID, 4}, {TPD01_ID, 6}, {TPD02_ID, 8}, {SDO_TX_ID, 7},
            {HEARTBEAT_ID, 11}, {PDU_RESPONSE_ID, 7}, {PDU_ALERT_ID, 5}
        };
        auto it = priority_map.find(can_id);
        return (it != priority_map.end()) ? it->second : 12;
    }

    void handle_mcu_frame(const can_frame& frame) {
        switch (frame.can_id) {
            case EMCY_ID: handle_emergency(frame); break;
            case TPD01_ID: handle_tpdo1(frame); break;
            case TPD02_ID: handle_tpdo2(frame); break;
            case HEARTBEAT_ID: handle_heartbeat(frame); break;
            case SDO_TX_ID: handle_sdo_response(frame); break;
        }
    }

    void handle_emergency(const can_frame& frame) {
        std::cerr << "Emergency! Code: " << int(frame.data[0])
                  << " | Error Reg: " << int(frame.data[1])
                  << " | Vendor: " << int(frame.data[2]) << "\n";
    }

    void handle_tpdo1(const can_frame& frame) {
        uint16_t rpm = (frame.data[1] << 8) | frame.data[0];
        uint8_t current = frame.data[2];
        uint8_t temp = frame.data[3];
        uint16_t speed = (frame.data[5] << 8) | frame.data[4];
        
        std::cout << "Motor Data - RPM: " << rpm
                  << " | Current: " << int(current)
                  << "A | Temp: " << int(temp)
                  << "°C | Speed: " << speed << "\n";
    }

    void handle_tpdo2(const can_frame& frame) {
        uint16_t voltage = (frame.data[1] << 8) | frame.data[0];
        uint16_t current = (frame.data[3] << 8) | frame.data[2];
        uint8_t soc = frame.data[4];
        uint8_t temp = frame.data[5];
        
        std::cout << "Battery Data - Voltage: " << voltage
                  << "V | Current: " << current
                  << "A | SOC: " << int(soc)
                  << "% | Controller Temp: " << int(temp) << "°C\n";
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

    void handle_pdu_response(const can_frame& frame) {
        uint8_t opcode = frame.data[0];
        {
            std::lock_guard<std::mutex> lock(response_mutex_);
            responses_[static_cast<OpCode>(opcode)] = frame;
        }
        response_cv_.notify_all();
        std::cout << "PDU Response - OpCode: 0x" << std::hex << static_cast<int>(opcode) 
                  << ", Status: " << get_status_description(extract_status(frame.data[1])) << std::dec << "\n";
    }

    void handle_pdu_alert(const can_frame& frame) {
        std::cout << "PDU Alert - Data: ";
        for (int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << "\n";
        uint8_t opcode = frame.data[0];
        if (alert_callbacks_.find(static_cast<OpCode>(opcode)) != alert_callbacks_.end()) {
            for (auto& callback : alert_callbacks_[static_cast<OpCode>(opcode)]) {
                callback(frame);
            }
        }
    }

    std::string get_status_description(uint8_t status) {
        switch (status) {
            case SUCCESS: return "Success";
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

    uint8_t extract_status(uint8_t status_byte) {
        return (status_byte >> 2);
    }

    void register_default_alert_callbacks() {
        register_alert_callback(static_cast<OpCode>(0x50), [this](const struct can_frame& frame) {
            uint8_t channel_id = frame.data[2];
            std::cout << "Trip Alert received for channel " << static_cast<int>(channel_id) << "\n";

            bool is_on, is_tripped, is_battle;
            uint8_t group_id;
            if (get_channel_status_2(channel_id, &is_on, &is_tripped, &is_battle, &group_id)) {
                std::cout << "Confirmed Channel " << static_cast<int>(channel_id) << " status:"
                          << " On=" << is_on << ", Tripped=" << is_tripped << ", Battle=" << is_battle 
                          << ", Group=" << (group_id == 251 ? "None" : std::to_string(group_id - 100)) << "\n";

                if (is_tripped) {
                    std::cout << "Attempting to reset channel " << static_cast<int>(channel_id) << "\n";
                    if (channel_control_1(channel_id, false)) {
                        std::cout << "Channel " << static_cast<int>(channel_id) << " turned OFF\n";
                    } else {
                        std::cerr << "Failed to turn OFF channel " << static_cast<int>(channel_id) << "\n";
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    if (channel_control_1(channel_id, true)) {
                        std::cout << "Channel " << static_cast<int>(channel_id) << " turned ON - Reset complete\n";
                    } else {
                        std::cerr << "Failed to turn ON channel " << static_cast<int>(channel_id) << "\n";
                    }
                } else {
                    std::cout << "Channel " << static_cast<int>(channel_id) << " not tripped, no reset needed\n";
                }
            } else {
                std::cerr << "Failed to confirm status for channel " << static_cast<int>(channel_id) << "\n";
            }
        });
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
    std::cout << "VCU Controller - Unified Command Interface\n"
              << "========================================\n"
              << "MCU Commands:\n"
              << "  n [cmd][node] - NMT command (e.g., n 0101 for start node 1)\n"
              << "  y             - Send SYNC message\n"
              << "  m             - Send motor command\n"
              << "  s             - Show NMT state\n"
              << "  r [i] [s]     - SDO Read (i=index, s=subindex, hex)\n"
              << "  w [i] [s] [d] - SDO Write (i=index, s=subindex, d=data, hex)\n"
              << "  p [n] [d...]  - Send RPDO (n=1|2, d=data)\n"
              << "PDU Commands:\n"
              << "  on [id] [0-149]    - Turn channel/group ON\n"
              << "  off [id] [0-149]   - Turn channel/group OFF\n"
              << "  battle [id] [0-149]- Enable battle mode\n"
              << "  normal [id] [0-149]- Disable battle mode\n"
              << "  status1 [id] [0-149]- Get current/voltage\n"
              << "  status2 [id] [0-149]- Get state info\n"
              << "  input              - Get input voltage/current\n"
              << "  temp               - Get board temperature\n"
              << "  group [id] [0-49]  - Get channels in group\n"
              << "  reset              - Reset PDU\n"
              << "  addr [a] [0-7]     - Set PDU address\n"
              << "General Commands:\n"
              << "  help               - Display this help\n"
              << "  q                  - Quit\n";
}

int main() {
    try {
        std::cout << "Initializing VCU Controller on vcan0...\n";
        VCUController vcu("vcan0");
        std::cout << "VCU Controller initialized successfully.\n";
        display_help();
        std::atomic<bool> running(true);

        std::thread input_thread([&]() {
            while (running) {
                std::string input;
                std::cout << "> ";
                std::getline(std::cin, input);
                if (input.empty()) continue;

                std::istringstream iss(input);
                std::vector<std::string> tokens;
                std::string token;
                while (iss >> token) {
                    tokens.push_back(token);
                }

                if (tokens.empty()) continue;
                std::string cmd = tokens[0];
                if (cmd == "q") {
                    running = false;
                } else if (cmd == "help") {
                    display_help();
                } else if (cmd == "n") {
                    if (tokens.size() > 1 && tokens[1].length() == 4) {
                        try {
                            std::string cmd_str = tokens[1].substr(0, 2);
                            std::string node_str = tokens[1].substr(2, 2);
                            uint8_t nmt_cmd = static_cast<uint8_t>(parse_hex(cmd_str));
                            uint8_t node_id = static_cast<uint8_t>(parse_hex(node_str));
                            vcu.send_nmt_command(nmt_cmd, node_id);
                            std::cout << "Sent NMT command: 0x" << std::hex << static_cast<int>(nmt_cmd) 
                                      << " to node: 0x" << static_cast<int>(node_id) << std::dec << "\n";
                        } catch (...) {
                            std::cout << "Invalid NMT command format. Use: n [cmd][nodeid]\n";
                        }
                    } else {
                        std::cout << "Please specify NMT command (e.g., n 0101)\n";
                    }
                } else if (cmd == "y") {
                    vcu.send_sync();
                    std::cout << "Sent SYNC message\n";
                } else if (cmd == "m") {
                    if (vcu.get_nmt_state() == VCUController::NMT_State::OPERATIONAL) {
                        vcu.send_motor_command(1500, 100, true);
                        std::cout << "Sent motor command\n";
                    } else {
                        std::cout << "Cannot send motor commands in current NMT state\n";
                    }
                } else if (cmd == "s") {
                    auto state = vcu.get_nmt_state();
                    std::cout << "Current NMT state: ";
                    switch (state) {
                        case VCUController::NMT_State::INITIALIZING: std::cout << "Initializing"; break;
                        case VCUController::NMT_State::PRE_OPERATIONAL: std::cout << "Pre-operational"; break;
                        case VCUController::NMT_State::OPERATIONAL: std::cout << "Operational"; break;
                        case VCUController::NMT_State::STOPPED: std::cout << "Stopped"; break;
                    }
                    std::cout << "\n";
                } else if (cmd == "r") {
                    if (tokens.size() >= 3) {
                        try {
                            uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                            uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                            vcu.send_sdo_command(VCUController::SDO_Command::UPLOAD, index, subindex);
                            std::cout << "Sent SDO Read - Index: 0x" << std::hex << index
                                      << ", Subindex: 0x" << static_cast<int>(subindex) << std::dec << "\n";
                        } catch (...) {
                            std::cout << "Invalid SDO read format. Use: r [index] [subindex]\n";
                        }
                    } else {
                        std::cout << "Please specify index and subindex for SDO read\n";
                    }
                } else if (cmd == "w") {
                    if (tokens.size() >= 4) {
                        try {
                            uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                            uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                            uint32_t data = parse_hex(tokens[3]);
                            vcu.send_sdo_command(VCUController::SDO_Command::DOWNLOAD, index, subindex, data);
                            std::cout << "Sent SDO Write - Index: 0x" << std::hex << index
                                      << ", Subindex: 0x" << static_cast<int>(subindex)
                                      << ", Data: 0x" << data << std::dec << "\n";
                        } catch (...) {
                            std::cout << "Invalid SDO write format. Use: w [index] [subindex] [data]\n";
                        }
                    } else {
                        std::cout << "Please specify index, subindex, and data for SDO write\n";
                    }
                } else if (cmd == "p") {
                    if (tokens.size() >= 2) {
                        try {
                            int pdo_num = std::stoi(tokens[1]);
                            if (pdo_num == 1 && tokens.size() >= 5) {
                                uint16_t target_speed = static_cast<uint16_t>(parse_hex(tokens[2]));
                                uint16_t torque_limit = static_cast<uint16_t>(parse_hex(tokens[3]));
                                uint8_t motor_enable = static_cast<uint8_t>(parse_hex(tokens[4]));
                                vcu.send_rpdo1(target_speed, torque_limit, motor_enable);
                                std::cout << "Sent RPDO1 - Speed: 0x" << std::hex << target_speed
                                          << ", Torque: 0x" << torque_limit
                                          << ", Enable: 0x" << static_cast<int>(motor_enable) << std::dec << "\n";
                            } else if (pdo_num == 2 && tokens.size() >= 5) {
                                uint16_t max_speed = static_cast<uint16_t>(parse_hex(tokens[2]));
                                uint8_t regen_level = static_cast<uint8_t>(parse_hex(tokens[3]));
                                uint8_t assist_level = static_cast<uint8_t>(parse_hex(tokens[4]));
                                vcu.send_rpdo2(max_speed, regen_level, assist_level);
                                std::cout << "Sent RPDO2 - Max Speed: 0x" << std::hex << max_speed
                                          << ", Regen: 0x" << static_cast<int>(regen_level)
                                          << ", Assist: 0x" << static_cast<int>(assist_level) << std::dec << "\n";
                            } else {
                                std::cout << "Invalid RPDO format. Use: p 1 [speed] [torque] [enable] or p 2 [max_speed] [regen] [assist]\n";
                            }
                        } catch (...) {
                            std::cout << "Invalid RPDO format. Use: p 1 [speed] [torque] [enable] or p 2 [max_speed] [regen] [assist]\n";
                        }
                    } else {
                        std::cout << "Please specify PDO number and data\n";
                    }
                } else if (cmd == "addr") {
                    if (tokens.size() < 2) {
                        std::cout << "Current PDU address: " << static_cast<int>(vcu.get_address()) << "\n";
                    } else {
                        try {
                            uint8_t addr = std::stoi(tokens[1]);
                            vcu.set_address(addr);
                            std::cout << "PDU address set to " << static_cast<int>(addr) << "\n";
                        } catch (...) {
                            std::cerr << "Invalid address\n";
                        }
                    }
                } else if (cmd == "on" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        if (vcu.channel_control_1(element, true)) {
                            std::cout << "Channel/group " << static_cast<int>(element) << " turned ON successfully\n";
                        } else {
                            std::cerr << "Failed to turn ON channel/group " << static_cast<int>(element) << "\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid channel/group ID\n";
                    }
                } else if (cmd == "off" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        if (vcu.channel_control_1(element, false)) {
                            std::cout << "Channel/group " << static_cast<int>(element) << " turned OFF successfully\n";
                        } else {
                            std::cerr << "Failed to turn OFF channel/group " << static_cast<int>(element) << "\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid channel/group ID\n";
                    }
                } else if (cmd == "battle" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        if (vcu.channel_control_2(element, true)) {
                            std::cout << "Battle mode ENABLED for channel/group " << static_cast<int>(element) << "\n";
                        } else {
                            std::cerr << "Failed to enable battle mode for channel/group " << static_cast<int>(element) << "\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid channel/group ID\n";
                    }
                } else if (cmd == "normal" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        if (vcu.channel_control_2(element, false)) {
                            std::cout << "Battle mode DISABLED for channel/group " << static_cast<int>(element) << "\n";
                        } else {
                            std::cerr << "Failed to disable battle mode for channel/group " << static_cast<int>(element) << "\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid channel/group ID\n";
                    }
                } else if (cmd == "status1" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        float current, voltage;
                        if (vcu.get_channel_status_1(element, &current, &voltage)) {
                            if (element < 100) {
                                std::cout << "Channel " << static_cast<int>(element) << " status:\n";
                            } else {
                                std::cout << "Group " << static_cast<int>(element - 100) << " status:\n";
                            }
                            bool is_on = (current > 0.0f || voltage > 0.0f);
                            if (is_on) {
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
                } else if (cmd == "status2" && tokens.size() > 1) {
                    try {
                        uint8_t element = std::stoi(tokens[1]);
                        bool is_on, is_tripped, is_battle;
                        uint8_t group_id;
                        if (vcu.get_channel_status_2(element, &is_on, &is_tripped, &is_battle, &group_id)) {
                            std::cout << "Channel/group " << static_cast<int>(element) << " state:\n";
                            std::cout << "  Power state: " << (is_on ? "ON" : "OFF") << "\n";
                            std::cout << "  Trip state: " << (is_tripped ? "TRIPPED" : "NORMAL") << "\n";
                            std::cout << "  Battle mode: " << (is_battle ? "ENABLED" : "DISABLED") << "\n";
                            std::cout << "  Group ID: " << (group_id == 251 ? "NONE" : std::to_string(group_id - 100)) << "\n";
                        } else {
                            std::cerr << "Failed to get state for channel/group " << static_cast<int>(element) << "\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid channel/group ID\n";
                    }
                } else if (cmd == "input") {
                    float current, voltage;
                    if (vcu.get_input_status(&current, &voltage)) {
                        std::cout << "Input status:\n";
                        std::cout << " database Current: " << current << " A\n";
                        std::cout << "  Voltage: " << voltage << " V\n";
                    } else {
                        std::cerr << "Failed to get input status\n";
                    }
                } else if (cmd == "temp") {
                    float temperature;
                    if (vcu.get_temperature(&temperature)) {
                        std::cout << "Board temperature: " << temperature << " °C\n";
                    } else {
                        std::cerr << "Failed to get temperature\n";
                    }
                } else if (cmd == "group" && tokens.size() > 1) {
                    try {
                        uint8_t group_id = std::stoi(tokens[1]);
                        if (group_id > 49) {
                            std::cerr << "Group ID must be between 0 and 49\n";
                            continue;
                        }
                        std::vector<uint8_t> channel_ids;
                        if (vcu.get_grouped_channels(group_id, &channel_ids)) {
                            std::cout << "Group " << static_cast<int>(group_id) << " contains channels: ";
                            for (size_t i = 0; i < channel_ids.size(); i++) {
                                std::cout << static_cast<int>(channel_ids[i]);
                                if (i < channel_ids.size() - 1) {
                                    std::cout << ", ";
                                }
                            }
                            std::cout << "\n";
                        } else {
                            std::cerr << "Failed to get group information\n";
                        }
                    } catch (...) {
                        std::cerr << "Invalid group ID\n";
                    }
                } else if (cmd == "reset") {
                    if (vcu.reset_device()) {
                        std::cout << "PDU reset successfully\n";
                    } else {
                        std::cerr << "Failed to reset PDU\n";
                    }
                } else {
                    std::cerr << "Unknown command. Type 'help' for commands.\n";
                }
            }
        });

        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        input_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Program terminated.\n";
    return 0;
}