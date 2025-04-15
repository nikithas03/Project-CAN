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

struct CanMessage {
    uint8_t priority;
    can_frame frame;
    
    bool operator<(const CanMessage& other) const {
        return priority > other.priority; // Lower priority value first
    }
};

class VCU_CAN_Controller {
public:
    static constexpr uint8_t VCU_NODE_ID = 0x02;
    static constexpr uint8_t MCU_NODE_ID = 0x01;
    
    // CAN IDs - Aligned with MCU implementation
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

    VCU_CAN_Controller(const std::string& interface = "vcan0")
        : interface_(interface), running_(true), nmt_state_(NMT_State::INITIALIZING),
          last_sync_time_(0), rpdo1_timer_(0) {
        initialize_socket();
        start_threads();
    }

    ~VCU_CAN_Controller() {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        if (receive_thread_.joinable()) receive_thread_.join();
        close(sockfd_);
    }

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
    
        // Only change our own state if the command is for our node
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
        rpdo1_timer_ = last_sync_time_; // Reset RPDO1 timer

        // Send RPDO1 immediately after SYNC
        send_rpdo1(1500, 100, 1); // Example values, adjust as needed
    }

    void send_motor_command(int16_t speed, int16_t torque, bool enable) {
        if (nmt_state_ != NMT_State::OPERATIONAL) {
            std::cerr << "Cannot send motor commands in current NMT state\n";
            return;
        }

        // Matches MCU RPDO1 mapping
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

        // Matches MCU RPDO1 mapping (0x2007:1A, 0x2007:1E, 0x2007:1D)
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

        // Matches MCU RPDO2 mapping (0x2003:26, 0x2002:1D, 0x2005:14)
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
                        // Removed redundant logging to avoid clutter
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
                // Periodic RPDO1 transmission after SYNC
                if (nmt_state_ == NMT_State::OPERATIONAL && last_sync_time_ > 0) {
                    uint64_t now = get_current_time_ms();
                    if (now - rpdo1_timer_ >= 200) { // 200ms period
                        send_rpdo1(1500, 100, 1); // Example values
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
        // Skip logging for RPD01_ID (0x202)
        if (frame.can_id != RPD01_ID) {
            std::cout << "Received CAN ID 0x" << std::hex << frame.can_id 
                      << " (Priority: " << std::dec << int(priority) << ")\n";
        }

        switch (frame.can_id) {
            case EMCY_ID: handle_emergency(frame); break;
            case TPD01_ID: handle_tpdo1(frame); break;
            case TPD02_ID: handle_tpdo2(frame); break;
            case HEARTBEAT_ID: handle_heartbeat(frame); break;
            case SDO_TX_ID: handle_sdo_response(frame); break;
        }
    }

    uint8_t get_receive_priority(uint32_t can_id) const {
        static const std::unordered_map<uint32_t, uint8_t> priority_map = {
            {EMCY_ID, 4}, {TPD01_ID, 6}, {TPD02_ID, 8}, {SDO_TX_ID, 7},
            {HEARTBEAT_ID, 11}
        };
        auto it = priority_map.find(can_id);
        return (it != priority_map.end()) ? it->second : 12;
    }

    void handle_emergency(const can_frame& frame) {
        std::cerr << "Emergency! Code: " << int(frame.data[0])
                  << " | Error Reg: " << int(frame.data[1])
                  << " | Vendor: " << int(frame.data[2]) << "\n";
    }

    void handle_tpdo1(const can_frame& frame) {
        // Matches MCU TPDO1 mapping (0x2004:08, 0x2004:07, 0x2004:06, 0x2004:09)
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
        // Matches MCU TPDO2 mapping (0x2004:0A, 0x2004:0B, 0x2004:0C, 0x2004:04)
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
};

uint32_t parse_hex(const std::string& s) {
    uint32_t value;
    std::stringstream ss;
    ss << std::hex << s;
    ss >> value;
    return value;
}

int main() {
    try {
        VCU_CAN_Controller vcu("vcan0");
        std::atomic<bool> running(true);
        
        std::thread input_thread([&]() {
            std::cout << "VCU CAN Controller - Node ID 2\n"
                      << "Available commands:\n"
                      << "  n [cmd]    - NMT command (01=start, 02=stop, 80=pre-op)\n"
                      << "  y          - Send SYNC message\n"
                      << "  m          - Send motor command\n"
                      << "  s          - Show NMT state\n"
                      << "  r i s      - SDO Read (i=index, s=subindex)\n"
                      << "  w i s d    - SDO Write (i=index, s=subindex, d=data)\n"
                      << "  p n d...   - Send RPDO (n=1|2, d=data)\n"
                      << "  q          - Quit\n";
                      
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
                
                char cmd = tokens[0][0];
                switch (cmd) {
                    case 'n': case 'N': {
                        if (tokens.size() > 1) {
                            try {
                                if (tokens[1].length() == 4) {
                                    std::string cmd_str = tokens[1].substr(0, 2);
                                    std::string node_str = tokens[1].substr(2, 2);
                                    
                                    uint8_t nmt_cmd = static_cast<uint8_t>(parse_hex(cmd_str));
                                    uint8_t node_id = static_cast<uint8_t>(parse_hex(node_str));
                                    
                                    vcu.send_nmt_command(nmt_cmd, node_id);
                                    std::cout << "Sent NMT command: 0x" << std::hex << static_cast<int>(nmt_cmd) 
                                              << " to node: 0x" << static_cast<int>(node_id) << std::dec << "\n";
                                } else {
                                    std::cout << "Invalid NMT command format. Use: n [cmd][nodeid] (e.g., n 0101)\n";
                                }
                            } catch (...) {
                                std::cout << "Invalid NMT command format. Use: n [cmd][nodeid] (e.g., n 0101)\n";
                            }
                        } else {
                            std::cout << "Please specify NMT command (e.g., n 0101 for start node 1)\n";
                        }
                        break;
                    }
                    case 'y': case 'Y': {
                        vcu.send_sync();
                        std::cout << "Sent SYNC message\n";
                        break;
                    }
                    case 'm': case 'M': {
                        if (vcu.get_nmt_state() == VCU_CAN_Controller::NMT_State::OPERATIONAL) {
                            vcu.send_motor_command(1500, 100, true);
                            std::cout << "Sent motor command\n";
                        } else {
                            std::cout << "Cannot send motor commands in current NMT state\n";
                        }
                        break;
                    }
                    case 's': case 'S': {
                        auto state = vcu.get_nmt_state();
                        std::cout << "Current NMT state: ";
                        switch (state) {
                            case VCU_CAN_Controller::NMT_State::INITIALIZING: std::cout << "Initializing"; break;
                            case VCU_CAN_Controller::NMT_State::PRE_OPERATIONAL: std::cout << "Pre-operational"; break;
                            case VCU_CAN_Controller::NMT_State::OPERATIONAL: std::cout << "Operational"; break;
                            case VCU_CAN_Controller::NMT_State::STOPPED: std::cout << "Stopped"; break;
                        }
                        std::cout << "\n";
                        break;
                    }
                    case 'r': case 'R': {
                        if (tokens.size() >= 3) {
                            try {
                                uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                                uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                                vcu.send_sdo_command(VCU_CAN_Controller::SDO_Command::UPLOAD, 
                                                    index, subindex);
                                std::cout << "Sent SDO Read - Index: 0x" << std::hex << index
                                          << ", Subindex: 0x" << static_cast<int>(subindex) << std::dec << "\n";
                            } catch (...) {
                                std::cout << "Invalid SDO read format. Use: r [index] [subindex] (hex values)\n";
                            }
                        } else {
                            std::cout << "Please specify index and subindex for SDO read (e.g., r 2004 08)\n";
                        }
                        break;
                    }
                    case 'w': case 'W': {
                        if (tokens.size() >= 4) {
                            try {
                                uint16_t index = static_cast<uint16_t>(parse_hex(tokens[1]));
                                uint8_t subindex = static_cast<uint8_t>(parse_hex(tokens[2]));
                                uint32_t data = parse_hex(tokens[3]);
                                vcu.send_sdo_command(VCU_CAN_Controller::SDO_Command::DOWNLOAD, 
                                                   index, subindex, data);
                                std::cout << "Sent SDO Write - Index: 0x" << std::hex << index
                                          << ", Subindex: 0x" << static_cast<int>(subindex)
                                          << ", Data: 0x" << data << std::dec << "\n";
                            } catch (...) {
                                std::cout << "Invalid SDO write format. Use: w [index] [subindex] [data] (hex values)\n";
                            }
                        } else {
                            std::cout << "Please specify index, subindex and data for SDO write (e.g., w 2007 1A 1234)\n";
                        }
                        break;
                    }
                    case 'p': case 'P': {
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
                        break;
                    }
                    case 'q': case 'Q': {
                        running = false;
                        break;
                    }
                    default: {
                        std::cout << "Unknown command\n";
                        break;
                    }
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
    return 0;
}
