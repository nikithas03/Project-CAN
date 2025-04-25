#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <ctime>
#include <csignal>
#include <mutex>
#include <atomic>
#include <map>
#include <bitset>
#include <vector>

// Configuration
#define CAN_INTERFACE "/dev/ttyUSB1"
#define LOG_FILE "can_telematics_log.csv"
#define MAX_LOG_SIZE_MB 10
#define MAX_LOG_FILES 5
#define DEBUG 1

// CAN ID Types
#define CAN_11BIT_ID_MASK 0x000007FF
#define CAN_29BIT_ID_MASK 0x1FFFFFFF

// CANopen Constants
#define NMT_COB_ID 0x000
#define SYNC_COB_ID 0x080
#define EMCY_COB_ID_BASE 0x080
#define HEARTBEAT_COB_ID_BASE 0x700
#define SDO_TX_COB_ID_BASE 0x580
#define SDO_RX_COB_ID_BASE 0x600
#define TPDO1_COB_ID_BASE 0x180
#define TPDO2_COB_ID_BASE 0x280
#define RPDO1_COB_ID_BASE 0x200
#define RPDO2_COB_ID_BASE 0x300

// Node IDs
#define VCU_NODE_ID 0x20
#define PDU_NODE_ID 0x30
#define MCU_NODE_ID 0x40
#define TELEMATICS_NODE_ID 0x50

// PDU CAN IDs from PduSimulator
#define PDU_COMMAND_ID 0xC77E00F1
#define PDU_RESPONSE_ID 0x18EFF160
#define PDU_ALERT_ID 0x18FFF160

// PDU OpCodes and Status Codes from PduSimulator
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
    GROUPED_CHANNELS_RSP = 0x36,
    TRIP_ALERT = 0x50
};

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

// Object Dictionary Mappings
struct ObjectDictionaryEntry {
    std::string description;
    std::string access;
    std::string pdoMapping;
    int size; // in bytes
};

std::map<uint32_t, ObjectDictionaryEntry> objectDictionary = {
    {0x200001, {"Actual RPM", "RO", "TPDO1 Byte 0-1", 2}},
    {0x200002, {"Voltage", "RO", "TPDO1 Byte 2", 1}},
    {0x200003, {"Current", "RO", "TPDO1 Byte 3", 1}},
    {0x200004, {"Motor Temp", "RO", "TPDO1 Byte 4", 1}},
    {0x200101, {"SOC", "RO", "TPDO2 Byte 0", 1}},
    {0x200102, {"Motor Mode", "RO", "TPDO2 Byte 1", 1}},
    {0x200103, {"Battery Temp", "RO", "TPDO2 Byte 2", 1}},
    {0x200104, {"Error Flags", "RO", "TPDO2 Byte 3", 1}},
    {0x300001, {"Target Speed", "RW", "RPDO1 Byte 0-1", 2}},
    {0x300002, {"Torque Limit", "RW", "RPDO1 Byte 2-3", 2}},
    {0x300003, {"Direction", "RW", "RPDO1 Byte 4", 1}},
    {0x300004, {"Motor Enable", "RW", "RPDO1 Byte 5", 1}},
    {0x300101, {"Max Speed", "RW", "RPDO2 Byte 0-1", 2}},
    {0x300102, {"Regen Level", "RW", "RPDO2 Byte 2", 1}},
    {0x300103, {"Operation Mode", "RW", "RPDO2 Byte 3", 1}}
};

// Global variables
std::mutex log_mutex;
std::atomic<bool> running{true};
std::map<uint8_t, std::string> node_states;

// CAN Frame Structure (for parsing compatibility)
struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

// CAN Message Structure (for USB-CAN)
struct CANMessage {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

// CANHandler Class
class CANHandler {
public:
    CANHandler(const std::string& device, int baudrate = 1000000)
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

    bool receiveFrame(CANMessage& msg, uint32_t timeout_ms) {
        if (fd_ == -1) {
            std::cerr << "Device not open" << std::endl;
            return false;
        }

        struct timeval tv;
        gettimeofday(&tv, nullptr);
        uint32_t start_ms = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
        uint32_t elapsed_ms = 0;

        while (elapsed_ms < timeout_ms) {
            uint8_t byte;
            ssize_t n = read(fd_, &byte, 1);
            if (n > 0) {
                if (byte == 0xAA) {
                    buffer_.clear();
                    buffer_.push_back(byte);
                } else if (!buffer_.empty()) {
                    buffer_.push_back(byte);
                    if (byte == 0x55 && buffer_.size() >= 5) {
                        if (processFrame(msg)) {
                            buffer_.clear();
                            return true;
                        }
                        buffer_.clear();
                    }
                }
            }

            gettimeofday(&tv, nullptr);
            elapsed_ms = ((tv.tv_sec * 1000) + (tv.tv_usec / 1000)) - start_ms;
            if (n <= 0) {
                usleep(1000);
            }
        }

        return false;
    }

private:
    bool processFrame(CANMessage& msg) {
        if (buffer_.size() < 5 || buffer_[0] != 0xAA || buffer_.back() != 0x55) {
            std::cerr << "Invalid frame format" << std::endl;
            return false;
        }

        uint8_t frame_info = buffer_[1];
        uint8_t dlc = frame_info & 0x0F;
        bool extended = (frame_info & 0x20) != 0;

        if (buffer_.size() != static_cast<size_t>(dlc + (extended ? 7 : 5))) {
            std::cerr << "Frame length mismatch" << std::endl;
            return false;
        }

        if (extended) {
            msg.id = (buffer_[5] << 24) | (buffer_[4] << 16) | (buffer_[3] << 8) | buffer_[2];
        } else {
            msg.id = (buffer_[3] << 8) | buffer_[2];
        }
        msg.len = dlc;
        std::memcpy(msg.data, buffer_.data() + (extended ? 6 : 4), dlc);

        return true;
    }

    std::string device_;
    int baudrate_;
    int fd_;
    std::vector<uint8_t> buffer_;
};

// Function declarations
void init_log_file();
void parse_canopen_message(struct can_frame &frame, const std::string &timestamp);
void parse_j1939_message(struct can_frame &frame, const std::string &timestamp);
void parse_pdu_message(struct can_frame &frame, const std::string &timestamp);
void parse_can_message(struct can_frame &frame);
void log_data(const std::string &timestamp, const std::string &protocol, uint32_t cob_id, 
              const std::string &source, const std::string &dest, uint8_t dlc, 
              uint8_t *data, const std::string &description);
std::string decode_object_dictionary(uint16_t index, uint8_t subindex, uint8_t *data, uint8_t length);
std::string get_timestamp();

// Timestamp function
std::string get_timestamp() {
    time_t now = time(0);
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
    return std::string(buffer);
}

// Signal handler
void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        std::ofstream file(LOG_FILE, std::ios::app);
        if (file.is_open()) {
            file << get_timestamp() << ",SYSTEM,0x00000000,Telematics,ALL,0,," 
                 << "Application shutdown (signal " << signal << ")" << std::endl;
            file.flush();
            file.close();
        }
    }
    
    running = false;
}

// Log file management
void rotate_log_if_needed() {
    std::lock_guard<std::mutex> lock(log_mutex);
    
    std::ifstream in(LOG_FILE, std::ifstream::ate | std::ifstream::binary);
    if (in.is_open() && in.tellg() > MAX_LOG_SIZE_MB * 1024 * 1024) {
        in.close();
        
        std::string oldest = std::string(LOG_FILE) + std::to_string(MAX_LOG_FILES);
        std::remove(oldest.c_str());
        
        for (int i = MAX_LOG_FILES - 1; i > 0; i--) {
            std::string old_name = std::string(LOG_FILE) + std::to_string(i);
            std::string new_name = std::string(LOG_FILE) + std::to_string(i + 1);
            std::rename(old_name.c_str(), new_name.c_str());
        }
        
        std::string first_backup = std::string(LOG_FILE) + "1";
        std::rename(LOG_FILE, first_backup.c_str());
        
        std::ofstream new_file(LOG_FILE);
        new_file.close();
    }
}

void init_log_file() {
    std::lock_guard<std::mutex> lock(log_mutex);
    std::ifstream test(LOG_FILE);
    if (!test.good()) {
        std::ofstream file(LOG_FILE);
        if (file.is_open()) {
            file << "Timestamp,Protocol,COB-ID,Source,Destination,Length,Data,Description" << std::endl;
            file.close();
        }
    }
    test.close();
}

void log_data(const std::string &timestamp, const std::string &protocol, uint32_t cob_id, 
              const std::string &source, const std::string &dest, uint8_t dlc, 
              uint8_t *data, const std::string &description) {
    rotate_log_if_needed();
    
    std::lock_guard<std::mutex> lock(log_mutex);
    std::ofstream file(LOG_FILE, std::ios::app);
    if (file.is_open()) {
        file << timestamp << "," << protocol << ","
             << std::hex << "0x" << std::setw(8) << std::setfill('0') << cob_id << ","
             << source << "," << dest << ","
             << std::dec << static_cast<int>(dlc) << ",";
        
        for (int i = 0; i < dlc && i < 8; i++) {
            file << std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(data[i]) << (i < dlc - 1 ? " " : "");
        }
        
        file << "," << description << std::endl;
        file.flush();
        file.close();
    }
}

// Object Dictionary Decoder
std::string decode_object_dictionary(uint16_t index, uint8_t subindex, uint8_t *data, uint8_t length) {
    uint32_t key = (index << 8) | subindex;
    auto entry = objectDictionary.find(key);
    
    if (entry == objectDictionary.end()) {
        return "Unknown OD Entry";
    }

    std::string value_str;
    const ObjectDictionaryEntry& od = entry->second;
    
    if (od.size == 1) {
        value_str = std::to_string(data[0]);
    } 
    else if (od.size == 2) {
        uint16_t value = (data[1] << 8) | data[0];
        value_str = std::to_string(value);
    } 
    else if (od.size == 4) {
        uint32_t value = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        value_str = std::to_string(value);
    }

    return od.description + ": " + value_str + " (" + od.access + ")";
}

// CANopen Message Parsing (VCU-related, unchanged)
void parse_nmt_message(struct can_frame &frame, const std::string &timestamp) {
    uint8_t command = frame.data[0];
    uint8_t node_id = frame.data[1];
    
    std::string command_str;
    switch (command) {
        case 0x01: command_str = "Start Operational"; break;
        case 0x02: command_str = "Stop"; break;
        case 0x80: command_str = "Enter Pre-Operational"; break;
        case 0x81: command_str = "Reset Node"; break;
        case 0x82: command_str = "Reset Communication"; break;
        default: command_str = "Unknown NMT Command";
    }
    
    std::string description = "NMT " + command_str + " for Node 0x" + std::to_string(node_id);
    
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, "VCU", "MCU", frame.can_dlc, frame.data, description);
}

void parse_sync_message(struct can_frame &frame, const std::string &timestamp) {
    std::string description = "SYNC Message";
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, "VCU", "ALL", frame.can_dlc, frame.data, description);
}

void parse_heartbeat_message(struct can_frame &frame, const std::string &timestamp) {
    uint8_t node_id = frame.can_id & 0x7F;
    uint8_t state = frame.data[0];
    
    std::string state_str;
    switch (state) {
        case 0x00: state_str = "Boot up"; break;
        case 0x04: state_str = "Stopped"; break;
        case 0x05: state_str = "Operational"; break;
        case 0x7F: state_str = "Pre-Operational"; break;
        default: state_str = "Unknown State";
    }
    
    node_states[node_id] = state_str;
    
    std::string description = "Heartbeat from Node 0x" + std::to_string(node_id) + 
                            " - State: " + state_str;
    
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, "MCU", "VCU", frame.can_dlc, frame.data, description);
}

void parse_emcy_message(struct can_frame &frame, const std::string &timestamp) {
    uint8_t node_id = frame.can_id & 0x7F;
    uint16_t emergency_code = (frame.data[1] << 8) | frame.data[0];
    uint8_t error_register = frame.data[2];
    uint8_t faults2 = frame.data[3];
    uint8_t faults1 = frame.data[4];
    uint16_t warnings = (frame.data[6] << 8) | frame.data[5];
    
    std::string description = "EMCY from Node 0x" + std::to_string(node_id) + 
                            "\n  Code: 0x" + std::to_string(emergency_code);
    
    if (emergency_code == 0x1000) {
        description += " (Manufacturer Defined Error)";
    } else {
        description += " (Unknown Emergency Code)";
    }
    
    description += "\n  Error Register: " + std::bitset<8>(error_register).to_string();
    if (error_register & 0x01) description += "\n    - Generic Error";
    if (error_register & 0x02) description += "\n    - Current Error";
    if (error_register & 0x04) description += "\n    - Voltage Error";
    if (error_register & 0x08) description += "\n    - Temperature Error";
    if (error_register & 0x10) description += "\n    - Communication Error";
    if (error_register & 0x20) description += "\n    - Device Profile Specific";
    if (error_register & 0x80) description += "\n    - Manufacturer Specific";
    
    description += "\n  Faults2: " + std::bitset<8>(faults2).to_string();
    if (faults2 & 0x01) description += "\n    - Parameter CRC Fault";
    if (faults2 & 0x02) description += "\n    - Current Scaling Fault";
    if (faults2 & 0x04) description += "\n    - Voltage Scaling Fault";
    if (faults2 & 0x08) description += "\n    - Headlight Under Voltage";
    if (faults2 & 0x10) description += "\n    - Torque Sensor Fault";
    if (faults2 & 0x20) description += "\n    - CAN Bus Fault";
    if (faults2 & 0x40) description += "\n    - Hall Stall";
    if (faults2 & 0x80) description += "\n    - Bootloader Fault";
    
    description += "\n  Faults1: " + std::bitset<16>((faults1 << 8) | frame.data[5]).to_string().substr(8);
    if (faults1 & 0x01) description += "\n    - Controller Over Voltage";
    if (faults1 & 0x02) description += "\n    - Phase Over Current";
    if (faults1 & 0x04) description += "\n    - Current Sensor Calibration";
    if (faults1 & 0x08) description += "\n    - Current Sensor Over Current";
    if (faults1 & 0x10) description += "\n    - Controller Over Temperature";
    if (faults1 & 0x20) description += "\n    - Motor Hall Sensor Fault";
    if (faults1 & 0x40) description += "\n    - Controller Under Voltage";
    if (faults1 & 0x80) description += "\n    - POST Static Gating Test";
    
    description += "\n  Warnings: " + std::bitset<16>(warnings).to_string();
    if (warnings & 0x0001) description += "\n    - Communication Timeout";
    if (warnings & 0x0002) description += "\n    - Hall Sensor Warning";
    if (warnings & 0x0004) description += "\n    - Hall Stall Warning";
    if (warnings & 0x0008) description += "\n    - Wheel Speed Sensor Warning";
    if (warnings & 0x0010) description += "\n    - CAN Bus Warning";
    if (warnings & 0x0020) description += "\n    - Hall Illegal Sector";
    if (warnings & 0x0040) description += "\n    - Hall Illegal Transition";
    if (warnings & 0x0080) description += "\n    - Vdc Low Foldback";
    if (warnings & 0x0100) description += "\n    - Vdc High Foldback";
    if (warnings & 0x0200) description += "\n    - Motor Temperature Foldback";
    if (warnings & 0x0400) description += "\n    - Control Temperature Foldback";
    if (warnings & 0x0800) description += "\n    - Low SOC Foldback";
    if (warnings & 0x1000) description += "\n    - Hi SOC Foldback";
    if (warnings & 0x2000) description += "\n    - I2t Foldback";
    if (warnings & 0x8000) description += "\n    - BMS timeout";
    
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, "MCU", "VCU", frame.can_dlc, frame.data, description);
}

void parse_sdo_message(struct can_frame &frame, const std::string &timestamp) {
    bool is_response = (frame.can_id & 0x580) == SDO_TX_COB_ID_BASE;
    uint8_t node_id = frame.can_id - (is_response ? 0x580 : 0x600);
    
    uint8_t command = frame.data[0];
    uint16_t index = (frame.data[2] << 8) | frame.data[1];
    uint8_t sub_index = frame.data[3];
    
    std::string type = is_response ? "SDO-RX" : "SDO-TX";
    std::string description = type + " Node 0x" + std::to_string(node_id) + 
                           " Index: 0x" + std::to_string(index) +
                           " SubIndex: 0x" + std::to_string(sub_index);
    
    if (is_response) {
        switch (command) {
            case 0x43: description += " Read-4B"; break;
            case 0x47: description += " Read-3B"; break;
            case 0x4B: description += " Read-2B"; break;
            case 0x4F: description += " Read-1B"; break;
            case 0x60: description += " Write-OK"; break;
            case 0x80: description += " Error"; break;
            default: description += " Unknown-CMD";
        }
        
        if (command >= 0x43 && command <= 0x4F) {
            description += " Data:";
            for (int i = 4; i < frame.can_dlc; i++) {
                description += " " + std::to_string(frame.data[i]);
            }
            
            description += "\n  " + decode_object_dictionary(index, sub_index, &frame.data[4], frame.can_dlc-4);
        }
    } else {
        switch (command) {
            case 0x40: description += " Read-Req"; break;
            case 0x23: description += " Write-4B"; break;
            case 0x27: description += " Write-3B"; break;
            case 0x2B: description += " Write-2B"; break;
            case 0x2F: description += " Write-1B"; break;
            default: description += " Unknown-CMD";
        }
    }
    
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, is_response ? "MCU" : "VCU", 
             is_response ? "VCU" : "MCU", frame.can_dlc, frame.data, description);
}

void parse_pdo_message(struct can_frame &frame, const std::string &timestamp, bool is_tpdo) {
    uint8_t node_id = frame.can_id & 0x7F;
    std::string type;
    uint8_t expected_dlc = 0;
    std::string description;
    
    if (is_tpdo) {
        if ((frame.can_id & 0xF80) == 0x180) {
            type = "TPDO1 (Motor Data)";
            expected_dlc = 5;
            
            if (frame.can_dlc >= expected_dlc) {
                uint16_t rpm = (frame.data[1] << 8) | frame.data[0];
                uint8_t voltage = frame.data[2];
                uint8_t current = frame.data[3];
                uint8_t temp = frame.data[4];
                
                description = type + " from Node 0x" + std::to_string(node_id) + 
                           "\n  RPM: " + std::to_string(rpm) +
                           "\n  Voltage: " + std::to_string(voltage) + "V" +
                           "\n  Current: " + std::to_string(current) + "A" +
                           "\n  Motor Temp: " + std::to_string(temp) + "°C";
            }
        }
        else if ((frame.can_id & 0xF80) == 0x280) {
            type = "TPDO2 (System Data)";
            expected_dlc = 4;
            
            if (frame.can_dlc >= expected_dlc) {
                uint8_t soc = frame.data[0];
                uint8_t mode = frame.data[1];
                uint8_t batt_temp = frame.data[2];
                uint8_t errors = frame.data[3];
                
                description = type + " from Node 0x" + std::to_string(node_id) + 
                           "\n  SOC: " + std::to_string(soc) + "%" +
                           "\n  Mode: 0x" + std::to_string(mode) +
                           "\n  Battery Temp: " + std::to_string(batt_temp) + "°C" +
                           "\n  Errors: " + std::bitset<8>(errors).to_string();
            }
        }
    } 
    else {
        if ((frame.can_id & 0xF80) == 0x200) {
            type = "RPDO1 (Motor Cmd)";
            expected_dlc = 6;
            
            if (frame.can_dlc >= expected_dlc) {
                uint16_t speed = (frame.data[1] << 8) | frame.data[0];
                uint16_t torque = (frame.data[3] << 8) | frame.data[2];
                uint8_t direction = frame.data[4];
                uint8_t enable = frame.data[5];
                
                description = type + " to Node 0x" + std::to_string(node_id) + 
                           "\n  Speed: " + std::to_string(speed) + " RPM" +
                           "\n  Torque: " + std::to_string(torque) + "%" +
                           "\n  Direction: " + std::string(direction ? "Reverse" : "Forward") +
                           "\n  Enable: " + std::string(enable ? "On" : "Off");
            }
        }
        else if ((frame.can_id & 0xF80) == 0x300) {
            type = "RPDO2 (Config)";
            expected_dlc = 4;
            
            if (frame.can_dlc >= expected_dlc) {
                uint16_t max_speed = (frame.data[1] << 8) | frame.data[0];
                uint8_t regen = frame.data[2];
                uint8_t mode = frame.data[3];
                
                description = type + " to Node 0x" + std::to_string(node_id) + 
                           "\n  Max Speed: " + std::to_string(max_speed) + " RPM" +
                           "\n  Regen Level: " + std::to_string(regen) +
                           "\n  Mode: 0x" + std::to_string(mode);
            }
        }
    }
    
    if (description.empty()) {
        if (type.empty()) {
            type = is_tpdo ? "Unknown TPDO" : "Unknown RPDO";
        }
        description = type + " from Node 0x" + std::to_string(node_id) + 
                     " (Unrecognized format or wrong DLC)";
    }
    
    std::cout << "[" << timestamp << "] " << description << "\n";
    log_data(timestamp, "CANopen", frame.can_id, 
             is_tpdo ? "MCU" : "VCU", 
             is_tpdo ? "VCU" : "MCU", 
             frame.can_dlc, frame.data, description);
}

void parse_canopen_message(struct can_frame &frame, const std::string &timestamp) {
    if (frame.can_id == NMT_COB_ID) {
        parse_nmt_message(frame, timestamp);
    } 
    else if (frame.can_id == SYNC_COB_ID) {
        parse_sync_message(frame, timestamp);
    } 
    else if ((frame.can_id & 0x700) == HEARTBEAT_COB_ID_BASE) {
        parse_heartbeat_message(frame, timestamp);
    } 
    else if ((frame.can_id & 0xF80) == EMCY_COB_ID_BASE) {
        parse_emcy_message(frame, timestamp);
    } 
    else if ((frame.can_id & 0x580) == SDO_TX_COB_ID_BASE || 
             (frame.can_id & 0x600) == SDO_RX_COB_ID_BASE) {
        parse_sdo_message(frame, timestamp);
    } 
    else if ((frame.can_id & 0xF80) == 0x180) {
        parse_pdo_message(frame, timestamp, true);
    }
    else if ((frame.can_id & 0xF80) == 0x280) {
        parse_pdo_message(frame, timestamp, true);
    }
    else if ((frame.can_id & 0xF80) == 0x200) {
        parse_pdo_message(frame, timestamp, false);
    }
    else if ((frame.can_id & 0xF80) == 0x300) {
        parse_pdo_message(frame, timestamp, false);
    }
    else {
        std::string description = "Unknown CANopen Message";
        std::cout << "[" << timestamp << "] " << description << ": ID=0x" 
                  << std::hex << frame.can_id << "\n";
        log_data(timestamp, "CANopen", frame.can_id, "Unknown", "Unknown", 
                 frame.can_dlc, frame.data, description);
    }
}

// PDU Message Parsing (aligned with pdu_usb1.cpp)
void parse_pdu_message(struct can_frame &frame, const std::string &timestamp) {
    uint32_t can_id = frame.can_id & CAN_29BIT_ID_MASK;
    std::string source, dest, description;
    uint8_t opcode = frame.can_dlc > 0 ? frame.data[0] : 0xFF;
    
    // Build raw data string for logging
    std::string raw_data;
    for (int i = 0; i < frame.can_dlc; ++i) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X", frame.data[i]);
        raw_data += hex;
        if (i < frame.can_dlc - 1) raw_data += " ";
    }
    
    // VCU sends commands to PDU
    if (can_id == (PDU_COMMAND_ID & CAN_29BIT_ID_MASK)) {
        source = "VCU";
        dest = "PDU";
        if (frame.can_dlc < 3) {
            description = "Invalid PDU Command (DLC too short: " + std::to_string(frame.can_dlc) + ")";
            std::cout << "[" << timestamp << "] PDU Message: " << source << "->" << dest 
                      << " - " << description << "\n";
            log_data(timestamp, "J1939-PDU", frame.can_id, source, dest, frame.can_dlc, frame.data, description);
            return;
        }

        uint8_t rwFlag = frame.data[1] & 0x03;
        uint8_t element = frame.data[2];
        uint8_t address = (frame.can_id >> 8) & 0x07;
        bool is_group = element >= 100;
        std::string target = is_group ? "Group " + std::to_string(element - 100) : "Channel " + std::to_string(element);
        
        description = "PDU Command from VCU (Address: " + std::to_string(address) + ", Raw Data: " + raw_data + ")";
        description += "\n  Target: " + target;
        description += "\n  RWFlag: ";
        switch (rwFlag) {
            case 0: description += "Read"; break;
            case 1: description += "Write"; break;
            case 2: description += "Macro"; break;
            default: description += "Unknown (" + std::to_string(rwFlag) + ")";
        }

        switch (opcode) {
            case CHANNEL_CONTROL_1_CMD:
                if (frame.can_dlc < 4) {
                    description += "\n  Error: DLC too short for Channel Control 1";
                    break;
                }
                description += "\n  Opcode: Channel Control 1 (0x01)";
                description += "\n  State: " + std::string(frame.data[3] ? "On" : "Off");
                break;
            case CHANNEL_CONTROL_2_CMD:
                if (frame.can_dlc < 4) {
                    description += "\n  Error: DLC too short for Channel Control 2";
                    break;
                }
                description += "\n  Opcode: Channel Control 2 (0x03)";
                description += "\n  Battle Mode: " + std::string(frame.data[3] == 0x0D ? "Enabled" : "Disabled");
                break;
            case RESET_CMD:
                description += "\n  Opcode: Reset (0x07)";
                break;
            case CHANNEL_STATUS_1_CMD:
                description += "\n  Opcode: Channel Status 1 Request (0x21)";
                break;
            case CHANNEL_STATUS_2_CMD:
                description += "\n  Opcode: Channel Status 2 Request (0x25)";
                break;
            case INPUT_STATUS_CMD:
                description += "\n  Opcode: Input Status Request (0x27)";
                description += "\n  Element: " + (element == 190 ? "Valid (190)" : "Invalid (" + std::to_string(element) + ")");
                break;
            case TEMPERATURE_CMD:
                description += "\n  Opcode: Temperature Request (0x33)";
                description += "\n  Element: " + (element == 152 ? "Valid (152)" : "Invalid (" + std::to_string(element) + ")");
                break;
            case GROUPED_CHANNELS_CMD:
                description += "\n  Opcode: Grouped Channels Request (0x35)";
                break;
            default:
                description += "\n  Opcode: Unknown (0x" + std::to_string(opcode) + ")";
        }
    }
    // PDU sends responses to VCU
    else if (can_id == PDU_RESPONSE_ID) {
        source = "PDU";
        dest = "VCU";
        if (frame.can_dlc < 3) {
            description = "Invalid PDU Response (DLC too short: " + std::to_string(frame.can_dlc) + ")";
            std::cout << "[" << timestamp << "] PDU Message: " << source << "->" << dest 
                      << " - " << description << "\n";
            log_data(timestamp, "J1939-PDU", frame.can_id, source, dest, frame.can_dlc, frame.data, description);
            return;
        }

        uint8_t status = frame.data[1] >> 2;
        uint8_t element = frame.data[2];
        bool is_group = element >= 100;
        std::string target = is_group ? "Group " + std::to_string(element - 100) : "Channel " + std::to_string(element);
        
        std::string status_str;
        switch (status) {
            case SUCCESS: status_str = "Success"; break;
            case GENERAL_RW_ERROR: status_str = "General R/W Error"; break;
            case READ_NOT_SUPPORTED: status_str = "Read Not Supported"; break;
            case WRITE_NOT_SUPPORTED: status_str = "Write Not Supported"; break;
            case ERROR_WRITING_FLASH: status_str = "Error Writing Flash"; break;
            case WRONG_ELEMENT: status_str = "Wrong Element"; break;
            case CHANNEL_DOESNT_EXIST: status_str = "Channel Doesn't Exist"; break;
            case GROUP_DOESNT_EXIST: status_str = "Group Doesn't Exist"; break;
            case SENSOR_DOESNT_EXIST: status_str = "Sensor Doesn't Exist"; break;
            case BOARD_DOESNT_EXIST: status_str = "Board Doesn't Exist"; break;
            case WRONG_ADDRESS: status_str = "Wrong Address"; break;
            case WRONG_PAGE: status_str = "Wrong Page"; break;
            case WRONG_STATE: status_str = "Wrong State"; break;
            case WRONG_FLASH_INDEX: status_str = "Wrong Flash Index"; break;
            case WRONG_FLASH_KEY: status_str = "Wrong Flash Key"; break;
            case GROUP_IS_EMPTY: status_str = "Group Is Empty"; break;
            case CURRENT_OUT_OF_BOUNDS: status_str = "Current Out of Bounds"; break;
            case OPCODE_DOESNT_EXIST: status_str = "Opcode Doesn't Exist"; break;
            case GENERAL_ERROR: status_str = "General Error"; break;
            default: status_str = "Unknown (0x" + std::to_string(status) + ")";
        }
        
        description = "PDU Response to VCU (Raw Data: " + raw_data + ")";
        description += "\n  Target: " + target;
        description += "\n  Status: " + status_str;

        switch (opcode) {
            case CHANNEL_CONTROL_1_RSP:
                description += "\n  Opcode: Channel Control 1 Response (0x02)";
                if (frame.can_dlc >= 4) {
                    description += "\n  State: " + std::string(frame.data[3] ? "On" : "Off");
                }
                break;
            case CHANNEL_CONTROL_2_RSP:
                description += "\n  Opcode: Channel Control 2 Response (0x04)";
                if (frame.can_dlc >= 4) {
                    description += "\n  Battle Mode: " + std::string(frame.data[3] == 0x0D ? "Enabled" : "Disabled");
                }
                break;
            case RESET_RSP:
                description += "\n  Opcode: Reset Response (0x08)";
                break;
            case CHANNEL_STATUS_1_RSP:
                description += "\n  Opcode: Channel Status 1 Response (0x22)";
                if (frame.can_dlc >= 8) {
                    bool is_on = frame.data[3] == 0x01;
                    uint16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                    uint16_t voltage_raw = (frame.data[6] << 8) | frame.data[7];
                    float current = current_raw * 0.001f;
                    float voltage = voltage_raw * 0.05f - 1606.0f;
                    description += "\n  On: " + std::string(is_on ? "Yes" : "No");
                    if (is_on) {
                        description += "\n  Current: " + std::to_string(current) + " A";
                        description += "\n  Voltage: " + std::to_string(voltage) + " V";
                    }
                } else {
                    description += "\n  Error: Insufficient DLC (" + std::to_string(frame.can_dlc) + ")";
                }
                break;
            case CHANNEL_STATUS_2_RSP:
                description += "\n  Opcode: Channel Status 2 Response (0x26)";
                if (frame.can_dlc >= 6) {
                    uint16_t state_bits = (frame.data[3] << 8) | frame.data[4];
                    bool is_on = state_bits & 0x01;
                    bool is_tripped = (state_bits >> 8) & 0x01;
                    bool is_battle = (state_bits >> 12) & 0x01;
                    uint8_t group_id = frame.data[5];
                    description += "\n  On: " + std::string(is_on ? "Yes" : "No");
                    description += "\n  Tripped: " + std::string(is_tripped ? "Yes" : "No");
                    description += "\n  Battle Mode: " + std::string(is_battle ? "Enabled" : "Disabled");
                    description += "\n  Group ID: " + (group_id >= 100 ? std::to_string(group_id - 100) : "None");
                } else {
                    description += "\n  Error: Insufficient DLC (" + std::to_string(frame.can_dlc) + ")";
                }
                break;
            case INPUT_STATUS_RSP:
                description += "\n  Opcode: Input Status Response (0x28)";
                if (frame.can_dlc >= 8) {
                    uint32_t current_raw = (frame.data[3] << 16) | (frame.data[4] << 8) | frame.data[5];
                    uint16_t voltage_raw = (frame.data[6] << 8) | frame.data[7];
                    float current = current_raw * 0.01f - 80000.0f;
                    float voltage = voltage_raw * 0.05f - 1606.0f;
                    description += "\n  Current: " + std::to_string(current) + " A";
                    description += "\n  Voltage: " + std::to_string(voltage) + " V";
                } else {
                    description += "\n  Error: Insufficient DLC (" + std::to_string(frame.can_dlc) + ")";
                }
                break;
            case TEMPERATURE_RSP:
                description += "\n  Opcode: Temperature Response (0x34)";
                if (frame.can_dlc >= 5) {
                    uint16_t temp_raw = (frame.data[3] << 8) | frame.data[4];
                    float temperature = temp_raw * 0.03125f - 273.0f;
                    description += "\n  Temperature: " + std::to_string(temperature) + " °C";
                } else {
                    description += "\n  Error: Insufficient DLC (" + std::to_string(frame.can_dlc) + ")";
                }
                break;
            case GROUPED_CHANNELS_RSP:
                description += "\n  Opcode: Grouped Channels Response (0x36)";
                if (frame.can_dlc >= 5) {
                    uint16_t membership = (frame.data[3] << 8) | frame.data[4];
                    description += "\n  Channel Membership: " + std::bitset<16>(membership).to_string();
                } else {
                    description += "\n  Error: Insufficient DLC (" + std::to_string(frame.can_dlc) + ")";
                }
                break;
            default:
                description += "\n  Opcode: Unknown (0x" + std::to_string(opcode) + ")";
        }
    }
    // PDU sends alerts to all nodes, including VCU
    else if (can_id == PDU_ALERT_ID) {
        source = "PDU";
        dest = "ALL";
        if (opcode == TRIP_ALERT && frame.can_dlc >= 3) {
            uint8_t channel = frame.data[2];
            description = "PDU Alert (Raw Data: " + raw_data + ")";
            description += "\n  Opcode: Channel Trip Alert (0x50)";
            description += "\n  Channel: " + std::to_string(channel);
            description += "\n  Type: Overcurrent Trip";
        } else {
            description = "Unknown PDU Alert (Opcode: 0x" + std::to_string(opcode) + ", Raw Data: " + raw_data + ")";
        }
    }
    else {
        source = "UNKNOWN";
        dest = "UNKNOWN";
        description = "Unrecognized PDU Message (Opcode: 0x" + std::to_string(opcode) + ", Raw Data: " + raw_data + ")";
    }

    std::cout << "[" << timestamp << "] PDU Message: " << source << "->" << dest 
              << " - " << description << "\n";
    log_data(timestamp, "J1939-PDU", frame.can_id, source, dest, frame.can_dlc, frame.data, description);
}

// J1939 Message Parsing
void parse_j1939_message(struct can_frame &frame, const std::string &timestamp) {
    uint32_t can_id = frame.can_id & CAN_29BIT_ID_MASK;
    
    if (can_id == (PDU_COMMAND_ID & CAN_29BIT_ID_MASK) || 
        can_id == PDU_RESPONSE_ID || 
        can_id == PDU_ALERT_ID) {
        parse_pdu_message(frame, timestamp);
        return;
    }

    uint8_t priority = (can_id >> 26) & 0x7;
    uint16_t pgn = (can_id >> 8) & 0x3FFFF;
    uint8_t sa = can_id & 0xFF;
    
    std::string source;
    switch(sa) {
        case VCU_NODE_ID: source = "VCU"; break;
        case PDU_NODE_ID: source = "PDU"; break;
        case MCU_NODE_ID: source = "MCU"; break;
        case TELEMATICS_NODE_ID: source = "Telematics"; break;
        default: source = "Unknown";
    }

    std::string description = "J1939 Message PGN: 0x" + std::to_string(pgn) + 
                            ", Priority: " + std::to_string(priority);

    std::cout << "[" << timestamp << "] J1939: " << source << " - " << description << "\n";
    log_data(timestamp, "J1939", frame.can_id, source, "Broadcast", frame.can_dlc, frame.data, description);
}

// Main message parser
void parse_can_message(struct can_frame &frame) {
    std::string timestamp = get_timestamp();

    #if DEBUG
    std::cout << "Received frame: ID=0x" << std::hex << frame.can_id 
              << " Len=" << std::dec << (int)frame.can_dlc << "\n";
    #endif

    if (frame.can_id <= CAN_11BIT_ID_MASK) {
        parse_canopen_message(frame, timestamp);
    } else {
        parse_j1939_message(frame, timestamp);
    }
}

// Convert CANMessage to can_frame
void convert_to_can_frame(const CANMessage& msg, struct can_frame& frame) {
    frame.can_id = msg.id;
    frame.can_dlc = msg.len;
    std::memcpy(frame.data, msg.data, msg.len);
}

int main() {
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    init_log_file();
    
    CANHandler can_handler(CAN_INTERFACE, 1000000);
    if (!can_handler.open()) {
        std::cerr << "Error: Failed to initialize CAN handler" << std::endl;
        return 1;
    }

    std::cout << "CAN Telematics Node running. Supports:\n";
    std::cout << "- CANopen (11-bit IDs) for VCU\n";
    std::cout << "- SAE J1939 (29-bit IDs) for PDU and others\n";
    std::cout << "- Enhanced PDU message parsing (VCU->PDU commands, PDU->VCU responses)\n";
    std::cout << "- Object Dictionary decoding\n";
    std::cout << "Listening on " << CAN_INTERFACE << "\n";

    while (running) {
        CANMessage msg;
        if (can_handler.receiveFrame(msg, 1000)) {
            struct can_frame frame;
            convert_to_can_frame(msg, frame);
            parse_can_message(frame);
        }
    }

    can_handler.close();
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}