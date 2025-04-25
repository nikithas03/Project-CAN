#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <stdlib.h>

// CANopen Object Dictionary Index Definitions
#define OD_DEVICE_TYPE 0x1000
#define OD_ERROR_REGISTER 0x1001
#define OD_MANUFACTURER_STATUS 0x1002
#define OD_PREDEFINED_ERROR_FIELD 0x1003
#define OD_COB_ID_SYNC 0x1005
#define OD_COMMUNICATION_CYCLE 0x1006
#define OD_MANUFACTURER_DEVICE_NAME 0x1008
#define OD_HARDWARE_VERSION 0x1009
#define OD_SOFTWARE_VERSION 0x100A
#define OD_IDENTITY_OBJECT 0x1018
#define OD_SYNC_COUNTER 0x1019
#define OD_EMCY_CONSUMER 0x1028
#define OD_HEARTBEAT_CONSUMER 0x1016
#define OD_HEARTBEAT_PRODUCER 0x1017
#define OD_SDO_SERVER 0x1200
#define OD_SDO_CLIENT 0x1280
#define OD_RPDO_COMM_PARAM 0x1400
#define OD_RPDO_MAPPING 0x1600
#define OD_TPDO_COMM_PARAM 0x1800
#define OD_TPDO_MAPPING 0x1A00
#define OD_MOTOR_PARAMS 0x2000
#define OD_BATTERY_PARAMS 0x2001
#define OD_CONTROL_PARAMS 0x3000
#define OD_CONFIG_PARAMS 0x3001

// NMT States
typedef enum {
    BOOT_UP = 0x00,
    INITIALIZING = 0x00,
    STOPPED = 0x04,
    OPERATIONAL = 0x05,
    PRE_OPERATIONAL = 0x7F
} NMT_State;

// Emergency Error Codes
typedef enum {
    EMCY_GENERIC_ERROR = 0x1000,
    EMCY_CURRENT_OVERLOAD = 0x2310,
    EMCY_SHORT_CIRCUIT = 0x2320,
    EMCY_OVER_TEMP = 0x3210,
    EMCY_LOW_VOLTAGE = 0x4210,
    EMCY_OVER_VOLTAGE = 0x4211,
    EMCY_COMMUNICATION_ERROR = 0x8110
} EMCY_ErrorCode;

// TPDO Transmission Types
typedef enum {
    TPDO_SYNC_ACYCLIC = 0x00,
    TPDO_SYNC_N = 0x01,
    TPDO_EVENT_DRIVEN = 0xFE,
    TPDO_EVENT_DRIVEN_SYNC = 0xFF
} TPDO_TransmissionType;

// Object Dictionary Structure
typedef struct {
    uint32_t device_type;
    uint8_t error_register;
    uint16_t manufacturer_status;
    uint8_t predefined_error_field[8];
    uint32_t cob_id_sync;
    uint16_t communication_cycle;
    char manufacturer_device_name[32];
    char hardware_version[16];
    char software_version[16];
    uint32_t identity_object[4];
    uint8_t sync_counter;
    uint32_t emcy_consumer[8]; 
    uint32_t last_vcu_heartbeat;
    uint16_t heartbeat_producer;
    uint32_t heartbeat_consumer[8];
    uint32_t rpdo1_comm_params[3];
    uint32_t rpdo2_comm_params[3];
    uint32_t tpdo1_comm_params[3];
    uint32_t tpdo2_comm_params[3];
    uint32_t rpdo1_mapping[4];
    uint32_t rpdo2_mapping[4];
    uint32_t tpdo1_mapping[4];
    uint32_t tpdo2_mapping[4];
    uint16_t actual_rpm;
    uint16_t prev_actual_rpm;
    uint8_t voltage;
    uint8_t prev_voltage;
    uint8_t current;
    uint8_t prev_current;
    uint8_t motor_temp;
    uint8_t prev_motor_temp;
    uint8_t soc;
    uint8_t prev_soc;
    uint8_t motor_mode;
    uint8_t prev_motor_mode;
    uint8_t battery_temp;
    uint8_t prev_battery_temp;
    uint8_t error_flags;
    uint8_t prev_error_flags;
    uint16_t target_speed;
    uint16_t torque_limit;
    uint8_t direction;
    uint8_t motor_enable;
    uint16_t max_speed;
    uint8_t regen_level;
    uint8_t operation_mode;
} ObjectDictionary;

// CAN Message Structure
typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} CANMessage;

// Global Variables
ObjectDictionary OD = {
    .device_type = 0x000000F1,
    .error_register = 0,
    .manufacturer_status = 0x2A00,
    .predefined_error_field = {0},
    .cob_id_sync = 0x80,
    .communication_cycle = 100,
    .manufacturer_device_name = "EV_MCU_Controller",
    .hardware_version = "HW_1.0",
    .software_version = "SW_1.0",
    .identity_object = {0x12345678, 0x0000, 0x0000, 0x0000},
    .sync_counter = 1,
    .emcy_consumer = {0},
    .last_vcu_heartbeat = 0,
    .heartbeat_producer = 1000,
    .heartbeat_consumer = {0},
    .rpdo1_comm_params = {0x40000202, 0xFE, 0},
    .rpdo2_comm_params = {0x40000302, 0xFE, 0},
    .tpdo1_comm_params = {0x40000180, 0x01, 100},
    .tpdo2_comm_params = {0x40000280, 0xFF, 100},
    .rpdo1_mapping = {
        0x30000110,
        0x30000210,
        0x30000308,
        0x30000408
    },
    .rpdo2_mapping = {
        0x30010110,
        0x30020208,
        0x30030308,
        0x00000000
    },
    .tpdo1_mapping = {
        0x20000110,
        0x20000208,
        0x20000308,
        0x20000408
    },
    .tpdo2_mapping = {
        0x20010108,
        0x20010208,
        0x20010308,
        0x20010408
    },
    .actual_rpm = 0,
    .prev_actual_rpm = 0,
    .voltage = 0x30,
    .prev_voltage = 0x30,
    .current = 0x00,
    .prev_current = 0x00,
    .motor_temp = 25,
    .prev_motor_temp = 25,
    .soc = 100,
    .prev_soc = 100,
    .motor_mode = 1,
    .prev_motor_mode = 1,
    .battery_temp = 25,
    .prev_battery_temp = 25,
    .error_flags = 0,
    .prev_error_flags = 0,
    .target_speed = 0,
    .torque_limit = 100,
    .direction = 1,
    .motor_enable = 1,
    .max_speed = 2500,
    .regen_level = 30,
    .operation_mode = 1
};

const uint8_t mcu_node_id = 1;
const uint8_t vcu_node_id = 2;
NMT_State current_state = INITIALIZING;
uint32_t last_tpdo1 = 0;
uint32_t last_tpdo2 = 0;
uint32_t last_hb = 0;
uint32_t last_sync = 0;
uint8_t sync_counter = 0;
bool tpdo1_event_flag = false;
bool tpdo2_event_flag = false;
bool sync_received = false; // New flag to track SYNC reception

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

        log_file_.open("can_log.csv", std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            std::cerr << "Error: Failed to open can_log.csv for writing" << std::endl;
            close();
            return false;
        }
        log_file_.seekp(0, std::ios::end);
        if (log_file_.tellp() == 0) {
            log_file_ << "Timestamp,CAN_ID,Data\n";
        }

        return true;
    }

    void close() {
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    bool sendCANFrame(const CANMessage& msg) {
        if (fd_ == -1) {
            std::cerr << "Device not open" << std::endl;
            return false;
        }

        if (msg.len > 8) {
            std::cerr << "Data too long (max 8 bytes)" << std::endl;
            return false;
        }

        std::vector<uint8_t> frame;
        frame.push_back(0xAA);
        frame.push_back(0xC0 | (msg.len & 0x0F));
        frame.push_back(msg.id & 0xFF);
        frame.push_back((msg.id >> 8) & 0xFF);
        frame.insert(frame.end(), msg.data, msg.data + msg.len);
        frame.push_back(0x55);

        ssize_t written = write(fd_, frame.data(), frame.size());
        if (written != static_cast<ssize_t>(frame.size())) {
            perror("Failed to write frame");
            return false;
        }

        std::cout << "TX: ID 0x" << std::hex << std::setw(3) << std::setfill('0') << msg.id 
                  << " LEN " << std::dec << static_cast<int>(msg.len) << " DATA ";
        for (int i = 0; i < msg.len; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(msg.data[i]) << " ";
        }
        std::cout << std::dec << std::endl;

        return true;
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
            std::cerr << "Frame Fatal error: length mismatch" << std::endl;
            return false;
        }

        if (extended) {
            msg.id = (buffer_[5] << 24) | (buffer_[4] << 16) | (buffer_[3] << 8) | buffer_[2];
        } else {
            msg.id = (buffer_[3] << 8) | buffer_[2];
        }
        msg.len = dlc;
        std::memcpy(msg.data, buffer_.data() + (extended ? 6 : 4), dlc);

        struct timeval tv;
        gettimeofday(&tv, nullptr);
        double timestamp = tv.tv_sec + tv.tv_usec / 1000000.0;

        std::cout << "RX: " << std::fixed << std::setprecision(6) << timestamp
                  << " ID 0x" << std::hex << std::setw(3) << std::setfill('0') << msg.id
                  << " LEN " << std::dec << static_cast<int>(msg.len) << " DATA ";
        std::stringstream data_stream;
        for (int i = 0; i < msg.len; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(msg.data[i]) << " ";
            data_stream << std::hex << std::setw(2) << std::setfill('0') 
                        << static_cast<int>(msg.data[i]);
        }
        std::cout << std::dec << std::endl;

        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                      << std::hex << std::setw(3) << std::setfill('0') << msg.id << ","
                      << data_stream.str() << "\n";
            log_file_.flush();
            if (log_file_.fail()) {
                std::cerr << "Error: Failed to write to can_log.csv" << std::endl;
            }
        }

        return true;
    }

    std::string device_;
    int baudrate_;
    int fd_;
    std::ofstream log_file_;
    std::vector<uint8_t> buffer_;
};

// Function Prototypes
void can_send(CANMessage *msg);
uint32_t get_current_time_ms();
void send_heartbeat();
void send_tpdo1();
void send_tpdo2();
void send_emcy(uint16_t error_code, uint8_t error_register, uint8_t error_field[5]);
void send_bootup_message();
void handle_nmt(CANMessage *msg);
void handle_sync(CANMessage *msg);
void handle_emcy(CANMessage *msg);
void handle_rpdo1(CANMessage *msg);
void handle_rpdo2(CANMessage *msg);
void handle_sdo_request(CANMessage *msg);
void canopen_process();
void abort_response(CANMessage *response, uint8_t code1, uint8_t code2, uint8_t code3);
void handle_can_message(CANMessage *msg);
void check_errors();
void update_tpdo_event_flags();
bool should_send_tpdo1(uint32_t now);
bool should_send_tpdo2(uint32_t now);
void generate_random_tpdo_data();

// Global CANHandler
CANHandler* can_handler = nullptr;

// CAN Send Function
void can_send(CANMessage *msg) {
    if (!can_handler->sendCANFrame(*msg)) {
        std::cerr << "Failed to send CAN frame" << std::endl;
    }
}

// Get Current Time in Milliseconds
uint32_t get_current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

// Send Heartbeat Message
void send_heartbeat() {
    CANMessage msg;
    msg.id = 0x700 + mcu_node_id;
    msg.len = 1;
    msg.data[0] = current_state;
    can_send(&msg);
}

// Send Boot-up Message
void send_bootup_message() {
    CANMessage msg;
    msg.id = 0x700 + mcu_node_id;
    msg.len = 1;
    msg.data[0] = BOOT_UP;
    can_send(&msg);
    current_state = PRE_OPERATIONAL;
}

// Send Emergency Message
void send_emcy(uint16_t error_code, uint8_t error_register, uint8_t error_field[5]) {
    CANMessage msg;
    msg.id = 0x080 + mcu_node_id;
    msg.len = 8;
    msg.data[0] = error_code & 0xFF;
    msg.data[1] = (error_code >> 8) & 0xFF;
    msg.data[2] = error_register;
    std::memcpy(&msg.data[3], error_field, 5);
    can_send(&msg);
}

// Send TPDO1
void send_tpdo1() {
    CANMessage msg;
    msg.id = 0x180 + mcu_node_id;
    msg.len = 0;
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.tpdo1_mapping[i];
        if (mapping == 0) break;
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        uint8_t size_bytes = (size_bits + 7) / 8;
        switch(index) {
            case 0x2000:
                switch(subindex) {
                    case 0x01:
                        msg.data[msg.len++] = OD.actual_rpm & 0xFF;
                        msg.data[msg.len++] = (OD.actual_rpm >> 8) & 0xFF;
                        break;
                    case 0x02:
                        msg.data[msg.len++] = OD.voltage;
                        break;
                    case 0x03:
                        msg.data[msg.len++] = OD.current;
                        break;
                    case 0x04:
                        msg.data[msg.len++] = OD.motor_temp;
                        break;
                }
                break;
        }
    }
    printf("Sending TPDO1 with motor data: ");
    for (int i = 0; i < msg.len; i++) printf("%02X ", msg.data[i]);
    printf("\n");
    can_send(&msg);
    last_tpdo1 = get_current_time_ms();
    tpdo1_event_flag = false;
}

// Send TPDO2
void send_tpdo2() {
    CANMessage msg;
    msg.id = 0x280 + mcu_node_id;
    msg.len = 0;
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.tpdo2_mapping[i];
        if (mapping == 0) break;
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        uint8_t size_bytes = (size_bits + 7) / 8;
        switch(index) {
            case 0x2001:
                switch(subindex) {
                    case 0x01:
                        msg.data[msg.len++] = OD.soc;
                        break;
                    case 0x02:
                        msg.data[msg.len++] = OD.motor_mode;
                        break;
                    case 0x03:
                        msg.data[msg.len++] = OD.battery_temp;
                        break;
                    case 0x04:
                        msg.data[msg.len++] = OD.error_flags;
                        break;
                }
                break;
        }
    }
    printf("Sending TPDO2 with battery data: ");
    for (int i = 0; i < msg.len; i++) printf("%02X ", msg.data[i]);
    printf("\n");
    can_send(&msg);
    last_tpdo2 = get_current_time_ms();
    tpdo2_event_flag = false;
}

// Generate Random TPDO Data
void generate_random_tpdo_data() {
    if (current_state != OPERATIONAL) {
        return;
    }

    OD.actual_rpm = rand() % (OD.max_speed + 1);
    OD.voltage = 24 + (rand() % 20);
    OD.current = rand() % 50;
    OD.motor_temp = 25 + (rand() % 60);
    OD.soc = rand() % 101;
    OD.battery_temp = 15 + (rand() % 45);
    if ((rand() % 100) < 5) {
        OD.error_flags = 1 << (rand() % 8);
    } else {
        OD.error_flags = 0;
    }
    static uint32_t last_mode_change = 0;
    uint32_t now = get_current_time_ms();
    if (now - last_mode_change > 5000) {
        OD.motor_mode = 1 + (rand() % 3);
        last_mode_change = now;
    }
}

// Handle NMT Messages
void handle_nmt(CANMessage *msg) {
    if (msg->len < 1) return;
    uint8_t command = msg->data[0];
    uint8_t target_node = (msg->len > 1) ? msg->data[1] : 0;
    if (target_node != 0 && target_node != vcu_node_id) return;
    switch(command) {
        case 0x01: 
            current_state = OPERATIONAL; 
            printf("Entering OPERATIONAL state\n");
            break;
        case 0x80: 
            current_state = PRE_OPERATIONAL; 
            printf("Entering PRE-OPERATIONAL state\n");
            break;
        case 0x02: 
            current_state = STOPPED; 
            printf("Entering STOPPED state\n");
            break;
        case 0x81: 
            current_state = INITIALIZING; 
            printf("Entering INITIALIZING state\n");
            break;
    }
    send_heartbeat();
}

// Handle SYNC Messages
void handle_sync(CANMessage *msg) {
    if (msg->id != 0x080) {
        return;
    }

    printf("SYNC received (ID: 0x%03X)\n", msg->id);
    uint32_t now = get_current_time_ms();
    sync_counter++;
    sync_received = true; // Set flag to start continuous TPDO transmission
    update_tpdo_event_flags(); // Update event flags for consistency

    // Send TPDO1 immediately (SYNC_N, transmission type 0x01)
    uint16_t tpdo1_inhibit_time = (OD.tpdo1_comm_params[2] >> 16) & 0xFFFF;
    if (now - last_tpdo1 >= tpdo1_inhibit_time) {
        printf("Sending TPDO1 due to SYNC\n");
        send_tpdo1();
    }

    // Send TPDO2 immediately (EVENT_DRIVEN_SYNC, transmission type 0xFF)
    uint16_t tpdo2_inhibit_time = (OD.tpdo2_comm_params[2] >> 16) & 0xFFFF;
    if (now - last_tpdo2 >= tpdo2_inhibit_time) {
        printf("Sending TPDO2 due to SYNC\n");
        send_tpdo2();
    }

    last_sync = now;
}

// Handle EMCY Messages
void handle_emcy(CANMessage *msg) {
    printf("EMCY Received - ID: 0x%03X, Data: ", msg->id);
    for(int i=0; i<msg->len; i++) printf("%02X ", msg->data[i]);
    printf("\n");

    if (msg->id != 0x082 || msg->len < 8) return;

    uint16_t error_code = (msg->data[1] << 8) | msg->data[0];
    uint8_t faults_1 = msg->data[4];

    if (error_code == 0x1000) {
        if (faults_1 & 0x01) {
            OD.motor_enable = 0;
            printf("EMERGENCY: Disabled motor (Overvoltage)\n");
        }
        else if (faults_1 & 0x40) {
            OD.torque_limit /= 2;
            printf("WARNING: Torque reduced by 50%% (Undervoltage)\n");
        }
    }
}

// Handle RPDO1 Messages
void handle_rpdo1(CANMessage *msg) {
    if (current_state != OPERATIONAL) {
        printf("RPDO1 rejected: Not in OPERATIONAL state\n");
        return;
    }

    printf("Processing RPDO1...\n");
    uint8_t pos = 0;
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.rpdo1_mapping[i];
        if (mapping == 0) break;
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        uint8_t size_bytes = (size_bits + 7) / 8;
        printf(" Mapping %d: Index 0x%04X, Subindex %d, Size %d bits\n",
               i, index, subindex, size_bits);
        if (pos + size_bytes > msg->len) {
            printf(" Not enough data in message\n");
            break;
        }
        switch(index) {
            case 0x3000:
                switch(subindex) {
                    case 0x01:
                        OD.target_speed = (msg->data[pos+1] << 8) | msg->data[pos];
                        printf(" Set target_speed = %u (0x%04X)\n", OD.target_speed, OD.target_speed);
                        pos += 2;
                        break;
                    case 0x02:
                        OD.torque_limit = (msg->data[pos+1] << 8) | msg->data[pos];
                        printf(" Set torque_limit = %u (0x%04X)\n", OD.torque_limit, OD.torque_limit);
                        pos += 2;
                        break;
                    case 0x03:
                        OD.direction = msg->data[pos];
                        printf(" Set direction = %u\n", OD.direction);
                        pos += 1;
                        break;
                    case 0x04:
                        OD.motor_enable = msg->data[pos];
                        printf(" Set motor_enable = %u\n", OD.motor_enable);
                        pos += 1;
                        break;
                }
                break;
        }
    }
}

// Handle RPDO2 Messages
void handle_rpdo2(CANMessage *msg) {
    if (current_state != OPERATIONAL) {
        printf("RPDO2 rejected: Not in OPERATIONAL state\n");
        return;
    }

    printf("Processing RPDO2...\n");
    uint8_t pos = 0;
    if (pos + 2 <= msg->len) {
        OD.max_speed = (msg->data[pos+1] << 8) | msg->data[pos];
        printf(" Set max_speed = %u (0x%04X)\n", OD.max_speed, OD.max_speed);
        pos += 2;
    }

    if (pos + 1 <= msg->len) {
        OD.regen_level = msg->data[pos];
        printf(" Set regen_level = %u (0x%02X)\n", OD.regen_level, OD.regen_level);
        pos += 1;
    }

    if (pos + 1 <= msg->len) {
        OD.operation_mode = msg->data[pos];
        printf(" Set operation_mode = %u (0x%02X)\n", OD.operation_mode, OD.operation_mode);
        pos += 1;
    }

    if (pos < msg->len) {
        printf(" Warning: %d bytes unprocessed in RPDO2\n", msg->len - pos);
    }
}

// Abort Response
void abort_response(CANMessage *response, uint8_t code1, uint8_t code2, uint8_t code3) {
    response->data[0] = 0x80;
    response->data[4] = code1;
    response->data[5] = code2;
    response->data[6] = code3;
    response->data[7] = 0x00;
}

// Handle SDO Requests
void handle_sdo_request(CANMessage *msg) {
    uint8_t client_id = (msg->id - 0x600) & 0x7F;
    if (client_id != vcu_node_id) {
        printf("Ignoring SDO request from non-VCU node %d\n", client_id);
        return;
    }

    if (msg->len < 4) return;
    uint8_t cs = msg->data[0];
    uint16_t index = (msg->data[2] << 8) | msg->data[1];
    uint8_t subindex = msg->data[3];
    CANMessage response;
    response.id = 0x580 + mcu_node_id;
    response.len = 8;
    memset(response.data, 0, 8);
    response.data[1] = msg->data[1];
    response.data[2] = msg->data[2];
    response.data[3] = subindex;

    if ((cs & 0xE0) == 0x40) {
        response.data[0] = 0x43;
        switch(index) {
            case 0x2000:
                switch(subindex) {
                    case 0x01:
                        response.data[4] = (OD.actual_rpm >> 8) & 0xFF;
                        response.data[5] = OD.actual_rpm & 0xFF;
                        break;
                    case 0x02:
                        response.data[4] = OD.voltage;
                        break;
                    case 0x03:
                        response.data[4] = OD.current;
                        break;
                    case 0x04:
                        response.data[4] = OD.motor_temp;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            case 0x2001:
                switch(subindex) {
                    case 0x01:
                        response.data[4] = OD.soc;
                        break;
                    case 0x02:
                        response.data[4] = OD.motor_mode;
                        break;
                    case 0x03:
                        response.data[4] = OD.battery_temp;
                        break;
                    case 0x04:
                        response.data[4] = OD.error_flags;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            case 0x3000:
                switch(subindex) {
                    case 0x01:
                        response.data[4] = (OD.target_speed >> 8) & 0xFF;
                        response.data[5] = OD.target_speed & 0xFF;
                        break;
                    case 0x02:
                        response.data[4] = (OD.torque_limit >> 8) & 0xFF;
                        response.data[5] = OD.torque_limit & 0xFF;
                        break;
                    case 0x03:
                        response.data[4] = OD.direction;
                        break;
                    case 0x04:
                        response.data[4] = OD.motor_enable;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            case 0x3001:
                switch(subindex) {
                    case 0x01:
                        response.data[4] = (OD.max_speed >> 8) & 0xFF;
                        response.data[5] = OD.max_speed & 0xFF;
                        break;
                    case 0x02:
                        response.data[4] = OD.regen_level;
                        break;
                    case 0x03:
                        response.data[4] = OD.operation_mode;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            default:
                abort_response(&response, 0x00, 0x02, 0x06);
        }
    }
    else if ((cs & 0xE0) == 0x20) {
        uint8_t size = 4 - ((cs >> 2) & 0x03);
        uint32_t data = 0;
        memcpy(&data, &msg->data[4], 4);
        response.data[0] = 0x60;
        switch(index) {
            case 0x3000:
                switch(subindex) {
                    case 0x01:
                        OD.target_speed = data;
                        break;
                    case 0x02:
                        OD.torque_limit = data;
                        break;
                    case 0x03:
                        OD.direction = data;
                        break;
                    case 0x04:
                        OD.motor_enable = data;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            case 0x3001:
                switch(subindex) {
                    case 0x01:
                        OD.max_speed = data;
                        break;
                    case 0x02:
                        OD.regen_level = data;
                        break;
                    case 0x03:
                        OD.operation_mode = data;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
            default:
                abort_response(&response, 0x00, 0x04, 0x05);
        }
    }
    else {
        abort_response(&response, 0x00, 0x04, 0x05);
    }

    can_send(&response);
    printf("SDO Response to VCU: 0x%04X:%02X: ", index, subindex);
    for(int i = 0; i < response.len; i++) printf("%02X ", response.data[i]);
    printf("\n");
}

// Check Errors
void check_errors() {
    uint8_t new_error_register = 0;
    if (OD.voltage < ((OD.manufacturer_status & 0xFF) - 10)) {
        new_error_register |= 0x01;
    }
    OD.error_register = new_error_register;
}

// Update TPDO Event Flags
void update_tpdo_event_flags() {
#define RPM_THRESHOLD 10
#define VOLTAGE_THRESHOLD 1
#define TEMP_THRESHOLD 2
#define SOC_THRESHOLD 1
    if (abs(OD.actual_rpm - OD.prev_actual_rpm) > RPM_THRESHOLD ||
        abs(OD.voltage - OD.prev_voltage) > VOLTAGE_THRESHOLD ||
        abs(OD.motor_temp - OD.prev_motor_temp) > TEMP_THRESHOLD) {
        tpdo1_event_flag = true;
    }
    if (abs(OD.soc - OD.prev_soc) > SOC_THRESHOLD ||
        OD.motor_mode != OD.prev_motor_mode ||
        abs(OD.battery_temp - OD.prev_battery_temp) > TEMP_THRESHOLD ||
        OD.error_flags != OD.prev_error_flags) {
        tpdo2_event_flag = true;
    }
    OD.prev_actual_rpm = OD.actual_rpm;
    OD.prev_voltage = OD.voltage;
    OD.prev_motor_temp = OD.motor_temp;
    OD.prev_soc = OD.soc;
    OD.prev_motor_mode = OD.motor_mode;
    OD.prev_battery_temp = OD.battery_temp;
    OD.prev_error_flags = OD.error_flags;
}

// Should Send TPDO1
bool should_send_tpdo1(uint32_t now) {
    uint8_t transmission_type = OD.tpdo1_comm_params[1] & 0xFF;
    uint16_t inhibit_time = (OD.tpdo1_comm_params[2] >> 16) & 0xFFFF;
    uint16_t event_timer = OD.tpdo1_comm_params[2] & 0xFFFF;
    switch(transmission_type) {
        case TPDO_SYNC_ACYCLIC:
            return false;
        case TPDO_SYNC_N:
            return false;
        case TPDO_EVENT_DRIVEN:
            return tpdo1_event_flag && 
                   (now - last_tpdo1 >= inhibit_time) &&
                   (event_timer == 0 || now - last_tpdo1 >= event_timer);
        case TPDO_EVENT_DRIVEN_SYNC:
            return false;
        default:
            if (transmission_type >= 1 && transmission_type <= 240) {
                return false;
            } else {
                return (now - last_tpdo1 >= (uint32_t)OD.tpdo1_comm_params[2]);
            }
    }
}

// Should Send TPDO2
bool should_send_tpdo2(uint32_t now) {
    uint8_t transmission_type = OD.tpdo2_comm_params[1] & 0xFF;
    uint16_t inhibit_time = (OD.tpdo2_comm_params[2] >> 16) & 0xFFFF;
    uint16_t event_timer = OD.tpdo2_comm_params[2] & 0xFFFF;
    switch(transmission_type) {
        case TPDO_SYNC_ACYCLIC:
            return false;
        case TPDO_SYNC_N:
            return false;
        case TPDO_EVENT_DRIVEN:
            return tpdo2_event_flag && 
                   (now - last_tpdo2 >= inhibit_time) &&
                   (event_timer == 0 || now - last_tpdo2 >= event_timer);
        case TPDO_EVENT_DRIVEN_SYNC:
            return false;
        default:
            if (transmission_type >= 1 && transmission_type <= 240) {
                return false;
            } else {
                return (now - last_tpdo2 >= (uint32_t)OD.tpdo2_comm_params[2]);
            }
    }
}

// CANopen Process
void canopen_process() {
    uint32_t now = get_current_time_ms();
    update_tpdo_event_flags();
    if (now - last_hb >= OD.heartbeat_producer) {
        send_heartbeat();
        last_hb = now;
    }
    if (current_state == OPERATIONAL && sync_received) {
        // Send TPDO1 every 50ms
        uint16_t tpdo1_inhibit_time = (OD.tpdo1_comm_params[2] >> 16) & 0xFFFF;
        if (now - last_tpdo1 >= 50 && now - last_tpdo1 >= tpdo1_inhibit_time) {
            printf("Sending TPDO1 (periodic, 50ms)\n");
            send_tpdo1();
        }
        // Send TPDO2 every 500ms
        uint16_t tpdo2_inhibit_time = (OD.tpdo2_comm_params[2] >> 16) & 0xFFFF;
        if (now - last_tpdo2 >= 500 && now - last_tpdo2 >= tpdo2_inhibit_time) {
            printf("Sending TPDO2 (periodic, 500ms)\n");
            send_tpdo2();
        }
    }
    if (current_state == OPERATIONAL) {
        if (should_send_tpdo1(now)) {
            send_tpdo1();
        }
        if (should_send_tpdo2(now)) {
            send_tpdo2();
        }
    }
    check_errors();
}

// Handle CAN Messages
void handle_can_message(CANMessage *msg) {
    if (msg->id == 0x081) {
        return;
    }

    printf("RX: ID 0x%03X LEN %d DATA ", msg->id, msg->len);
    for (int i = 0; i < msg->len; i++) {
        printf("%02X ", msg->data[i]);
    }
    printf("\n");

    if (msg->id == 0x000) {
        handle_nmt(msg);
    } 
    else if (msg->id == 0x080) {
        handle_sync(msg);
    }
    else if ((msg->id & 0xF80) == 0x080) {
        handle_emcy(msg);
    }
    else if (msg->id == 0x202) {
        handle_rpdo1(msg);
    }
    else if (msg->id == 0x302) {
        handle_rpdo2(msg);
    }
    else if ((msg->id & 0x780) == 0x600) {
        handle_sdo_request(msg);
    }
}

int main() {
    srand(time(NULL));
    std::string device = "/dev/ttyUSB0";
    int baudrate = 1000000;
    can_handler = new CANHandler(device, baudrate);
    if (!can_handler->open()) {
        std::cerr << "Error: Failed to initialize CAN handler" << std::endl;
        return 1;
    }

    printf("MCU Node ID: %d\n", mcu_node_id);
    printf("CAN initialized on %s\n", device.c_str());

    send_bootup_message();
    uint32_t last_data_update = 0;
    while (true) {
        uint32_t now = get_current_time_ms();
        if (current_state == OPERATIONAL && now - last_data_update > 100) {
            generate_random_tpdo_data();
            last_data_update = now;
        }
        canopen_process();
        CANMessage msg;
        if (can_handler->receiveFrame(msg, 10)) {
            handle_can_message(&msg);
        }
    }
    can_handler->close();
    delete can_handler;
    return 0;
}