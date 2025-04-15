#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>
#include <stdlib.h>
#include <sys/select.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>

// CANopen Object Dictionary Index Definitions
#define OD_DEVICE_TYPE             0x1000
#define OD_ERROR_REGISTER          0x1001
#define OD_MANUFACTURER_STATUS     0x1002
#define OD_PREDEFINED_ERROR_FIELD  0x1003
#define OD_COB_ID_SYNC             0x1005
#define OD_COMMUNICATION_CYCLE     0x1006
#define OD_MANUFACTURER_DEVICE_NAME 0x1008
#define OD_HARDWARE_VERSION        0x1009
#define OD_SOFTWARE_VERSION        0x100A
#define OD_IDENTITY_OBJECT         0x1018
#define OD_SYNC_COUNTER            0x1019
#define OD_EMCY_CONSUMER           0x1028
#define OD_HEARTBEAT_CONSUMER      0x1016
#define OD_HEARTBEAT_PRODUCER      0x1017
#define OD_SDO_SERVER              0x1200
#define OD_SDO_CLIENT              0x1280
#define OD_RPDO_COMM_PARAM         0x1400
#define OD_RPDO_MAPPING            0x1600
#define OD_TPDO_COMM_PARAM         0x1800
#define OD_TPDO_MAPPING            0x1A00

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
    // Communication Parameters
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
    
    // PDO Communication Parameters
    uint32_t rpdo1_comm_params[3];
    uint32_t rpdo2_comm_params[3];
    uint32_t tpdo1_comm_params[3];
    uint32_t tpdo2_comm_params[3];
    
    // PDO Mapping Parameters (Updated to ICD addresses)
    uint32_t rpdo1_mapping[4];
    uint32_t rpdo2_mapping[4];
    uint32_t tpdo1_mapping[4];
    uint32_t tpdo2_mapping[4];
    
    // Application Objects (ICD-compliant)
    // TPDO1 Objects
    uint16_t motor_rpm;
    uint8_t motor_current;
    uint8_t motor_temp;
    uint16_t motor_speed;
    
    // TPDO2 Objects
    uint16_t battery_voltage;
    uint16_t battery_current;
    uint8_t battery_soc;
    uint8_t controller_temp;
    
    // RPDO1 Objects
    uint16_t target_speed;
    uint16_t torque_limit;
    uint8_t direction;
    uint8_t motor_enable;
    
    // RPDO2 Objects
    uint16_t max_speed;
    uint8_t regen_level;
    uint8_t assist_level;
    
    // Previous values
    uint16_t prev_motor_rpm;
    uint8_t prev_motor_current;
    uint8_t prev_motor_temp;
    uint16_t prev_motor_speed;
    uint16_t prev_battery_voltage;
    uint16_t prev_battery_current;
    uint8_t prev_battery_soc;
    uint8_t prev_controller_temp;
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
    
    // PDO Communication Parameters
    .rpdo1_comm_params = {0x40000202, 0xFF, 200}, // Periodic, 200ms
    .rpdo2_comm_params = {0x40000302, 0xFE, 0},
    .tpdo1_comm_params = {0x40000180, 0xFF, 100}, // Periodic, 100ms
    .tpdo2_comm_params = {0x40000280, 0xFF, 150}, // Periodic, 150ms
    
    // Updated PDO Mappings (ICD-compliant)
    .rpdo1_mapping = {
        0x20071A10, // Target Speed (0x2007:1A, 16-bit) - Speed command
        0x20071E10, // Torque Limit (0x2007:1E, 16-bit) - Applied Torque command (%)
        0x20071D08, // Motor Enable (0x2007:1D, 8-bit) - State command
        0x00000000  // Unused
    },
    .rpdo2_mapping = {
        0x20032610, // Max Speed (0x2003:26, 16-bit)
        0x20021D08, // Regen Level (0x2002:1D, 8-bit)
        0x20051408, // Assist Level (0x2005:14, 8-bit)
        0x00000000  // Unused
    },
    .tpdo1_mapping = {
        0x20040810, // Motor RPM (0x2004:08, 16-bit)
        0x20040708, // Motor Current (0x2004:07, 8-bit)
        0x20040608, // Motor Temp (0x2004:06, 8-bit)
        0x20040910  // Motor Speed % (0x2004:09, 16-bit)
    },
    .tpdo2_mapping = {
        0x20040A10, // Battery Voltage (0x2004:0A, 16-bit)
        0x20040B10, // Battery Current (0x2004:0B, 16-bit)
        0x20040C08, // Battery SOC (0x2004:0C, 8-bit)
        0x20040408  // Controller Temp (0x2004:04, 8-bit)
    },
    
    // Initialize application objects
    .motor_rpm = 0,
    .motor_current = 0,
    .motor_temp = 25,
    .motor_speed = 0,
    .battery_voltage = 0,
    .battery_current = 0,
    .battery_soc = 100,
    .controller_temp = 25,
    .target_speed = 0,
    .torque_limit = 100,
    .direction = 1,
    .motor_enable = 1,
    .max_speed = 2500,
    .regen_level = 30,
    .assist_level = 1,
    
    // Initialize previous values
    .prev_motor_rpm = 0,
    .prev_motor_current = 0,
    .prev_motor_temp = 25,
    .prev_motor_speed = 0,
    .prev_battery_voltage = 0,
    .prev_battery_current = 0,
    .prev_battery_soc = 100,
    .prev_controller_temp = 25
};

const uint8_t mcu_node_id = 1;
const uint8_t vcu_node_id = 2;
NMT_State current_state = INITIALIZING;
int can_socket = -1;
uint32_t last_tpdo1 = 0;
uint32_t last_tpdo2 = 0;
uint32_t last_rpdo1 = 0;
uint32_t last_hb = 0;
uint32_t last_sync = 0;
uint8_t sync_counter = 0;

void can_init(const char* ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation error");
        exit(1);
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("IOCTL error");
        close(can_socket);
        exit(1);
    }

    struct can_filter rfilter[2] = {
        {.can_id = 0x081, .can_mask = (CAN_SFF_MASK | CAN_INV_FILTER)},
        {.can_id = 0x000, .can_mask = 0x000}
    };

    if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter)) < 0) {
        perror("Filter setup error");
        close(can_socket);
        exit(1);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind error");
        close(can_socket);
        exit(1);
    }
}

void can_send(CANMessage *msg) {
    struct can_frame frame;
    frame.can_id = msg->id;
    frame.can_dlc = msg->len;
    memcpy(frame.data, msg->data, msg->len);

    if (write(can_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("Write");
    }
}

uint32_t get_current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

void send_heartbeat() {
    CANMessage msg;
    msg.id = 0x700 + mcu_node_id;
    msg.len = 1;
    msg.data[0] = current_state;
    can_send(&msg);
}

void send_bootup_message() {
    CANMessage msg;
    msg.id = 0x700 + mcu_node_id;
    msg.len = 1;
    msg.data[0] = BOOT_UP;
    can_send(&msg);
    current_state = PRE_OPERATIONAL;
}

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
        
        switch(index) {
            case 0x2004:
                switch(subindex) {
                    case 0x08:
                        msg.data[msg.len++] = OD.motor_rpm & 0xFF;
                        msg.data[msg.len++] = (OD.motor_rpm >> 8) & 0xFF;
                        break;
                    case 0x07:
                        msg.data[msg.len++] = OD.motor_current;
                        break;
                    case 0x06:
                        msg.data[msg.len++] = OD.motor_temp;
                        break;
                    case 0x09:
                        msg.data[msg.len++] = OD.motor_speed & 0xFF;
                        msg.data[msg.len++] = (OD.motor_speed >> 8) & 0xFF;
                        break;
                }
                break;
        }
    }
    
    can_send(&msg);
    last_tpdo1 = get_current_time_ms();
}

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
        
        switch(index) {
            case 0x2004:
                switch(subindex) {
                    case 0x0A:
                        msg.data[msg.len++] = OD.battery_voltage & 0xFF;
                        msg.data[msg.len++] = (OD.battery_voltage >> 8) & 0xFF;
                        break;
                    case 0x0B:
                        msg.data[msg.len++] = OD.battery_current & 0xFF;
                        msg.data[msg.len++] = (OD.battery_current >> 8) & 0xFF;
                        break;
                    case 0x0C:
                        msg.data[msg.len++] = OD.battery_soc;
                        break;
                    case 0x04:
                        msg.data[msg.len++] = OD.controller_temp;
                        break;
                }
                break;
        }
    }
    
    can_send(&msg);
    last_tpdo2 = get_current_time_ms();
}

void send_rpdo1() {
    CANMessage msg;
    msg.id = 0x202;
    msg.len = 0;
    
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.rpdo1_mapping[i];
        if (mapping == 0) break;
        
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        
        switch(index) {
            case 0x2007:
                switch(subindex) {
                    case 0x1A:
                        msg.data[msg.len++] = OD.target_speed & 0xFF;
                        msg.data[msg.len++] = (OD.target_speed >> 8) & 0xFF;
                        break;
                    case 0x1E:
                        msg.data[msg.len++] = OD.torque_limit & 0xFF;
                        msg.data[msg.len++] = (OD.torque_limit >> 8) & 0xFF;
                        break;
                    case 0x1D:
                        msg.data[msg.len++] = OD.motor_enable;
                        break;
                }
                break;
        }
    }
    
    can_send(&msg);
    last_rpdo1 = get_current_time_ms();
}

void generate_random_rpdo1_data() {
    if (current_state != OPERATIONAL) return;
    
    OD.target_speed = rand() % (OD.max_speed + 1);
    OD.torque_limit = 50 + (rand() % 51); // 50-100%
    OD.motor_enable = (rand() % 2); // 0 or 1
}

void generate_random_tpdo_data() {
    if (current_state != OPERATIONAL) return;

    OD.motor_rpm = rand() % (OD.max_speed + 1);
    OD.motor_current = rand() % 50;
    OD.motor_temp = 25 + (rand() % 60);
    OD.motor_speed = (OD.motor_rpm * 4096) / OD.max_speed;
    
    OD.battery_voltage = 360 + (rand() % 80);
    OD.battery_current = rand() % 50;
    OD.battery_soc = rand() % 101;
    OD.controller_temp = 25 + (rand() % 40);
    
    static uint32_t last_assist_change = 0;
    uint32_t now = get_current_time_ms();
    if (now - last_assist_change > 5000) {
        OD.assist_level = 1 + (rand() % 3);
        last_assist_change = now;
    }
}

void handle_nmt(CANMessage *msg) {
    if (msg->len < 1) return;
    
    uint8_t command = msg->data[0];
    uint8_t target_node = (msg->len > 1) ? msg->data[1] : 0;
    
    if (target_node != 0 && target_node != vcu_node_id) return;
    
    switch(command) {
        case 0x01: current_state = OPERATIONAL; break;
        case 0x80: current_state = PRE_OPERATIONAL; break;
        case 0x02: current_state = STOPPED; break;
        case 0x81: current_state = INITIALIZING; break;
    }
    send_heartbeat();
}

void handle_sync(CANMessage *msg) {
    if (msg->id != 0x080) return;

    uint32_t now = get_current_time_ms();
    sync_counter++;
    
    last_sync = now;
    last_rpdo1 = now; // Reset timers on SYNC
    last_tpdo1 = now;
    last_tpdo2 = now;
}

void handle_emcy(CANMessage *msg) {
    if (msg->id != 0x082 || msg->len < 8) return;

    uint16_t error_code = (msg->data[1] << 8) | msg->data[0];
    uint8_t faults_1 = msg->data[4];

    if (error_code == 0x1000) {
        if (faults_1 & 0x01) OD.motor_enable = 0;
        else if (faults_1 & 0x40) OD.torque_limit /= 2;
    }
}

void handle_rpdo1(CANMessage *msg) {
    if (current_state != OPERATIONAL) return;

    uint8_t pos = 0;
    
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.rpdo1_mapping[i];
        if (mapping == 0) break;
        
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        uint8_t size_bytes = (size_bits + 7) / 8;
        
        if (pos + size_bytes > msg->len) break;
        
        switch(index) {
            case 0x2007:
                switch(subindex) {
                    case 0x1A:
                        OD.target_speed = (msg->data[pos] << 8) | msg->data[pos+1];
                        pos += 2;
                        break;
                    case 0x1E:
                        OD.torque_limit = (msg->data[pos] << 8) | msg->data[pos+1];
                        pos += 2;
                        break;
                    case 0x1D:
                        OD.motor_enable = msg->data[pos];
                        pos += 1;
                        break;
                }
                break;
        }
    }
}

void handle_rpdo2(CANMessage *msg) {
    if (current_state != OPERATIONAL) return;

    uint8_t pos = 0;
    
    for (int i = 0; i < 4; i++) {
        uint32_t mapping = OD.rpdo2_mapping[i];
        if (mapping == 0) break;
        
        uint16_t index = (mapping >> 16) & 0xFFFF;
        uint8_t subindex = (mapping >> 8) & 0xFF;
        uint8_t size_bits = mapping & 0xFF;
        uint8_t size_bytes = (size_bits + 7) / 8;
        
        if (pos + size_bytes > msg->len) break;
        
        switch(index) {
            case 0x2003:
                switch(subindex) {
                    case 0x26:
                        OD.max_speed = (msg->data[pos] << 8) | msg->data[pos+1];
                        pos += 2;
                        break;
                }
                break;
            case 0x2002:
                switch(subindex) {
                    case 0x1D:
                        OD.regen_level = msg->data[pos];
                        pos += 1;
                        break;
                }
                break;
            case 0x2005:
                switch(subindex) {
                    case 0x14:
                        OD.assist_level = msg->data[pos];
                        pos += 1;
                        break;
                }
                break;
        }
    }
}

void abort_response(CANMessage *response, uint8_t code1, uint8_t code2, uint8_t code3) {
    response->data[0] = 0x80;
    response->data[4] = code1;
    response->data[5] = code2;
    response->data[6] = code3;
    response->data[7] = 0x00;
}

void handle_sdo_request(CANMessage *msg) {
    uint8_t client_id = (msg->id - 0x600) & 0x7F;
    if (client_id != vcu_node_id) return;

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
            case 0x2004:
                switch(subindex) {
                    case 0x08:
                        response.data[4] = (OD.motor_rpm >> 8) & 0xFF;
                        response.data[5] = OD.motor_rpm & 0xFF;
                        break;
                    case 0x07:
                        response.data[4] = OD.motor_current;
                        break;
                    case 0x06:
                        response.data[4] = OD.motor_temp;
                        break;
                    case 0x09:
                        response.data[4] = (OD.motor_speed >> 8) & 0xFF;
                        response.data[5] = OD.motor_speed & 0xFF;
                        break;
                    case 0x0A:
                        response.data[4] = (OD.battery_voltage >> 8) & 0xFF;
                        response.data[5] = OD.battery_voltage & 0xFF;
                        break;
                    case 0x0B:
                        response.data[4] = (OD.battery_current >> 8) & 0xFF;
                        response.data[5] = OD.battery_current & 0xFF;
                        break;
                    case 0x0C:
                        response.data[4] = OD.battery_soc;
                        break;
                    case 0x04:
                        response.data[4] = OD.controller_temp;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2007:
                switch(subindex) {
                    case 0x1A:
                        response.data[4] = (OD.target_speed >> 8) & 0xFF;
                        response.data[5] = OD.target_speed & 0xFF;
                        break;
                    case 0x1D:
                        response.data[4] = OD.motor_enable;
                        break;
                    case 0x1E:
                        response.data[4] = (OD.torque_limit >> 8) & 0xFF;
                        response.data[5] = OD.torque_limit & 0xFF;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2003:
                switch(subindex) {
                    case 0x26:
                        response.data[4] = (OD.max_speed >> 8) & 0xFF;
                        response.data[5] = OD.max_speed & 0xFF;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2002:
                switch(subindex) {
                    case 0x1D:
                        response.data[4] = OD.regen_level;
                        break;
                    default: 
                        abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2005:
                switch(subindex) {
                    case 0x14:
                        response.data[4] = OD.assist_level;
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
            case 0x2007:
                switch(subindex) {
                    case 0x1A: OD.target_speed = data; break;
                    case 0x1D: OD.motor_enable = data; break;
                    case 0x1E: OD.torque_limit = data; break;
                    default: abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2003:
                switch(subindex) {
                    case 0x26: OD.max_speed = data; break;
                    default: abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2002:
                switch(subindex) {
                    case 0x1D: OD.regen_level = data; break;
                    default: abort_response(&response, 0x11, 0x09, 0x06);
                }
                break;
                
            case 0x2005:
                switch(subindex) {
                    case 0x14: OD.assist_level = data; break;
                    default: abort_response(&response, 0x11, 0x09, 0x06);
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
}

void check_errors() {
    uint8_t new_error_register = 0;
    if (OD.battery_voltage < 300) new_error_register |= 0x01;
    OD.error_register = new_error_register;
}

void canopen_process() {
    uint32_t now = get_current_time_ms();
    
    if (now - last_hb >= OD.heartbeat_producer) {
        send_heartbeat();
        last_hb = now;
    }
    
    if (current_state == OPERATIONAL && last_sync > 0) {
        if (now - last_rpdo1 >= OD.rpdo1_comm_params[2]) {
            generate_random_rpdo1_data();
            send_rpdo1();
        }
        if (now - last_tpdo1 >= OD.tpdo1_comm_params[2]) {
            generate_random_tpdo_data();
            send_tpdo1();
        }
        if (now - last_tpdo2 >= OD.tpdo2_comm_params[2]) {
            generate_random_tpdo_data();
            send_tpdo2();
        }
    }
    
    check_errors();
}

void handle_can_message(CANMessage *msg) {
    if (msg->id == 0x081) return;

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
    system("sudo killall candump 2>/dev/null");
    system("sudo ip link set dev vcan0 down");
    system("sudo ip link set dev vcan0 up");

    srand(time(NULL));
    
    can_init("vcan0");
    send_bootup_message();
    
    while (true) {
        canopen_process();
        
        struct can_frame frame;
        fd_set readSet;
        struct timeval timeout = {0, 10000};
        
        FD_ZERO(&readSet);
        FD_SET(can_socket, &readSet);
        
        if (select(can_socket + 1, &readSet, NULL, NULL, &timeout) > 0) {
            if (FD_ISSET(can_socket, &readSet)) {
                int nbytes = read(can_socket, &frame, sizeof(frame));
                
                if (nbytes > 0) {
                    CANMessage msg;
                    msg.id = frame.can_id;
                    msg.len = frame.can_dlc;
                    memcpy(msg.data, frame.data, frame.can_dlc);
                    handle_can_message(&msg);
                }
            }
        }
    }
    
    close(can_socket);
    return 0;
}
