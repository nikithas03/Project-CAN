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

const canid_t PDU_COMMAND_ID = 0xC77E00F1;
const canid_t PDU_RESPONSE_ID = 0x18EFF1600;
const canid_t PDU_ALERT_ID = 0x18FFF1600;

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

enum RWFlag {
    READ = 0,
    WRITE = 1,
    MACRO = 2
};

class PduCommunicator {
private:
    int canSocket;
    std::atomic<bool> running;
    std::thread receiveThread;
    std::mutex responseMutex;
    std::condition_variable responseCV;
    std::map<OpCode, struct can_frame> responses;
    std::map<OpCode, std::vector<std::function<void(const struct can_frame&)>>> alertCallbacks;
    uint8_t currentAddress = 0;

    std::string getStatusDescription(uint8_t status) {
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

    uint8_t extractStatus(uint8_t statusByte) {
        return (statusByte >> 2);
    }

    void receiveLoop() {
        struct can_frame frame;
        while (running) {
            int nbytes = read(canSocket, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("Error reading from CAN socket");
                continue;
            }
            if (frame.can_id == PDU_RESPONSE_ID) {
                uint8_t opCode = frame.data[0];
                {
                    std::lock_guard<std::mutex> lock(responseMutex);
                    responses[static_cast<OpCode>(opCode)] = frame;
                }
                responseCV.notify_all();
                std::cout << "Response received - OpCode: 0x" << std::hex << static_cast<int>(opCode) 
                          << ", Status: " << getStatusDescription(extractStatus(frame.data[1])) << std::dec << std::endl;
            } else if (frame.can_id == PDU_ALERT_ID) {
                std::cout << "Alert received - Data: ";
                for (int i = 0; i < frame.can_dlc; i++) {
                    std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
                }
                std::cout << std::dec << std::endl;
                uint8_t opCode = frame.data[0];
                if (alertCallbacks.find(static_cast<OpCode>(opCode)) != alertCallbacks.end()) {
                    for (auto& callback : alertCallbacks[static_cast<OpCode>(opCode)]) {
                        callback(frame);
                    }
                }
            }
        }
    }

    void registerDefaultAlertCallbacks() {
        registerAlertCallback(static_cast<OpCode>(0x50), [this](const struct can_frame& frame) {
            uint8_t channelId = frame.data[2];
            std::cout << "Trip Alert received for channel " << static_cast<int>(channelId) << std::endl;

            // Request status to confirm the trip
            bool isOn, isTripped, isBattle;
            uint8_t groupId;
            if (getChannelStatus2(channelId, &isOn, &isTripped, &isBattle, &groupId)) {
                std::cout << "Confirmed Channel " << static_cast<int>(channelId) << " status:" 
                          << " On=" << isOn << ", Tripped=" << isTripped << ", Battle=" << isBattle 
                          << ", Group=" << (groupId == 251 ? "None" : std::to_string(groupId - 100)) << std::endl;

                // Reset the channel if tripped
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

public:
    PduCommunicator(const std::string& interface = "vcan0") : running(false) {
        if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Error opening CAN socket");
            throw std::runtime_error("Failed to open CAN socket");
        }
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface.c_str());
        ioctl(canSocket, SIOCGIFINDEX, &ifr);
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error binding CAN socket");
            close(canSocket);
            throw std::runtime_error("Failed to bind CAN socket");
        }
        running = true;
        receiveThread = std::thread(&PduCommunicator::receiveLoop, this);
        registerDefaultAlertCallbacks();
    }

    ~PduCommunicator() {
        running = false;
        if (receiveThread.joinable()) {
            receiveThread.join();
        }
        close(canSocket);
    }

    void setAddress(uint8_t addr) {
        if (addr > 7) {
            std::cerr << "Warning: Address must be between 0-7, using 0" << std::endl;
            currentAddress = 0;
        } else {
            currentAddress = addr;
        }
    }

    uint8_t getAddress() const {
        return currentAddress;
    }

    bool sendCommand(const struct can_frame& frame, OpCode responseOpCode = static_cast<OpCode>(0), 
                    struct can_frame* response = nullptr, int timeoutMs = 1000) {
        if (write(canSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Error writing to CAN socket");
            return false;
        }
        if (responseOpCode != static_cast<OpCode>(0) && response != nullptr) {
            std::unique_lock<std::mutex> lock(responseMutex);
            responses.clear();  // Clear old responses to avoid confusion
            bool received = responseCV.wait_for(lock, std::chrono::milliseconds(timeoutMs), 
                [this, responseOpCode]() { return responses.find(responseOpCode) != responses.end(); });
            if (received) {
                *response = responses[responseOpCode];
                responses.erase(responseOpCode);
                return true;
            }
            return false;
        }
        return true;
    }

    void registerAlertCallback(OpCode alertOpCode, std::function<void(const struct can_frame&)> callback) {
        alertCallbacks[alertOpCode].push_back(callback);
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
        bool result = sendCommand(frame, CHANNEL_CONTROL_1_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, CHANNEL_CONTROL_2_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, RESET_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, CHANNEL_STATUS_1_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, CHANNEL_STATUS_2_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, INPUT_STATUS_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, TEMPERATURE_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status != SUCCESS) {
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
        bool result = sendCommand(frame, GROUPED_CHANNELS_RSP, &response, 1000);
        if (result) {
            uint8_t status = extractStatus(response.data[1]);
            if (status == GROUP_DOESNT_EXIST) {
                std::cerr << "Group " << static_cast<int>(groupId) << " doesn't exist" << std::endl;
                return false;
            }
            if (status != SUCCESS) {
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
};

void displayHelp() {
    std::cout << "PDU Communication Utility" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  on <channel/group> [0-149]     Turn channel or group ON" << std::endl;
    std::cout << "  off <channel/group> [0-149]    Turn channel or group OFF" << std::endl;
    std::cout << "  battle <channel/group> [0-149] Enable battle mode for channel or group" << std::endl;
    std::cout << "  normal <channel/group> [0-149] Disable battle mode for channel or group" << std::endl;
    std::cout << "  status1 <channel/group> [0-149] Get current and voltage of channel or group" << std::endl;
    std::cout << "  status2 <channel/group> [0-149] Get state info of channel or group" << std::endl;
    std::cout << "  input                          Get input voltage and current" << std::endl;
    std::cout << "  temp                           Get board temperature" << std::endl;
    std::cout << "  group <groupId> [0-49]         Get channels in group" << std::endl;
    std::cout << "  reset                          Reset the PDU" << std::endl;
    std::cout << "  addr <address> [0-7]           Set PDU address" << std::endl;
    std::cout << "  help                           Display this help" << std::endl;
    std::cout << "  exit                           Exit the program" << std::endl;
}

int main() {
    try {
        std::cout << "Initializing PDU Communicator on vcan0..." << std::endl;
        PduCommunicator pdu("vcan0");
        std::cout << "PDU Communicator initialized successfully." << std::endl;
        displayHelp();
        std::string command;
        while (true) {
            std::cout << "> ";
            std::getline(std::cin, command);
            std::vector<std::string> parts;
            std::size_t start = 0, end = 0;
            while ((end = command.find(' ', start)) != std::string::npos) {
                parts.push_back(command.substr(start, end - start));
                start = end + 1;
            }
            parts.push_back(command.substr(start));
            if (parts.empty()) continue;
            if (parts[0] == "exit") {
                break;
            } else if (parts[0] == "help") {
                displayHelp();
            } else if (parts[0] == "addr") {
                if (parts.size() < 2) {
                    std::cout << "Current address: " << static_cast<int>(pdu.getAddress()) << std::endl;
                } else {
                    try {
                        uint8_t addr = std::stoi(parts[1]);
                        pdu.setAddress(addr);
                        std::cout << "Address set to " << static_cast<int>(addr) << std::endl;
                    } catch (...) {
                        std::cerr << "Invalid address" << std::endl;
                    }
                }
            } else if (parts[0] == "on" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    if (pdu.channelControl1(element, true)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " turned ON successfully" << std::endl;
                    } else {
                        std::cerr << "Failed to turn ON channel/group " << static_cast<int>(element) << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID" << std::endl;
                }
            } else if (parts[0] == "off" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    if (pdu.channelControl1(element, false)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " turned OFF successfully" << std::endl;
                    } else {
                        std::cerr << "Failed to turn OFF channel/group " << static_cast<int>(element) << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID" << std::endl;
                }
            } else if (parts[0] == "battle" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    if (pdu.channelControl2(element, true)) {
                        std::cout << "Battle mode ENABLED for channel/group " << static_cast<int>(element) << std::endl;
                    } else {
                        std::cerr << "Failed to enable battle mode for channel/group " << static_cast<int>(element) << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID" << std::endl;
                }
            } else if (parts[0] == "normal" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    if (pdu.channelControl2(element, false)) {
                        std::cout << "Battle mode DISABLED for channel/group " << static_cast<int>(element) << std::endl;
                    } else {
                        std::cerr << "Failed to disable battle mode for channel/group " << static_cast<int>(element) << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID" << std::endl;
                }
            } else if (parts[0] == "status1" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    float current, voltage;
                    if (pdu.getChannelStatus1(element, &current, &voltage)) {
                        if (element < 100) {
                            std::cout << "Channel " << static_cast<int>(element) << " status:" << std::endl;
                        } else {
                            std::cout << "Group " << static_cast<int>(element - 100) << " status:" << std::endl;
                        }
                        bool isOn = (current > 0.0f || voltage > 0.0f);
                        if (isOn) {
                            std::cout << "  Current: " << current << " A" << std::endl;
                            std::cout << "  Voltage: " << voltage << " V" << std::endl;
                        } else {
                            std::cout << "  State: OFF" << std::endl;
                        }
                    } else {
                        std::cerr << "Failed to get status" << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid element ID" << std::endl;
                }
            } else if (parts[0] == "status2" && parts.size() > 1) {
                try {
                    uint8_t element = std::stoi(parts[1]);
                    bool isOn, isTripped, isBattle;
                    uint8_t groupId;
                    if (pdu.getChannelStatus2(element, &isOn, &isTripped, &isBattle, &groupId)) {
                        std::cout << "Channel/group " << static_cast<int>(element) << " state:" << std::endl;
                        std::cout << "  Power state: " << (isOn ? "ON" : "OFF") << std::endl;
                        std::cout << "  Trip state: " << (isTripped ? "TRIPPED" : "NORMAL") << std::endl;
                        std::cout << "  Battle mode: " << (isBattle ? "ENABLED" : "DISABLED") << std::endl;
                        std::cout << "  Group ID: " << (groupId == 251 ? "NONE" : std::to_string(groupId - 100)) << std::endl;
                    } else {
                        std::cerr << "Failed to get state for channel/group " << static_cast<int>(element) << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid channel/group ID" << std::endl;
                }
            } else if (parts[0] == "input") {
                float current, voltage;
                if (pdu.getInputStatus(&current, &voltage)) {
                    std::cout << "Input status:" << std::endl;
                    std::cout << "  Current: " << current << " A" << std::endl;
                    std::cout << "  Voltage: " << voltage << " V" << std::endl;
                } else {
                    std::cerr << "Failed to get input status" << std::endl;
                }
            } else if (parts[0] == "temp") {
                float temperature;
                if (pdu.getTemperature(&temperature)) {
                    std::cout << "Board temperature: " << temperature << " °C" << std::endl;
                } else {
                    std::cerr << "Failed to get temperature" << std::endl;
                }
            } else if (parts[0] == "group" && parts.size() > 1) {
                try {
                    uint8_t groupId = std::stoi(parts[1]);
                    if (groupId > 49) {
                        std::cerr << "Group ID must be between 0 and 49" << std::endl;
                        continue;
                    }
                    std::vector<uint8_t> channelIds;
                    if (pdu.getGroupedChannels(groupId, &channelIds)) {
                        std::cout << "Group " << static_cast<int>(groupId) << " contains channels: ";
                        for (size_t i = 0; i < channelIds.size(); i++) {
                            std::cout << static_cast<int>(channelIds[i]);
                            if (i < channelIds.size() - 1) {
                                std::cout << ", ";
                            }
                        }
                        std::cout << std::endl;
                    } else {
                        std::cerr << "Failed to get group information" << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Invalid group ID" << std::endl;
                }
            } else if (parts[0] == "reset") {
                if (pdu.resetDevice()) {
                    std::cout << "PDU reset successfully" << std::endl;
                } else {
                    std::cerr << "Failed to reset PDU" << std::endl;
                }
            } else {
                std::cerr << "Unknown command. Type 'help' for available commands." << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Program terminated." << std::endl;
    return 0;
}