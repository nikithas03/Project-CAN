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
#include <thread>
#include <atomic>
#include <chrono>
#include <random>
#include <map>
#include <termios.h>  // For non-blocking keyboard input
#include <fcntl.h>

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

class PduSimulator {
private:
    int canSocket;
    std::atomic<bool> running;
    std::thread receiveThread;
    std::random_device rd;
    std::mt19937 gen;
    uint8_t deviceAddress;

    struct ChannelState {
        bool isOn;
        bool isBattle;
        bool isTripped;
        uint8_t groupId;
        float current;
        float voltage;
    };

    std::map<uint8_t, ChannelState> channels;
    std::map<uint8_t, std::vector<uint8_t>> groups;
    float inputCurrent;
    float inputVoltage;
    float boardTemperature;

    uint8_t createStatusByte(StatusCode status) {
        return static_cast<uint8_t>(status) << 2;
    }

    void initializeSimulatedData() {
        for (uint8_t i = 0; i < 8; i++) {
            channels[i] = {false, false, false, static_cast<uint8_t>(i % 2 + 100), 0.0f, 28.0f};
        }
        for (uint8_t i = 0; i < 2; i++) {
            std::vector<uint8_t> groupChannels;
            for (uint8_t j = 0; j < 4; j++) {
                groupChannels.push_back(i * 4 + j);
            }
            groups[i] = groupChannels;
        }
        inputCurrent = 0.0f;
        inputVoltage = 28.0f;
        boardTemperature = 25.0f;

        std::uniform_real_distribution<> currentDist(0.1, 25.0);
        std::uniform_real_distribution<> voltageDist(27.5, 28.5);
        for (auto& pair : channels) {
            pair.second.current = currentDist(gen);
            pair.second.voltage = voltageDist(gen);
        }
        inputCurrent = currentDist(gen) * 2;
        inputVoltage = voltageDist(gen);
    }

    void updateSimulatedData() {
        std::uniform_real_distribution<> currentNoise(-0.5, 0.5);
        std::uniform_real_distribution<> voltageNoise(-0.1, 0.1);
        std::uniform_real_distribution<> tempNoise(-0.5, 0.5);

        float totalCurrent = 0.0f;
        for (auto& pair : channels) {
            if (pair.second.isOn) {
                pair.second.current += currentNoise(gen);
                if (pair.second.current < 0.05f) pair.second.current = 0.05f;
                if (pair.second.isBattle) pair.second.current *= 1.2f;
                totalCurrent += pair.second.current;
            } else {
                pair.second.current = 0.0f;
            }
            pair.second.voltage += voltageNoise(gen);
            if (pair.second.voltage < 6.0f) pair.second.voltage = 6.0f;
            if (pair.second.voltage > 33.0f) pair.second.voltage = 33.0f;
        }
        inputCurrent = totalCurrent * 1.1f;
        inputVoltage += voltageNoise(gen);
        if (inputVoltage < 6.0f) inputVoltage = 6.0f;
        if (inputVoltage > 33.0f) inputVoltage = 33.0f;
        boardTemperature = 25.0f + (totalCurrent * 0.5f) + tempNoise(gen);
        if (boardTemperature < -55.0f) boardTemperature = -55.0f;
        if (boardTemperature > 105.0f) boardTemperature = 105.0f;
    }

    void sendTestAlert(uint8_t channelId) {
        struct can_frame alert;
        memset(&alert, 0, sizeof(alert));
        alert.can_id = PDU_ALERT_ID;
        alert.can_dlc = 8;
        alert.data[0] = 0x50;  // Alert opcode for trip
        alert.data[1] = 0x01;  // Alert type (trip)
        alert.data[2] = channelId;  // Channel to "trip"
        write(canSocket, &alert, sizeof(struct can_frame));
        std::cout << "Test Alert Sent: Channel " << (int)channelId << " tripped" << std::endl;

        // Simulate trip state for consistency
        if (channelId < 8) {
            channels[channelId].isTripped = true;
            channels[channelId].isOn = false;
        }
    }

    void processChannelControl1(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = CHANNEL_CONTROL_1_RSP;

        uint8_t element = request.data[2];
        bool turnOn = request.data[3] == 1;
        bool isMacro = (request.data[1] & 0x03) == 2;

        StatusCode status = SUCCESS;

        if (element < 100) {
            if (element < 8) {
                if (channels[element].isTripped && turnOn) {
                    status = WRONG_STATE;
                } else {
                    channels[element].isOn = turnOn;
                    if (!turnOn) channels[element].isTripped = false;
                    if (isMacro) {
                        uint8_t groupId = channels[element].groupId - 100;
                        if (groups.find(groupId) != groups.end()) {
                            for (uint8_t channelId : groups[groupId]) {
                                channels[channelId].isOn = turnOn;
                                if (!turnOn) channels[channelId].isTripped = false;
                            }
                        }
                    }
                }
            } else {
                status = CHANNEL_DOESNT_EXIST;
            }
        } else if (element >= 100 && element < 150) {
            uint8_t groupId = element - 100;
            if (groups.find(groupId) != groups.end()) {
                for (uint8_t channelId : groups[groupId]) {
                    if (channels[channelId].isTripped && turnOn) {
                        status = WRONG_STATE;
                        break;
                    }
                    channels[channelId].isOn = turnOn;
                    if (!turnOn) channels[channelId].isTripped = false;
                }
            } else {
                status = GROUP_DOESNT_EXIST;
            }
        } else {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Channel Control 1: Element=" << (int)element 
                  << ", State=" << (turnOn ? "ON" : "OFF")
                  << ", Status=" << (int)status << std::endl;
    }

    void processChannelControl2(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = CHANNEL_CONTROL_2_RSP;

        uint8_t element = request.data[2];
        bool enableBattle = request.data[3] == 0x0D;
        bool isMacro = (request.data[1] & 0x03) == 2;

        StatusCode status = SUCCESS;

        if (element < 100) {
            if (element < 8) {
                channels[element].isBattle = enableBattle;
                if (isMacro) {
                    uint8_t groupId = channels[element].groupId - 100;
                    if (groups.find(groupId) != groups.end()) {
                        for (uint8_t channelId : groups[groupId]) {
                            channels[channelId].isBattle = enableBattle;
                        }
                    }
                }
            } else {
                status = CHANNEL_DOESNT_EXIST;
            }
        } else if (element >= 100 && element < 150) {
            uint8_t groupId = element - 100;
            if (groups.find(groupId) != groups.end()) {
                for (uint8_t channelId : groups[groupId]) {
                    channels[channelId].isBattle = enableBattle;
                }
            } else {
                status = GROUP_DOESNT_EXIST;
            }
        } else {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Channel Control 2: Element=" << (int)element 
                  << ", Battle=" << (enableBattle ? "ON" : "OFF")
                  << ", Status=" << (int)status << std::endl;
    }

    void processReset(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = RESET_RSP;
        response.data[1] = createStatusByte(SUCCESS);

        for (auto& pair : channels) {
            pair.second.isOn = false;
            pair.second.isBattle = false;
            pair.second.isTripped = false;
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Reset command processed" << std::endl;
    }

    void processChannelStatus1(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = CHANNEL_STATUS_1_RSP;

        uint8_t element = request.data[2];
        StatusCode status = SUCCESS;

        float current = 0.0f;
        float voltage = 0.0f;
        bool isOn = false;

        if (element < 100) {
            if (element < 8) {
                isOn = channels[element].isOn;
                if (isOn) {
                    current = channels[element].current;
                    voltage = channels[element].voltage;
                }
            } else {
                status = CHANNEL_DOESNT_EXIST;
            }
        } else if (element >= 100 && element < 150) {
            uint8_t groupId = element - 100;
            if (groups.find(groupId) != groups.end()) {
                for (uint8_t channelId : groups[groupId]) {
                    if (channels[channelId].isOn) {
                        isOn = true;
                        current += channels[channelId].current;
                        if (voltage < channels[channelId].voltage) {
                            voltage = channels[channelId].voltage;
                        }
                    }
                }
            } else {
                status = GROUP_DOESNT_EXIST;
            }
        } else {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        if (status == SUCCESS) {
            response.data[3] = isOn ? 0x01 : 0x00;
            if (isOn) {
                uint16_t currentRaw = static_cast<uint16_t>(current / 0.001f);
                response.data[4] = (currentRaw >> 8) & 0xFF;
                response.data[5] = currentRaw & 0xFF;
                uint16_t voltageRaw = static_cast<uint16_t>((voltage + 1606.0f) / 0.05f);
                response.data[6] = (voltageRaw >> 8) & 0xFF;
                response.data[7] = voltageRaw & 0xFF;
            } else {
                memset(&response.data[4], 0, 4);
            }
        } else {
            memset(&response.data[3], 0, 5);
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Channel Status 1: Element=" << (int)element 
                  << ", On=" << isOn 
                  << (isOn ? (", Current=" + std::to_string(current) + "A, Voltage=" + std::to_string(voltage) + "V") : "")
                  << ", Status=" << (int)status << std::endl;
    }

    void processChannelStatus2(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = CHANNEL_STATUS_2_RSP;

        uint8_t element = request.data[2];
        StatusCode status = SUCCESS;

        bool isOn = false;
        bool isTripped = false;
        bool isBattle = false;
        uint8_t groupId = 251;

        if (element < 100) {
            if (element < 8) {
                isOn = channels[element].isOn;
                isTripped = channels[element].isTripped;
                isBattle = channels[element].isBattle;
                groupId = channels[element].groupId;
            } else {
                status = CHANNEL_DOESNT_EXIST;
            }
        } else if (element >= 100 && element < 150) {
            uint8_t groupIdReq = element - 100;
            if (groups.find(groupIdReq) != groups.end()) {
                bool anyOn = false, anyTripped = false, anyBattle = false;
                for (uint8_t channelId : groups[groupIdReq]) {
                    if (channels[channelId].isOn) anyOn = true;
                    if (channels[channelId].isTripped) anyTripped = true;
                    if (channels[channelId].isBattle) anyBattle = true;
                }
                isOn = anyOn;
                isTripped = anyTripped;
                isBattle = anyBattle;
                groupId = element;
            } else {
                status = GROUP_DOESNT_EXIST;
            }
        } else {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        if (status == SUCCESS) {
            uint16_t stateBits = 0;
            if (isOn) stateBits |= 0x01;
            if (isTripped) stateBits |= (0x01 << 8);
            if (isBattle) stateBits |= (0x01 << 12);

            response.data[3] = (stateBits >> 8) & 0xFF;
            response.data[4] = stateBits & 0xFF;
            response.data[5] = groupId;
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Sending CHANNEL_STATUS_2_RSP for element " << (int)element 
                  << ": On=" << isOn << ", Tripped=" << isTripped
                  << ", Battle=" << isBattle << ", Group=" << (groupId == 251 ? "None" : std::to_string(groupId - 100))
                  << ", Status=" << (int)status << std::endl;
    }

    void processInputStatus(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = INPUT_STATUS_RSP;

        StatusCode status = SUCCESS;
        uint8_t element = request.data[2];

        if (element != 190) {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        if (status == SUCCESS) {
            uint32_t currentRaw = static_cast<uint32_t>((inputCurrent + 80000.0f) / 0.01f);
            response.data[3] = (currentRaw >> 16) & 0xFF;
            response.data[4] = (currentRaw >> 8) & 0xFF;
            response.data[5] = currentRaw & 0xFF;
            uint16_t voltageRaw = static_cast<uint16_t>((inputVoltage + 1606.0f) / 0.05f);
            response.data[6] = (voltageRaw >> 8) & 0xFF;
            response.data[7] = voltageRaw & 0xFF;
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Input Status: Current=" << inputCurrent << "A, Voltage=" << inputVoltage << "V"
                  << ", Status=" << (int)status << std::endl;
    }

    void processTemperature(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = TEMPERATURE_RSP;

        StatusCode status = SUCCESS;
        uint8_t element = request.data[2];

        if (element != 152) {
            status = WRONG_ELEMENT;
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        if (status == SUCCESS) {
            uint16_t tempRaw = static_cast<uint16_t>((boardTemperature + 273.0f) / 0.03125f);
            response.data[3] = (tempRaw >> 8) & 0xFF;
            response.data[4] = tempRaw & 0xFF;
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Temperature: " << boardTemperature << "°C, Status=" << (int)status << std::endl;
    }

    void processGroupedChannels(const struct can_frame& request) {
        struct can_frame response;
        memset(&response, 0, sizeof(response));
        response.can_id = PDU_RESPONSE_ID;
        response.can_dlc = 8;
        response.data[0] = GROUPED_CHANNELS_RSP;

        uint8_t element = request.data[2];
        StatusCode status = SUCCESS;

        if (element < 100 || element >= 150) {
            status = WRONG_ELEMENT;
        } else {
            uint8_t groupId = element - 100;
            if (groups.find(groupId) == groups.end()) {
                status = GROUP_DOESNT_EXIST;
            }
        }

        response.data[1] = createStatusByte(status);
        response.data[2] = element;

        if (status == SUCCESS) {
            uint8_t groupId = element - 100;
            uint16_t membership = 0;

            for (uint8_t channelId : groups[groupId]) {
                if (channelId < 8) {
                    membership |= (0x01 << (channelId * 2));
                }
            }

            response.data[3] = (membership >> 8) & 0xFF;
            response.data[4] = membership & 0xFF;
        }

        write(canSocket, &response, sizeof(struct can_frame));
        std::cout << "Grouped Channels: Group=" << (int)(element - 100)
                  << ", Status=" << (int)status << std::endl;
    }

    void receiveLoop() {
        struct can_frame frame;
        while (running) {
            int nbytes = read(canSocket, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("Error reading from CAN socket");
                continue;
            }
            if (frame.can_id == PDU_COMMAND_ID) {
                std::cout << "Received command - OpCode: 0x" << std::hex 
                          << static_cast<int>(frame.data[0]) << std::dec << std::endl;
                switch (frame.data[0]) {
                    case CHANNEL_CONTROL_1_CMD: processChannelControl1(frame); break;
                    case CHANNEL_CONTROL_2_CMD: processChannelControl2(frame); break;
                    case RESET_CMD: processReset(frame); break;
                    case CHANNEL_STATUS_1_CMD: processChannelStatus1(frame); break;
                    case CHANNEL_STATUS_2_CMD: processChannelStatus2(frame); break;
                    case INPUT_STATUS_CMD: processInputStatus(frame); break;
                    case TEMPERATURE_CMD: processTemperature(frame); break;
                    case GROUPED_CHANNELS_CMD: processGroupedChannels(frame); break;
                    default:
                        struct can_frame errorResponse;
                        memset(&errorResponse, 0, sizeof(errorResponse));
                        errorResponse.can_id = PDU_RESPONSE_ID;
                        errorResponse.can_dlc = 8;
                        errorResponse.data[0] = frame.data[0] + 1;
                        errorResponse.data[1] = createStatusByte(OPCODE_DOESNT_EXIST);
                        write(canSocket, &errorResponse, sizeof(struct can_frame));
                        std::cout << "Unknown opcode response sent" << std::endl;
                }
                updateSimulatedData();
            }
        }
    }

public:
    PduSimulator(const std::string& interface = "vcan0") 
        : running(false), gen(rd()), deviceAddress(0) {
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
        initializeSimulatedData();
    }

    ~PduSimulator() {
        running = false;
        if (receiveThread.joinable()) {
            receiveThread.join();
        }
        close(canSocket);
    }

    void start() {
        running = true;
        receiveThread = std::thread(&PduSimulator::receiveLoop, this);
        std::cout << "PDU Simulator started" << std::endl;
    }

    void stop() {
        running = false;
        if (receiveThread.joinable()) {
            receiveThread.join();
        }
        std::cout << "PDU Simulator stopped" << std::endl;
    }

    void setAddress(uint8_t addr) {
        if (addr > 7) {
            std::cerr << "Warning: Address must be between 0-7, using 0" << std::endl;
            deviceAddress = 0;
        } else {
            deviceAddress = addr;
        }
    }

    // Expose sendTestAlert for main to call
    void triggerTestAlert(uint8_t channelId) {
        sendTestAlert(channelId);
    }
};

// Non-blocking keyboard input function
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char* argv[]) {
    std::string interface = "vcan0";
    if (argc > 1) {
        interface = argv[1];
    }
    try {
        std::cout << "Starting PDU Simulator on " << interface << "..." << std::endl;
        PduSimulator simulator(interface);
        simulator.start();
        std::cout << "PDU Simulator running. Press 'o' to send a test alert for channel 2, or Enter to exit." << std::endl;

        while (true) {
            if (kbhit()) {
                char c = getchar();
                if (c == 'o') {
                    simulator.triggerTestAlert(2);  // Send alert for channel 2 when 'o' is pressed
                } else if (c == '\n') {
                    break;  // Exit on Enter
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Avoid busy-waiting
        }

        simulator.stop();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "PDU Simulator terminated." << std::endl;
    return 0;
}