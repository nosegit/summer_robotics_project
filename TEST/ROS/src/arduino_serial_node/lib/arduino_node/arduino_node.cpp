#include "arduino_node/arduino_node.hpp"

ArduinoNode::ArduinoNode(const std::string& port, int baudrate, const std::string& name)
    : port_(port), baud_(baudrate), name_(name) {
    last_packet_time_ = std::chrono::steady_clock::now();
}

ArduinoNode::~ArduinoNode() {
    stop();
}

void ArduinoNode::start() {
    if (port_.empty()) return;

    try {
        serial_.Open(port_);
        
        // Match Python's 115200 baud setting
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);

        running_ = true;
        thread_ = std::thread(&ArduinoNode::readLoop, this);
        std::cout << "[" << name_ << "] Thread successfully started on " << port_ << std::endl;
        
    } catch (const LibSerial::OpenFailed& e) {
        std::cerr << "[" << name_ << "] Connection failed on " << port_ << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[" << name_ << "] Error: " << e.what() << std::endl;
    }
}

void ArduinoNode::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (serial_.IsOpen()) {
        serial_.Close();
    }
    std::cout << "[" << name_ << "] Serial port closed." << std::endl;
}

void ArduinoNode::sendCommand(float w1_target, float w2_target, float servo_target) {
    if (!serial_.IsOpen()) return;

    // Scale values matching Python's approach
    CommandPayload payload;
    payload.w1_target = static_cast<int16_t>(w1_target * 1000.0f);
    payload.w2_target = static_cast<int16_t>(w2_target * 1000.0f);
    
    // Constrain servo target
    servo_target = std::max(0.0f, std::min(180.0f, servo_target));
    payload.servo_target = static_cast<uint8_t>(servo_target);

    // Compute Checksum
    uint8_t checksum = 0;
    uint8_t* payload_bytes = reinterpret_cast<uint8_t*>(&payload);
    for (size_t i = 0; i < sizeof(CommandPayload); i++) {
        checksum ^= payload_bytes[i];
    }

    // Construct full packet
    LibSerial::DataBuffer packet;
    packet.push_back(COMMAND_HEADER);
    for (size_t i = 0; i < sizeof(CommandPayload); i++) {
        packet.push_back(payload_bytes[i]);
    }
    packet.push_back(checksum);

    try {
        serial_.Write(packet);
        // serial_.DrainWriteBuffer();
    } catch (const std::exception& e) {
        std::cerr << "[" << name_ << "] Write Error: " << e.what() << std::endl;
    }
}

ArduinoData ArduinoNode::getData() {
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Active Heartbeat: considered connected if packet received within 1.5s
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_packet_time_;
    data_.connected = (elapsed.count() < 1.5);

    return data_;
}

void ArduinoNode::readLoop() {
    // Persistent buffer to handle fragmented packets over USB
    std::vector<uint8_t> rx_buffer;
    
    while (running_) {
        try {
            size_t available = serial_.GetNumberOfBytesAvailable();
            
            if (serial_.IsOpen() && available > 0) {
                LibSerial::DataBuffer chunk;
                // Read whatever bytes are instantly available right now
                serial_.Read(chunk, available, 10);
                rx_buffer.insert(rx_buffer.end(), chunk.begin(), chunk.end());
                
                // Process all complete packets trapped in our buffer
                while (rx_buffer.size() >= TELEMETRY_PACKET_SIZE) {
                    
                    // 1. Scan for the Header Byte
                    auto it = std::find(rx_buffer.begin(), rx_buffer.end(), TELEMETRY_HEADER);
                    
                    if (it != rx_buffer.begin()) {
                        if (it == rx_buffer.end()) {
                            rx_buffer.clear(); // No header found, clear garbage
                            break;
                        }
                        // Erase garbage bytes before the header
                        rx_buffer.erase(rx_buffer.begin(), it);
                    }
                    
                    // Check if we still have a full packet after trimming garbage
                    if (rx_buffer.size() < TELEMETRY_PACKET_SIZE) {
                        break; 
                    }
                    
                    // 2. Validate Checksum
                    uint8_t received_cs = rx_buffer[TELEMETRY_PACKET_SIZE - 1];
                    uint8_t calc_cs = 0;
                    
                    for (size_t i = 1; i <= TELEMETRY_PAYLOAD_SIZE; i++) {
                        calc_cs ^= rx_buffer[i];
                    }
                    
                    // 3. Decode payload if checksum matches
                    if (calc_cs == received_cs) {
                        TelemetryPayload payload;
                        // Copy data skipping the 1-byte header
                        std::memcpy(&payload, rx_buffer.data() + 1, TELEMETRY_PAYLOAD_SIZE);
                        
                        {
                            std::lock_guard<std::mutex> lock(mtx_);
                            data_.timestamp = payload.timestamp;
                            data_.theta1_actual = payload.theta1_act / 1000.0f;
                            data_.theta2_actual = payload.theta2_act / 1000.0f;
                            data_.w1_actual = payload.w1_actual / 1000.0f;
                            data_.w2_actual = payload.w2_actual / 1000.0f;
                            data_.ir_distance_1 = payload.ir_distance_1;
                            data_.ir_distance_2 = payload.ir_distance_2;
                            data_.servo_angle = static_cast<float>(payload.servo_angle);
                            
                            data_.qx = payload.qx / 10000.0f;
                            data_.qy = payload.qy / 10000.0f;
                            data_.qz = payload.qz / 10000.0f;
                            data_.qw = payload.qw / 10000.0f;
                            data_.gx = payload.gx / 1000.0f;
                            data_.gy = payload.gy / 1000.0f;
                            data_.gz = payload.gz / 1000.0f;
                            data_.ax = payload.ax / 1000.0f;
                            data_.ay = payload.ay / 1000.0f;
                            data_.az = payload.az / 1000.0f;
                            
                            last_packet_time_ = std::chrono::steady_clock::now();
                        }
                        
                        // Erase the processed packet from the buffer
                        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + TELEMETRY_PACKET_SIZE);
                    } else {
                        // Checksum failed (likely a false header). Drop the fake header byte and scan again.
                        rx_buffer.erase(rx_buffer.begin());
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            if (running_) {
                std::cerr << "[" << name_ << "] Read Error: " << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}