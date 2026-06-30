#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <mbed.h>
#include <rtos.h>
#include "arduinostate.h" 

// --- NEW DISTRIBUTED PROTOCOL DEFINITIONS ---

// 1. OUTGOING: Telemetry (Arduino -> Raspberry Pi)
// Size: 13 Bytes (Start + Timestamp + 3*2 + 1 + Checksum)
struct __attribute__((packed)) ArduinoTelemetryPacket {
    uint8_t startByte = 0xA5;    // Sync Header
    uint32_t timestamp;          // 4 bytes (millis)
    int16_t w1_act;              // 2 bytes (Actual Motor 1 Speed: rad/s * 1000)
    int16_t w2_act;              // 2 bytes (Actual Motor 2 Speed: rad/s * 1000)
    uint16_t ir_dist;            // 2 bytes (IR Distance: raw value or mm)
    uint8_t servo_ang;           // 1 bytes (Current Servo Angle: degree)
    uint8_t checksum;            // 1 byte (XOR payload check)
};

// 2. INCOMING: Commands (Raspberry Pi -> Arduino)
// Size: 8 Bytes (Start + 3*2 + Checksum)
struct __attribute__((packed)) ArduinoCommandPacket {
    uint8_t startByte = 0x5A;    // Sync Header
    int16_t w1_targ;             // 2 bytes (Target Motor 1 Speed: rad/s * 1000)
    int16_t w2_targ;             // 2 bytes (Target Motor 2 Speed: rad/s * 1000)
    uint8_t servo_targ;          // 2 bytes (Target Servo Angle: radians * 1000)
    uint8_t checksum;            // 1 byte (XOR payload check)
};

class COMMUNICATION {
private:
    rtos::Thread thread;
    ARDUINO_STATE* _state;       
    Stream* _serialPort;
    bool _running;

    // Parsing State Machine
    enum ParseState {
        WAIT_SYNC,
        READ_DATA,
        CHECK_CS
    };
    ParseState _pState = WAIT_SYNC;
    uint8_t _rxBuffer[sizeof(ArduinoCommandPacket)];
    uint8_t _rxIndex = 0;

    void loop() {
        while (true) {
            if (_serialPort != nullptr && _state != nullptr) {
                // 1. Handle Incoming Data from Raspberry Pi (Non-blocking)
                while (_serialPort->available() > 0) {
                    processIncomingByte(_serialPort->read());
                }

                // 2. Send Local Telemetry up to Raspberry Pi at ~30Hz
                sendTelemetry();
            }
            
            rtos::ThisThread::sleep_for(33); 
        }
    }

    void processIncomingByte(uint8_t byte) {
        switch (_pState) {
            case WAIT_SYNC:
                if (byte == 0x5A) { 
                    _rxIndex = 0;
                    _rxBuffer[_rxIndex++] = byte;
                    _pState = READ_DATA;
                }
                break;

            case READ_DATA:
                _rxBuffer[_rxIndex++] = byte;
                if (_rxIndex >= sizeof(ArduinoCommandPacket)) {
                    _pState = CHECK_CS;
                    
                    if (validatePacket()) {
                        applyCommand();
                    }
                    _pState = WAIT_SYNC;
                }
                break;
                
            case CHECK_CS:
                 _pState = WAIT_SYNC;
                 break;
        }
    }

    bool validatePacket() {
        ArduinoCommandPacket* pkt = (ArduinoCommandPacket*)_rxBuffer;
        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)pkt;
        
        for (size_t i = 1; i < sizeof(ArduinoCommandPacket) - 1; i++) {
            calc_cs ^= ptr[i];
        }
        
        return (calc_cs == pkt->checksum);
    }

    void applyCommand() {
        ArduinoCommandPacket* pkt = (ArduinoCommandPacket*)_rxBuffer;
        
        // Convert integer scaling back to floats
        float w1_target = (float)pkt->w1_targ / 1000.0f; 
        float w2_target = (float)pkt->w2_targ / 1000.0f; 
        float servo_target = (float)pkt->servo_targ; 
        
        // Write to local Arduino state layout
        _state->setTargetSpeeds(w1_target, w2_target);
        _state->setTargetServo(servo_target); 
        
    }

    void sendTelemetry() {
        LocalState data = _state->getSnapshot();
        
        ArduinoTelemetryPacket pkt;
        pkt.timestamp = millis();
        
        pkt.w1_act = (int16_t)(data.actual_w1 * 1000.0f);
        pkt.w2_act = (int16_t)(data.actual_w2 * 1000.0f);
        pkt.ir_dist = (uint16_t)data.ir_distance;
        pkt.servo_ang = (uint8_t)(data.servo_angle);

        pkt.checksum = 0;
        uint8_t* ptr = (uint8_t*)&pkt;
        for (size_t i = 1; i < sizeof(ArduinoTelemetryPacket) - 1; i++) {
            pkt.checksum ^= ptr[i];
        }

        _serialPort->write((uint8_t*)&pkt, sizeof(ArduinoTelemetryPacket));
    }

public:
    COMMUNICATION() : thread(osPriorityAboveNormal, 4096), _running(false) {}

    void begin(ARDUINO_STATE* state_ptr, Stream* serial_ptr) {
        _state = state_ptr;
        _serialPort = serial_ptr;
        _running = true;
        thread.start(mbed::callback(this, &COMMUNICATION::loop));
    }
};

#endif // COMMUNICATION_H