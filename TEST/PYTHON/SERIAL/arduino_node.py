import serial
import struct
import threading
import time
from dataclasses import dataclass

# --- FIXED CONFIGURATIONS ---
# Struct format: 1 uint32 (I), 2 int32 (i), 2 int16 (h), 2 uint16 (H), 1 uint8 (B), 10 int16 (h)
TELEMETRY_FMT = '<IiihhHHBhhhhhhhhhh'   
TELEMETRY_PAYLOAD_SIZE = 41 
TELEMETRY_PACKET_SIZE = 43  
COMMAND_FMT = '<hhB'       

@dataclass
class ArduinoData:
    timestamp: int = 0
    theta1_actual: float = 0.0
    theta2_actual: float = 0.0
    w1_actual: float = 0.0
    w2_actual: float = 0.0
    ir_distance_1: int = 0
    ir_distance_2: int = 0
    servo_angle: float = 0.0
    
    # IMU Data
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    
    connected: bool = False

class ArduinoNode:
    """Manages isolated serial communication for a single Arduino board."""
    def __init__(self, port, baudrate=115200, name="Arduino"):
        self.port = port
        self.baud = baudrate
        self.name = name
        self.ser = None
        self.data = ArduinoData()
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.last_packet_time = 0

    def start(self):
        if not self.port:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"[{self.name}] Thread successfully started on {self.port}")
        except serial.SerialException as e:
            print(f"[{self.name}] Connection failed on {self.port}: {e}")

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print(f"[{self.name}] Serial port closed.")

    def send_command(self, w1_target, w2_target, servo_target):
        if not self.ser or not self.ser.is_open:
            return
        w1_int = int(w1_target * 1000)
        w2_int = int(w2_target * 1000)
        servo_int = max(0, min(180, int(servo_target)))
        try:
            payload = struct.pack(COMMAND_FMT, w1_int, w2_int, servo_int)
            checksum = 0
            for b in payload:
                checksum ^= b
            packet = b'\x5A' + payload + bytes([checksum])
            self.ser.write(packet)
        except Exception as e:
            print(f"[{self.name}] Write Error: {e}")

    def get_data(self):
        with self.lock:
            # Active Heartbeat: consider connected only if we got a valid packet in last 1.5s
            is_alive = (time.time() - self.last_packet_time) < 1.5
            
            # Return a fresh copy of the expanded dataclass
            return ArduinoData(
                self.data.timestamp, 
                self.data.theta1_actual, self.data.theta2_actual,
                self.data.w1_actual, self.data.w2_actual,
                self.data.ir_distance_1, self.data.ir_distance_2, self.data.servo_angle,
                self.data.qx, self.data.qy, self.data.qz, self.data.qw,
                self.data.gx, self.data.gy, self.data.gz,
                self.data.ax, self.data.ay, self.data.az,
                is_alive
            )

    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting >= TELEMETRY_PACKET_SIZE:
                    if self.ser.read(1) != b'\xA5':
                        continue 
                    buffer = self.ser.read(TELEMETRY_PAYLOAD_SIZE + 1)
                    if len(buffer) != TELEMETRY_PAYLOAD_SIZE + 1:
                        continue
                    payload = buffer[:-1]
                    received_checksum = buffer[-1]
                    calc_cs = 0
                    for b in payload:
                        calc_cs ^= b
                        
                    if calc_cs == received_checksum:
                        unpacked = struct.unpack(TELEMETRY_FMT, payload)
                        with self.lock:
                            # Base data
                            self.data.timestamp = unpacked[0]
                            self.data.theta1_actual = unpacked[1] / 1000.0
                            self.data.theta2_actual = unpacked[2] / 1000.0
                            self.data.w1_actual = unpacked[3] / 1000.0
                            self.data.w2_actual = unpacked[4] / 1000.0
                            
                            self.data.ir_distance_1 = unpacked[5]
                            self.data.ir_distance_2 = unpacked[6]
                            self.data.servo_angle = float(unpacked[7])
                            
                            # IMU Quaternions (scaled by 10000)
                            self.data.qx = unpacked[8] / 10000.0
                            self.data.qy = unpacked[9] / 10000.0
                            self.data.qz = unpacked[10] / 10000.0
                            self.data.qw = unpacked[11] / 10000.0
                            
                            # IMU Gyro & Accel (scaled by 1000)
                            self.data.gx = unpacked[12] / 1000.0
                            self.data.gy = unpacked[13] / 1000.0
                            self.data.gz = unpacked[14] / 1000.0
                            self.data.ax = unpacked[15] / 1000.0
                            self.data.ay = unpacked[16] / 1000.0
                            self.data.az = unpacked[17] / 1000.0

                            self.last_packet_time = time.time()
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"[{self.name}] Read Error: {e}")
                time.sleep(0.01)