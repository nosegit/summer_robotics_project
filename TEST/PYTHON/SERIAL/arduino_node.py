import serial
import struct
import threading
import time
from dataclasses import dataclass

# --- FIXED CONFIGURATIONS ---
TELEMETRY_FMT = '<IhhHB'   
TELEMETRY_PAYLOAD_SIZE = 11 
TELEMETRY_PACKET_SIZE = 13  
COMMAND_FMT = '<hhB'       

@dataclass
class ArduinoData:
    timestamp: int = 0
    w1_actual: float = 0.0
    w2_actual: float = 0.0
    ir_distance: int = 0
    servo_angle: float = 0.0
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
        self.last_packet_time = 0  # Tracks last valid telemetry packet timestamp

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
            return ArduinoData(
                self.data.timestamp, self.data.w1_actual, self.data.w2_actual,
                self.data.ir_distance, self.data.servo_angle, is_alive
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
                            self.data.timestamp = unpacked[0]
                            self.data.w1_actual = unpacked[1] / 1000.0
                            self.data.w2_actual = unpacked[2] / 1000.0
                            self.data.ir_distance = unpacked[3]
                            self.data.servo_angle = float(unpacked[4])
                            self.last_packet_time = time.time()  # Feed the heartbeat
                else:
                    # CRITICAL FIX: Relinquish control to the GIL so Flask can run
                    time.sleep(0.002)
            except Exception as e:
                if self.running:
                    print(f"[{self.name}] Read Error: {e}")
                time.sleep(0.01)