import time
from arduino_node import ArduinoNode

# --- CONFIGURATION ---
# Change this to match your actual port
# SERIAL_PORT = '/dev/ttyACM0' 
SERIAL_PORT = 'COM5' 
BAUDRATE = 115200

def main():
    print(f"Starting connection to Arduino on {SERIAL_PORT}...")
    
    # Initialize the standalone Arduino node
    node = ArduinoNode(port=SERIAL_PORT, baudrate=BAUDRATE, name="TelemetryReader")
    
    # Start the background serial reading thread
    node.start()
    
    # Give the serial port a second to complete the handshake
    time.sleep(1.0)
    
    print("\nListening for incoming telemetry packets... Press Ctrl+C to stop.\n")

    node.send_command(w1_target=.0, w2_target=.0, servo_target=90.0)  # Example command to set initial positions
    
    try:
        while True:
            # Safely extract the latest decoded snapshot from the node
            data = node.get_data()
            
            if data.connected:
                # Clear terminal and print block format to accommodate the large amount of data
                print("\033[H\033[J", end="")  # ANSI escape to clear screen and move cursor home
                
                print(f"=== TELEMETRY SNAPSHOT | Time: {data.timestamp} ms ===")
                print(f"MOTORS: W1: {data.w1_actual:+.3f} rad/s  | W2: {data.w2_actual:+.3f} rad/s")
                print(f"SENSOR: IR1: {data.ir_distance_1} mm | IR2: {data.ir_distance_2} mm | Servo: {data.servo_angle:.1f}°")
                print("-" * 50)
                print(f"IMU QUAT:  Qx: {data.qx:+.4f} | Qy: {data.qy:+.4f} | Qz: {data.qz:+.4f} | Qw: {data.qw:+.4f}")
                print(f"IMU GYRO:  Gx: {data.gx:+.3f} | Gy: {data.gy:+.3f} | Gz: {data.gz:+.3f} rad/s")
                print(f"IMU ACCEL: Ax: {data.ax:+.3f} | Ay: {data.ay:+.3f} | Az: {data.az:+.3f} m/s^2")
                print("==================================================")
            else:
                # If serial port drops or fails to open
                print("Connecting/Waiting for data packets...", end="\r")
                
            # Control the terminal print rate (10Hz / every 100ms) to prevent flooding
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopping telemetry reader...")
    finally:
        # Crucial: Always stop the thread and release the hardware serial port
        node.stop()
        print("Serial port successfully closed.")

if __name__ == '__main__':
    main()