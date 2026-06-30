import time
from arduino_node import ArduinoNode

# --- CONFIGURATION ---
# Change this to match your actual port (e.g., '/dev/ttyACM0' on Pi or 'COM3' on Windows)
# SERIAL_PORT = '/dev/ttyUSB0' 
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
    
    # Print a clean tabular header
    header = f"{'Timestamp (ms)':<15} | {'W1 Act (rad/s)':<15} | {'W2 Act (rad/s)':<15} | {'IR Dist':<10} | {'Servo Angle':<12}"
    print(header)
    print("-" * len(header))
    
    try:
        while True:
            # Safely extract the latest decoded snapshot from the node
            data = node.get_data()
            
            if data.connected:
                # Print the decoded floats/integers directly
                print(f"{data.timestamp:<15} | {data.w1_actual:<15.3f} | {data.w2_actual:<15.3f} | {data.ir_distance:<10} | {data.servo_angle:<12.1f}")
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