from flask import Flask, render_template
from flask_socketio import SocketIO
from arduino_node import ArduinoNode
import serial.tools.list_ports  # Added to scan for COM ports
import time

app = Flask(__name__)
# Force threading mode for stable background tasks
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Initialize nodes with NO default ports. The UI will assign them now!
nodes = {
    "arduino1": ArduinoNode(port=None, name="Arduino 1"),
    "arduino2": ArduinoNode(port=None, name="Arduino 2") 
}

def telemetry_broadcast_loop():
    """Background task reading hardware states and pushing to the Web UI at 20Hz."""
    print("[Web Server] Background broadcast thread started.")
    while True:
        telemetry_packet = {}
        for key, node in nodes.items():
            # Fix: Check port first, THEN pull data, THEN check if connected
            if node and node.port:
                d = node.get_data() 
                
                if d.connected:
                    telemetry_packet[key] = {
                        "connected": True,
                        "timestamp": d.timestamp,
                        "w1_actual": round(d.w1_actual, 2),
                        "w2_actual": round(d.w2_actual, 2),
                        "ir_distance": d.ir_distance,
                        "servo_angle": round(d.servo_angle, 2)
                    }
                else:
                    telemetry_packet[key] = {"connected": False}
            else:
                telemetry_packet[key] = {"connected": False}
        
        socketio.emit('telemetry_update', telemetry_packet)
        socketio.sleep(0.05)
        
@app.route('/')
def index():
    return render_template('index.html')

# --- NEW: Dynamic Port Management ---

@socketio.on('request_ports')
def handle_request_ports():
    """Scans the host computer for available serial ports and sends them to the UI."""
    ports = [port.device for port in serial.tools.list_ports.comports()]
    socketio.emit('ports_list', ports)

@socketio.on('connect_node')
def handle_connect_node(data):
    """Dynamically assigns a port and starts the serial thread."""
    target_node = data.get('node')
    port = data.get('port')
    
    if target_node in nodes and port:
        print(f"[Web Server] Connecting {target_node} to {port}...")
        nodes[target_node].stop() # Stop any existing connection safely
        nodes[target_node].port = port
        nodes[target_node].start()

@socketio.on('disconnect_node')
def handle_disconnect_node(data):
    """Safely shuts down a serial thread and frees the port."""
    target_node = data.get('node')
    if target_node in nodes:
        print(f"[Web Server] Disconnecting {target_node}...")
        nodes[target_node].stop()
        nodes[target_node].port = None

# ------------------------------------

@socketio.on('ui_command')
def handle_ui_command(data):
    """Listens for command events sent from the Web UI sliders/inputs."""
    target_node = data.get('node')
    if target_node in nodes and nodes[target_node]:
        nodes[target_node].send_command(
            w1_target=float(data['w1']),
            w2_target=float(data['w2']),
            servo_target=float(data['servo'])
        )

if __name__ == '__main__':
    # Start background loop (Arduinos will start dynamically from the UI)
    socketio.start_background_task(telemetry_broadcast_loop)
    print("[Web Server] Launching web controller on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)