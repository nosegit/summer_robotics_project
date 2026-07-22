from flask import Flask, render_template
from flask_socketio import SocketIO
from arduino_node import ArduinoNode
import serial.tools.list_ports
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Initialize working ArduinoNode instances
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
            if node and node.port:
                d = node.get_data() 
                
                if d and getattr(d, 'connected', False):
                    telemetry_packet[key] = {
                        "connected": True,
                        "timestamp": getattr(d, 'timestamp', 0),
                        "w1_actual": round(getattr(d, 'w1_actual', 0.0), 2),
                        "w2_actual": round(getattr(d, 'w2_actual', 0.0), 2),
                        "ir_distance_1": getattr(d, 'ir_distance_1', getattr(d, 'ir_distance', 0)),
                        "ir_distance_2": getattr(d, 'ir_distance_2', 0),
                        "servo_angle": round(getattr(d, 'servo_angle', 0.0), 2),
                        # IMU Data Fields
                        "qx": getattr(d, 'qx', 0.0),
                        "qy": getattr(d, 'qy', 0.0),
                        "qz": getattr(d, 'qz', 0.0),
                        "qw": getattr(d, 'qw', 1.0),
                        "gx": getattr(d, 'gx', 0.0),
                        "gy": getattr(d, 'gy', 0.0),
                        "gz": getattr(d, 'gz', 0.0),
                        "ax": getattr(d, 'ax', 0.0),
                        "ay": getattr(d, 'ay', 0.0),
                        "az": getattr(d, 'az', 0.0)
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

@socketio.on('request_ports')
def handle_request_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    socketio.emit('ports_list', ports)

@socketio.on('connect_node')
def handle_connect_node(data):
    target_node = data.get('node')
    port = data.get('port')
    
    if target_node in nodes and port:
        print(f"[Web Server] Connecting {target_node} to {port}...")
        nodes[target_node].stop()
        nodes[target_node].port = port
        nodes[target_node].start()

@socketio.on('disconnect_node')
def handle_disconnect_node(data):
    target_node = data.get('node')
    if target_node in nodes:
        print(f"[Web Server] Disconnecting {target_node}...")
        nodes[target_node].stop()
        nodes[target_node].port = None

@socketio.on('ui_command')
def handle_ui_command(data):
    target_node = data.get('node')
    if target_node in nodes and nodes[target_node]:
        nodes[target_node].send_command(
            w1_target=float(data['w1']),
            w2_target=float(data['w2']),
            servo_target=float(data['servo'])
        )

if __name__ == '__main__':
    socketio.start_background_task(telemetry_broadcast_loop)
    print("[Web Server] Launching web controller on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)