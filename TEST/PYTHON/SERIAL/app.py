from flask import Flask, render_template
from flask_socketio import SocketIO
from arduino_node import ArduinoNode
import time

app = Flask(__name__)
# Force threading mode for stable background tasks on Windows/Local networks
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# --- CONFIGURE YOUR ARDUINOS HERE ---
# Update 'COM5' if your port changes
nodes = {
    "arduino1": ArduinoNode(port='COM5', name="Arduino 1"),
    "arduino2": ArduinoNode(port=None, name="Arduino 2") 
}

def telemetry_broadcast_loop():
    """Background task reading hardware states and pushing to the Web UI at 20Hz."""
    print("[Web Server] Background broadcast thread started.")
    while True:
        telemetry_packet = {}
        for key, node in nodes.items():
            if node and node.port:
                d = node.get_data() # Pull data safely using the thread lock
                
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
        
        # CRITICAL FIX: explicit broadcast and namespace forces the data to the browser
        socketio.emit('telemetry_update', telemetry_packet, namespace='/')
        socketio.sleep(0.05) # 50ms interval

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print("[Web Server] SUCCESS: A browser window connected!")

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
    # Start the hardware background threads
    for node in nodes.values():
        if node:
            node.start()

    # Spin up the WebSocket broadcast loop
    socketio.start_background_task(telemetry_broadcast_loop)

    print("[Web Server] Launching web controller on port 5000...")
    # Run web server on port 5000, visible to the whole local network (0.0.0.0)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)