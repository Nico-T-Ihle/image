import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt

# Set the GPIO pins you are using according to your connections for Motor A side
IN1 = 17
IN2 = 18
ENABLE_A = 12

# Set the GPIO pins you are using according to your connections for Motor B side
IN3 = 23
IN4 = 24
ENABLE_B = 25

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENABLE_A, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENABLE_B, GPIO.OUT)

# PWM frequency (Hz) and initial speed (0% to 100%)
PWM_FREQ = 1000
INITIAL_SPEED = 100

# Create PWM objects for Motor A pins
pwm_in1 = GPIO.PWM(IN1, PWM_FREQ)
pwm_in2 = GPIO.PWM(IN2, PWM_FREQ)
pwm_enable_a = GPIO.PWM(ENABLE_A, PWM_FREQ)

# Create PWM objects for Motor B pins
pwm_in3 = GPIO.PWM(IN3, PWM_FREQ)
pwm_in4 = GPIO.PWM(IN4, PWM_FREQ)
pwm_enable_b = GPIO.PWM(ENABLE_B, PWM_FREQ)

# Function to drive Motor A forward
def motor_a_forward(speed):
    pwm_in1.start(speed)
    pwm_in2.start(0)
    pwm_enable_a.start(speed)

# Function to drive Motor A backward
def motor_a_backward(speed):
    pwm_in1.start(0)
    pwm_in2.start(speed)
    pwm_enable_a.start(speed)

# Function to stop Motor A
def motor_a_stop():
    pwm_in1.start(0)
    pwm_in2.start(0)
    pwm_enable_a.start(0)

# Function to drive Motor B forward
def motor_b_forward(speed):
    pwm_in3.start(speed)
    pwm_in4.start(0)
    pwm_enable_b.start(speed)

# Function to drive Motor B backward
def motor_b_backward(speed):
    pwm_in3.start(0)
    pwm_in4.start(speed)
    pwm_enable_b.start(speed)

# Function to stop Motor B
def motor_b_stop():
    pwm_in3.start(0)
    pwm_in4.start(0)
    pwm_enable_b.start(0)

def motor_both_backward(speed, duration_ms=2):
    pwm_in1.start(0)
    pwm_in2.start(speed)
    pwm_enable_a.start(speed)

    pwm_in3.start(0)
    pwm_in4.start(speed)
    pwm_enable_b.start(speed)

    # Run for the given time
    time.sleep(duration_ms / 1000.0)

    # Stop motors
    pwm_in1.start(0)
    pwm_in2.start(0)
    pwm_enable_a.start(0)

    pwm_in3.start(0)
    pwm_in4.start(0)
    pwm_enable_b.start(0)

def motor_both_forward(speed, duration_ms=2):
    pwm_in1.start(speed)
    pwm_in2.start(0)
    pwm_enable_a.start(speed)

    pwm_in3.start(speed)
    pwm_in4.start(0)
    pwm_enable_b.start(speed)

    # Run for the given time
    time.sleep(duration_ms / 1000.0)

    # Stop motors
    pwm_in1.start(0)
    pwm_in2.start(0)
    pwm_enable_a.start(0)

    pwm_in3.start(0)
    pwm_in4.start(0)
    pwm_enable_b.start(0)
    

# Callback function triggered when an MQTT message is received
def on_message(client, userdata, message):
    command = message.payload.decode("utf-8")
    print("Received message:", command)

    if command == "start_a":
        print("Starting Motor A...")
        motor_a_forward(INITIAL_SPEED)
    elif command == "stop_a":
        print("Stopping Motor A...")
        motor_a_stop()
    elif command == "start_b":
        print("Starting Motor B...")
        motor_b_forward(INITIAL_SPEED)
    elif command == "stop_b":
        print("Stopping Motor B...")
        motor_b_stop()
    elif command == "start_both":
        print("Starting both motors...")
        motor_a_forward(INITIAL_SPEED)
        motor_b_forward(INITIAL_SPEED)
    elif command == "stop_both":
        print("Stopping both motors...")
        motor_a_stop()
        motor_b_stop()
    elif command == "both_back":
        print("Both motors backward...")
        motor_both_backward(INITIAL_SPEED, duration_ms=600)
    elif command == "both_forward":
        print("Both motors forward...")
        motor_both_forward(INITIAL_SPEED, duration_ms=600)

# Create MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect("mqtt.eclipseprojects.io", 1883, 60)

# Subscribe to MQTT topic
mqtt_client.subscribe("motor/control")

# Set callback for incoming messages
mqtt_client.on_message = on_message

try:
    # Start MQTT client and wait for messages
    mqtt_client.loop_start()

    while True:  # Infinite loop
        time.sleep(0)

except KeyboardInterrupt:
    # Exit script when Ctrl+C is pressed
    motor_a_stop()
    motor_b_stop()
    pwm_in1.stop()
    pwm_in2.stop()
    pwm_in3.stop()
    pwm_in4.stop()
    pwm_enable_a.stop()
    pwm_enable_b.stop()
    GPIO.cleanup()
    mqtt_client.disconnect()
    mqtt_client.loop_stop()



################################

import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import paho.mqtt.client as mqtt
from random import uniform
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# Set the GPIO pins you are using according to your connections for Motor A side
IN1 = 17
IN2 = 18
ENABLE_A = 12

# Set the GPIO pins you are using according to your connections for Motor B side
IN3 = 23
IN4 = 24
ENABLE_B = 25

# Initialize the GPIO library
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENABLE_A, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENABLE_B, GPIO.OUT)

# PWM frequency (Hz) and initial speed (0% to 100%)
PWM_FREQ = 1000
INITIAL_SPEED = 100

# Create PWM objects for Motor A pins
pwm_in1 = GPIO.PWM(IN1, PWM_FREQ)
pwm_in2 = GPIO.PWM(IN2, PWM_FREQ)
pwm_enable_a = GPIO.PWM(ENABLE_A, PWM_FREQ)

# Create PWM objects for Motor B pins
pwm_in3 = GPIO.PWM(IN3, PWM_FREQ)
pwm_in4 = GPIO.PWM(IN4, PWM_FREQ)
pwm_enable_b = GPIO.PWM(ENABLE_B, PWM_FREQ)

# --- The motor functions are the same as above, unchanged ---

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/server.html')
            self.end_headers()
        elif self.path == '/server.html':
            with open('server.html', 'rb') as file:
                content = file.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                self.handle_streaming()
            except ConnectionResetError:
                logging.warning('Connection reset by client')
                camera_stream.stop()
            except Exception as e:
                logging.warning('Exception occurred in streaming: %s', str(e))
                camera_stream.stop()
        else:
            self.send_error(404)
            self.end_headers()

    def handle_streaming(self):
        while True:
            with camera_stream.output.condition:
                camera_stream.output.condition.wait()
                frame = camera_stream.output.frame
            try:
                # Perform face detection on the frame
                frame_with_faces = detect_faces(frame)

                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame_with_faces))
                self.end_headers()
                self.wfile.write(frame_with_faces)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except BrokenPipeError:
                logging.warning('Broken pipe error occurred')
                camera_stream.stop()
                break

def detect_faces(frame):
    nparr = np.frombuffer(frame, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    if img is None:
        print("Error: Image decoding failed.")
        return frame  # Return original frame if decoding fails

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(faces) > 0:
        print(f"{len(faces)} face(s) detected.")
        # Place actions here if faces are detected
    else:
        print("No faces detected.")
        # Place actions here if no faces are detected

    # Draw rectangles around detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

class CameraStream(object):
    def __init__(self):
        self.camera = picamera.PiCamera(resolution='640x480', framerate=24)
        self.output = StreamingOutput()
        self.server = None
        self.is_streaming = False

    def start(self):
        print("is Starting")
        self.camera.start_recording(self.output, format='mjpeg')
        address = ('', 8000)
        self.server = StreamingServer(address, StreamingHandler)
        self.server.serve_forever()

    def stop(self):
        if self.camera.recording:
            self.camera.stop_recording()
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        self.is_streaming = False

def on_connect(client, userdata, flags, rc):
    print("Client connected")
    client.subscribe("camera/control")

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print("Received message: ", message)
    if message == "start":
        print("Starting camera...")
        if not camera_stream.is_streaming:
            camera_stream.start()
            camera_stream.is_streaming = True
    elif message == "both_back":
        motor_both_backward(INITIAL_SPEED, duration_ms=600)
        if camera_stream.is_streaming:
            camera_stream.stop()
            camera_stream.is_streaming = False
    elif message == "both_forward":
        if not camera_stream.is_streaming:
            camera_stream.start()
            camera_stream.is_streaming = True
        motor_both_forward(INITIAL_SPEED, duration_ms=600)

# Load face cascade classifier
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

client = mqtt.Client("Camera_Streaming_Server")
client.on_connect = on_connect
client.on_message = on_message

client.connect("mqtt.eclipseprojects.io")

camera_stream = CameraStream()

try:
    # Start MQTT client and wait for messages
    client.loop_start()

    while True:  # Infinite loop
        time.sleep(0)

except KeyboardInterrupt:
    # Exit script when Ctrl+C is pressed
    motor_a_stop()
    motor_b_stop()
    pwm_in1.stop()
    pwm_in2.stop()
    pwm_in3.stop()
    pwm_in4.stop()
    pwm_enable_a.stop()
    pwm_enable_b.stop()
    GPIO.cleanup()
    client.disconnect()
    client.loop_stop()
