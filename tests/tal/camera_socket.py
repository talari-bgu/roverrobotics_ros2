import cv2
import socket
import struct
import pickle
import threading
import signal
import sys
import time

# Socket setup function
def setup_socket(port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(5)
    print(f"Waiting for connection on port {port}...")
    conn, addr = server_socket.accept()
    print(f"Connection from {addr} on port {port}")
    return server_socket, conn

# Stream function for each camera
def stream_camera(camera_id, port, width=640, height=480, fps= 15):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Camera {camera_id} could not be opened.")
        return

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    server_socket, conn = setup_socket(port)
    delay = 1 / fps

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Failed to capture frame from camera {camera_id}. Exiting...")
                break

            # Size of each frame is : Height X Width X Channels)

            # Serialize frame
            data = pickle.dumps(frame)
            size = len(data)

            # Send frame size and data
            conn.sendall(struct.pack(">L", size) + data)

            # Control frequency
            time.sleep(delay)

    except Exception as e:
        print(f"An error occurred in camera {camera_id}: {e}")
    finally:
        cap.release()
        conn.close()
        server_socket.close()

# Cleanup function
def cleanup_and_exit(signum, frame):
    print("\nTerminating program...")
    sys.exit(0)

# Register signal handlers
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Start threads for each camera
if __name__ == "__main__":
    # Define cameras and their respective ports
    camera_threads = [
        threading.Thread(target=stream_camera, args=(6, 6000, 640, 480, 15)),  # Camera 0 on port 8000
        threading.Thread(target=stream_camera, args=(4, 6001, 848, 480 ,15)),  # Camera 1 on port 8001
    ]

    for thread in camera_threads:
        thread.start()

    for thread in camera_threads:
        thread.join()
