import cv2
import socket
import struct
import pickle
import threading
import signal
import sys
import time


# Stream function for each client
def handle_client(conn, addr, camera_id, width, height, fps):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Camera {camera_id} could not be opened.")
        conn.close()
        return

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    delay = 1 / fps

    try:
        print(f"Streaming to {addr} from camera {camera_id}...")
        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Failed to capture frame from camera {camera_id}.")
                break

            # Serialize frame
            data = pickle.dumps(frame)
            size = len(data)

            # Send frame size and data
            conn.sendall(struct.pack(">L", size) + data)

            # Control frequency
            time.sleep(delay)
    except Exception as e:
        print(f"Connection to {addr} closed or error occurred: {e}")
    finally:
        cap.release()
        conn.close()


# Server function to accept connections
def server(camera_id, port, width=640, height=480, fps=15):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(5)
    print(f"Server started on port {port} for camera {camera_id}...")

    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"Connection from {addr} on port {port}")
            client_thread = threading.Thread(target=handle_client, args=(conn, addr, camera_id, width, height, fps))
            client_thread.start()
    except Exception as e:
        print(f"Server error on port {port}: {e}")
    finally:
        server_socket.close()


# Cleanup function
def cleanup_and_exit(signum, frame):
    print("\nTerminating program...")
    sys.exit(0)


# Register signal handlers
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Start servers for each camera
def main():
    # Define cameras and their respective ports
    camera_servers = [
        threading.Thread(target=server, args=(6, 6000, 640, 480, 15)),  # Camera 6 on port 6000
        threading.Thread(target=server, args=(4, 6001, 848, 480, 15)),  # Camera 4 on port 6001
    ]

    for thread in camera_servers:
        thread.start()

    for thread in camera_servers:
        thread.join()

if __name__ == "__main__":
    main()