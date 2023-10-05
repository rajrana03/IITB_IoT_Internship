import socket
import struct
import time
# from datetime import datetime

# Perform some operations or wait for some time...

# Define the server's IP address and port
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 53  # Use the same port as specified in your ESP32 code
count = 0
# Create a socket object
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    # Set the socket option to allow quick reuse of the address and port
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind the socket to the specified address and port
    server_socket.bind((HOST, PORT))
    
    # Listen for incoming connections
    server_socket.listen()
    print(f"Listening on {HOST}:{PORT}...")
    
    while True:
        # Accept incoming connections
        conn, addr = server_socket.accept()
        with conn:
            print(f"Connected by {addr}")
            timestamp1 = time.time_ns()
            # Continuously read and process incoming data
            # while True:
            for i in range (1000):
                data = conn.recv(2)  # Adjust buffer size as needed
                # print(data)
                # if not data:
                #     # The connection is closed by the remote end
                #     print("Connection closed by remote end")
                #     break  # Exit the data receiving loop
                
                if len(data) == 2:  # Assuming it's a 4-byte integer (adjust as needed)
                    my_int = struct.unpack('h', data)[0]
                    # print(f"Received Integer: {my_int}")
                    # count+=1
                else:
                    print("Received Unrecognized Binary Data")

                                        
            # Get the second timestamp
            timestamp2 = time.time_ns()
            # Calculate the difference in microseconds
            time_difference = ((timestamp2 - timestamp1)/1000)
            print(count)
            print(f"Time difference in microseconds: {time_difference}")

       



