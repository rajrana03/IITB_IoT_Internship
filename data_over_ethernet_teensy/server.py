# import socket

# # Define the server's IP address and port
# HOST = '192.168.0.113'  # Listen on all available network interfaces
# PORT = 12345  # Use a port of your choice (e.g., 12345)

# # Create a socket object
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
#     # Bind the socket to the specified address and port
#     server_socket.bind((HOST, PORT))
    
#     # Listen for incoming connections
#     server_socket.listen()
#     print(f"Listening on {HOST}:{PORT}...")
    
#     # Accept incoming connections
#     conn, addr = server_socket.accept()
#     with conn:
#         print(f"Connected by {addr}")
        
#         # Receive and process incoming data
#         while True:
#             data = conn.recv(1024)  # Adjust buffer size as needed
#             if not data:
#                 break
#             print(f"Received: {data.decode('utf-8')}")


# import socket
# import struct

# # Define the server's IP address and port
# HOST = '0.0.0.0'  # Listen on all available network interfaces
# PORT = 12345  # Use the same port as specified in your ESP32 code

# # Create a socket object
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
#     # Bind the socket to the specified address and port
#     server_socket.bind((HOST, PORT))

#     # Listen for incoming connections
#     server_socket.listen()
#     print(f"Listening on {HOST}:{PORT}...")

#     # Accept incoming connections
#     conn, addr = server_socket.accept()
#     with conn:
#         print(f"Connected by {addr}")

#         # Continuously read and process incoming data
#         while True:
#             data = conn.recv(2)  # Adjust buffer size as needed
#             print(data)
#             if not data:
#                 break
            
#             # try:
#             #     # Attempt to decode data as a string
#             #     data_str = data.decode('utf-8')
#             #     print(f"Received String: {data_str}")
                
#             #     # If the decoding is successful, it's a string
#             #     # Handle the string data as needed
#             #     # Example: You can check for specific commands or messages in the string
                
#             # except UnicodeDecodeError:
#                 # If decoding as a string fails, it's binary data
#                 # Handle the binary data (e.g., convert it to integers or floats)
                
#             if len(data) == 2:  # Assuming it's a 4-byte integer (adjust as needed)
#                 my_int = struct.unpack('h', data)[0]
#                 print(f"Received Integer: {my_int}")
            
#             # elif len(data) == 8:  # Assuming it's an 8-byte float (adjust as needed)
#             #     my_float = struct.unpack('d', data)[0]
#             #     print(f"Received Float: {my_float}")
            
#             else:
#                 print("Received Unrecognized Binary Data")




import socket
import struct
import time
from datetime import datetime



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
            timestamp1 = datetime.now()
            # Continuously read and process incoming data
            while True:
            # for i in range (1000):
                data = conn.recv(100)  # Adjust buffer size as needed
                # print(data)
                # if not data:
                #     # The connection is closed by the remote end
                #     print("Connection closed by remote end")
                #     break  # Exit the data receiving loop
                
                if len(data) == 2:  # Assuming it's a 4-byte integer (adjust as needed)
                    my_int = struct.unpack('h', data)[0]
                    print(f"Received Integer: {my_int}")

                else:
                    print("Received Unrecognized Binary Data")

                                        
            # Get the second timestamp
            timestamp2 = datetime.now()
            # Calculate the difference in microseconds
            time_difference = (timestamp2 - timestamp1).microseconds

            print(f"Time difference in microseconds: {time_difference}")

       



