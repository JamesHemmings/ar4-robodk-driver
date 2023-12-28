import socket
from text_2_speech import text_to_speech
from voice_2_text import recognize_speech_from_mic

test_string = "LLA1B0C0D0E0F0G0H0I0" + "J0K-3.96L0M1.334N30.005O10.5P0Q0R0" + "\n"

def connect_to_server():
    host = "raspberrypi.local"  # Raspberry Pi's IP address
    port = 5000  # The same port as used by the server

    client_socket = socket.socket()
    client_socket.connect((host, port))
    text_to_speech(f"client connected to {host}")

    return client_socket

def send_message(client_socket, message):
    client_socket.send(message.encode())
    data = client_socket.recv(1024).decode()
    print('Received from server: ' + data)
    return data

if __name__ == '__main__':
    client_socket = connect_to_server()
    message = input(" -> ")
    message = test_string

    while message.lower().strip() != 'bye':
        response = send_message(client_socket, message)
        message = input(" -> ")

    client_socket.close()