import socket
from text_2_speech import text_to_speech
from voice_2_text import recognize_speech_from_mic


def client_program():
    host = "raspberrypi.local"  # Replace with Raspberry Pi's IP address
    port = 5000  # The same port as used by the server

    client_socket = socket.socket()
    client_socket.connect((host, port))
    text_to_speech(f"client connected to{host}")

    message = input(" -> ")

    while message.lower().strip() != 'bye':
        text_to_speech("Please say a command")
        client_socket.send(message.encode())
        data = client_socket.recv(1024).decode()
        print('Received from server: ' + data)
        message = recognize_speech_from_mic()

    client_socket.close()


if __name__ == '__main__':
    client_program()






