import socket
import threading

# ESP32's IP address and port
host = '192.168.1.38'
port = 12345

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(10)  # Set timeout to avoid waiting indefinitely


def send_message(host, port):
    print("Enter control commands (type 'exit' to quit):")
    while True:
        message = input("Command: ").strip().lower()  # Take user input and normalize case

        if message == "exit":
            print("Exiting...")
            break  # Stop the loop if user types 'exit'

        # Send the message
        sock.sendto(message.encode(), (host, port))
        print(f"Sent: {message}")


def receive_messages():
    while True:
        try:
            data, addr = sock.recvfrom(1024)  # Receive up to 1024 bytes
            message = data.decode().strip()
            print(f"Received: {message}")

        except socket.timeout:
            pass  # Ignore timeout errors in the receive thread


def main():
    # Start a thread for receiving messages
    receive_thread = threading.Thread(target=receive_messages, daemon=True)
    receive_thread.start()

    # Start a thread for sending messages
    send_thread = threading.Thread(target=send_message, args=(host, port), daemon=True)
    send_thread.start()

    # Wait for both threads to complete
    send_thread.join()  # Wait for the send thread to finish
    sock.close()  # Close socket on exit


if __name__ == "__main__":
    main()
