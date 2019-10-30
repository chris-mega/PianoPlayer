import socket
import sys
import rospy
from std_msgs.msg import String

# Create a UDP socket
def client(ip, message):
    # pub = rospy.Publish('/udp_message',String, queue_size=1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server_address = (ip, 10000)

    try:

        # Send data
        print(sys.stderr, 'sending "%s"' % message)
        sent = sock.sendto(message.encode(), server_address)

        # # Receive response
        # print(sys.stderr, 'waiting to receive')

        # data, server = sock.recvfrom(4096)
        # print(sys.stderr, 'received "%s"' % data)

        # pub.publish(message)

    finally:
        print(sys.stderr, 'closing socket')
        sock.close()


def server(ip):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the port
    server_address = (ip, 10000)
    print(sys.stderr, 'starting up on %s port %s' % server_address)
    sock.bind(server_address)

    while True:
        print('waiting to receive message')
        data, address = sock.recvfrom(4096)
        print('received {} bytes from {}'.format(len(data), address))
        print("data ", data)
        if data:
            sent = sock.sendto(data, address)
            print('sent {} bytes back to {}'.format(sent, address))
            return