"""UDP door command transport."""

import socket


class DoorClient:
    def __init__(self, node, doors):
        self.node = node
        self.doors = doors

    def send(self, door_id, command):
        destination = self.doors[door_id]
        self.node.get_logger().info(
            f'Sending door {door_id} command {command!r} to '
            f'{destination[0]}:{destination[1]}'
        )
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
                udp_socket.sendto(command.encode('ascii'), destination)
        except OSError as error:
            raise RuntimeError(
                f'failed to send door {door_id} command {command!r}: {error}'
            ) from error
