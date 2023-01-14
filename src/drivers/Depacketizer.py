import collections
from drivers.Packet import OVERHEAD, MAX_PAYLOAD, PacketizerException, Packet
BUFFER_LEN = 2048
class Depacketizer:
    def __init__(self):
        self.buffer = []
    

    # each time we read new bytes from the serial port, we call this function.
    def write(self, bytes):
        self.buffer.extend(bytes)
    
    # each time time the coms system checks for new packets, it calls this function.
    def read_packets_from_buffer(self):
        try:
            test_index = self.buffer.index(b'\x69')
            # since all packets start with 0x6969, we can find the start of the next packet by searching for 0x69.
        except ValueError:
            # if we don't find it, we can assume that the buffer is garbage and we can clear it.
            self.buffer = []
            return []
        if test_index + 1 >= len(self.buffer):
            # in this case, there is still a chance we have a good packet, but we don't have enough bytes to check
            # therefore we return no packets, but leave the buffer as is.
            return []
        if self.buffer[test_index + 1] != b'\x69':
            # if the next byte is not 0x69, we know that this is not a hit, so we remove the first bytes and try again.
            self.buffer.pop(test_index)
            # however, we still need to check if there are other packets in the buffer.
            return self.read_packets_from_buffer()
        # if we get here, we think that we have a packet starting at test_index.
        # we need to check if the packet is complete.
        packet_length = self.packet_bytes(self.buffer[test_index:])
        if packet_length == 0:
            # the packet is not complete, so we return no packets, but leave the buffer as is.
            return []
        if packet_length == -1:
            # the packet is invalid, remove it from the buffer and check for more packets.
            self.buffer = self.buffer[test_index + OVERHEAD:]
            return self.read_packets_from_buffer()
        # if we get here, we have a complete packet, with an unverified checksum.
        try:
            packet = self.de_packetizer(self.buffer[test_index:test_index + packet_length])
            # if we get here, the packet is valid, so we remove it from the buffer and return it.
            self.buffer = self.buffer[test_index + packet_length:]
            return [packet] + self.read_packets_from_buffer()
        except PacketizerException as P:
            print(P)
            # if we get here, the packet is invalid, so we remove it from the buffer and check for more packets.
            self.buffer = self.buffer[test_index + OVERHEAD:]
            return self.read_packets_from_buffer()



    
    def de_packetizer(self, packet : bytes) -> tuple[int, bytes]:
        if len(packet) <= OVERHEAD:
            raise PacketizerException("Packet is too small to be valid")
        if packet[:2] != b'\x69\x69':
            raise PacketizerException("Packet does not start with 0x6969")
        packet_id = int.from_bytes(packet[2:6], 'little')
        print(packet_id)
        payload_length = int.from_bytes(packet[6:8], 'little')
        if len(packet) != OVERHEAD + payload_length:
            raise PacketizerException("Packet length does not match payload length")
        payload = packet[OVERHEAD:]
        if hashlib.sha256(packet[2:6] + payload).digest()[:4] != packet[8:12]:
            raise PacketizerException("Packet hash does not match payload hash")
        return Packet(packet_id, payload)

    def packet_bytes(self, packet : bytes) -> int:
        if len(packet) < OVERHEAD:
            return 0
        payload_length = int.from_bytes(packet[6:8], 'little')
        if payload_length > MAX_PAYLOAD or payload_length < 0:
            return -1
        if len(packet) < OVERHEAD + payload_length:
            return 0
        return OVERHEAD + payload_length
        