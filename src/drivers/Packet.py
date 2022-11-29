# Written by Walden Hillegass 
# details of packet structure are in this document :
# https://docs.google.com/document/d/e/2PACX-1vRbm1WKCdh5X4Ji4nGYI22LL2Y-oLjkN6hBZEl6U7xsOcApdzz1ynh-aBq82YZGA1eBMbsIvnchxTQM/pub

import hashlib
MAX_PAYLOAD = 500
OVERHEAD = 12

class PacketizerException(Exception):
    pass

class Packet:
    def __init__(self, packet_id : int, payload : bytes):
        if (len(payload) > MAX_PAYLOAD):
            raise PacketizerException("Payload is larger than max size. Use multi_packetizer() instead")
        id_bytes = packet_id.to_bytes(4, 'little')
        payload_length = len(payload).to_bytes(2, 'little')
        b = []
        b.append(b'\x69\x69')
        b.append(id_bytes)
        b.append(payload_length)
        b.append(hashlib.sha256(id_bytes + payload).digest()[:4])
        b.append(payload)

        self.bytes = b"".join(b)
        

# Returns a list of packets
def multi_packetizer(payload : bytes, starting_packet_id : int) -> list:
    packets = []
    packet_id = starting_packet_id
    while len(payload) > 0:
        if len(payload) > MAX_PAYLOAD:
            packets.append(Packet(packet_id, payload[:MAX_PAYLOAD]))
            payload = payload[MAX_PAYLOAD:]
        else:
            packets.append(Packet(packet_id, payload))
            payload = b""
        packet_id += 1
    return packets


def de_packetizer(packet : bytes) -> tuple[int, bytes]:
    if len(packet) < OVERHEAD:
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
    return (packet_id, payload)



        
if __name__ == "__main__":
    p = Packet(5000000, b'It was the best of times, it was the worst of times')
    print(p.bytes)
    print(de_packetizer(p.bytes))




