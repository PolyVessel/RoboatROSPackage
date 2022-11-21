# Written by Walden Hillegass 
# details of packet structure are in this document :
# https://docs.google.com/document/d/e/2PACX-1vRbm1WKCdh5X4Ji4nGYI22LL2Y-oLjkN6hBZEl6U7xsOcApdzz1ynh-aBq82YZGA1eBMbsIvnchxTQM/pub

import hashlib
MAX_PAYLOAD = 500
OVERHEAD = 12

class Packet:
    def __init__(self, packet_id : int, payload : bytes):
        if (len(payload) > MAX_PAYLOAD):
            raise PacketizerException("Payload is larger than max size. Use multi_packetizer() instead")
        id_bytes = packet_id.to_bytes(4, 'little')
        b = []
        b.append(b'\x69\x69')
        b.append(id_bytes)
        b.append(payload)
        b.append(b"\x03") # EOT character
        b.append(hashlib.sha256(id_bytes + payload).digest()[:4])
        b.append(b"\04")

        self.bytes = b"".join(b)
        

def multi_packetizer():
    pass


        
if __name__ == "__main__":
    p = Packet(5000000, b'It was the best of times, it was the worst of times')
    print(p.bytes)




