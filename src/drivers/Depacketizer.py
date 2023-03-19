import collections, hashlib
from Packet import OVERHEAD, MAX_PAYLOAD, PacketizerException, Packet, multi_packetizer
BUFFER_LEN = 2048
class Depacketizer:
    def __init__(self):
        self.buffer = bytes()
        self.packets = []


    # each time we read new bytes from the serial port, we call this function.
    def write(self, bts: bytes):
        self.buffer += bts
        self.packets = []
    
    def remove_transmission(self, transmission_id):
        for packet in self.packets:
            if packet.transmission_id == transmission_id:
                self.packets.remove(packet)

    def build_bytes_from_packets(self, packets):
        output = []
        for packet in packets:
            output.append(packet.payload)
        return b"".join(output)

    def read_messages(self, missing_packets_callback: callable([int])) -> list[bytes]:
        self.packets.extend(self.read_packets_from_buffer())
        self.packets.sort(key=lambda x: x.packet_id)
        if len(self.packets) == 0:
            return []
        #check the transmission id of the first packet
        transmission_id = self.packets[0].transmission_id
        if transmission_id != self.packets[0].packet_id:
            #this is a hanging packet.
            self.packets.pop()
        #if we get here we have the first packet of a transmission.
        #check if we have all the packets.
        transmission_count = self.packets[0].transmission_count
        if transmission_count > len(self.packets):
            #we are missing packets wait until the next call to read_messages
            return []
        for i,packet in enumerate(self.packets[0:transmission_count]):
            if packet.transmission_id != transmission_id:
                #we have a packet from a different transmission, wait until the next call to read_messages
                self.packets.remove(packet)
            if packet.packet_id != transmission_id + i:
                #we have a packet with the wrong id, wait until the next call to read_messages
                missing_packets_callback(transmission_id)
                self.remove_transmission(transmission_id)
                return self.read_messages(missing_packets_callback)
            data = self.build_bytes_from_packets(self.packets[0:transmission_count])
            self.packets = self.packets[transmission_count:]
            return [data] + self.read_messages(missing_packets_callback)

    
    # each time time the coms system checks for new packets, it calls this function.
    def read_packets_from_buffer(self):
        try:
            test_index = self.buffer.index(b'\x83')
            # since all packets start with 0x8383, we can find the start of the next packet by searching for 0x83.
        except ValueError:
            # if we don't find it, we can assume that the buffer is garbage and we can clear it.
            self.buffer = []
            return []
        if test_index + 1 >= len(self.buffer):
            # in this case, there is still a chance we have a good packet, but we don't have enough bytes to check
            # therefore we return no packets, but leave the buffer as is.
            return []
        if self.buffer[test_index + 1] != b'\x83'[0]:
            # if the next byte is not 0x83, we know that this is not a hit, so we remove the first bytes and try again.
            self.buffer = self.buffer[test_index + 1:]
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

    
    def de_packetizer(self, packet : bytes) -> Packet:
        if len(packet) <= OVERHEAD:
            raise PacketizerException("Packet is too small to be valid")
        if packet[:2] != b'\x83\x83':
            raise PacketizerException("Packet does not start with 0x6969")
        packet_id = int.from_bytes(packet[2:6], 'little')
        print(packet_id)
        payload_length = int.from_bytes(packet[6:8], 'little')
        if len(packet) != OVERHEAD + payload_length:
            raise PacketizerException("Packet length does not match payload length")
        transmission_id     = int.from_bytes(packet[8:12], 'little', signed=False) # this is hash of the packet id and payload
        transmission_length = packet[12] # this is the length of the transmission
        payload = packet[OVERHEAD:]
        hash = hashlib.sha256(packet[2:6] + packet[8:12] +  packet[12].to_bytes(1, 'little') + payload).digest()[:4]
        print(hash)
        if hash != packet[13:17]:
            raise PacketizerException("Packet hash does not match payload hash")
        return Packet(packet_id, payload, transmission_id, transmission_length)

    def packet_bytes(self, packet : bytes) -> int:
        if len(packet) < OVERHEAD:
            return 0
        payload_length = int.from_bytes(packet[6:8], 'little')
        if payload_length > MAX_PAYLOAD or payload_length < 0:
            return -1
        if len(packet) < OVERHEAD + payload_length:
            return 0
        return OVERHEAD + payload_length
        

if __name__ == "__main__":
    def missing_packets_callback(transmission_id):
        print("missing packets, transmission id: " + str(transmission_id))
    print("testing packetizer")
    #test_message = b"hello world"
    #test_message2 = b"goooodbye world"
    test_message = b"""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"""
    test_message2 = b"""6fctApD2gzD8OeaMg6gMZSTPXqNp7l0O7aCeUVu0IAYqQmqtOaoiun72uB6njri1SBxBo2xoWygbg8qz9INZX8CTtE995GoFozVZh0kdcvr9dy7fo1N9EHw6h3mjASNBZgN0BTYRCzUiXTcJJjUWiwU1OfBW1twNU95mF03jN0JylpC3LJOaQhL5EaETtCzwOGHnvJK8vr81pcp4Rdshf1d7gnx3UgY32cUIQHGxcHEr6jlXxtiELFz9u4iGtnX0r8y3v5unBjaBBGWtGQat6LRhqTjli5eyFn7Njo5DEIiVEwzu8zZDuhUr7dg5eAaQdXbsessKT5mD1LQo1pNW16DHvauaKi5ZKVDkUvU9VMEDc0OnXY4791ewE9Nl3pEgHAMbjnCCxBcIxJqWPkxAfiaIhEn7JqL8v4uojzCFo5lcvkbyxi9pStZ6IfF3KzWpKXyWo0efJ7mUU6UIqiyFfqgtNPdhSbjoH4pV80x2KouG6ZAoUOFMm8wOUGFnu75CQXY2c8DCcAyjq3oA0dkfZNWeDX99Uwi2IdeodbrcOJ4yPRZCRvrt7ftToUyxhC0NF9WpJHcm3DuLYq0abrhmVVNqv1WOK0B94uUNqzdeMUH3WH6JQ3lem7TcMGTfmssWMWLzuEwMqDnuA5c4PtsRQCgyyk1JZVS2ikeCqMOP2GGeLiuNSF5e8T9pSBBcG8MNxXQGNKxVDNp3N4AeyUTlWlKWckqxWlPe3BPvqEOXSE9KCq0bOV8yvqGVdZ9KIfXqygvAY3DNfgotreHFpFQXNAk5fQrBXBUygPTMfauOO0Q0xq1DQ8EM3aCcJLXvTvkz8oSCL6NYidQgCjSOyjU1HspxYXcQwduvxTcW8qi9dkpAFCJK6AxC3ttn8huhw6DI7ZODgVpBdfSryhY3QE7xYZgeIfyFAMG7ul1x5CkxL7qeC1wEPqk81sJfUNiIhc8hLUM2cJZzj0CNnKcqh27iKl7PjKpdDXyQiE9u7Iw2"""
    packets1 = multi_packetizer(test_message, 1)
    print(packets1[0].get_bytes())
    packets2 = multi_packetizer(test_message2, 100)
    depacketizer = Depacketizer()
    for p in packets1:
        depacketizer.write(p.get_bytes())
    
    for p in packets2:
        depacketizer.write(p.get_bytes())

    msgs =  depacketizer.read_messages(missing_packets_callback)
    print(msgs)
    print(msgs[0] == test_message)
    print(msgs[1] == test_message2)
    
    msgs =  depacketizer.read_messages(missing_packets_callback)
    msgs[0] == test_message2

    