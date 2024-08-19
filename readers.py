import os

class PCAPReader:
    ''' Based off PCAP format spec here: https://datatracker.ietf.org/doc/id/draft-gharris-opsawg-pcap-00.html
    Note that this has only been tested on PCAP files captured using the network loopback
    interface (link type = 0) and may not work on other link types'''
    
    def __init__(self, file: str):
        self.file = file
        
        with open(file, 'rb') as fstream:
            header = fstream.read(24) #file header is 24 bytes

            #magic number stuff
            if header[0:4] == bytes.fromhex('A1B2C3D4'):
                self.endian = 'big'
                self.timestamp_format = 'seconds_microseconds'
            elif header[0:4] == bytes.fromhex('A1B23C4D'):
                self.endian = 'big'
                self.timestamp_format = 'seconds_nanoseconds'
            elif header[0:4][::-1] == bytes.fromhex('A1B2C3D4'):
                self.endian = 'little'
                self.timestamp_format = 'seconds_microseconds'
            elif header[0:4][::-1] == bytes.fromhex('A1B23C4D'):
                self.endian = 'little'
                self.timestamp_format = 'seconds_nanoseconds'
            else:
                raise Exception('Invalid PCAP magic number. Must equal 0xA1B2C3D4 or 0xA1B23C4D in either big or little endian.')

            self.major_version = int.from_bytes(header[4:6], self.endian, signed = False)
            self.minor_version = int.from_bytes(header[6:8], self.endian, signed = False)
            self.reserved_1 = header[8:12]
            self.reserved_2 = header[12:16]

            #snap_len is max number of bytes captured from each packet
            self.snap_len = int.from_bytes(header[16:20], self.endian, signed = False)
            self.fcs_f = header[20:21]
            self.link_type = int.from_bytes(header[21:24], self.endian, signed = False)

            if self.link_type != 0:
                #see all link types here: https://www.tcpdump.org/linktypes.html
                print(f'WARNING: This basic PCAP reader is only coded for link_type = 0. Current link type: {self.link_type}. Other link types MIGHT have issues.')

            self.file_size_bytes = os.stat(file).st_size

    def __str__(self):
        #This function is for fancy printing of this object. Mostly used for debugging!
        return f'Filename: {self.file}\nFile Size Bytes: {self.file_size_bytes}\nPCAP Version: {self.major_version}.{self.minor_version}\nEndian: {self.endian}\nTimestamp Format: {self.timestamp_format}\nSnap Length: {self.snap_len} bytes\nLinktype: {self.link_type}'

    def __iter__(self):
        with open(self.file, 'rb') as fstream:
            fstream.seek(24) #seek to first packet (after header)

            while True:
                #read packet header
                header = fstream.read(16) #packet header is 16 bytes
                if len(header) < 16:
                    break #end of file

                timestamp_seconds = int.from_bytes(header[0:4], self.endian, signed = False)
                timestamp_micronano_seconds = int.from_bytes(header[4:8], self.endian, signed = False)
                captured_packet_len = int.from_bytes(header[8:12], self.endian, signed = False)
                orig_packet_len = int.from_bytes(header[12:16], self.endian, signed = False)
                packet_data = fstream.read(captured_packet_len)

                yield packet_data
