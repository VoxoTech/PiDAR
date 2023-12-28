import binascii



byte_string = b',n\x11\x1c\x1b\x9e\x01\xe9\x96\x01\xe9\x92\x01\xea\x98\x01\xe8\xa2\x01\xdc\xf0\x00\xd5\xad\x01T,\x11c$\x03\xd3\xec\x01\xd3\x01\xd4\x9e\x01\xd3\x01\xd2Y\x01\xd2\x01\xdb'

hex_string = binascii.hexlify(byte_string).decode()
spaced_string = ' '.join([hex_string[i:i+2] for i in range(0, len(hex_string), 2)])
hex_list = spaced_string.split(' ')
print(len(hex_list), hex_list)
