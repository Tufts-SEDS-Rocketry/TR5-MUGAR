import struct
import sys

infname = sys.argv[1]
outfname = sys.argv[2]
print(f"Unpacking file {infname}, writing to file {outfname}")

format = "<iffffffffffff"

infile = open(infname, mode='rb')
outfile = open(outfname, mode='a+') # dont write over other file
data = bytearray(infile.read())

outfile.write("time,ax,ay,az,gx,gy,gz\n")

first_packet_format = "<iiffffff"

offset = 0
while True:
    init_packet = struct.unpack_from(first_packet_format, data, offset)
    if (init_packet[1] != 0): 
        #only triggers after the first 0.0 extension, so we rewrite
        offset -= struct.calcsize(first_packet_format)
        break
    outfile.write(str(init_packet[0]))
    outfile.write(',')
    outfile.write(",".join([str(x) for x in init_packet[2:]]))
    outfile.write('\n')
    offset += struct.calcsize(first_packet_format)

outfile.write("time,proportion,orientation real,orientation i,orientation j,orientation k,vx,vy,vz,x,y,z\n")

time_threshold = 100 #ms
num_time_over = 0
avg_time = 0
num_packets = 0
max_time_gap = 0
last_time = 0

while True:
    if len(data) - offset < struct.calcsize(format):
        break
    packet = struct.unpack_from(format, data, offset)

    outfile.write(','.join([str(x) for x in packet]))
    outfile.write("\n")
    time = packet[0]
    if last_time != 0:
        time_gap = time - last_time
        avg_time = num_packets / (num_packets + 1) * avg_time + 1 / (num_packets + 1) * time_gap
        if time_gap > time_threshold:
            print(num_packets)
            num_time_over += 1
        if time_gap > max_time_gap:
            max_time_gap = time_gap

    last_time = time
    num_packets += 1
    offset += struct.calcsize(format)

outfile.close()
infile.close()

print(f"read {num_packets} data packets, average time gap was {avg_time} ms, number of time gaps over threshold of {time_threshold} ms was {num_time_over}, max time gap was {max_time_gap}")