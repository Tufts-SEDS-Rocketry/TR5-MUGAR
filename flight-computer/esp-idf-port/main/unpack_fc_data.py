import struct
import sys

infname = sys.argv[1]
outfname = sys.argv[2]
print(f"Unpacking file {infname}, writing to file {outfname}")

format = "<i" + "f" * 14

infile = open(infname, mode='rb')
outfile = open(outfname, mode='a+') # dont write over other file
data = bytearray(infile.read())

outfile.write("time,ax,ay,az,gx,gy,gz,b1_temp,b1_pressure,b2_temp,b2_pressure,h_ax,h_ay,h_az,temp\n")

time_threshold = 100 #ms
num_time_over = 0
avg_time = 0
num_packets = 0
max_time_gap = 0
last_time = 0

offset = 0

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