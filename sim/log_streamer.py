# reader.py
import os

fifo_path = "/tmp/mincopter_log"

# Ensure the FIFO exists (in case reader is started first)
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

print("Waiting for data...")

def parse(frame):
    print(frame)
    return

with open(fifo_path, "rb") as fifo:

    while True:
        data = fifo.read(2048)

        if not data:
            break

        frame = bytes()

        in_frame = False
        remaining = 0

        for b in data:

            ## If we are in a frame currently and still have bytes left then we just append
            if in_frame and remaining>0:
                frame += bytes(b)
                remaining -= 1
                continue

            ## If we are in a frame but have no bytes then we have a valid frame and should parse
            if in_frame and remaining==0:
                parse(frame)
                frame = bytes()
                in_frame = False
                continue

            ## If we are not in a valid frame, we are either waiting for the sync bytes 
            if not in_frame and len(frame)==4:
                remaining = frame[-1] - 1 
                frame += bytes(b)
                in_frame = True
                continue

            elif not in_frame:
                if len(frame)==0 and b==0x2A:
                    frame += bytes(b)
                else:
                    ## Reset frame
                    frame = bytes()
                    continue

                if len(frame)==1 and b==0x4E:
                    frame += bytes(b)
                else:
                    ## Reset frame
                    frame = bytes()
                    continue


