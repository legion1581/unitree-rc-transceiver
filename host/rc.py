#!/usr/bin/env python3
# Serial helper for the T3-S3 receiver/transmitter.
#   rc.py <seconds> [cmd1] [cmd2] ...
# Opens /dev/ttyACM0 @115200, sends each cmd (newline), then prints all RX
# for <seconds>.
import sys, time, glob, serial

ports = sorted(glob.glob("/dev/ttyACM*"))
PORT = ports[-1] if ports else "/dev/ttyACM0"
sys.stderr.write("[port %s]\n" % PORT)
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
cmds = sys.argv[2:]

s = serial.Serial(PORT, 115200, timeout=0.2)
time.sleep(0.3)
s.reset_input_buffer()
for c in cmds:
    s.write((c + "\n").encode())
    s.flush()
    time.sleep(0.25)

end = time.time() + secs
while time.time() < end:
    data = s.read(4096)
    if data:
        sys.stdout.write(data.decode("utf-8", "replace"))
        sys.stdout.flush()
s.close()
