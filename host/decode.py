#!/usr/bin/env python3
# Full live decoder for the 42-byte Unitree RC packet (format verified 2026-11-07).
#   decode.py [seconds] [cmd...]
import sys, time, glob, struct

B1=["R1","L1","Start","Select","R2","L2","F1","F2"]      # byte[4]
B2=["A","B","X","Y","Up","Right","Down","Left"]          # byte[5]

def bits(v,names): return [n for i,n in enumerate(names) if v&(1<<i)]

def decode(b):
    if len(b)!=42: return "len=%d (unexpected)"%len(b)
    lx=struct.unpack_from("<f",b,6)[0]
    rx=struct.unpack_from("<f",b,10)[0]
    ry=struct.unpack_from("<f",b,14)[0]
    ly=struct.unpack_from("<f",b,22)[0]
    btn=bits(b[4],B1)+bits(b[5],B2)
    cks=(b[40]<<8)|b[41]; ok=(sum(b[4:40])&0xFFFF)==cks
    s=("id=%02x%02x LX=%+.2f LY=%+.2f RX=%+.2f RY=%+.2f"%(b[0],b[1],lx,ly,rx,ry))
    if btn: s+="  BTN["+",".join(btn)+"]"
    if not ok: s+="  CKSUM-BAD"
    return s

def main():
    import serial
    secs=float(sys.argv[1]) if len(sys.argv)>1 else 30.0
    cmds=sys.argv[2:]
    port=sorted(glob.glob("/dev/ttyACM*"))[-1]; sys.stderr.write("[port %s]\n"%port)
    s=serial.Serial(port,115200,timeout=0.2); time.sleep(0.3); s.reset_input_buffer()
    for c in cmds: s.write((c+"\n").encode()); s.flush(); time.sleep(0.25)
    buf=b""; end=time.time()+secs; last=None
    while time.time()<end:
        buf+=s.read(4096)
        while b"\n" in buf:
            line,buf=buf.split(b"\n",1)
            t=line.decode("utf-8","replace")
            if "Radio Data:" in t:
                hx=t.split(":",1)[1].split("[")[0].strip().split()
                try: b=bytes(int(x,16) for x in hx)
                except ValueError: continue
                d=decode(b)
                if d!=last: print(time.strftime("%H:%M:%S"),d); last=d
    s.close()

if __name__=="__main__": main()
