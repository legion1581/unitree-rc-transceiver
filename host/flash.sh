#!/bin/bash
# Robust flasher for the bootlooping/renumbering T3-S3 USB-JTAG.
cd "$(dirname "$0")/.." || exit 1
BIN=.pio/build/t3s3/firmware.bin; BOOT=.pio/build/t3s3/bootloader.bin; PART=.pio/build/t3s3/partitions.bin
ES=~/.local/bin/esptool.py
pick(){ ls /dev/ttyACM* 2>/dev/null | sort | tail -1; }

ok=0
for a in $(seq 1 30); do
  P=$(pick); [ -z "$P" ] && continue
  out=$(timeout 12 $ES --chip esp32s3 --port "$P" --baud 460800 \
        --before usb-reset --after no-reset \
        write-flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB \
        0x0000 "$BOOT" 0x8000 "$PART" 0x10000 "$BIN" 2>&1)
  if echo "$out" | grep -q "Hash of data verified"; then echo "FLASH OK (attempt $a, $P)"; ok=1; break; fi
  printf "%d " "$a"
done
[ $ok -eq 0 ] && { echo; echo "FLASH FAILED"; echo "$out" | tail -6; exit 2; }

# run the app via watchdog reset (reliable for USB-JTAG)
P=$(pick)
timeout 15 $ES --chip esp32s3 --port "$P" --before usb-reset --after watchdog-reset flash-id 2>&1 | tail -2
echo "RESET done"
