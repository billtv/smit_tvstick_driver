#!/bin/sh
set -eu

echo "=== host ==="
hostname
uname -a
date -Is

echo "=== usb/dvb ==="
lsusb -d 29df:0001 || true
ls -l /dev/dvb/adapter* 2>/dev/null || true
lsmod | awk '$1 == "smitdtmb" || NR == 1'

echo "=== frontend ==="
if command -v dvb-fe-tool >/dev/null 2>&1; then
	dvb-fe-tool -a 0 2>&1 || true
else
	echo "dvb-fe-tool not installed"
fi

echo "=== smitdtmb dmesg ==="
dmesg -T | grep -E 'smitdtmb|buffer negotiation|create_t_c|CI/RM|CI/SAS|SAS connect|open_session|TUNE|STATUS|TS URB|start feed|starting TS|stopping TS|APDU response|unexpected|ignoring' | tail -n 240 || true

echo "=== docker containers ==="
if command -v docker >/dev/null 2>&1; then
	docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}' || true
else
	echo "docker not installed"
fi

echo "=== tvheadend compose ==="
if [ -d /root/tvheadend ]; then
	(cd /root/tvheadend && docker compose ps 2>&1) || true
	(cd /root/tvheadend && docker compose logs --tail=160 2>&1) || true
else
	echo "/root/tvheadend not present or not readable"
fi

echo "=== listening ports ==="
if command -v ss >/dev/null 2>&1; then
	ss -ltnp 2>/dev/null | grep -E '(:9981|:9982|tvheadend|docker)' || true
else
	netstat -ltnp 2>/dev/null | grep -E '(:9981|:9982|tvheadend|docker)' || true
fi
