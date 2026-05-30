#!/usr/bin/env bash
set -euo pipefail

VIDPID="${VIDPID:-29df:0001}"
USB_DEV_PATH="${USB_DEV_PATH:-/sys/bus/usb/devices/9-2}"
PCI_DEV="${PCI_DEV:-0000:06:10.0}"
RESET_METHOD="${RESET_METHOD:-all}"

need_root() {
	if [ "$(id -u)" -ne 0 ]; then
		exec sudo -E "$0" "$@"
	fi
}

wait_for_dvb() {
	local i

	for i in $(seq 1 20); do
		if [ -e /dev/dvb/adapter0/frontend0 ]; then
			return 0
		fi
		sleep 0.5
	done

	return 1
}

reset_usb_device() {
	if command -v usbreset >/dev/null 2>&1; then
		usbreset "$VIDPID"
		return
	fi

	if [ -e "$USB_DEV_PATH/authorized" ]; then
		echo 0 >"$USB_DEV_PATH/authorized"
		sleep 1
		echo 1 >"$USB_DEV_PATH/authorized"
		return
	fi

	return 1
}

reset_controller() {
	local drv_path
	local drv_name

	drv_path="$(readlink -f "/sys/bus/pci/devices/$PCI_DEV/driver")"
	drv_name="$(basename "$drv_path")"

	echo "$PCI_DEV" >"$drv_path/unbind"
	sleep 1
	echo "$PCI_DEV" >"/sys/bus/pci/drivers/$drv_name/bind"
}

reset_pci_rescan() {
	echo 1 >"/sys/bus/pci/devices/$PCI_DEV/remove"
	sleep 3
	echo 1 >"/sys/bus/pci/rescan"
}

need_root "$@"

pkill -x dvbv5-scan 2>/dev/null || true
pkill -x dvbv5-zap 2>/dev/null || true
modprobe -r smitdtmb 2>/dev/null || true

case "$RESET_METHOD" in
usbreset)
	reset_usb_device
	;;
authorized)
	if [ -e "$USB_DEV_PATH/authorized" ]; then
		echo 0 >"$USB_DEV_PATH/authorized"
		sleep 1
		echo 1 >"$USB_DEV_PATH/authorized"
	else
		exit 1
	fi
	;;
controller)
	reset_controller
	;;
pci-rescan)
	reset_pci_rescan
	;;
all)
	reset_usb_device || true
	sleep 1
	if [ -e "$USB_DEV_PATH/authorized" ]; then
		echo 0 >"$USB_DEV_PATH/authorized" || true
		sleep 1
		echo 1 >"$USB_DEV_PATH/authorized" || true
	fi
	sleep 2
	reset_controller || true
	sleep 2
	reset_pci_rescan || true
	;;
*)
	echo "unknown RESET_METHOD=$RESET_METHOD" >&2
	exit 2
	;;
esac

sleep 2
modprobe smitdtmb
wait_for_dvb

lsusb -d "$VIDPID"
ls -l /dev/dvb/adapter0
