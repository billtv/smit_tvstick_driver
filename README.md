# smitdtmb driver

This repository contains a Linux DVB driver for the SMIT USB DTMB stick.

## Ubuntu install

Install the build dependencies:

```sh
sudo apt update
sudo apt install git dkms build-essential clang llvm lld linux-headers-$(uname -r) linux-modules-extra-$(uname -r)
```

Clone the repository and install it with DKMS:

```sh
git clone https://github.com/billtv/smit_tvstick_driver.git
cd smit_tvstick_driver

sudo install -d /usr/src/smitdtmb-0.1
sudo cp -a Makefile dkms.conf smitdtmb.c /usr/src/smitdtmb-0.1/
sudo dkms add -m smitdtmb -v 0.1
sudo dkms build -m smitdtmb -v 0.1
sudo dkms install -m smitdtmb -v 0.1
```

Load the module:

```sh
sudo modprobe smitdtmb
```

## Verify

After the module loads, the DVB device should appear under `/dev/dvb/adapter0/`.

For tvheadend, select that adapter and frontend in the tvheadend configuration.

If Secure Boot is enabled, Ubuntu may require enrolling the DKMS signing key
before the module can load.

## Remove

```sh
sudo dkms remove -m smitdtmb -v 0.1 --all
```

## License

This project is licensed under the GNU General Public License v2.0 or later.
