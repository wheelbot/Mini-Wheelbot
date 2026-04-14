
# Cloning and first build
Clone the repo:
```
git clone --recursive git@git.rwth-aachen.de:dsme-projects/wheelbot/wheelbot-buildroot.git
cd wheelbot-buildroot/buildroot
```

Configure buildroot
```
make defconfig BR2_DEFCONFIG=../wheelbot_tree/configs/wheelbot_cm4_defconfig
```

Now you can build for the first time. This usually takes about 20-60min:
```
make BR2_EXTERNAL=/path/to/wheelbot-buildroot/wheelbot_tree
```

After first build, next builds are faster!

# Flashing the eMMC on the CM4
Put the CM4 on the official IO board. On J2, fit the jumper to disable eMMC boot.

You'll need [rpiboot](https://github.com/raspberrypi/usbboot) to mount the eMMC. First, call
```
rpiboot
```

Then power on the CM4. You should see output like:
```
$ rpiboot
RPIBOOT: build-date Jul 10 2023 version 20221215~105525 c4b12f85
Waiting for BCM2835/6/7/2711...
Loading embedded: bootcode4.bin
Sending bootcode.bin
Successful read 4 bytes
Waiting for BCM2835/6/7/2711...
Loading embedded: bootcode4.bin
Second stage boot server
Cannot open file config.txt
Cannot open file pieeprom.sig
Loading embedded: start4.elf
File read: start4.elf
Cannot open file fixup4.dat
Second stage boot server done
```

And with `lsblk`, a disk called `sdX` should appear.

Now you can use the copying tool of your trust, e.g., `dd` (beware, `dd` is dangerous):
```
cd output/images
dd if=sdcard.img of=/dev/sdX bs=64k status=progress && sync
```

# Accessing the Pi
While still on IO board, you can hook up monitor, mouse and keyboar.

Once you place the CM4 on the wheelbot compute carrier, you can only access over WIFI, i.e. the `botnet` Wifi via `ssh`:
```
ssh root@192.168.10.XXX
```


# Notes

```
rm -rf output/build/linux-* && rm -rf output/target/root/wheelbot-ros && make defconfig BR2_DEFCONFIG=../wheelbot_tree/configs/wheelbot_cm4_defconfig && make BR2_EXTERNAL=/home/hose/projects/dsme/wheelbot-buildroot/wheelbot_tree

rm -rf output/build/linux-* && rm -rf output/target/root/wheelbot-ros && make defconfig BR2_DEFCONFIG=../wheelbot_tree/configs/wheelbot_cm4_defconfig && make linux-menuconfig CONFIG_EXPERT=y
```