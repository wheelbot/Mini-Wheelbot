#!/bin/sh

BOARD_DIR="$(dirname $0)"

set -u
set -e

cp package/busybox/S10mdev ${TARGET_DIR}/etc/init.d/S10mdev
chmod 755 ${TARGET_DIR}/etc/init.d/S10mdev

cp package/busybox/mdev.conf ${TARGET_DIR}/etc/mdev.conf
cp ${BOARD_DIR}/../common/interfaces ${TARGET_DIR}/etc/network/interfaces
cp ${BOARD_DIR}/../common/wpa_supplicant.conf ${TARGET_DIR}/etc/wpa_supplicant.conf

cp ${BOARD_DIR}/../common/expandfs.sh ${TARGET_DIR}/etc/init.d/expandfs.sh
chmod +x ${TARGET_DIR}/etc/init.d/expandfs.sh

cp ${BOARD_DIR}/../common/S99dsme ${TARGET_DIR}/etc/init.d/S99dsme
chmod +x ${TARGET_DIR}/etc/init.d/S99dsme

cp -r ${BOARD_DIR}/../common/chrony ${TARGET_DIR}/etc/

cp ${BOARD_DIR}/../common/bashrc ${TARGET_DIR}/root/.bashrc
cp ${BOARD_DIR}/../common/profile ${TARGET_DIR}/root//../common.profile

# exnsure overlays exists for genimage
mkdir -p "${BINARIES_DIR}/rpi-firmware/overlays"
