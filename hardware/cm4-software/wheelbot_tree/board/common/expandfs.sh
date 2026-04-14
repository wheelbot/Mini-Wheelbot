#!/bin/sh

do_expand_rootfs() {
  ROOT_PART="$(findmnt / -o source -n)"
  ROOT_DEV="/dev/$(lsblk -no pkname "$ROOT_PART")"

  PART_NUM="$(echo "$ROOT_PART" | grep -o "[[:digit:]]*$")"

  LAST_PART_NUM=$(parted "$ROOT_DEV" -ms unit s p | tail -n 1 | cut -f 1 -d:)
  if [ "$LAST_PART_NUM" -ne "$PART_NUM" ]; then
    echo '#####'
    echo "$ROOT_PART is not the last partition. Don't know how to expand" 20 60 2
    echo '#####'
    return 0
  fi

  # Get the starting offset of the root partition
  PART_START=$(parted "$ROOT_DEV" -ms unit s p | grep "^${PART_NUM}" | cut -f 2 -d: | sed 's/[^0-9]//g')
  [ "$PART_START" ] || return 1
  # Return value will likely be error for fdisk as it fails to reload the
  # partition table because the root fs is mounted
  fdisk "$ROOT_DEV" <<EOF
p
d
$PART_NUM
n
p
$PART_NUM
$PART_START

p
w
EOF
  echo "## Will resize filesystem now ##"
  resize2fs "$ROOT_PART"
  echo "## Expanding done ##"
}


do_expand_rootfs
printf "Rebooting now!\n"
mv /etc/init.d/expandfs.sh /etc/init.d/expandfs.sh.done
reboot
exit 0
