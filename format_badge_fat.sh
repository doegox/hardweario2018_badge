#!/bin/bash

SCSIPATH=/sys/bus/scsi/devices
dev=""

function be_root() {
    if [[ $EUID -ne 0 ]]; then
        echo "Must be run as root!"
        exit 1;
    fi
}

function wait_plug() {
    echo -e "\n\nWaiting for device to be plugged... CTRL-C to abort"
    while :; do
        sleep 1
        for SCSI in $(ls -1 $SCSIPATH/|grep ":.*:.*:"); do
            if cat $SCSIPATH/$SCSI/vendor|grep -q "^LUFA"; then
                dev="/dev/$(ls -1 $SCSIPATH/$SCSI/block/)"
                break 2;
            fi
        done
    done
    if [ "$dev" == "" ]; then
        echo "Aborting!"
        exit 1;
    fi
}

function wait_unplug() {
    echo -e "\n\nWaiting for device to be unplugged... CTRL-C to abort"
    while :; do
        sleep 1
        found=false
        for SCSI in $(ls -1 $SCSIPATH/|grep ":.*:.*:"); do
            if cat $SCSIPATH/$SCSI/vendor|grep -q "^LUFA"; then
                found=true
                break;
            fi
        done
        if ! $found; then
            break;
        fi
    done
    if $found; then
        echo "Aborting!"
        exit 1;
    fi
}


be_root
wait_plug
echo "Found $dev"
if ! sfdisk -d $dev /dev/null ; then
    echo "Creating partitions table..."
    echo "${dev}1 : start=           1, size=        8191, type=4"|sfdisk $dev -w always -W always
    wait_unplug
    wait_plug
fi
if ! dd if=${dev}1 bs=1 count=5 skip=$((0x36)) 2>/dev/null|grep "FAT12"; then
    echo "Formatting ${dev}1"
    mkdosfs -F 12 -n "HWIO_2018" ${dev}1
fi
wait_unplug
wait_plug
echo -e "\n\nDone!"
