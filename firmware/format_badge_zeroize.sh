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

be_root
wait_plug
echo "Found $dev"
umount ${dev}1
# Remove count to erase it all, it takes a long time!
dd if=/dev/zero of=${dev}1 bs=512 count=100
echo -e "\n\nDone!"
