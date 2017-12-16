#!/bin/bash

# Unmount the SSDs if they are mounted

sudo umount /dev/sda
sudo umount /dev/disk/by-label/pod2dataA
sudo umount /dev/sdc
sudo umount /dev/disk/by-label/pod2dataB

## Mound disk B (currently on ACS)
sudo mount -t ntfs /dev/disk/by-label/pod2dataB /media/pod2data/ -o uid=1000,gid=1000,utf8,dmask=027,fmask=137 