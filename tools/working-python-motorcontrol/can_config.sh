#/bin/bash

ip l set can2 down
ip l set can2 type can bitrate 500000
#ip l set can2 type can tq 50 prop-seg 17 phase-seg1 17 phase-seg2 5 sjw 1
ip l set can2 txqueuelen 1000
ip l set can2 up
ip -det -stat link show can1


