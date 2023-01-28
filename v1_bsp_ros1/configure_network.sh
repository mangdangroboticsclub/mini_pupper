#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <ssid> <wifi password>"
    exit 1
fi

cat > /tmp/mini-pupper.yaml << EOF
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    wifis:
        wlan0:
            access-points:
                $1:
                    password: "$2"
            dhcp4: true
            optional: true
    version: 2
EOF

sudo rm -f /etc/netplan/*
sudo cp /tmp/mini-pupper.yaml /etc/netplan/
