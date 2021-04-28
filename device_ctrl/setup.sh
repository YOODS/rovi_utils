#!/bin/sh

echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"04d8\", ATTRS{idProduct}==\"ffee\", GROUP=\"dialout\", MODE=\"0666\"" > /etc/udev/rules.d/99-devantech.rules
