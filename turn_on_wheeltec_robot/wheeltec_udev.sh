#!/bin/bash
# mini_omni 实车 udev 规则 — 仅保留 STM32 底盘控制器 + 雷达

# CP2102 串口号0002 → /dev/wheeltec_controller
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
# CH9102 (安装了驱动) 串口号0002
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller2.rules
# CH9102 (未安装驱动) 串口号0002
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller3.rules

# CP2102 串口号0001 → /dev/wheeltec_lidar
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
# CH9102 (安装了驱动) 串口号0001
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar2.rules
# CH9102 (未安装驱动) 串口号0001
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar3.rules

service udev reload
sleep 2
service udev restart
