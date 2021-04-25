# Connecting Multiple Bebop 2's to Linux (Deprecated)
Attempts were made to connect the Bebop 2 to a network rather than the host connecting to the Bebop 2's ad-hoc network to enable multiple Bebop 2 control. This was ultimately unsuccessful. This page will be retained as an archive for future work.

## Reverting to a older Bebop 2 Firmware
[Archived older Bebop 2 Firmware](https://www.drone-forum.com/forum/viewtopic.php?t=9764)

[Youtube Video Demonstration](https://youtu.be/mSMJF7Y-E6I)

1. Download older Firmware

2. Save to a USB Stick formatted in FAT32 with the name ```bebopdrone_usb_update```.

3. Turn on Bebop 2 drone with full battery

4. Plug in USB stick to Bebop 2 micro USB slot with OTG cable

5. Wait 8 minutes. DO NOT do anything to the Bebop 2. It will not show any signs of updating and you should not attempt to connect any applications to the Bebop 2 to check progress.

6. After 8 minutes, unplug the USB stick and restart the Bebop 2

## Installing Prerequisites

### Installing ADB
The Bebop 2 runs on android and can be reconfigured with a USB connection. ADB (Android Debug Bridge) is required to interface with the Bebop 2. This will primarily be used to install the custom scripts to enable the Bebop 2 to connect to wireless networks rather than a host PC connect to a singular Bebop 2.

Source: [Install ADB & Fastboot on Ubuntu 18.04 / Linux Mint 19](https://computingforgeeks.com/install-adb-fastboot-on-ubuntu-mint/)
```
sudo apt update
sudo apt-get install android-tools-adb
```
To get ADB version details
```
adb version
```
To get a list of connected devices. This will also run the adb server if it has not already started.
```
adb devices
```

### Installing Net Tools for inspection
This is used to help in various debugging and configuration for the Bebop 2.
```
sudo apt install net-tools
```

## Enabling Multiple Bebop capability (Quick)
The repository by tnaegeli provides instructions and files required to be configured and loaded onto the Bebop 2.
```
cd ~
git clone https://github.com/tnaegeli/multiple_bebops.git
```

1. Connect to the Bebop 2
    Connect to the drone (in default configuration)
    ```
    adb connect 192.168.43.1:9050
    ```
2. Edit the shell script `shortpress_3.sh` on the host PC. Fill in the `ssid`, `password` and `IP` (Local) for the drone.
3. Run the automated script which loads and sets permissions within the Bebop 2 firmware.
    ```
    ./copy_files.sh
    ```
4. Reboot the Bebop 2 and then press the power button 3 times to hear a beep. After 10-30s the connection should be establised and you can ping the drone.

## Enabling Multiple Bebop capability (Manual)
The repository by tnaegeli provides instructions and files required to be configured and loaded onto the Bebop 2.
```
cd ~
git clone https://github.com/tnaegeli/multiple_bebops.git
```
1. Connect to the Bebop 2
    Connect to the drone (in default configuration)
    ```
    adb connect 192.168.43.1:9050
    ```
    Make the partition writable
    ```
    adb shell mount -o remount,rw /
    ```

2. Add the files from github to the drone
    
    Use `adb push` to copy each of the files into the Bebop 2

    ```
    cd multiple_bebops
    adb push libiw.so /lib/libiw.so
    adb push libiw.so.29 /lib/libiw.so.29
    adb push ifrename /sbin/ifrename
    adb push iwconfig /sbin/iwconfig
    adb push iwevent /sbin/iwevent
    adb push iwgetid /sbin/iwgetid
    adb push iwlist /sbin/iwlist
    adb push iwpriv /sbin/iwpriv
    adb push iwspy /sbin/iwspy
    adb push wpa_cli /bin/wpa_cli
    adb push wpa_passphrase /bin/wpa_passphrase
    adb push wpa_supplicant /bin/wpa_supplicant
    ```

3. Enable Executable
    Switch to the Bebop 2 shell in adb. This is essentially a file explorer
    ```
    adb shell
    ```
    Navigate to the folders `/lib`, `/bin`, `/sbin` with `cd <path>` to change the mode and execute using `chmod 777 <filename>`
4. In a new terminal, run the script `connect` with the following arguments:
    ```
    ./connect "<essid>" -p "<password>" [-a <address>] [-d <droneip>]
    ```
    - `<essid>` Name of the WPA2 network the drone will connect to.

    - `<password>` Password of the network.

    - `<address>` (Optional) IP address that will be set on drone when connected. Set to "auto" or "dhcp" to automatically assign an IP.

    - `<droneip>` (Optional) Current drone's IP address.

    In this instance, we will use:
    ```
    ./connect "Radiance" -p "nancenuchaAeuD1" [-a 192.168.0.123]

    ./connect "Shironokuro" -p "nancenuchaAeuD6" [-a 192.168.0.123]
    ```


## Translated from [Multiple_Bebops](https://github.com/guozhenglong/multiple_bebops/)

Setting up connection to the router.

If you have previously set up to connect to another router, you will need to restore the factory settings first. All settings are performed in a Linux environment.
To restore the factory settings of bebop2.

  1. turn on the power.
  2. wait until the LEDs stop blinking and then press and hold the power button for about 10 seconds.
  3. next bebop2 will reboot automatically. Then the power LED will start blinking and this process may last for a few minutes.

Pre-configure the environment and prepare.

  1. install the android debug tool, type sudo apt-get install android-tools-adb -y in the Linux terminal and press enter.
  2. edit the file shortpress_3.sh, SSID is modified to your router wifi name, PW is WiFi password, IP is the new ip address of bebop, generally set to 192.168.1.x, x can be any three digits; then save the file.

Operation steps.

 1. turn on the power.
 2. connect to the computer with the USB cable.
3. press the power button four times continuously.
 4. type ifconfig in the Linux terminal, you should see a new network interface called usb0 (the new plane is also called by another name, a string ending in 0) ; if it does not appear, repeat 1 to 3.
 5. to connect to bebop2, type adb connect 192.168.43.1:9050 in the terminal.
 6. After successful connection, type in the terminal . copy_files.sh ;
 7. 6 completed, unplug the USB cable, shut down the computer, after rebooting (LED no longer blinking) press the power button three times in a row, you will hear a zipping sound, wait about 10 to 30 seconds.
 8. the computer side can ping 192.168.1.x to check whether the connection is successful.


Translated with www.DeepL.com/Translator (free version)