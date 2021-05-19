# Pi Hw Interface
A Ros2 hardware interface that works with Raspi GPIO through libgpiod interface

Requires: gpiod, libgpiod

In order to run the executable, the system must have a set of Udev rules along with a new user group created.
Use the following steps to configure the Udev rule and accompanying groups.
1. Configure the UDEV rule 

    Note: run this command as root (use sudo su in order to get a superuser prompt)

    ```
    echo '# udev rules for gpio port access through libgpiod
    SUBSYSTEM=="gpio", KERNEL=="gpiochip[0-4]", GROUP="gpiod", MODE="0660"' > /etc/udev/rules.d/60-gpiod.rules 
    ```
2. Create the gpiod user group

    `sudo groupadd gpiod`

3. Add the current user to the gpiod group

    `sudo usermod -G gpiod <user>`


This node also relies on an updated linux kernel for the Raspi for full functionality, including handling for the pull up and pull down
resistors on the GPIO pins. A flag can be set to disable this on systems that cannot be upgraded. For full functionality, the current kernel
used is based on 5.10 and can be downloaded from github. From there take the following steps to build and upgrade the kernel from this site: 
https://www.raspberrypi.org/documentation/linux/kernel/building.md

Followed Steps to build and install the kernel:
1. Clone the kernel repo and check out the desired tag

    It is recommended to use the release tagged versions rather than just the master branch.

    `git clone --depth=1 --branch raspberrypi-kernel_1.20210430-1 https://github.com/raspberrypi/linux`

2. make the config

    For ras pi 3: `make bcmrpi3_defconfig`

    For ras pi 4: `make bcm2711_defconfig`

3. Make the kernel and its dependents

    `make -j4 Image modules dtbs`

    Note: the `-j4` allows all cpus on the pi to be used to compile, which speeds up the process significantly

4. Install the kernel






