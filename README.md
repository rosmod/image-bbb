# image-bbb
Instructions for making a new image for the BBB that is compatible with ROSMOD and contains useful programs

# Operations

Install the latest Debian Image and ensure it has an updated linux kernel

Expand the image's file system

Install ROS

Install ROSMOD

Install KRPC and Protobuff

Install OpenCV

Install emacs

Set up SSH keys

Set up BBB cape dtbi and dtbo files

Ensure initialization scripts are present to easily updated a new BBB with the new image

# Setup

Clone this repo 

`git clone git@github.com:rosmod/image-bbb`

# Debian Image

Run `Win32DiskImager-0.9.5-install.exe` 

Download and unpack the [latest debian image.](https://beagleboard.org/latest-images) Currently using `Debian 8.5 2016-05-13 4GB SD LXQT`

Plug in an SD card with at least 8 GB, run win32DiskImager, and point it towards the image file and the sd card. Click `Write`

# Basic BBB setup

Insert the SD card into the BBB

Press the `Boot` button on the BBB (the one next to the SD card slot) while powering the BBB in order for it to boot from the SD card. Keep pressing the button until the LEDs light in order.

The rest of this readme is run **From the BBB itself**. SSH into it using the default user. Default ip if connected through usb is `192.168.7.2`

## Expand the file system

Follow the instructions [found here](http://elinux.org/Beagleboard:Expanding_File_System_Partition_On_A_microSD) for expanding the filesystem. **Be sure to expand the file system to 7GB instead of the entire SD which is the default value**

## Updating the image and installing emacs

Run `sudo apt-get update` and `sudo apt-get upgrade`

## Remove unused programs

`sudo apt-get remove apache2 avahi-daemon nodejs bonescript`

## Setup passwordless ssh login

For every user

`cat ~/.ssh/public_key >> ~/.ssh/authorized_keys`

If a public key needs to be generated from a private key

`ssh-keygen -y -f ~/.ssh/private_key >> ~/.ssh/public_key`

Disable logging in by password

`sudo emacs /etc/ssh/sshd_config`
```
ChallengeResponseAuthentication no
PasswordAuthentication no
UsePAM no
```

## Setup Fail2Ban

`sudo apt-get install fail2ban`

Default settings are fine

## Install optional programs

If desired, `sudo apt-get install emacs`

# Install ROSMOD

Follow the instruction located [in the rosmod-comm repo](https://github.com/rosmod/rosmod-comm)

# Install KRPC and Protobuff

Follow the instructions located on [the KRPC C++ documentation page](https://krpc.github.io/krpc/cpp/client.html#installing-the-library) for installing KRPC and its dependencies (ASIO and Protobuff)

Asio can be installed by running `apt-get install libasio-dev`

Follow the instructions above for instaling protobuff

# Install OpenCV

Follow the [instructions for installing OpenCV](http://docs.opencv.org/3.0-last-rst/doc/tutorials/introduction/linux_install/linux_install.html)
