# image-bbb
Instructions for making a new image for the BBB that is compatible with ROSMOD and contains useful programs

# Order of Operations

1. Install the latest Debian Image and ensure it has an updated linux kernel
2. Expand the image's file system
3. Install ROS
4. Install ROSMOD
5. Install KRPC and Protobuff
6. Install OpenCV
7. Set up SSH keys
8. Set up BBB cape dtbi and dtbo files
9. Ensure initialization scripts are present to easily updated a new BBB with the new image

# Setup

Clone this repo 

`git clone git@github.com:rosmod/image-bbb`

# Debian Image

Run 
