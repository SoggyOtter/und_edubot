# UND_EDUBot Overview

[Build Instructions Playlist](https://www.youtube.com/playlist?list=PLYcNE-BB9RCs-w1YGQMMMpfctwIbQ31T8)

# Build the robot

The parts list for the is in the parts list excel file.  Purchase the needed components to build the robot.

If you are building the robot with others several people can build one robot with multi packs of screws and wire.

The CAD models for printing are found in the /build/CAD Models directory.  The .3mf files can be imported directly to the slicer of your choosing for printing.  It is reccommended to use support material, however, acceptable results can be achieved without supports.

Soldering and assembly of the electrical components should be done following the wiring diagram found in the wiring diagram power point in the /build directory

## Tools Needed:

- 3D printer
- Soldering iron & solder
- Metric hex key set
- Small flat head screwdriver
- Small pliers

# Build Instructions

Build instructions will be provided by youtube videos which will be posted.  Following the video instructions and diagrams provide all the information needed to build the robot.

# Powering up for the first time

Once the robot is built you can power up the pi with a flashed micro SD card.

In the /software/Pi Image directory the image for the raspberry pi is provided.  Download the .img file and use the Raspberry Pi Imager to flash the micro SD card with the image.

TODO add images

Select Pi 5 for the pi version

Click on operating system, scroll down and select use custom.  Then find the .img file you downloaded.

Click choose storage and select the micro SD card.  You will be prompted to set the host name and password, choose a name and password you will remember.

# UND Students Pi Image Files

Duplicate the SD card from https://ndusbpos-my.sharepoint.com/:u:/g/personal/john_merila_ndus_edu/EXtqe4LofwhBoTBZhCYkA34BFa9aUKt46IkEdTdlvLatog?e=Mr8oee and follow the steps to flash a SD card

I am looking at alternative hosting or compression methods to allow for an image that can be hosted on github.

# Getting the robot running
{::comment}
The EDUBot uses ROS2 in a Docker container to manage the control of the robot.

Once the pi powers on open VS code, the easiest way is to open a terminal and run



```bash
code .
```

Once in vs code open the PiBot directory using file -> open folder -> PiBot

With the PiBot folder open use the hotkey ctrl+shift+p and type open in container.  This opens the docker container and allows you to control the robot.

BASIC STARTUP
Basic Startup:
{:/comment}
1. open terminal and open VS code

code .

2.
file -> open folder
select pibot_repo
click on pop up Reopen in container
if no pop up ctrl-shift-p reopen in container
if it fails ctrl-shift-p rebuild and reopen container

3.

Follow readme located in ws/src/PiBot/Readme.txt
