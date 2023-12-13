#!/bin/bash

#This installer downloads and installs the components required to preform automated commandline stitching on a Raspberry Pi. 

# Install the Raspberry Pi GPIO
echo "Installing Raspberry Pi GPIO library"
sudo apt-get install python-rpi.gpio
echo "Completed RPi.GPIO install"

#Download and install Hugin Command Line Tools
echo "Installing Hugin Command Line Tools"
sudo apt-get install hugin-tools
# cd ~/Downloads/ 
# wget "http://http.us.debian.org/debian/pool/main/h/hugin/hugin-tools_2020.0.0+dfsg-2_armhf.deb"
# sudo dpkg -i /home/pi/Downloads/hugin-tools_2020.0.0+dfsg-2_armhf.deb 
# sudo apt-get install -f
echo "Completed Hugin Tools install" #This doesn't actually check if the installation was successful, so it is possible for the install to fail but this message to still appear. 


#Download and install enblend
echo "Installing enblend"
sudo apt-get install enblend
echo "Completed enblend install"  #This doesn't actually check if the installation was successful, so it is possible for the install to fail but this message to still appear. 


#Create software directory
mkdir ~/Time-Lapse-Software

#Create software directory for StereoPi
mkdir ~/Time-Lapse-Software/StereoPi

#Create photo directory
mkdir ~/Time-Lapses


#Move files to correct location
cp run-timelapse.sh ~/Time-Lapse-Software
cp stitch-pictures.sh ~/Time-Lapse-Software
cp take-pictures.py ~/Time-Lapse-Software
cp template.pto ~/Time-Lapse-Software

#Move StereoPi files to correct location
cp StereoPi/*.* ~/Time-Lapse-Software/StereoPi


#INCLUDE SAMPLE PHOTOS
# mv "sample photos" ~/Time-Lapse-Software/

# Making scripts executable
cd ~/Time-Lapse-Software/
chmod +x *.sh

# Making scripts executable
cd ~/Time-Lapse-Software/StereoPi/
chmod +x *.sh

echo "Install complete. Navigate to ~/Time-Lapse-Software and run 'bash run-timelapse.sh' to start taking a time-lapse."