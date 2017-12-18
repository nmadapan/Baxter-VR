# Baxter-VR
Controlling the Baxter robot using Virtual reality device (HTC VIVE). It requires two computers: Windows 10 (Client) and Ubuntu 14.04 (Server). Client computer connects to VR hardware and extracts complete VIVE controller status and sends this data to the Server. The server computer receives the controller information, transforms into Baxter coordinate system, estimates inverse kinematics solutions of Baxter and sends joint increments to Baxter core. 

# VIVE controller information
triad_openvr is an open source software built on top of the openvr library. It provides a list of basic functions to access controller information, touch pad coordinates, trigger status, etc. Link: https://github.com/TriadSemi/triad_openvr.git. <br />
In my project, I used triad_openvr library to extract VIVE controller information (Code is present in 'Client_VR' directory)

# How to run
1. Install SteamVR (http://store.steampowered.com/app/323910/SteamVR_Performance_Test/) and Unreal 
Copy the Client_VR folder

# Video
https://youtu.be/GOF_M8wtWZE <br />
https://youtu.be/dS_DJDMJGNU
