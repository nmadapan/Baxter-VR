# Baxter-VR
Controlling the Baxter robot using Virtual reality device (HTC VIVE). It requires two computers: Windows 10 (Client) and Ubuntu 14.04 (Server). Client computer connects to VR hardware and extracts complete VIVE controller status and sends this data to the Server. The server computer receives the controller information, transforms into Baxter coordinate system, estimates inverse kinematics solutions of Baxter and sends joint increments to Baxter core. 

# VIVE controller information
triad_openvr is an open source software built on top of the openvr library. It provides a list of basic functions to access controller information, touch pad coordinates, trigger status, etc. Link: https://github.com/TriadSemi/triad_openvr.git. <br />
In my project, I used triad_openvr library to extract VIVE controller information (Code is present in 'Client_VR' directory)

# Windows Computer
0. Check for the computer hardware requrirements [here] (http://store.steampowered.com/app/323910/SteamVR_Performance_Test/)
1. Install [Steam VR](https://support.steampowered.com/kb_article.php?ref=2001-UXCM-4439) and [Epics Launcher - Unreal Engine] (https://www.epicgames.com/unrealtournament/download). 
2. Install the headset and controllers properly. 
3. Download the 'Client_VR' folder present in this repository. 
4. Run the 'controller_test.py'. If it runs without errors, then VIVE hardware is successfully installed. 
5. Open 'main.py', change the 'TCP_IP' variable to the IP address of your Ubuntu computer. 
6. Run 'main.py'

# Ubuntu Computer
0. Ubuntu 14.04 is recommended. 
1. Install [ROS Indigo] (http://wiki.ros.org/indigo/Installation/Ubuntu), [Baxter Workstation Setup] (http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
2. Download the 'Server_Baxter' folder present in this repository. Copy this folder into catkin_ws/src/.
3. Open 'main.py', and change the 'TCP_IP' variable to the IP address of Ubuntu computer. 
4. Run 'main.py'

Note: These instructions does not include live streaming the first view of Baxter on to the VIVE headset. In order to do this, you need to mount a camera on Baxter's head, connect it to Windows computer. Use the Unreal engine (installed in the Windows computer) to live stream webcam in the VIVE headset. 

# Video
https://youtu.be/GOF_M8wtWZE <br />
https://youtu.be/dS_DJDMJGNU
