import triad_openvr
import time
import sys

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[0])
else:
    print("Invalid number of arguments")
    interval = False

controller_name = 'controller_2'
    
print(v.devices[controller_name].get_pose_euler())
# if interval:
#     while(True):
#         start = time.time()
#         txt = ""
#         for each in v.devices["controller_1"].get_pose_euler():
#             txt += "%.4f" % each
#             txt += " "
#         print("\r" + txt, end="")
#         sleep_time = interval-(time.time()-start)
#         if sleep_time>0:
#             time.sleep(sleep_time)

while True:
    state_id, state = v.vr.getControllerState(v.devices[controller_name].index)
    print('Controller State',state_id, state.ulButtonPressed, state.ulButtonTouched, state.unPacketNum)
    print(state.rAxis[0].x, state.rAxis[0].y)
    time.sleep(1)
