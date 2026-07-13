from Robotic_Arm.rm_robot_interface import *
import pprint

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

try:
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
    print("connected, handle id:", handle.id)

    ret = arm.rm_get_current_arm_state()
    print("current arm state:")
    pprint.pprint(ret)

finally:
    arm.rm_delete_robot_arm()
