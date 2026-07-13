from Robotic_Arm.rm_robot_interface import *
import pprint
import time

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

try:
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
    print("connected, handle id:", handle.id)

    print("before clear:")
    pprint.pprint(arm.rm_get_current_arm_state())

    ret = arm.rm_clear_system_err()
    print("clear error ret:", ret)

    time.sleep(1)

    print("after clear:")
    pprint.pprint(arm.rm_get_current_arm_state())

finally:
    arm.rm_delete_robot_arm()
