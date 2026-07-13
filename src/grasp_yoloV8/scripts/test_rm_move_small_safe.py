from Robotic_Arm.rm_robot_interface import *
import pprint
import time

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

def has_real_error(state):
    err_info = state.get("err", {})
    err_list = err_info.get("err", [])
    # SDK 有时返回 ['0'] 表示无错误
    real_errors = [e for e in err_list if str(e) != "0"]
    return real_errors

arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

try:
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
    print("connected, handle id:", handle.id)

    state_ret = arm.rm_get_current_arm_state()
    print("current state:")
    pprint.pprint(state_ret)

    if state_ret[0] != 0:
        raise RuntimeError(f"get state failed, ret={state_ret[0]}")

    state = state_ret[1]
    errors = has_real_error(state)

    if errors:
        print("机械臂当前仍有错误，不执行运动:", errors)
        raise SystemExit

    joints = list(state["joint"])
    print("current joints:", joints)

    target = joints.copy()
    target[-1] += 2.0

    print("target joints:", target)

    input("确认机械臂周围安全，急停在手边。准备最后一个关节 +2 度，按 Enter 继续...")

    move_ret = arm.rm_movej(target, 5, 0, 0, 1)
    print("move_ret:", move_ret)

    time.sleep(1)

    input("准备回到原位，按 Enter 继续...")

    back_ret = arm.rm_movej(joints, 5, 0, 0, 1)
    print("back_ret:", back_ret)

    print("final state:")
    pprint.pprint(arm.rm_get_current_arm_state())

finally:
    arm.rm_delete_robot_arm()
