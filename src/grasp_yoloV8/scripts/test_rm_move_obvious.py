from Robotic_Arm.rm_robot_interface import *
import pprint
import time

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

try:
    handle = arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
    print("connected, handle id:", handle.id)

    print("power state:", arm.rm_get_arm_power_state())
    print("set power on:", arm.rm_set_arm_power(1))
    time.sleep(1)

    print("clear err:", arm.rm_clear_system_err())
    time.sleep(0.5)

    ret = arm.rm_get_current_arm_state()
    print("before move:")
    pprint.pprint(ret)

    if ret[0] != 0:
        raise RuntimeError(f"get state failed: {ret[0]}")

    state = ret[1]
    err_list = state.get("err", {}).get("err", [])
    real_errors = [e for e in err_list if str(e) != "0"]
    if real_errors:
        print("当前有错误，不运动:", real_errors)
        raise SystemExit

    joints = list(state["joint"])
    target = joints.copy()

    # 只转第6轴，末端姿态变化明显，但末端位置基本不变
    target[5] += 10.0

    print("current joints:", joints)
    print("target joints:", target)

    input("确认安全，准备第6轴 +10 度，按 Enter...")

    move_ret = arm.rm_movej(target, 5, 0, 0, 1)
    print("move_ret:", move_ret)

    print("after move:")
    pprint.pprint(arm.rm_get_current_arm_state())

    print("停 5 秒，请观察末端有没有转动")
    time.sleep(5)

    input("准备回到原位，按 Enter...")

    back_ret = arm.rm_movej(joints, 5, 0, 0, 1)
    print("back_ret:", back_ret)

    print("final state:")
    pprint.pprint(arm.rm_get_current_arm_state())

finally:
    arm.rm_delete_robot_arm()
