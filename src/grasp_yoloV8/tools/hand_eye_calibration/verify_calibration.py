# coding=utf-8
"""
标定精度验证脚本（眼在手上 Eye-in-Hand）

原理：
  使用棋盘格固定不动，机械臂在不同姿态下拍摄，通过手眼标定结果
  将棋盘格角点从相机坐标系转换到基座坐标系，比较一致性。

用法：
  python3 verify_calibration.py
  s — 采集 | q — 退出

验证结果（2026-07-08）:
  平均误差 25.6mm，主要问题在Z轴（49.5mm），XY约15mm
  结论：精度一般，可写10cm以上大字
"""

import os
import json
import socket
import time
import logging

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs

# ---------- 配置 ----------
# 手眼标定结果（眼在手上：相机→末端）
ROTATION_MATRIX = np.array([
    [-0.02626938, -0.99964788,  0.00374701],
    [ 0.99920019, -0.0261442,   0.03025652],
    [-0.0301479,   0.00453883,  0.99953514]
])

TRANSLATION_VECTOR = np.array([0.09392612, -0.0322434, 0.00873646])

# 棋盘格参数
CHESSBOARD_XX = 11
CHESSBOARD_YY = 8
CHESSBOARD_L = 0.015  # 米

# 机械臂
ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080
# ------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)


def send_cmd(client, cmd, get_pose=True):
    client.send(cmd.encode('utf-8'))
    if not get_pose:
        time.sleep(0.05)
        client.recv(1024).decode('utf-8')
        return True
    time.sleep(0.1)
    response = client.recv(4096).decode('utf-8')
    try:
        decoder = json.JSONDecoder()
        data_list = []
        index = 0
        while index < len(response):
            while index < len(response) and response[index].isspace():
                index += 1
            if index >= len(response):
                break
            obj, idx = decoder.raw_decode(response[index:])
            data_list.append(obj)
            index += idx
        target_data = None
        for data in reversed(data_list):
            if data.get("state") == "current_arm_state":
                target_data = data
                break
        if not target_data:
            return False, "未找到有效的机械臂状态响应"
        if target_data["arm_state"]["err"] != [0]:
            return False, f"机械臂报错: {target_data['arm_state']['err']}"
        pose_raw = target_data["arm_state"]["pose"]
        pose_converted = [
            pose_raw[0] / 1000000.0,
            pose_raw[1] / 1000000.0,
            pose_raw[2] / 1000000.0,
            pose_raw[3] / 1000.0,
            pose_raw[4] / 1000.0,
            pose_raw[5] / 1000.0
        ]
        return True, pose_converted
    except Exception as e:
        return False, f"处理响应出错: {str(e)}"


def connect_robot(ip, port):
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        client.connect((ip, port))
        cmd = '{"command":"set_change_work_frame","frame_name":"Base"}'
        send_cmd(client, cmd, get_pose=False)
        return client
    except Exception as e:
        logger.error(f"机械臂连接失败: {e}")
        return None


def detect_chessboard(color_image, depth_image, camera_matrix):
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    ret, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_XX, CHESSBOARD_YY), None)
    if not ret:
        return None, None, None
    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    cv2.drawChessboardCorners(color_image, (CHESSBOARD_XX, CHESSBOARD_YY), corners2, ret)
    objp = np.zeros((CHESSBOARD_XX * CHESSBOARD_YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_XX, 0:CHESSBOARD_YY].T.reshape(-1, 2)
    objp = CHESSBOARD_L * objp
    depth_h, depth_w = depth_image.shape
    pts_3d = []
    for pt in corners2[0]:
        u, v = int(pt[0]), int(pt[1])
        if u < 0 or u >= depth_w or v < 0 or v >= depth_h:
            pts_3d.append([0, 0, 0])
            continue
        depth_roi = depth_image[max(0,v-2):min(depth_h,v+3), max(0,u-2):min(depth_w,u+3)]
        valid_depths = depth_roi[depth_roi > 0]
        if len(valid_depths) == 0:
            pts_3d.append([0, 0, 0])
            continue
        z = np.median(valid_depths) / 1000.0
        fx, fy = camera_matrix[0,0], camera_matrix[1,1]
        cx, cy = camera_matrix[0,2], camera_matrix[1,2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts_3d.append([x, y, z])
    return corners2, np.array(pts_3d, dtype=np.float32), objp


def cam_to_base(pts_in_camera, end_effector_pose):
    T_cam2end = np.eye(4)
    T_cam2end[:3, :3] = ROTATION_MATRIX
    T_cam2end[:3, 3] = TRANSLATION_VECTOR
    x, y, z, rx, ry, rz = end_effector_pose
    R_end2base = R.from_euler('xyz', [rx, ry, rz], degrees=False).as_matrix()
    t_end2base = np.array([x, y, z])
    T_end2base = np.eye(4)
    T_end2base[:3, :3] = R_end2base
    T_end2base[:3, 3] = t_end2base
    T_cam2base = T_end2base @ T_cam2end
    pts_homo = np.ones((len(pts_in_camera), 4))
    pts_homo[:, :3] = pts_in_camera
    pts_base_homo = (T_cam2base @ pts_homo.T).T
    return pts_base_homo[:, :3]


def main():
    print("=" * 60)
    print("手眼标定精度验证 — 眼在手上 (Eye-in-Hand)")
    print("=" * 60)
    print()
    print("操作：s=采集  q=退出\n")

    client = connect_robot(ROBOT_IP, ROBOT_PORT)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        pipeline.start(config)
        for _ in range(10):
            pipeline.wait_for_frames()
        profile = pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float64)
    except Exception as e:
        logger.error(f"相机启动失败: {e}")
        pipeline.stop()
        if client:
            client.close()
        return

    collected_data = []
    count = 0
    cv2.namedWindow("验证标定", cv2.WINDOW_NORMAL)

    try:
        consecutive_timeouts = 0
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                consecutive_timeouts += 1
                if consecutive_timeouts >= 3:
                    break
                continue
            consecutive_timeouts = 0
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            corners, corners_3d, objp = detect_chessboard(color_image, depth_image, camera_matrix)
            display = color_image.copy()
            if corners is not None:
                cv2.putText(display, "Chessboard OK! Press 's'",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display, "No chessboard detected",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("验证标定", display)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('s') and corners is not None:
                count += 1
                if client is not None:
                    success, pose = send_cmd(client, '{"command": "get_current_arm_state"}')
                    if not success:
                        continue
                else:
                    print("手动输入末端位姿 [x y z rx ry rz]:")
                    inp = input("> ")
                    pose = [float(v) for v in inp.strip().split()]
                pts_base = cam_to_base(corners_3d, pose)
                first_corner_base = pts_base[0]
                first_corner_cam = corners_3d[0]
                logger.info(f"采集第{count}组")
                logger.info(f"  末端位姿: {pose}")
                logger.info(f"  棋盘格第1角点（基座系）: {first_corner_base}")
                logger.info(f"  棋盘格第1角点（相机系）: {first_corner_cam}")
                collected_data.append({'index': count, 'end_pose': pose, 'first_corner_base': first_corner_base})
            elif key == ord('q') or key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if client:
            client.close()

    # 计算结果...（略，详见现场输出）
    print(f"\n采集了 {len(collected_data)} 组数据")
    if len(collected_data) >= 2:
        errors = []
        for i in range(len(collected_data)):
            for j in range(i+1, len(collected_data)):
                err = np.linalg.norm(collected_data[i]['first_corner_base'] - collected_data[j]['first_corner_base'])
                errors.append(err)
        print(f"平均误差: {np.mean(errors)*1000:.1f} mm")
        print(f"最大误差: {np.max(errors)*1000:.1f} mm")
        out_path = os.path.join(os.path.dirname(__file__), "calibration_verify_result.txt")
        with open(out_path, 'w') as f:
            f.write(f"平均误差: {np.mean(errors)*1000:.1f} mm\n")
            f.write(f"最大误差: {np.max(errors)*1000:.1f} mm\n")
        logger.info(f"结果已保存到 {out_path}")


if __name__ == '__main__':
    main()
