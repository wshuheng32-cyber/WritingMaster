# coding=utf-8
"""
毛笔字轨迹提取 — 从字体渲染提取"创"字的笔画路径
输出: 机器人可执行的轨迹点（基座坐标系）

用法:
  python3 extract_char_trajectory.py               # 生成轨迹+预览图
  python3 extract_char_trajectory.py --send        # 生成轨迹+发送到机械臂执行
  python3 extract_char_trajectory.py --send --test # 只写外框轮廓测试
"""

import os
import argparse
import json
import socket
import time
import logging
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# ========== 配置参数 ==========
CHAR = "创"
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_SIZE = 400

# 书写区域（基座坐标系下）
WRITE_ORIGIN_X = -0.15
WRITE_ORIGIN_Y = -0.10
WRITE_SIZE = 0.10
WRITE_Z = 0.02

PEN_UP_Z = 0.08
PEN_DOWN_Z = 0.02
POINT_SPACING = 0.002

# 末端姿态：朝下 (欧拉角 rx=π, ry=0, rz=0 -> 四元数)
END_EULER = [3.14159, 0.0, 0.0]
END_QUAT = R.from_euler('xyz', END_EULER).as_quat()

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(TOOL_DIR, "chuang_trajectory.json")
# =================================


def render_char(char, font_path, font_size=400):
    """Render Chinese character to binary image. Returns (image, bbox)"""
    font = ImageFont.truetype(font_path, font_size)
    # Get exact bounding box first
    mask = Image.new('L', (1, 1), 0)
    bbox = ImageDraw.Draw(mask).textbbox((0, 0), char, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    pad = font_size // 4
    img_size = max(tw, th) + pad * 2
    img = Image.new('L', (img_size, img_size), 255)
    draw = ImageDraw.Draw(img)
    x = (img_size - tw) // 2 - bbox[0]
    y = (img_size - th) // 2 - bbox[1]
    draw.text((x, y), char, fill=0, font=font)
    return np.array(img), img_size


def zhang_suen(binary):
    """Zhang-Suen thinning. Returns skeleton as 0/255 uint8."""
    img = (binary < 128).astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in [0, 1]:
            marker = np.zeros_like(img)
            for i in range(1, img.shape[0] - 1):
                for j in range(1, img.shape[1] - 1):
                    if img[i, j] != 1:
                        continue
                    p2, p3 = img[i-1,j], img[i-1,j+1]
                    p4, p5 = img[i,j+1], img[i+1,j+1]
                    p6, p7 = img[i+1,j], img[i+1,j-1]
                    p8, p9 = img[i,j-1], img[i-1,j-1]
                    A = sum(1 for k, pk in enumerate([p2,p3,p4,p5,p6,p7,p8,p9])
                            if pk == 0 and [p2,p3,p4,p5,p6,p7,p8,p9][(k+1)%8] == 1)
                    B = p2+p3+p4+p5+p6+p7+p8+p9
                    cond1 = 2 <= B <= 6 and A == 1
                    if step == 0:
                        if cond1 and p2*p4*p6 == 0 and p4*p6*p8 == 0:
                            marker[i, j] = 1
                    else:
                        if cond1 and p2*p4*p8 == 0 and p2*p6*p8 == 0:
                            marker[i, j] = 1
            img[marker == 1] = 0
            changed = changed or (np.sum(marker) > 0)
    return (img * 255).astype(np.uint8)


def trace_skeleton(skeleton):
    """
    Trace skeleton to extract stroke paths.
    Returns: list of paths, each path is list of (y, x) tuples in reading order.
    """
    h, w = skeleton.shape
    binary = (skeleton > 0).astype(np.uint8)
    visited = np.zeros_like(binary)

    def neighbors(y, x):
        n = []
        for dy, dx in [(-1,0),(-1,1),(0,1),(1,1),(1,0),(1,-1),(0,-1),(-1,-1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and binary[ny, nx] and not visited[ny, nx]:
                n.append((ny, nx))
        return n

    def count_all_neighbors(y, x):
        c = 0
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dy == 0 and dx == 0: continue
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and binary[ny, nx]:
                    c += 1
        return c

    # Find all endpoints (1 neighbor)
    endpoints = []
    for y in range(h):
        for x in range(w):
            if binary[y, x]:
                c = count_all_neighbors(y, x)
                if c == 1:
                    endpoints.append((y, x))

    # Trace from every endpoint
    paths = []
    for start in endpoints:
        if visited[start]:
            continue
        path = [start]
        visited[start] = 1
        cy, cx = start

        while True:
            nbrs = neighbors(cy, cx)
            if not nbrs:
                break
            # If multiple neighbors, pick the one closest to image bottom (笔画走向)
            nbrs.sort(key=lambda p: (-p[0], p[1]))  # prefer downward, then rightward
            ny, nx = nbrs[0]
            # Check if next point is a junction (3+ total neighbors)
            if count_all_neighbors(ny, nx) >= 3 and len(path) > 3:
                visited[(ny, nx)] = 1
                path.append((ny, nx))
                break
            path.append((ny, nx))
            visited[(ny, nx)] = 1
            cy, cx = ny, nx

        paths.append(path)

    # Also trace unvisited skeleton loops/patches
    remaining = np.where((binary > 0) & (visited == 0))
    if len(remaining[0]) > 0:
        for ry, rx in zip(remaining[0], remaining[1]):
            if visited[(ry, rx)]:
                continue
            path = [(ry, rx)]
            visited[(ry, rx)] = 1
            stack = [(ry, rx)]
            while stack:
                cy, cx = stack.pop()
                for ny, nx in neighbors(cy, cx):
                    if not visited[(ny, nx)]:
                        visited[(ny, nx)] = 1
                        path.append((ny, nx))
                        stack.append((ny, nx))
            if len(path) >= 10:
                paths.append(path)

    # Keep only the longest paths (笔画), sort by position (top-to-bottom, left-to-right)
    paths.sort(key=len, reverse=True)
    paths = [p for p in paths if len(p) >= 20]

    # For "创", keep top ~8 paths and sort by position
    paths = paths[:8]

    # Sort paths by position: top components first, then left-to-right
    def path_sort_key(path):
        ys = [p[0] for p in path]
        xs = [p[1] for p in path]
        return (min(ys), min(xs))

    paths.sort(key=path_sort_key)

    # Orient each path: start from the end closest to (0, img_width) i.e. top-left
    oriented = []
    for p in paths:
        if p[0][1] > p[-1][1]:  # start x > end x → reverse
            oriented.append(list(reversed(p)))
        else:
            oriented.append(p)

    return oriented


def to_robot_coords(strokes, img_size, ox, oy, wsize, wz, spacing):
    """Convert pixel strokes to robot base coordinates."""
    result = []
    for stroke in strokes:
        pts = np.array(stroke, dtype=np.float32)  # (y, x)
        ny = pts[:, 0] / img_size  # normalized Y (pixel row)
        nx = pts[:, 1] / img_size  # normalized X (pixel col)
        rx = ox + (1.0 - nx) * wsize  # mirror X
        ry = oy + ny * wsize
        pts_3d = np.stack([rx, ry, np.full_like(rx, wz)], axis=1)
        # Resample
        sampled = resample(pts_3d, spacing)
        result.append(sampled)
    return result


def resample(points, spacing):
    """Evenly resample a path."""
    if len(points) < 2:
        return [points[0]] if len(points) == 1 else []

    diffs = np.diff(points, axis=0)
    dists = np.sqrt(np.sum(diffs ** 2, axis=1))
    cum = np.zeros(len(points))
    cum[1:] = np.cumsum(dists)
    total = cum[-1]

    if total < spacing:
        return [points[0], points[-1]]

    n = int(total / spacing) + 1
    targets = np.linspace(0, total, n)
    out = []
    for t in targets:
        idx = np.searchsorted(cum, t)
        if idx == 0:
            out.append(points[0])
        elif idx >= len(points):
            out.append(points[-1])
        else:
            s = (t - cum[idx-1]) / (cum[idx] - cum[idx-1])
            out.append(points[idx-1] + s * (points[idx] - points[idx-1]))
    return np.array(out)


def save_trajectory(strokes, path):
    data = {
        "char": CHAR,
        "num_strokes": len(strokes),
        "write_origin": {"x": WRITE_ORIGIN_X, "y": WRITE_ORIGIN_Y, "z": WRITE_Z},
        "write_size": WRITE_SIZE,
        "strokes": [{"index": i+1, "num_points": len(s), "points": np.array(s).tolist()}
                    for i, s in enumerate(strokes)]
    }
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    logger.info(f"轨迹已保存: {path}")


def visualize(strokes_robot):
    """Generate a preview image showing all strokes with numbers."""
    scale = 500 / WRITE_SIZE
    margin = 50
    side = int(WRITE_SIZE * scale + margin * 2)
    img = np.ones((side, side, 3), dtype=np.uint8) * 255

    colors = [
        (200, 0, 0), (0, 160, 0), (0, 0, 200),
        (200, 130, 0), (130, 0, 200), (0, 180, 180),
        (100, 100, 100), (200, 0, 200)
    ]

    for i, stroke in enumerate(strokes_robot):
        c = colors[i % len(colors)]
        pts = []
        for pt in stroke:
            px = int((pt[0] - WRITE_ORIGIN_X) * scale + margin)
            py = int((pt[1] - WRITE_ORIGIN_Y) * scale + margin)
            pts.append((px, py))
        for j in range(len(pts) - 1):
            cv2.line(img, pts[j], pts[j+1], c, 3)
        if pts:
            cv2.circle(img, pts[0], 7, c, -1)
            cv2.putText(img, str(i+1), (pts[0][0]+6, pts[0][1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)

    path = os.path.join(TOOL_DIR, "chuang_trajectory.png")
    cv2.imwrite(path, img)
    logger.info(f"预览图已保存: {path}")


class RobotArm:
    """ROS2方式控制RM65-B机械臂"""
    def __init__(self):
        self.pub = None
        self.node = None
        self._init_ros()

    def _init_ros(self):
        import rclpy
        from rclpy.node import Node
        from rm_ros_interfaces.msg import Movejp
        from geometry_msgs.msg import Point, Quaternion
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        rclpy.init()
        self.node = Node('calligraphy_arm')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        self.Movejp = Movejp
        self.Point = Point
        self.Quaternion = Quaternion
        self.pub = self.node.create_publisher(Movejp, '/rm_driver/movej_p_cmd', qos)
        time.sleep(1.0)  # 等待连接

    def _move(self, x, y, z, speed=20, block=True):
        msg = self.Movejp()
        msg.pose.position = self.Point(x=float(x), y=float(y), z=float(z))
        msg.pose.orientation = self.Quaternion(
            x=float(END_QUAT[0]), y=float(END_QUAT[1]),
            z=float(END_QUAT[2]), w=float(END_QUAT[3]))
        msg.speed = speed
        msg.trajectory_connect = 0
        msg.block = block
        self.pub.publish(msg)

    def move_l(self, x, y, z, speed=20, block=True):
        """直线移动到目标位姿"""
        self._move(x, y, z, speed, block)

    def move_j(self, x, y, z, speed=20, block=True):
        """关节移动到目标位姿"""
        self._move(x, y, z, speed, block)

    def close(self):
        if self.node:
            self.node.destroy_node()
        import rclpy
        rclpy.shutdown()

    def wait(self, t=2.0):
        time.sleep(t)


def execute_trajectory(arm, strokes, test=False):
    print(f"\n开始执行「{CHAR}」字 ({'测试模式' if test else '完整书写'})")

    for i, stroke in enumerate(strokes):
        print(f"  笔{i+1}: {len(stroke)}点")
        first = stroke[0]
        # 抬笔移动到起点上方
        arm.move_j(first[0], first[1], first[2] + PEN_UP_Z - PEN_DOWN_Z, speed=30)
        arm.wait(0.5)
        # 落笔到纸面
        arm.move_l(first[0], first[1], first[2], speed=10)
        arm.wait(0.3)
        # 描点
        for j, pt in enumerate(stroke[1:], 1):
            arm.move_l(pt[0], pt[1], pt[2], speed=10)
            if j % 10 == 0:
                arm.wait(0.1)  # 每10个点停一下防丢
        # 抬笔
        arm.move_l(first[0], first[1], first[2] + PEN_UP_Z - PEN_DOWN_Z, speed=20)
        arm.wait(0.3)

    print("✅ 书写完成!")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--send", action="store_true", help="发送到机械臂执行")
    parser.add_argument("--test", action="store_true", help="测试模式(只走轮廓)")
    args = parser.parse_args()

    print(f"毛笔字轨迹提取 — 「{CHAR}」")
    print(f"{'='*40}")

    # Step 1: Render
    print("\n[1/3] 渲染字体...")
    img, img_size = render_char(CHAR, FONT_PATH, FONT_SIZE)

    # Step 2: Skeletonize
    print("[2/3] 提取笔画骨架...")
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    skeleton = zhang_suen(binary)
    cv2.imwrite(os.path.join(TOOL_DIR, "chuang_skeleton.png"), skeleton)

    # Step 3: Trace
    print("[3/3] 追踪笔画路径...")
    strokes_pixel = trace_skeleton(skeleton)
    print(f"  提取到 {len(strokes_pixel)} 条笔画")

    # Convert to robot coordinates
    strokes_robot = to_robot_coords(strokes_pixel, img_size,
                                     WRITE_ORIGIN_X, WRITE_ORIGIN_Y,
                                     WRITE_SIZE, WRITE_Z, POINT_SPACING)

    # Save
    save_trajectory(strokes_robot, OUTPUT_FILE)
    visualize(strokes_robot)

    total_pts = sum(len(s) for s in strokes_robot)
    print(f"\n轨迹总览:")
    print(f"  书写区域: {WRITE_SIZE*100:.0f}cm × {WRITE_SIZE*100:.0f}cm")
    print(f"  原点: ({WRITE_ORIGIN_X:.3f}, {WRITE_ORIGIN_Y:.3f})")
    for i, s in enumerate(strokes_robot):
        f, l = s[0], s[-1]
        print(f"  笔{i+1}: {len(s)}点, ({f[0]:.3f},{f[1]:.3f})→({l[0]:.3f},{l[1]:.3f})")

    # Send to robot via ROS2
    if args.send:
        print("\n初始化ROS2控制...")
        arm = RobotArm()
        try:
            # Move to safe height
            print("移动到安全高度...")
            arm.move_j(-0.05, -0.05, 0.25, speed=20)
            arm.wait(2.0)

            if args.test:
                # Test: draw bounding box
                x, y = WRITE_ORIGIN_X, WRITE_ORIGIN_Y
                s = WRITE_SIZE
                z = WRITE_Z
                arm.move_j(x, y, z + PEN_UP_Z - PEN_DOWN_Z, speed=20)
                arm.wait(1.0)
                for bx, by in [(x+s, y), (x+s, y+s), (x, y+s), (x, y)]:
                    arm.move_l(bx, by, z, speed=10)
                    arm.wait(0.3)
                print("✅ 测试完成（外框已画出）")
            else:
                execute_trajectory(arm, strokes_robot)

            # Return to safe height
            arm.move_j(-0.05, -0.05, 0.25, speed=20)
            arm.wait(1.0)
            print("✅ 书写完成，已回到安全高度")

        except Exception as e:
            logger.error(f"执行错误: {e}")
        finally:
            arm.close()

    print("\n✅ 完成!")


if __name__ == "__main__":
    main()