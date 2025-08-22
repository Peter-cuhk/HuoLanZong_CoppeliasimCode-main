# -*- coding: utf-8 -*-
"""
连接 CoppeliaSim 4.7.0（ZeroMQ Remote API），读取 Kinect（或任意 Vision Sensor）RGB 图像，
遍历场景中名称包含“Cuboid”的 Shape，获取其世界系中心并投影到图像上，用红色加号标记。

使用方法：
1) 打开 CoppeliaSim 场景，确保有一个 Vision Sensor（作为 Kinect RGB），并运行仿真。
2) 安装/配置 Python 客户端：
   - 方法A（推荐）：pip install coppeliasim-zmqremoteapi-client
   - 方法B：将 CoppeliaSim/programming/zmqRemoteApi/clients/python 加到 PYTHONPATH
3) 运行：python kinect_cuboid_center_demo.py
   - 按键 q 退出

注意：
- 若你的相机名称不同，请修改下方 kinect_rgb_candidates。
- 这里“识别 cuboid”采用仿真真值：按名称包含 “Cuboid” 过滤。如需几何判定，可改用 getShapeGeomInfo（示例见函数 find_cuboids_by_name 的注释）。
"""

import sys
import time
import math
import numpy as np
import cv2

try:
    # 使用 pip 包：coppeliasim-zmqremoteapi-client
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print(
        "未找到 coppeliasim_zmqremoteapi_client。请先在当前 Python 环境中安装：\n"
        "  python -m pip install -U coppeliasim-zmqremoteapi-client\n"
        "或将 CoppeliaSim/programming/zmqRemoteApi/clients/python 加入 PYTHONPATH。"
    )
    raise

def main():
    client = RemoteAPIClient()  # 默认连接 127.0.0.1:23000
    sim = client.require('sim')

    # 可能的 Kinect RGB 视觉传感器路径/名称候选（按需修改/扩展）
    kinect_rgb_candidates = [
        '/kinect/rgb',
        '/Kinect/rgb',
        '/kinect_rgb',
        '/Kinect_rgb',
        '/Vision_sensor',
        '/Kinect/Vision_sensor',
        '/kinect/Vision_sensor',
    ]

    rgb_sensor = find_first_object(sim, kinect_rgb_candidates)
    if rgb_sensor is None:
        raise RuntimeError('未能找到 Kinect RGB Vision Sensor，请检查名称/层级路径。')

    # 获取分辨率与水平视场角（弧度），推导内参
    res = sim.getVisionSensorRes(rgb_sensor)  # [width, height]
    w, h = int(res[0]), int(res[1])
    fov_x = sim.getObjectFloatParam(rgb_sensor, sim.visionfloatparam_perspective_angle)  # 水平FOV（弧度）
    fx, fy, cx, cy = compute_intrinsics(w, h, fov_x)

    # 收集场景中名称包含 "Cuboid" 的形状
    cuboid_handles = find_cuboids_by_name(sim)

    print(f'找到视觉传感器: handle={rgb_sensor}, 分辨率={w}x{h}, 水平FOV={fov_x:.3f}rad')
    print(f'找到 Cuboid 数量: {len(cuboid_handles)}')
    print('按 q 退出窗口。')

    while True:
        # 读取图像（返回自底向上的缓冲，需要翻转）
        img_buf, res_now = sim.getVisionSensorImg(rgb_sensor)
        w_now, h_now = int(res_now[0]), int(res_now[1])
        if (w_now != w) or (h_now != h):
            w, h = w_now, h_now
            fx, fy, cx, cy = compute_intrinsics(w, h, fov_x)

        img = image_buffer_to_numpy(img_buf, (w, h))  # HxWx3, uint8, RGB, top-left origin

        # 相机位姿（世界系）
        cam_pose = sim.getObjectPose(rgb_sensor, -1)  # [x y z qx qy qz qw]
        T_WC = pose_to_matrix(cam_pose)               # 世界<-相机
        T_CW = np.linalg.inv(T_WC)                    # 相机<-世界

        # 将每个 cuboid 中心投影并画红色加号
        for hdl in list(cuboid_handles):
            try:
                pW = sim.getObjectPosition(hdl, -1)  # 世界坐标
            except Exception:
                # 可能被删除
                cuboid_handles.remove(hdl)
                continue
            uv = project_point(T_CW, pW, fx, fy, cx, cy)
            if uv is None:
                continue
            u, v = int(round(uv[0])), int(round(uv[1]))
            draw_plus(img, u, v, half_len=8, color=(0, 0, 255))  # BGR，红色

        # 叠加说明文字
        cv2.putText(img, f'Cuboids: {len(cuboid_handles)}  (q to quit)',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, f'Cuboids: {len(cuboid_handles)}  (q to quit)',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1, cv2.LINE_AA)

        cv2.imshow('Kinect Cuboid Center Demo', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


def find_first_object(sim, candidates):
    for name in candidates:
        try:
            return sim.getObject(name)
        except Exception:
            continue
    return None


def find_cuboids_by_name(sim):
    """
    策略1：按名称包含 "Cuboid"
    如需几何判定（更鲁棒），可使用 getShapeGeomInfo 判断 primitiveType 是否为 cuboid：
        isPure, _, primitiveType, _ = sim.getShapeGeomInfo(hdl)
        if isPure and primitiveType == sim.primitiveshape_cuboid: ...
    注意：不同版本返回参数可能略有不同，请以本机 CoppeliaSim 文档为准。
    """
    handles = []
    # 列出整个场景根（sim.handle_scene）下的所有 Shape
    all_shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    for hdl in all_shapes:
        alias = sim.getObjectAlias(hdl, 1)  # 带路径
        if alias and ('cuboid' in alias.lower()):
            handles.append(hdl)
    return handles


def compute_intrinsics(w, h, fov_x_rad):
    # 像素坐标系：原点左上，u向右，v向下
    fov_y = 2.0 * math.atan((h / max(w, 1e-9)) * math.tan(fov_x_rad / 2.0))
    fx = w / (2.0 * math.tan(fov_x_rad / 2.0))
    fy = h / (2.0 * math.tan(fov_y / 2.0))
    cx = (w - 1) / 2.0
    cy = (h - 1) / 2.0
    return fx, fy, cx, cy


def image_buffer_to_numpy(img_buf, res_wh):
    """
    CoppeliaSim 返回的图像缓冲是自底向上的 RGB 序列。
    转为 HxWx3 的 uint8，并翻转为左上角原点。
    """
    w, h = res_wh
    if isinstance(img_buf, (bytes, bytearray, memoryview)):
        # frombuffer 可能创建只读视图，这里 copy 以确保可写
        arr = np.frombuffer(img_buf, dtype=np.uint8).copy()
    else:
        arr = np.array(img_buf, dtype=np.uint8)
    if arr.size != w * h * 3:
        # 某些设置可能返回灰度或包含 alpha；此处简单防御
        arr = arr[:w * h * 3]
    img = arr.reshape(h, w, 3)     # 注意先按 (h, w, 3) reshape
    img = np.flipud(img).copy()    # 翻转到左上角为原点，并确保可写
    return img


def pose_to_matrix(pose):
    """
    pose: [x y z qx qy qz qw], CoppeliaSim 四元数为 [qx qy qz qw]
    返回 4x4 齐次矩阵：世界<-该物体
    """
    x, y, z, qx, qy, qz, qw = pose
    R = quat_to_rotm(qx, qy, qz, qw)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def quat_to_rotm(qx, qy, qz, qw):
    # 经典四元数到旋转矩阵公式（右手，单位四元数）
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1 - 2*(yy + zz),   2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),       1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),       2*(yz + wx),       1 - 2*(xx + yy)]
    ], dtype=np.float64)
    return R


def project_point(T_CW, pW, fx, fy, cx, cy):
    """
    世界点 pW -> 像素坐标 (u, v)，左上原点
    """
    Pw = np.array([pW[0], pW[1], pW[2], 1.0], dtype=np.float64)
    Pc = T_CW @ Pw
    X, Y, Z = Pc[0], Pc[1], Pc[2]
    if Z <= 0:
        return None
    u = fx * (X / Z) + cx
    v = cy - fy * (Y / Z)  # 上为负方向
    return (u, v)


def draw_plus(img_rgb, u, v, half_len=8, color=(0, 0, 255), thickness=2):
    """
    在 RGB 图像上画加号；color 为 BGR 以便与 OpenCV 一致，这里先转换。
    """
    h, w, _ = img_rgb.shape
    if not (0 <= u < w and 0 <= v < h):
        return
    # OpenCV 期望 BGR
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    pt_h1 = (max(0, u - half_len), v)
    pt_h2 = (min(w - 1, u + half_len), v)
    pt_v1 = (u, max(0, v - half_len))
    pt_v2 = (u, min(h - 1, v + half_len))
    cv2.line(img_bgr, pt_h1, pt_h2, color, thickness, cv2.LINE_AA)
    cv2.line(img_bgr, pt_v1, pt_v2, color, thickness, cv2.LINE_AA)
    # 再转回 RGB 以保持上层使用一致
    img_rgb[:] = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


if __name__ == '__main__':
    main()