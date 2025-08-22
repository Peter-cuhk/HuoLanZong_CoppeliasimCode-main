function kinect_cuboid_center_demo_legacy()
% 连接 CoppeliaSim 4.7.0（Legacy Remote API，simxStart），读取 Kinect RGB 图像，
% 将场景中名称包含“Cuboid”的 Shape 的3D中心投影到图像平面，在其像素中心处画红色加号。
%
% 使用方法：
% 1) 在 CoppeliaSim 中启用 “Remote API server” 服务（默认端口 19997），并运行仿真。
%    注意：4.7.0 可通过菜单 Add-ons > Remote API server 启动，或在场景脚本中手动启用。
% 2) 把 legacy Remote API 的 MATLAB 绑定加入路径（示例见下方 addpath）。
% 3) 运行本脚本。按 q 退出。
%
% 可选：将下行路径替换为你的 CoppeliaSim legacy Remote API MATLAB 绑定位置
% addpath('C:/Program Files/CoppeliaRobotics/CoppeliaSim/programming/legacyRemoteApi/remoteApiBindings/matlab/matlab');

% --------------- 建立连接（Legacy Remote API） ---------------
sim = remApi('remoteApi'); %#ok<REMAPI>
cleanupObj = onCleanup(@() cleanup(sim)); %#ok<NASGU>

sim.simxFinish(-1); % 关闭任何旧连接
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
assert(clientID >= 0, '无法连接到 CoppeliaSim Remote API server (127.0.0.1:19997)。');

% --------------- 找到 Kinect 的 RGB 视觉传感器 ---------------
kinectRgbCandidates = { ...
    '/kinect/rgb', '/Kinect/rgb', '/kinect_rgb', '/Kinect_rgb', ...
    '/Vision_sensor', '/Kinect/Vision_sensor', '/kinect/Vision_sensor' ...
};
rgbSensor = findFirstHandle(sim, clientID, kinectRgbCandidates);
assert(~isempty(rgbSensor), '未能找到 Kinect RGB Vision Sensor，请检查名称。');

% 读取相机水平视场角（弧度）
[rc, fovX] = sim.simxGetObjectFloatParameter( ...
    clientID, rgbSensor, sim.sim_visionfloatparam_perspective_angle, sim.simx_opmode_blocking);
assert(rc == sim.simx_return_ok, '无法读取相机视场角。');

% 预启动图像流并等待第一帧
sim.simxGetVisionSensorImage2(clientID, rgbSensor, 0, sim.simx_opmode_streaming);
sim.simxGetPingTime(clientID); % 让流真正开始
[img, res] = waitFirstImage(sim, clientID, rgbSensor);
w = double(res(1)); h = double(res(2));
[fx, fy, cx, cy] = computeIntrinsics(w, h, fovX);

% --------------- 收集 Cuboid 句柄 ---------------
cuboidHandles = findCuboidsByName(sim, clientID);
fprintf('找到视觉传感器 handle=%d, 分辨率=%dx%d, 水平FOV=%.3frad\n', rgbSensor, w, h, fovX);
fprintf('找到 Cuboid 数量: %d\n', numel(cuboidHandles));

% 预启动相机位姿流与各 cuboid 的位置流
sim.simxGetObjectPosition(clientID, rgbSensor, -1, sim.simx_opmode_streaming);
sim.simxGetObjectQuaternion(clientID, rgbSensor, -1, sim.simx_opmode_streaming);
for i = 1:numel(cuboidHandles)
    sim.simxGetObjectPosition(clientID, cuboidHandles(i), -1, sim.simx_opmode_streaming);
end
sim.simxGetPingTime(clientID);

figure('Name','Kinect Cuboid Center Demo (legacy simx)','NumberTitle','off');
set(gcf,'Color','w');

while true
    % 取图像（img: HxWx3 uint8, RGB；若显示上下颠倒，可改为 flipud(img)）
    [rc, resNow, img] = sim.simxGetVisionSensorImage2(clientID, rgbSensor, 0, sim.simx_opmode_buffer);
    if rc == sim.simx_return_ok
        if any([double(resNow(1)), double(resNow(2))] ~= [w, h])
            w = double(resNow(1)); h = double(resNow(2));
            [fx, fy, cx, cy] = computeIntrinsics(w, h, fovX);
        end

        % 相机位姿（世界系）
        [rcp, pC] = sim.simxGetObjectPosition(clientID, rgbSensor, -1, sim.simx_opmode_buffer);
        [rcq, qC] = sim.simxGetObjectQuaternion(clientID, rgbSensor, -1, sim.simx_opmode_buffer);
        if rcp == sim.simx_return_ok && rcq == sim.simx_return_ok
            camPose = [pC(:).' qC(:).']; % [x y z qx qy qz qw]
            T_WC = poseToT(camPose);
            T_CW = inv(T_WC);

            % 在图像上画每个 Cuboid 的中心
            for i = numel(cuboidHandles):-1:1
                hdl = cuboidHandles(i);
                [rcpW, pW] = sim.simxGetObjectPosition(clientID, hdl, -1, sim.simx_opmode_buffer);
                if rcpW ~= sim.simx_return_ok
                    % 可能被删除，移除之
                    cuboidHandles(i) = [];
                    continue;
                end
                uv = projectPoint(T_CW, pW, fx, fy, cx, cy);
                if ~isempty(uv)
                    img = drawPlus(img, round(uv(1)), round(uv(2)), 8, [255, 0, 0]);
                end
            end
        end

        imshow(img);
        title(sprintf('RGB image with cuboid centers (N=%d)  - press q to quit', numel(cuboidHandles)));
        drawnow;
    end

    % 按 q 退出
    if ~ishandle(gcf), break; end
    ch = get(gcf,'CurrentCharacter');
    if ~isempty(ch) && (ch=='q' || ch=='Q'), break; end
end

end

% --------------- 辅助函数 ---------------

function cleanup(sim)
% 退出时关闭连接，释放对象
try
    sim.simxFinish(-1);
catch
end
try
    sim.delete();
catch
end
end

function handle = findFirstHandle(sim, clientID, candidates)
handle = [];
for i = 1:numel(candidates)
    name = candidates{i};
    % legacy API 不支持路径，这里取最后一个斜杠后的名字
    if contains(name, '/')
        parts = regexp(name,'/+','split');
        name = parts{end};
    end
    try
        [rc, h] = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking);
        if rc == sim.simx_return_ok
            handle = h;
            return;
        end
    catch
    end
end
end

function [img, res] = waitFirstImage(sim, clientID, sensor)
t0 = tic;
img = []; res = [0 0];
while true
    [rc, resNow, imgNow] = sim.simxGetVisionSensorImage2(clientID, sensor, 0, sim.simx_opmode_buffer);
    if rc == sim.simx_return_ok
        img = imgNow;
        res = resNow;
        return;
    end
    if toc(t0) > 5
        error('等待相机第一帧超时。');
    end
    pause(0.01);
end
end

function handles = findCuboidsByName(sim, clientID)
% 按名称包含 "Cuboid" 过滤
handles = [];
[rc, allObjs] = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking);
if rc ~= sim.simx_return_ok
    warning('无法获取场景对象列表。');
    return;
end
for i = 1:numel(allObjs)
    h = allObjs(i);
    [rcn, nm] = sim.simxGetObjectName(clientID, h, sim.simx_opmode_blocking);
    if rcn == sim.simx_return_ok && ~isempty(nm)
        if contains(lower(nm), 'cuboid')
            handles(end+1) = h; %#ok<AGROW>
        end
    end
end
end

function [fx, fy, cx, cy] = computeIntrinsics(w, h, fovX)
fovY = 2*atan((h/max(w,1e-9))*tan(fovX/2));
fx = w/(2*tan(fovX/2));
fy = h/(2*tan(fovY/2));
cx = (w-1)/2;
cy = (h-1)/2;
end

function T = poseToT(pose)
% pose: [x y z qx qy qz qw]
p = pose(1:3);
q = pose(4:7);
R = quat2rotm_coppelia(q);
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = p(:);
end

function R = quat2rotm_coppelia(q)
% q = [qx qy qz qw]，单位四元数
qx=q(1); qy=q(2); qz=q(3); qw=q(4);
R = [1-2*(qy^2+qz^2),   2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx^2+qy^2)];
end

function uv = projectPoint(T_CW, pW, fx, fy, cx, cy)
Pw = [pW(:); 1];
Pc = T_CW * Pw;
X = Pc(1); Y = Pc(2); Z = Pc(3);
if Z <= 0
    uv = [];
    return;
end
u = fx * (X / Z) + cx;
v = cy - fy * (Y / Z); % 上为负方向
uv = [u, v];
end

function img = drawPlus(img, u, v, halfLen, color)
% 在 img (HxWx3, uint8, RGB) 上画加号
[h, w, ~] = size(img);
if isnan(u) || isnan(v), return; end
u = round(u); v = round(v);
% 横线
u1 = max(1, u - halfLen); u2 = min(w, u + halfLen);
v1 = min(max(1, v), h);
img(v1, u1:u2, 1) = color(1);
img(v1, u1:u2, 2) = color(2);
img(v1, u1:u2, 3) = color(3);
% 竖线
v1 = max(1, v - halfLen); v2 = min(h, v + halfLen);
u1 = min(max(1, u), w);
img(v1:v2, u1, 1) = color(1);
img(v1:v2, u1, 2) = color(2);
img(v1:v2, u1, 3) = color(3);
end