% clc;close all;
% VehicleAnimation2();
function VehicleAnimation()
    global obstacle_vertexes_ vehicle_TPBV_ environment_scale_ Nobs
    x = load("x.txt");
    y = load("y.txt");
    theta = load("theta.txt");
    phy = load("phy.txt");
    InitializeParams();
    videoFWriter = VideoWriter('Parking.mp4','MPEG-4');
    open(videoFWriter);    
    figure(3);
    set(0, 'DefaultLineLineWidth', 1);
    hold on;
    box on;
    grid minor;
    axis equal;
%     sz=get(0,'screensize');
%     figure('outerposition',sz);
    xmin = environment_scale_.environment_x_min;
    xmax = environment_scale_.environment_x_max;
    ymin = environment_scale_.environment_y_min;
    ymax = environment_scale_.environment_y_max;
    axis([xmin xmax ymin ymax]);
    set(gcf, 'outerposition', get(0,'screensize'));
    
    for ii = 1 : Nobs
        fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
    end
    
    Arrow([vehicle_TPBV_.x0, vehicle_TPBV_.y0], [vehicle_TPBV_.x0 + cos(vehicle_TPBV_.theta0), vehicle_TPBV_.y0 + sin(vehicle_TPBV_.theta0)], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    Arrow([vehicle_TPBV_.xtf, vehicle_TPBV_.ytf], [vehicle_TPBV_.xtf + cos(vehicle_TPBV_.thetatf), vehicle_TPBV_.ytf+ sin(vehicle_TPBV_.thetatf)],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    drawnow
    V = CreateVehiclePolygon(vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0);
    plot(V.x, V.y, 'g--', 'LineWidth', 2);

    V = CreateVehiclePolygon(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf);
    plot(V.x, V.y, 'r--', 'LineWidth', 2);
    
    X = [];
    Y = [];
    px = x(1);
    py = y(1);
    X = [X,px];
    Y = [Y,py];
    pth = theta(1);
    pph = phy(1);
    h3 = plot(X,Y,'b'); % 规划出来的轨迹，蓝色曲线  
    [vehx,vehy] = getVehTran(px,py,pth); % 根据后轴中心的位姿计算车辆边框的位姿
    [wheelx1,wheely1] = Wheel_point1(px,py,pth,pph);
    [wheelx2,wheely2] = Wheel_point2(px,py,pth,pph);    
    [wheelx3,wheely3] = Wheel_point3(px,py,pth);
    [wheelx4,wheely4] = Wheel_point4(px,py,pth);
    h1 = plot(vehx,vehy,'k'); % 车辆边框
    h2 = plot(px,px,'rx','MarkerSize',10); % 车辆后轴中心
    h4 = plot(wheelx3,wheely3,'k');
    h5 = plot(wheelx4,wheely4,'k');
    h6 = plot(wheelx1,wheely1,'k');
    h7 = plot(wheelx2,wheely2,'k');   
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2:length(theta)
        px = x(i);
        py = y(i);
        X = [X,px];
        Y = [Y,py];
        pth = theta(i);
        pph = phy(i);
        [vehx,vehy] = getVehTran(px,py,pth);
        [wheelx1,wheely1] = Wheel_point1(px,py,pth,pph);
        [wheelx2,wheely2] = Wheel_point2(px,py,pth,pph);
        [wheelx3,wheely3] = Wheel_point3(px,py,pth);
        [wheelx4,wheely4] = Wheel_point4(px,py,pth);
        h1.XData = vehx; %更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
        h1.YData = vehy;
        h2.XData = px; %更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
        h2.YData = py;
        h3.XData = X;
        h3.YData = Y;
        h4.XData = wheelx3; 
        h4.YData = wheely3;    
        h5.XData = wheelx4; 
        h5.YData = wheely4;
        h6.XData = wheelx1; 
        h6.YData = wheely1;    
        h7.XData = wheelx2; 
        h7.YData = wheely2;        
        img = getframe(gcf);%只提取图窗里面的内容，gcf是图窗句柄
        writeVideo(videoFWriter,img);
%         pause(0.005)
    end
    close(videoFWriter);
end

 % 根据后轴中心的位姿计算车辆边框的位姿
function [x,y] = getVehTran(x,y,theta)
    global vehicle_geometrics_
    W = vehicle_geometrics_.vehicle_width;
    LF = vehicle_geometrics_.vehicle_wheelbase+vehicle_geometrics_.vehicle_front_hang;
    LB = vehicle_geometrics_.vehicle_rear_hang;
    
    
    % 车辆的边框由四个角点确定
    Cornerfl = [LF, W/2]; % 左前方角点
    Cornerfr = [LF, -W/2]; % 右前方角点
    Cornerrl = [-LB, W/2]; % 左后方角点
    Cornerrr = [-LB, -W/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

    % 计算车辆轮子的顶点
function [x,y] = Wheel_point1(x,y,theta,phy)
    global vehicle_geometrics_
    W = vehicle_geometrics_.vehicle_width;
    WB = vehicle_geometrics_.vehicle_wheelbase;
    wheel_width = 0.3;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cc1 = [WB,W/2];
    Cornerfl = [wheel_length/2, wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, -wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, -wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm1 = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    dcm2 = angle2dcm(-(phy+theta), 0, 0);
    
    tvec = dcm1*[Cc1';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cc1 = tvec(1:2)+Pos;
    
    tvec1 = dcm2*[Cornerfl';0];
    tvec1 = tvec1';
    Cornerfl = tvec1(1:2)+Cc1;
    
    tvec1 = dcm2*[Cornerfr';0];
    tvec1 = tvec1';
    Cornerfr = tvec1(1:2)+Cc1;
    
    tvec1 = dcm2*[Cornerrl';0];
    tvec1 = tvec1';
    Cornerrl = tvec1(1:2)+Cc1;
    
    tvec1 = dcm2*[Cornerrr';0];
    tvec1 = tvec1';
    Cornerrr = tvec1(1:2)+Cc1;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function [x,y] = Wheel_point2(x,y,theta,phy)
    global vehicle_geometrics_
    W = vehicle_geometrics_.vehicle_width;
    WB = vehicle_geometrics_.vehicle_wheelbase;
    wheel_width = 0.3;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cc1 = [WB,-W/2];
    Cornerfl = [wheel_length/2, wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, -wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, -wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm1 = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    dcm2 = angle2dcm(-(phy+theta), 0, 0);
    
    tvec = dcm1*[Cc1';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cc1 = tvec(1:2)+Pos;
    
    tvec1 = dcm2*[Cornerfl';0];
    tvec1 = tvec1';
    Cornerfl = tvec1(1:2)+Cc1;
    
    tvec1 = dcm2*[Cornerfr';0];
    tvec1 = tvec1';
    Cornerfr = tvec1(1:2)+Cc1;
    
    
    tvec1 = dcm2*[Cornerrl';0];
    tvec1 = tvec1';
    Cornerrl = tvec1(1:2)+Cc1;   

    tvec1 = dcm2*[Cornerrr';0];
    tvec1 = tvec1';
    Cornerrr = tvec1(1:2)+Cc1; 
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%构建四个轮子(左后)
function [x,y] = Wheel_point3(x,y,theta)
    global vehicle_geometrics_
    W = vehicle_geometrics_.vehicle_width;
    wheel_width = 0.3;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [wheel_length/2, W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%构建四个轮子(左后)
function [x,y] = Wheel_point4(x,y,theta)
    global vehicle_geometrics_
    W = vehicle_geometrics_.vehicle_width;
    wheel_width = 0.3;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [wheel_length/2, -W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, -W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, -W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, -W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function InitializeParams()
global params_ vehicle_geometrics_
params_.vehicle.lw = vehicle_geometrics_.vehicle_wheelbase;       % vehicle wheelbase
params_.vehicle.lf = vehicle_geometrics_.vehicle_front_hang;      % vehicle front hang length
params_.vehicle.lr = vehicle_geometrics_.vehicle_rear_hang;     % vehicle rear hang length
params_.vehicle.lb = vehicle_geometrics_.vehicle_width;     % vehicle width
end

function V = CreateVehiclePolygon(x, y, theta)
global params_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = params_.vehicle.lb * 0.5;
AX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (params_.vehicle.lf + params_.vehicle.lw) * cos_theta + vehicle_half_width * sin_theta;
CX = x - params_.vehicle.lr * cos_theta + vehicle_half_width * sin_theta;
DX = x - params_.vehicle.lr * cos_theta - vehicle_half_width * sin_theta;
AY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (params_.vehicle.lf + params_.vehicle.lw) * sin_theta - vehicle_half_width * cos_theta;
CY = y - params_.vehicle.lr * sin_theta - vehicle_half_width * cos_theta;
DY = y - params_.vehicle.lr * sin_theta + vehicle_half_width * cos_theta;
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end

function Angle = limit(angle)
% 将角度限制在(-pi, pi)范围内
angle = mod(angle, 2*pi); % 将角度转换为0到2*pi的范围

% 将角度限制在(-pi, pi)范围内
if angle > pi
    Angle = angle - 2*pi;
end

end

function M = rot(angle)

% 创建绕z轴旋转的旋转矩阵
M = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

function R = rotationMatrix(axis, angle)
    % 将轴向量单位化
    axis = axis / norm(axis);
    
    % 提取轴向量的分量
    ux = axis(1);
    uy = axis(2);
    uz = axis(3);
    
    % 计算旋转矩阵的元素
    cosAngle = cos(angle);
    sinAngle = sin(angle);
    oneMinusCos = 1 - cosAngle;
    
    R = [cosAngle + ux^2 * oneMinusCos, ux * uy * oneMinusCos - uz * sinAngle, ux * uz * oneMinusCos + uy * sinAngle;
         uy * ux * oneMinusCos + uz * sinAngle, cosAngle + uy^2 * oneMinusCos, uy * uz * oneMinusCos - ux * sinAngle;
         uz * ux * oneMinusCos - uy * sinAngle, uz * uy * oneMinusCos + ux * sinAngle, cosAngle + uz^2 * oneMinusCos];
end
