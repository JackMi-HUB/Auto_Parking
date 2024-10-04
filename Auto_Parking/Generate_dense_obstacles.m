close all;clc;clear; clear global obstacle_vertexes_ vehicle_TPBV_; 
global obstacle_vertexes_ vehicle_TPBV_ environment_scale_ Nobs
%%
%采用随机分布生成12个六边形障碍
% vers = [];
% for i=1:12 
%     A = generate_ploygon();
%     A = A-22+44*rand;
%     if mod(i,2) == 0 
%         A(1,:) = -A(1,:);
%     end
%     vers = [vers;A];
% end
% obstacle_vertexes_ = ...
%     {struct('x',vers(1,:),'y',vers(2,:)),...
%      struct('x',vers(3,:),'y',vers(4,:)),...
%      struct('x',vers(5,:),'y',vers(6,:)),...
%      struct('x',vers(7,:),'y',vers(8,:)),...
%      struct('x',vers(9,:),'y',vers(10,:)),...
%      struct('x',vers(11,:),'y',vers(12,:)),...
%      struct('x',vers(13,:),'y',vers(14,:)),...
%      struct('x',vers(15,:),'y',vers(16,:)),...
%      struct('x',vers(17,:),'y',vers(18,:)),...
%      struct('x',vers(19,:),'y',vers(20,:)),...
%      struct('x',vers(21,:),'y',vers(22,:)),...
%      struct('x',vers(23,:),'y',vers(24,:)),...
%      };
%%
%采用自画3~6边形构造地图
obstacle_vertexes_ = {};
figure(1);
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
for i = 1 : 6 %随机生成14个obs，后续根据需要再修改
    [ob_x,ob_y] = ginput;%生成n*1的x、y向量
    obstacle_vertexes_ = {obstacle_vertexes_{:},struct('x',ob_x','y',ob_y')};
    for ii = 1 : i
        fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
    end
end
%%
% load('Dense_environments\\case2.mat');
InitParam();
countmap = rot90(costmap_);%这里旋转90度后才能使图像正常显示
figure(2)
imshow(~countmap);
% Location_M = [];
% bVertexs = get_bVertex(costmap_);
% for i=1:67
%     Location_M = [Location_M;ConvertIndexToX(bVertexs(i,1))-20,ConvertIndexToY(bVertexs(i,2))-20];
% end
% Plot basic setups
figure(3)
axis equal; box on;  axis([-30 30 -30 30]);
set(gcf,'outerposition',get(0,'screensize'));
hold on;
for ii = 1 : Nobs
    fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
end
% vehicle_TPBV_ = ...
%                 struct('x0',20,'y0',0,'theta0',pi/4,...
%                 'xtf',0,'ytf',20,'thetatf',pi/4);
[x,y]=ginput;
theta0 = qtan(y(1),x(1),y(2),x(2));
thetatf = qtan(y(3),x(3),y(4),x(4));
vehicle_TPBV_ = ...
                struct('x0',x(1),'y0',y(1),'theta0',theta0,...
                'xtf',x(3),'ytf',y(3),'thetatf',thetatf);                        
save('no_verify//case5.mat','obstacle_vertexes_','vehicle_TPBV_');


function vertex = generate_ploygon()
% 生成随机六边形的顶点坐标
numVertices = 6; % 顶点数量
radius = 1; % 六边形的半径

% 生成随机角度
theta = linspace(0, 2*pi, numVertices+1); % 均匀分布的角度
theta = theta(1:end-1); % 去掉最后一个角度，以避免重复

% 生成随机半径
randRadius = radius * (3 + 2*rand(size(theta))); % 随机生成的半径长度，范围在0.8到1.2倍半径之间

% 计算顶点的坐标
x = randRadius .* cos(theta);
y = randRadius .* sin(theta);

vertex = [x;y];
% 绘制六边形
% figure;
%plot([x x(1)], [y y(1)], 'r-o'); % 连接最后一个顶点和第一个顶点，形成闭合的六边形
%axis equal;  axis([-26 26 -26 26]);% 设置坐标轴比例相等
% 显示顶点坐标
% disp('顶点坐标：');
% disp([x' y']);
end

function theta=qtan(y1,x1,y2,x2)
    
theta=atan((y2-y1)/(x2-x1));
%First Quadrant
if (x2>x1)&&(y2>y1)
    theta =abs(theta);
    
%90 degrees
elseif (x2==x1)&&(y2>y1)
    theta = pi/2;
   
%180 degrees
elseif (x2<x1)&&(y2==y1)
    theta = pi;
    
%270 degrees
elseif (x2==x1)&&(y2<y1)
    theta =-pi/2;   
    
%Second Quadrant
elseif (x2<x1)&&(y2>y1)
    theta = pi - abs(theta);
    
%Third Quadrant
elseif (x2<x1)&&(y2<y1)
    theta = abs(theta) -pi;
    
%Fourth Quadrant
elseif (x2>x1)&&(y2<y1)
    theta = -abs(theta);
    
%Zero
elseif (x2>x1)&&(y2==y1)
    theta =0;
    
end

end