close all; clc
global obstacle_vertexes_
% load('Dense_environments\\case22.mat');
load('ds\\case13.mat');
% load('pose.mat');

figure(1)
InitParam();
countmap = rot90(costmap_aex);%这里旋转90度后才能使图像正常显示
Location_M = [];
% bVertexs = get_bVertex(costmap_);
% for i=1:67
%     Location_M = [Location_M;ConvertIndexToX(bVertexs(i,1))-20,ConvertIndexToY(bVertexs(i,2))-20];
% end
% Plot basic setups
axis equal; box on;  axis([-30 30 -30 30]);
set(gcf,'outerposition',get(0,'screensize'));
hold on;
for ii = 1 : Nobs
    fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
end
figure(2)
imshow(~countmap);%这里取反是因为显示0是黑色，1是白色；而期望结果恰恰相反
%将二值图像转用栅格图表示出来
%设置grids数目
%figure(3)
a=261;%横轴
b=261;%纵轴
length=1;%格子边长
%Inverse = ~costmap_;
B = imresize(costmap_,[a/length b/length]); %对矩阵与元素取反，使障碍区域用黑色表示，可行区域用白色表示
J= ~B;

axes('GridLineStyle','-');
hold on
grid on
axis([0,a,0,b]);
% set(gca,'xtick',0:1:a,'ytick',0:1:b);
for i=1:a/length
    for j=1:b/length
        if(J(i,j)==0)
            x=[i,i+1,i+1,i]*length;
            y=[j,j,j+1,j+1]*length;
            % k代表'black'（或 'k'），也可以使用'black'
            h=fill(x,y,'k');
            hold on
        end
    end
end


function [x1] = ConvertIndexToX(ind1)
global hybrid_astar_
x1 = ind1 * hybrid_astar_.resolution_x;
end

function [y1] = ConvertIndexToY(ind2)
global hybrid_astar_
y1 = ind2 * hybrid_astar_.resolution_y;
end