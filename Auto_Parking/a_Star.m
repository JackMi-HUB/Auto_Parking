function path = a_Star()
global costmap_ vehicle_TPBV_ environment_scale_
map = ~costmap_;%因为作图时0为黑色，1为白色，所以此处需要做一个取反操作，画出正常的二值图
%% 输入参数
X_Length = size(map,1);
Y_Length = size(map,2);
%a_star是在栅格地图上进行搜索的，所以为整数端点
Start_Node = ceil(([vehicle_TPBV_.x0,vehicle_TPBV_.y0] - environment_scale_.environment_x_min) .* (size(costmap_,1)/(environment_scale_.environment_x_max-environment_scale_.environment_x_min)));
Target_Node = ceil(([vehicle_TPBV_.xtf,vehicle_TPBV_.ytf] - environment_scale_.environment_x_min) .* (size(costmap_,1)/(environment_scale_.environment_x_max-environment_scale_.environment_x_min)));
% Start_Node = [115,45];
% Target_Node = [180,215];
Obs_Node_List = [];

for i = 1:size(map,1)
    for j = 1:size(map,2)
        if (map(i,j) == 0)
            Obs_Node_List = [Obs_Node_List;[i,j]];
        end
    end
end

%% Astar算法
tic
path = AstarSearch(Start_Node,Target_Node,Obs_Node_List,X_Length,Y_Length);
using_time = toc
end

%% Astar搜索过程
function Path_List = AstarSearch(Start_Node,Target_Node,Obs_Node_List,X_Length,Y_Length)
%% 初始化变量
Node = struct('PositionX',0,'PositionY',0,'Cost_F',0,'Cost_G',0,'Cost_H',0,'Father',[0,0]);
OpenList = repmat(Node,X_Length*Y_Length,1);
OpenList_Num = 0;
CloseList = repmat(Node,X_Length*Y_Length,1);
CloseList_Num = 0;
Start_Node_PositionX = Start_Node(1);
Start_Node_PositionY = Start_Node(2);
Target_Node_PositionX = Target_Node(1);
Target_Node_PositionY = Target_Node(2);

%% 将初始节点加入CloseList
Node_Father.PositionX = Start_Node_PositionX;
Node_Father.PositionY = Start_Node_PositionY;
Node_Father.Cost_G = 0;
Node_Father.Cost_H = abs(Start_Node_PositionX - Target_Node_PositionX) + abs(Start_Node_PositionY - Target_Node_PositionY);
Node_Father.Cost_F = Node_Father.Cost_G + Node_Father.Cost_H;
Node_Father.Father = [0,0];

CloseList_Num = CloseList_Num + 1;
CloseList(CloseList_Num) = Node_Father;

%% 打印地图网格和起始、终止、障碍物点
%PlotGridArea(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,OpenList,OpenList_Num,CloseList,CloseList_Num)

%% 循环查找路径
while 1
    Node_Child_List = repmat(Node,8,1);
    Node_Child_List(1).PositionX = Node_Father.PositionX + 1;Node_Child_List(1).PositionY = Node_Father.PositionY;
    Node_Child_List(2).PositionX = Node_Father.PositionX + 1;Node_Child_List(2).PositionY = Node_Father.PositionY + 1;
    Node_Child_List(3).PositionX = Node_Father.PositionX;Node_Child_List(3).PositionY = Node_Father.PositionY + 1;
    Node_Child_List(4).PositionX = Node_Father.PositionX - 1;Node_Child_List(4).PositionY = Node_Father.PositionY + 1;
    Node_Child_List(5).PositionX = Node_Father.PositionX - 1;Node_Child_List(5).PositionY = Node_Father.PositionY;
    Node_Child_List(6).PositionX = Node_Father.PositionX - 1;Node_Child_List(6).PositionY = Node_Father.PositionY - 1;
    Node_Child_List(7).PositionX = Node_Father.PositionX;Node_Child_List(7).PositionY = Node_Father.PositionY - 1;
    Node_Child_List(8).PositionX = Node_Father.PositionX + 1;Node_Child_List(8).PositionY = Node_Father.PositionY - 1;
    %计算子节点中的属性
    for i = 1:8
        PositionX = Node_Child_List(i).PositionX;
        PositionY = Node_Child_List(i).PositionY;
        %判断是否超地图范围，如果是则不考虑
        if PositionX <= 0 || PositionX > X_Length || PositionY <= 0 || PositionY > Y_Length
            continue;
        end
        %判断是否障碍物节点，如果是则不考虑
        IsObsNodeFlag = false;
        for j = 1:size(Obs_Node_List,1)
            Obs_Node_PositionX = Obs_Node_List(j,1);
            Obs_Node_PositionY = Obs_Node_List(j,2);
            if PositionX == Obs_Node_PositionX && PositionY == Obs_Node_PositionY
                IsObsNodeFlag = true;
                break;
            end
        end
        if IsObsNodeFlag
            continue;
        end
        %判断是否在CloseList中，如果是则不考虑
        IsInCloseListFlag = false;
        for j = 1:CloseList_Num
            if PositionX == CloseList(j).PositionX && PositionY == CloseList(j).PositionY
                IsInCloseListFlag = true;
                break;
            end
        end
        if IsInCloseListFlag
            continue;
        end
        %计算代价函数
        if abs(PositionX - Node_Father.PositionX) + abs(PositionY - Node_Father.PositionY) == 2
            Cost_G = 1.4 + Node_Father.Cost_G;
        else
            Cost_G = 1 + Node_Father.Cost_G;
        end
        Cost_H = abs(PositionX - Target_Node_PositionX) + abs(PositionY - Target_Node_PositionY);
        Cost_F = Cost_G + Cost_H;
        %判断节点是否在OpenList，是则考虑更新，不是则加入
        IsInOpenListFlag = false;
        Index_InOpenList = 0;
        for j = 1:OpenList_Num
            if PositionX == OpenList(j).PositionX && PositionY == OpenList(j).PositionY
                IsInOpenListFlag = true;
                Index_InOpenList = j;
                break;
            end
        end
        if IsInOpenListFlag %如果节点已经在Openlist，则需要判断新的路径的代价函数是不是更高
            if Cost_F < OpenList(Index_InOpenList).Cost_F
                OpenList(Index_InOpenList).Cost_F = Cost_F;
                OpenList(Index_InOpenList).Cost_G = Cost_G;
                OpenList(Index_InOpenList).Father = [Node_Father.PositionX,Node_Father.PositionY];
            end
        else %直接加入OpenList
            OpenList_Num = OpenList_Num + 1;
            OpenList(OpenList_Num).PositionX = PositionX;
            OpenList(OpenList_Num).PositionY = PositionY;
            OpenList(OpenList_Num).Cost_G = Cost_G;
            OpenList(OpenList_Num).Cost_H = Cost_H;
            OpenList(OpenList_Num).Cost_F = Cost_F;
            OpenList(OpenList_Num).Father = [Node_Father.PositionX,Node_Father.PositionY];
        end
        
    end
    %画出OpenList和CloseList
    %PlotGridArea(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,OpenList,OpenList_Num,CloseList,CloseList_Num);
    %判断OpenList是否已经出现了目标节点
    ExistTargetNodeFlag = false;
    for i = 1:OpenList_Num
        if OpenList(i).PositionX == Target_Node_PositionX && OpenList(i).PositionY == Target_Node_PositionY
            ExistTargetNodeFlag = true;
            CloseList_Num = CloseList_Num + 1;%将目标节点放入CloseList以便后面处理
            Target_Node_Cost_F = OpenList(i).Cost_F;
            CloseList(CloseList_Num) = OpenList(i);
        end        
    end
    if ExistTargetNodeFlag %OpenList中如果出现目标点则代表搜索完成找到路径
        break;
    end
    %判断OpenList是否为空
    IsOpenListEmpty = false;
    if OpenList_Num == 0
        IsOpenListEmpty = true;
    end
    if IsOpenListEmpty %OpenList为空则代表没找到路径
        break;
    end
    %挑出代价函数最小的子节点
    Index_Min_Cost = 1;
    Temp_Cost_F = OpenList(1).Cost_F;
    for i = 1:OpenList_Num
        if OpenList(i).Cost_F < Temp_Cost_F
            Temp_Cost_F = OpenList(i).Cost_F;
            Index_Min_Cost = i;
        end        
    end
    %加入CloseList并从OpenList中删除
    CloseList_Num = CloseList_Num + 1;
    CloseList(CloseList_Num) = OpenList(Index_Min_Cost);
    for i = Index_Min_Cost:OpenList_Num
        OpenList(i) = OpenList(i+1);    
    end
    OpenList_Num = OpenList_Num - 1;
    %将该节点作为父节点继续进行搜索
    Node_Father = CloseList(CloseList_Num);    
        
end

%% 循环完成后处理结果
if ExistTargetNodeFlag %OpenList中如果出现目标点则代表搜索完成找到路径,根据CloseList中的数据画出图像
    PositionX = Target_Node_PositionX;
    PositionY = Target_Node_PositionY;
    Path_List = [Target_Node_PositionX,Target_Node_PositionY,Target_Node_Cost_F];
    while 1        
        for i = 1:CloseList_Num % 寻找当前节点的父节点
            if PositionX == CloseList(i).PositionX && PositionY == CloseList(i).PositionY
                PositionX = CloseList(i).Father(1);
                PositionY = CloseList(i).Father(2);        
                break;
            end
        end
        for i = 1:CloseList_Num % 寻找父节点的Cost_F
            if PositionX == CloseList(i).PositionX && PositionY == CloseList(i).PositionY
                Cost_F = CloseList(i).Cost_F;
                Path_List = [[PositionX,PositionY,Cost_F];Path_List];  
                break;
            end
        end        
        if PositionX == Start_Node_PositionX && PositionY == Start_Node_PositionY
            break;
        end
    end
%     PlotPath(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,Path_List)
elseif IsOpenListEmpty %OpenList为空则代表没找到路径
    msgbox('未搜索到路径！');
end
    
end


%% 搜索出路径后，通过画图画出来
function PlotPath(X_Length,Y_Length,Start_Node,Target_Node,Obs_Node_List,Path_List)
figure(2);
global costmap_
map = ~costmap_;
map = rot90(map);
imshow(map);
hold on
title('y')
ylabel('x')
%标记起点与终点
plot(Start_Node(2),Start_Node(1),'g.','MarkerSize',20)
plot(Target_Node(2),Target_Node(1),'r.','MarkerSize',20)
hold on
plot(Path_List(:,2),Path_List(:,1),'g-','LineWidth',2)
%Path_List
% plot(Path_List(:,1)-0.5,Path_List(:,2)-0.5,'*');
% plot(Path_List(:,1)-0.5,Path_List(:,2)-0.5,'-');
%Cost_F
end
