function [costmap,costmap_ex,costmap_aex] = CreateDilatedCostmap()%根据给定的障碍物顶点填充障碍物图并进行膨胀
%%2023.08.10修改了：粗轨迹+0，优化+1
global environment_scale_ hybrid_astar_ vehicle_geometrics_
xmin = environment_scale_.environment_x_min;
ymin = environment_scale_.environment_y_min;
resolution_x = hybrid_astar_.resolution_x;
resolution_y = hybrid_astar_.resolution_y;
costmap = zeros(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);
global obstacle_vertexes_ %这里是障碍物点集全局变量 n*2

%以下是根据障碍物顶点进行填充
for ii = 1 : size(obstacle_vertexes_,2) %index 2:表示obstacles个数
    vx = obstacle_vertexes_{ii}.x;%赋予障碍物点的x和y坐标系
    vy = obstacle_vertexes_{ii}.y;
    x_lb = min(vx); x_ub = max(vx); y_lb = min(vy); y_ub = max(vy);
    [Nmin_x,Nmin_y] = ConvertXYToIndex(x_lb,y_lb);%将坐标系转换为Index
    [Nmax_x,Nmax_y] = ConvertXYToIndex(x_ub,y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj,kk) == 1)%如果已经为障碍物点，则跳过该点
                continue;
            end
            cur_x = xmin + (jj - 1) * resolution_x; %index（多少个cell） * 分辨率（cm/cell） = 位置
            cur_y = ymin + (kk - 1) * resolution_y;
            if (inpolygon(cur_x, cur_y, obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y) == 1)%若点（cur_x,cur_y）是在边缘内部（包括边缘上），则进行膨胀
                costmap(jj,kk) = 1;
            end
        end
    end
end
length_unit = 0.5 * (resolution_x + resolution_y);%分辨率代表一个最小的cell有多少cm
basic_elem = strel('disk',ceil(vehicle_geometrics_.radius / length_unit)+1);%小车的半径（将要膨胀的半径）
basic_elem_ex = strel('disk',ceil(vehicle_geometrics_.radius / length_unit));%小车的半径（将要膨胀的半径）
basic_elem_aex = strel('disk',ceil(vehicle_geometrics_.radius / length_unit)+1);%小车的半径（将要膨胀的半径）
costmap_ex = imdilate(costmap, basic_elem_ex);
costmap_aex =  imdilate(costmap, basic_elem_aex);
costmap = imdilate(costmap, basic_elem);%对原来的地图进行膨胀
end

function [ind1,ind2] = ConvertXYToIndex(x,y)
global hybrid_astar_ environment_scale_
ind1 = ceil((x - environment_scale_.environment_x_min) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((y - environment_scale_.environment_y_min) / hybrid_astar_.resolution_y) + 1;
if ((ind1 <= hybrid_astar_.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= hybrid_astar_.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
end

