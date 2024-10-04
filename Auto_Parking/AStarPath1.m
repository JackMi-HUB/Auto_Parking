function [closelist_,path_list] = AStarPath1()%画图时修改了costmap的指定
global hybrid_astar_ vehicle_TPBV_
begin_config1 = [vehicle_TPBV_.x0,vehicle_TPBV_.y0];
end_config1 = [vehicle_TPBV_.xtf,vehicle_TPBV_.ytf];
path_list = [];
begin_config = Convert2DimConfigToIndex(begin_config1);
end_config = Convert2DimConfigToIndex(end_config1);
grid_space_2D_ = cell(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);
init_node = zeros(1,11);
% Information of each element in each node:
% Dim # | Variable
%  1        x
%  2        y
%  3        f
%  4        g
%  5        h
%  6        is_in_openlist
%  7        is_in_closedlist
%  8-9      index of current node
%  10-11    index of parent node
init_node(1:2) = begin_config;
init_node(4) = 0;
init_node(5) = sum(abs(init_node(1:2) - end_config));
% init_node(3) = init_node(4) + hybrid_astar_.multiplier_H_for_A_star * init_node(5) + 0.001 * randn;
init_node(3) = init_node(4) + init_node(5);
init_node(6) = 1;
init_node(8:9) = Convert2DimConfigToIndex(begin_config1);
init_node(10:11) = [-999,-999];
openlist_ = init_node;
closelist_ = init_node;
goal_ind = Convert2DimConfigToIndex(end_config1);
grid_space_2D_{init_node(8), init_node(9)} = init_node;
expansion_pattern = [-1 1; -1 0; -1 -1; 0 1; 0 -1; 1 1; 1 0; 1 -1];
expansion_length = [1.414; 1; 1.414; 1; 1; 1.414; 1; 1.414] ;
iter = 0;

while ((~isempty(openlist_))&&(iter <= hybrid_astar_.num_nodes_x^2))
    iter = iter + 1;
    % Locate the node with smallest f value in the openlist, and then name
    % it as cur_node and prepare for extension
    cur_node_order = find(openlist_(:,3) == min(openlist_(:,3))); cur_node_order = cur_node_order(1);
    closelist_ = [closelist_;openlist_(cur_node_order,:)];
    cur_node = openlist_(cur_node_order, :);
    cur_config = cur_node(1:2);
    cur_ind = cur_node(8:9);
    cur_g = cur_node(4);
    % Remove cur_node from open list and add it in closed list
    openlist_(cur_node_order, :) = [];
    grid_space_2D_{cur_ind(1), cur_ind(2)}(6) = 0;
    grid_space_2D_{cur_ind(1), cur_ind(2)}(7) = 1;
    for ii = 1 : 8
        child_node_config = cur_config + expansion_pattern(ii,:);
        child_node_config(child_node_config == 0) = 1;%row:find(child_node_config == 0)
        child_node_ind = child_node_config;
        child_node_ind = limted(child_node_ind);
        child_g = cur_g + expansion_length(ii);
        child_h = sum(abs(child_node_config - end_config));
%         child_f = child_g + hybrid_astar_.multiplier_H_for_A_star * child_h;
        child_f = child_g +  child_h;
        child_node_prepare = [child_node_config, child_f, child_g, child_h, 1, 0, child_node_ind, cur_ind];
        % If the child node has been explored ever before
        if (~isempty(grid_space_2D_{child_node_ind(1), child_node_ind(2)}))
            % If the child has been within the closed list, abandon it and continue.
            if (grid_space_2D_{child_node_ind(1), child_node_ind(2)}(7) == 1)
                continue;
            end
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_2D_{child_node_ind(1), child_node_ind(2)}(4) > child_g + 0.1)
                child_node_order1 = find(openlist_(:,8) == child_node_ind(1));
                child_node_order2 = openlist_(child_node_order1,9) == child_node_ind(2);
                openlist_(child_node_order1(child_node_order2), :) = [];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
                
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (Is2DNodeValid(child_node_config, child_node_ind))
                % If the child node is close to the goal point, then exit
                % directly because we only need the length value rather than the path.
                openlist_ = [openlist_; child_node_prepare];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
                if (sum(abs(child_node_ind - goal_ind)) == 0)
                    idx = closelist_(end,10);
                    idy = closelist_(end,11);
                    closelist_ = [closelist_;child_node_prepare];
                    path_list = [closelist_(end,8),closelist_(end,9)];
%                     path_length = child_g;
                    while 1
                        for i = size(closelist_,1):-1:1
                            if idx == closelist_(i,8) && idy == closelist_(i,9)
                                px = closelist_(i,8);
                                py = closelist_(i,9);
                                path_list = [px,py;path_list];
                                idx = closelist_(i,10);
                                idy = closelist_(i,11);
                                closelist_(i,:) = [];
                                break;
                            end
                        end
                        if idx == closelist_(1,8) && idy == closelist_(1,9)
                            break;
                        end
                    end
%                     PlotPath(begin_config,end_config,path_list);
                    return;
                end
            else % If the child node involves collisons
                child_node_prepare(7) = 1;
                child_node_prepare(6) = 0;
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
            end
        end
    end
end
% PlotPath(begin_config1,end_config1,path_list);
% path_length = sum(abs(begin_config - end_config));
end

function is_collision_free = Is2DNodeValid(child_node_config, child_node_ind)
is_collision_free = 1;
global hybrid_astar_ costmap_aex
if (costmap_aex(child_node_ind(1), child_node_ind(2)) == 1)
    is_collision_free = 0;
    return;
end
if ((child_node_config(1) > hybrid_astar_.num_nodes_x) || (child_node_config(1) < 0) || (child_node_config(2) > hybrid_astar_.num_nodes_x) || (child_node_config(2) < 0))
    is_collision_free = 0;
    return;
end
end

function idx = Convert2DimConfigToIndex(config)
global hybrid_astar_ environment_scale_
ind1 = ceil((config(1) - environment_scale_.environment_x_min) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((config(2) - environment_scale_.environment_y_min) / hybrid_astar_.resolution_y) + 1;
idx = [ind1, ind2];
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
idx = [ind1, ind2];
end

function X = limted(x)
global hybrid_astar_
X = x;
if X(1) < 0
    X = 0;
end
if X(2) < 0
    X(1) = 0;
end
if X(1) > hybrid_astar_.num_nodes_x
    X(1) = hybrid_astar_.num_nodes_x;
end
if X(2) > hybrid_astar_.num_nodes_x
    X(2) = hybrid_astar_.num_nodes_x;
end
end

% function PlotPath(Start_Node,Target_Node,Path_List)
% figure(2);
% global costmap_
% map = ~costmap_;
% % map = rot90(map);
% imshow(map);
% 
% hold on
% %标记起点与终点
% plot(Start_Node(2),Start_Node(1),'g.','MarkerSize',20)
% plot(Target_Node(2),Target_Node(1),'r.','MarkerSize',20)
% hold on
% plot(Path_List(:,2),Path_List(:,1),'g-','LineWidth',2)
% end