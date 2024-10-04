function Planning_and_Optimize()
clear; close all; clc
global obstacle_vertexes_ vehicle_TPBV_ environment_scale_ param_
% load('no_verify\\case4.mat');%feasible:3(+1,+1) 4(0,+1，mf)
file_name = sprintf('case%d.mat',param_.case_id);
full_name = fullfile ('C:\Users\Administrator\Desktop\ampl_test\ds',file_name);
load(full_name);%feasible:1(0,+1)、2(0,+1)、3(+1,+1,s)、4(0,+1,mf)、5(0,+1)、6(+1,+1，s)、7(0,+1，s)
InitParam();
one_seg_flag = 0;
completeness_flag_s = 0;
completeness_flag_e = 0;
param_.plot_flag = 1;
% colour = [255,0,0;255,125,0;255,255,0;0,255,0;0,0,255;0,255,255;255,0,255];
% c = 1;
% completeness_flag = 0;
% x = [];
% y = [];
% theta = [];
% % Plot basic setups
DrawParkingScenario();
% % Hybrid A* search for a coarse path
% [x, y, theta, path_length, completeness_flag] = SearchHybridAStarPath();
% plot(x,y,'r'); drawnow 
% param4 = [-2.68965517241379,9.86206896551724,0,1.49425287356322,9.86206896551724,0];
%%%%%%%%%%%%%%global%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%[-16.0199005,-13.50746269,0.200398554,-11.39303483,-14.75124378,0.379494744]
% param_g = [23.7642585551331,0.836501901140689,-1.57079632679490,9.46360153256705,5.67816091954023,3.14132932697929];
% param_g = [vehicle_TPBV_.x0, vehicle_TPBV_.y0,vehicle_TPBV_.theta0,vehicle_TPBV_.xtf, vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf];
% % param_g = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf,vehicle_TPBV_.x0, vehicle_TPBV_.y0,vehicle_TPBV_.theta0];
% [x, y, theta, path_length, completeness_flag] = SearchHybridAStarPath(param_g(1,1:3),param_g(1,4:6),10);
% 
% plot(x,y,'m'); drawnow
% curvature_display(x,y);
%%%%%%%%%%%%%%%test%%%%%%%%%%%%%%%%%
% for i = 1:11
%     if c > 7
%         c = i-7;
%     end
%     param_g(3) = pi/6*i;
%     [x, y, theta, path_length, completeness_flag] = SearchHybridAStarPath(param_g(1,1:3),param_g(1,4:6));
%     plot(x,y,'Color',colour(c,:)./255); drawnow
%     c = c+1;
% end
%%%%%%%%%%%%%segments%%%%%%%%%%%%%%%%%
% if ~completeness_flag
% if 1  %test
tic
[~,path] = AStarPath1();
%     path = AStarSearch();
[CX,WX,SS] = Finding_narrow_SSs(path);%由于该函数的特殊性，因此此处需要代入path在grid map下的坐标
path = environment_scale_.environment_x_min + path.*((environment_scale_.environment_x_max-environment_scale_.environment_x_min)/size(costmap_,1));
% plot(path(:,1),path(:,2),'c'); drawnow
% param = Extract_param(SS);
%%%%%%%draw_narrow_segment%%%%%%%%%%
for i=1:size(CX,2)
    CX{i} = environment_scale_.environment_x_min + CX{i}.*((environment_scale_.environment_x_max-environment_scale_.environment_x_min)/size(costmap_,1));
%     plot(CX{i}(:,1),CX{i}(:,2),'m'); drawnow
end
for i=1:size(SS,2)
    SS{i} = environment_scale_.environment_x_min + SS{i}.*((environment_scale_.environment_x_max-environment_scale_.environment_x_min)/size(costmap_,1));
end
for i = size(path,1):-1:1
    if path(i,1) == SS{end}(1,1) && path(i,2) == SS{end}(1,2)
        last_segment_id = i;
        break;
    end
end
%%%%%%%b-spline%%%%%%%%%%
star_index = size(SS{1},1);
Tpoints_index = Find_turning_points(path(star_index:last_segment_id,:)) + (star_index-1);
Tpoints_modify = Smooth_path(Tpoints_index);
PX = path([size(SS{1},1),Tpoints_modify,last_segment_id],1);
PY = path([size(SS{1},1),Tpoints_modify,last_segment_id],2);
PX = PX';
PY = PY';
if size(SS,2) ~= 1 && size(SS,2) ~= 2%0819:修改为一二段视为全局
    if size(PX,2) == 2
        PX = [PX,path(last_segment_id+1,1)];
        PY = [PY,path(last_segment_id+1,1)];
        [x,y,theta] = ordinary_path(PX,PY);
    else
%         PX  = [-15.0421455938697,-11.4559386973180,-5.87739463601533,0.0996168582375461,0.0996168582375461];
%         PY = [-0.0996168582375496,3.48659003831417,3.48659003831417,9.46360153256705,13.0498084291188];
        [x,y,theta] = Spline_smooth(PX,PY);
    end
    plot(x,y,'r'); drawnow
%     curvature_display(x,y);
%%%%%%%H-a_star_segment%%%%%%%%%%
%     for i=1:size(param,1)
%         [xtemp, ytemp, theta_temp, path_length_temp, completeness_flag_temp] = SearchHybridAStarPath(param(i,1:3),param(i,4:6));
%         plot(xtemp,ytemp,'r'); drawnow
%         x = [x,xtemp];
%         y = [y,ytemp];
%         theta = [theta,theta_temp];
%     end

%%%%%%%%%%%%F-HA*%%%%%%%%%%%%%
    param_first = [vehicle_TPBV_.x0, vehicle_TPBV_.y0,vehicle_TPBV_.theta0,SS{1}(end,1),SS{1}(end,2),theta(1)];
    [xs, ys, theta_s, ~, completeness_flag_s] = SearchHybridAStarPath(param_first(1,1:3),param_first(1,4:6),2);
    if ~completeness_flag_s
        [xs, ys, theta_s, ~, completeness_flag_s] = SearchHybridAStarPath(param_first(1,4:6),param_first(1,1:3),2);
        xs = flip(xs);
        ys = flip(ys);
        theta_s = flip(theta_s);
    end
    plot(xs,ys,'g'); drawnow
%%%%%%%%%%%%L-HA*%%%%%%%%%%%%%    
    param_last = [path(last_segment_id,1),path(last_segment_id,2),theta(end),vehicle_TPBV_.xtf, vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf];
    [xe, ye, theta_e, ~, completeness_flag_e] = SearchHybridAStarPath(param_last(1,1:3),param_last(1,4:6),2);
else
    one_seg_flag = 1;
    param_last = [vehicle_TPBV_.x0, vehicle_TPBV_.y0,vehicle_TPBV_.theta0,vehicle_TPBV_.xtf, vehicle_TPBV_.ytf,vehicle_TPBV_.thetatf];
    [xe, ye, theta_e, ~, completeness_flag_e] = SearchHybridAStarPath(param_last(1,1:3),param_last(1,4:6),10);
end


if ~completeness_flag_e
    [xe, ye, theta_e, ~, completeness_flag_e] = SearchHybridAStarPath(param_last(1,4:6),param_last(1,1:3),10);
    xe = flip(xe);
    ye = flip(ye);
    theta_e = flip(theta_e);
end

if ~completeness_flag_e %0818:此处应当flap后再进行增距尝试，否则三段SS恐有后患
    for i = last_segment_id:-1:1
        if path(i,1) == SS{end-1}(1,1) && path(i,2) == SS{end-1}(1,2)
            last_before_segment_id = i;
            break;
        end
    end
    Tpoints_modify = Tpoints_modify(Tpoints_modify <= last_before_segment_id);
    PX = path([size(SS{1},1),Tpoints_modify,last_before_segment_id],1);
    PY = path([size(SS{1},1),Tpoints_modify,last_before_segment_id],2);
    PX = PX';
    PY = PY';
    [x,y,theta] = Spline_smooth(PX,PY);
    plot(x,y,'k'); drawnow
    param_last(1,1:3) = [path(last_before_segment_id,1),path(last_before_segment_id,2),theta(end)];
    [xe, ye, theta_e, ~, completeness_flag_e] = SearchHybridAStarPath(param_last(1,1:3),param_last(1,4:6),10);
end
toc
fid1 = fopen('time_consumption_multi.txt', 'a');
fprintf(fid1, '%g %g \r\n', param_.case_id , toc);
fclose(fid1);
plot(xe,ye,'g'); drawnow

if (one_seg_flag && completeness_flag_e) || (completeness_flag_e && completeness_flag_s)
    fid = fopen('plan_flag.txt', 'a');
    fprintf(fid, '%g %g \r\n', param_.case_id , 1);
    fclose(fid);
else
    param_.plot_flag = 0;
    fid = fopen('plan_flag.txt', 'a');
    fprintf(fid, '%g %g \r\n', param_.case_id , 0);
    fclose(fid);
    return;

end

if ~one_seg_flag 
    x = [xs,x(2:end),xe(2:end)];%,x,xe
    y = [ys,y(2:end),ye(2:end)];%,y,ye
    theta = [theta_s,theta(2:end),theta_e(2:end)];%,theta,theta_e
else
    x = xe;
    y = ye;
    theta = theta_e;
end

plot(x,y,'r'); drawnow
% end %*****************END_PLANNINNG***********************

% [x,y,theta] = Spline_smooth_all(PX,PY);
% plot(x,y,'c');
[x, y, theta, v, a, phy, w, tf] = ResamplePath(x, y, theta);
% plot(x,y,'g'); drawnow
tic
[bvr, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta);
toc 
plotclosePolygon(bvr);
WriteInitialGuess(x, y, theta, xr, yr, xf, yf, v, a, phy, w, tf);
WriteBoundaryValues();
%IPOPT的默认迭代次数为3000。无解，待尝试：初始规划+0，优化为+1
tic
!ampl rr.run
toc
data1 = importdata('x.txt');
data2 = importdata('y.txt');
plot(data1,data2,'b'); drawnow

end