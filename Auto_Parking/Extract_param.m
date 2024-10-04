function [param] = Extract_param(SS)
global vehicle_TPBV_ environment_scale_ costmap_
param = [];
for i=1:size(SS,2)
    star_points = environment_scale_.environment_x_min + SS{i}(1,1:2).*((environment_scale_.environment_x_max-environment_scale_.environment_x_min)/size(costmap_,1));
    end_points = environment_scale_.environment_x_min + SS{i}(end,1:2).*((environment_scale_.environment_x_max-environment_scale_.environment_x_min)/size(costmap_,1));
    theta0 = atan2((SS{i}(2,2)-SS{i}(1,2)),(SS{i}(2,1)-SS{i}(1,1)));
    thetatf = atan2((SS{i}(end,2)-SS{i}(end-1,2)),(SS{i}(end,1)-SS{i}(end-1,1)));
    param = [param; star_points, theta0, end_points, thetatf];
end

%将全局pose信息赋给端点
param(1,1:3) = [vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0];
param(end,4:6) = [vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf];
end
