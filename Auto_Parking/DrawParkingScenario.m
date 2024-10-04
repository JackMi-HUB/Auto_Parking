function DrawParkingScenario()
global obstacle_vertexes_ vehicle_TPBV_ environment_scale_ Nobs
InitializeParams();
figure(1);
set(0, 'DefaultLineLineWidth', 1);
hold on;
box on;
grid minor;
axis equal;
xlabel('x/m'); 
ylabel('y/m');
% xmin = min(params_.task.x0, params_.task.xf) - 15;
% xmax = max(params_.task.x0, params_.task.xf) + 15;
% ymin = min(params_.task.y0, params_.task.yf) - 15;
% ymax = max(params_.task.y0, params_.task.yf) + 15;
xmin = environment_scale_.environment_x_min;
xmax = environment_scale_.environment_x_max;
ymin = environment_scale_.environment_y_min;
ymax = environment_scale_.environment_y_max;
axis([xmin xmax ymin ymax]);
set(gcf, 'outerposition', get(0,'screensize'));

% for jj = 1 : params_.obstacle.num_obs
%     V = params_.obstacle.obs{jj};
%     fill(V.x, V.y, [0.5 0.5 0.5], 'EdgeColor', 'None');
% end
% 
% Arrow([params_.task.x0, params_.task.y0], [params_.task.x0 + cos(params_.task.theta0), params_.task.y0 + sin(params_.task.theta0)], 'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);
% Arrow([params_.task.xf, params_.task.yf], [params_.task.xf + cos(params_.task.thetaf), params_.task.yf + sin(params_.task.thetaf)], 'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);

for ii = 1 : Nobs
    fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
end



Arrow([vehicle_TPBV_.x0, vehicle_TPBV_.y0], [vehicle_TPBV_.x0 + cos(vehicle_TPBV_.theta0), vehicle_TPBV_.y0 + sin(vehicle_TPBV_.theta0)], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
Arrow([vehicle_TPBV_.xtf, vehicle_TPBV_.ytf], [vehicle_TPBV_.xtf + cos(vehicle_TPBV_.thetatf), vehicle_TPBV_.ytf+ sin(vehicle_TPBV_.thetatf)],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);

% Arrow([-19.6245210727969, -4.68199233716475], [-19.6245210727969 + cos(vehicle_TPBV_.theta0), -4.68199233716475 + sin(vehicle_TPBV_.theta0)], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
% Arrow([0.0996168582375461, 20.0229885057471], [0.0996168582375461 + cos(vehicle_TPBV_.thetatf), 20.0229885057471+ sin(vehicle_TPBV_.thetatf)],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);


V = CreateVehiclePolygon(vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0);
plot(V.x, V.y, 'g--', 'LineWidth', 2);

V = CreateVehiclePolygon(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf);
plot(V.x, V.y, 'r--', 'LineWidth', 2);

drawnow
% text(xmin + 2, ymax - 2, ['Case ', num2str(params_.user.case_id)], 'FontSize', 24, 'FontName', 'Arial Narrow', 'FontWeight', 'bold');
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