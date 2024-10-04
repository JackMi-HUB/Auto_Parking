function Static_show()
global obstacle_vertexes_ vehicle_TPBV_ environment_scale_ Nobs param_
if ~param_.plot_flag
    return;
end
x = load("x.txt");
y = load("y.txt");
theta = load("theta.txt");
% curvature_display(x,y);
figure(2);
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


% V = CreateVehiclePolygon(vehicle_TPBV_.x0, vehicle_TPBV_.y0, vehicle_TPBV_.theta0);
% plot(V.x, V.y, 'g--', 'LineWidth', 2);
% 
% V = CreateVehiclePolygon(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, vehicle_TPBV_.thetatf);
% plot(V.x, V.y, 'r--', 'LineWidth', 2);

plot(x,y,'r'); drawnow

for i = 1:size(x,2)
    V = CreateVehiclePolygon(x(i), y(i), theta(i));
    plot(V.x, V.y, 'b', 'LineWidth', 1);
    hold on;
end

figure_handle = figure(2); % 替换为你的图形句柄
file_name = sprintf('Case%d.fig',param_.case_id);
file_name1 = sprintf('Case%d.tif',param_.case_id);
file_path = fullfile('C:\Users\Administrator\Desktop\ampl_test\Result_bb',file_name); % 替换为你的文件路径和文件名
file_path1 = fullfile('C:\Users\Administrator\Desktop\ampl_test\Result_bb',file_name1);
% 使用 saveas 函数保存图形
saveas(figure_handle, file_path);
saveas(figure_handle, file_path1);

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