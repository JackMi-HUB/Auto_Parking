function curvature_display(x,y)
dx = gradient(x);
dy = gradient(y);
d2x = gradient(dx);
d2y = gradient(dy);
curvature = abs(dx.*d2y - dy.*d2x) ./ (dx.^2 + dy.^2).^(3/2); 
maxValue = max(curvature);
minValue = min(curvature);
% 目标范围
targetMin = 0.3;
targetMax = 0.32;
cc = curvature > 0.3;
% 线性插值
curvature(cc) = ((curvature(cc) - minValue) / (maxValue - minValue)) * (targetMax - targetMin) + targetMin;


% curvature(cc) = (curvature(cc) - m)./(M - m) .*0.3;
% figure;
axis([-26, 26, -26, 26])
hold on;
scatter(x, y, 10, curvature, 'filled');
colormap('jet');
colorbar;
xlabel('x/m'); 
ylabel('y/m');
title('Curvature heat map analysis');
end