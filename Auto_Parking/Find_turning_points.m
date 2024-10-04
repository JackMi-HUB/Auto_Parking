function turning_points = Find_turning_points(path)
% smoothed_y = smooth(y, window_size);
dy = diff(path(:,2));
dx = diff(path(:,1));
threshold = 0.001; % 转折点的阈值

turning_points = [];
for i = 2:length(dy)
    if ((dx(i-1) > threshold && dx(i) < threshold) || (dx(i-1) < threshold && dx(i) > threshold)||(dy(i-1) > threshold && dy(i) < threshold) || (dy(i-1) < threshold && dy(i) > threshold)|| ...
         (dx(i-1) > -threshold && dx(i) < -threshold) || (dx(i-1) < -threshold && dx(i) > -threshold)||(dy(i-1) > -threshold && dy(i) < -threshold) || (dy(i-1) < -threshold && dy(i) > -threshold)) 
        turning_points = [turning_points, i];
    end
end
% figure;
% plot(path(:,1),path(:,2));
% hold on;
% scatter(path(turning_points,1), path(turning_points,2), 'r');
% hold off;
%%%case3_broad_points
%     PX = [-19.8237547892720,-11.6551724137931,-5.67816091954023,0.0996168582375461,0.0996168582375461];
%     PY = [-4.88122605363985,3.28735632183908,3.28735632183908,9.06513409961686,20.0229885057471];
end

