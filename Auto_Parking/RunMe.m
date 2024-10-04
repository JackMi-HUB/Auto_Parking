clear; close all; clc
global param_
% fid1 = fopen('plan_flag.txt', 'w');
% delete('plan_flag.txt');
% fid2 = fopen('time_consumption_bb.txt', 'w');
% delete('time_consumption_bb.txt');
% fid3 = fopen('time_consumption_multi.txt', 'w');
% delete('time_consumption_multi.txt');
% fclose(fid1);
% fclose(fid2);
% fclose(fid3);
for i = 1:13
param_.case_id = i;
Planning_and_Optimize_bb();
Static_show();
VehicleAnimation();
end