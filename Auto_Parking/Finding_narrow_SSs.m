function [CX,PS,SS] = Finding_narrow_SSs(path) %0704:增加对边界的检测
global costmap_
narrow_flag = zeros(size(path,1),1);
cx_temp = [];
dx_temp = [];
CX = {};%complicated
PS = {};%
SS = {};%sum
path = [path,narrow_flag];
d = 40; % 40 is feasible
for i=1:size(path,1)
    if (size(find(costmap_(limitRange(path(i,1)-d):path(i,1),path(i,2))),1) > 0 && size(find(costmap_(path(i,1):limitRange(path(i,1)+d),path(i,2))),1) > 0) ...
    || (size(find(costmap_(path(i,1),limitRange(path(i,2)-d):path(i,2))),2) > 0 && size(find(costmap_(path(i,1),path(i,2):limitRange(path(i,2)+d))),2) > 0) ...
    || (size(find(costmap_(limitRange(path(i,1)-d):path(i,1),path(i,2))),1) > 0 && (path(i,1)+d) > size(costmap_ ,1))...
    || (size(find(costmap_(path(i,1):limitRange(path(i,1)+d),path(i,2))),1) > 0 && (path(i,1)-d) < 0)...
    || (size(find(costmap_(path(i,1),limitRange(path(i,2)-d):path(i,2))),2) > 0 && (path(i,2)+d) > size(costmap_ ,2))...
    || (size(find(costmap_(path(i,1),path(i,2):limitRange(path(i,2)+d))),2) > 0 && (path(i,2)-d) < 0)
        path(i,3) = 1;
        cx_temp = [cx_temp;path(i,:)];
        if size(dx_temp,1) > 0
            dx_temp = [dx_temp;path(i,:)];
            PS = {PS{:},dx_temp};
            SS = {SS{:},dx_temp};
            dx_temp = [];
        end
    else
        if size(cx_temp,1) ~=0
            dx_temp = [cx_temp(end,:);dx_temp;path(i,:)];
        end
        dx_temp = [dx_temp;path(i,:)];
        if size(cx_temp,1) > 0
            CX = {CX{:},cx_temp};
            SS = {SS{:},cx_temp};
            cx_temp = [];
        end
    end
end
if size(cx_temp,1) > 0
    CX = {CX{:},cx_temp};
    SS = {SS{:},cx_temp};
end
if size(dx_temp,1) > 0
    PS = {PS{:},dx_temp};
    SS = {SS{:},dx_temp};
end
for i=1:size(SS,2)
    k = size(SS,2);
    if i > k
        break
    end
    if size(SS{i},1) == 1
        if i == 1
            SS = {SS{2:end}};
        elseif i == size(SS,1)
            SS = {SS{1:i-1}};
        else
            SS={SS{1:i-1},SS{i+1:end}};
        end
        k = k-1;
    end
end
end

function result = limitRange(value)
    global costmap_
    if value < 1
        result = 1;
    elseif value > size(costmap_,1)
        result = size(costmap_,1);
    else
        result = value;
    end
end

% function [CX,PS,SS] = Finding_narrow_SSs(path)
% global costmap_
% narrow_flag = zeros(size(path,1),1);
% cx_temp = [];
% dx_temp = [];
% CX = {};
% PS = {};
% SS = {};
% path = [path,narrow_flag];
% d = 30; % 40 is feasible
% for i=1:size(path,1)
%     if (size(find(costmap_(limitRange(path(i,1)-d):path(i,1),path(i,2))),1) > 0 && size(find(costmap_(path(i,1):limitRange(path(i,1)+d),path(i,2))),1) > 0) ...
%     || (size(find(costmap_(path(i,1),limitRange(path(i,2)-d):path(i,2))),2) > 0 && size(find(costmap_(path(i,1),path(i,2):limitRange(path(i,2)+d))),2) > 0)
%         path(i,4) = 1;
%         cx_temp = [cx_temp;path(i,:)];
%         if size(dx_temp,1) > 0
%             dx_temp = [dx_temp;path(i,:)];
%             PS = {PS{:},dx_temp};
%             SS = {SS{:},dx_temp};
%             dx_temp = [];
%         end
%     else
%         if size(cx_temp,1) ~=0
%             dx_temp = [cx_temp(end,:);dx_temp;path(i,:)];
%         end
%         dx_temp = [dx_temp;path(i,:)];
%         if size(cx_temp,1) > 0
%             CX = {CX{:},cx_temp};
%             SS = {SS{:},cx_temp};
%             cx_temp = [];
%         end
%     end
% end
% if size(cx_temp,1) > 0
%     CX = {CX{:},cx_temp};
%     SS = {SS{:},cx_temp};
% end
% if size(dx_temp,1) > 0
%     PS = {PS{:},dx_temp};
%     SS = {SS{:},dx_temp};
% end
% for i=1:size(SS,2)
%     k = size(SS,2);
%     if i > k
%         break
%     end
%     if size(SS{i},1) == 1
%         if i == 1
%             SS = {SS{2:end}};
%         elseif i == size(SS,1)
%             SS = {SS{1:i-1}};
%         else
%             SS={SS{1:i-1},SS{i+1:end}};
%         end
%         k = k-1;
%     end
% end
% end