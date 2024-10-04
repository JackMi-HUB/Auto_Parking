function plotclosePolygon(pointList)%pointList: xmin, xmax, ymin, ymax
a=pointList;  %要连接的点坐标 x;y
m = size(a,1);
for i=1:m
    line([a(i,1),a(i,2)],[a(i,4),a(i,4)]);  %连接节点line([x1,x2],[y1,y2])
    line([a(i,2),a(i,2)],[a(i,4),a(i,3)]);
    line([a(i,2),a(i,1)],[a(i,3),a(i,3)]);
    line([a(i,1),a(i,1)],[a(i,3),a(i,4)]);
    hold on
end
end