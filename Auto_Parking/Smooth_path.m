% TPS = [72,135,136,145,146,147,159,160,182];
% a = Smooth_path1(TPS);
function tps = Smooth_path(TPS)
tps = [];
i = 1;

if size(TPS,2) == 1
    tps = TPS;
end

while(i<=size(TPS,2) && size(TPS,2)>1)
    if abs(TPS(i+1) - TPS(i)) <= 4
        for t = i:(size(TPS,2)-1)
            if abs(TPS(t+1) - TPS(t)) > 4
                tps = [tps,TPS(i),TPS(t)];
                i = t+1;
                break;
            end
            if t == size(TPS,2)-1
                tps = [tps,TPS(i),TPS(t+1)];
                i = t+2;%end_function
            end
        end
    else
        tps = [tps,TPS(i)];
        i = i+1;
    end
    if i == size(TPS,2)
        tps = [tps,TPS(i)];
        i = i+1; %end_function
    end
end
end
