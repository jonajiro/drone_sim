function gen_anime( data, target ,filename)
%gen_anime アニメーション生成
%   詳細説明をここに記述

[a ,~ ,c] = size(data);
skip_cnt = 100;
nframe = (a-1)/skip_cnt;
% preallocate
fr(1:round(nframe)) = struct('cdata',[],'colormap',[]);
cycle_cnt = 1;
% draw each step
for i = 1:skip_cnt:a
    %tmp(1,n)
    %1:time
    %2:X
    %3:Y
    %4:Z
    %5:roll[deg]
    %6:pitch[deg]
    %7:yaw[deg]
    %8:roll rate[rad/s]
    %9:pitch rate[rad/s]
    %10:yaw rate[rad/s]
    %11:目標位置クロスレンジ[m]
    %12:目標位置ダウンレンジ[m]
    %13:目標位置高度[m]
    %14:描画最高高度[m]
    %15:q1
    %16:q2
    %17:q3
    %18:q4

for j = 1:c
    tmp(1,1) = data(i,1,j);
    tmp(1,2) = data(i,12,j);
    tmp(1,3) = data(i,13,j);
    tmp(1,4) = -data(i,14,j);
    tmp(1,5) = rad2deg(data(i,23,j));
    tmp(1,6) = rad2deg(-data(i,24,j));
    tmp(1,7) = rad2deg(data(i,25,j));
    tmp(1,8) = rad2deg(0);
    tmp(1,9) = rad2deg(0);
    tmp(1,10) = rad2deg(0);
    tmp(1,11) = target.TARGXI;
    tmp(1,12) = target.TARGETA;
    tmp(1,13) = target.TARGALT;
    tmp(1,14) = target.MAXALT;
    tmp(1,15) = -data(i,5,j);
    tmp(1,16) = -data(i,6,j);
    tmp(1,17) = data(i,7,j);
    tmp(1,18) = data(i,8,j);
    Draw_now( tmp ,j);
end
    fr(cycle_cnt) = getframe(99);
    cycle_cnt = cycle_cnt +1;
end

% make an animation
mv = VideoWriter(filename,'MPEG-4');
dt = data(skip_cnt+1,1) - data(1,1);
mv.FrameRate = 1/dt; % ← fpsと同じ
open(mv)
writeVideo(mv,fr)
close(mv)
% close(figure(99));

end

