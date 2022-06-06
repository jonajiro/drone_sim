function omgd = u2omgd(u,omg,I)
%u2omgd 外力から角加速度を算出
%   外力から角加速度を算出（角速度と慣性能率を入力）
omgd = inv(I) * (u-cross(omg,I*omg));
end

