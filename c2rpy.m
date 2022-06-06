function rpy = c2rpy(c)
%c2rpy 方向余弦行列からロールピッチヨーを算出
%   方向余弦行列からロールピッチヨーを算出
    if 1-abs(c(1,3)) > 2*10^-4
        roll = atan(c(2,3)/c(3,3));
        pitch = asin(-c(1,3));
        yaw = atan(c(1,2)/c(1,1));
    else
        roll = 0;
        pitch = asin(-c(1,3));
        yaw = atan(-c(2,1)/c(2,2));
    end
    rpy = [roll pitch yaw]';
end

