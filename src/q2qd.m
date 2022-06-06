function qd4 = q2qd(q4,omg)
%q2qd 角速度からクォータニオン速度を算出
%   角速度からクォータニオン速度を算出
qd4 = 0.5*[ 0       omg(3) -omg(2) omg(1);
           -omg(3)  0       omg(1) omg(2);
            omg(2) -omg(1)  0      omg(3);
           -omg(1) -omg(2) -omg(3) 0]*q4;
end

