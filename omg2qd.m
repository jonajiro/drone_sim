function qd4 = omg2qd(omg,q4)
%omg2qd 角速度からクォータニオン速度を算出
%   角速度からクォータニオン速度を算出
q = zeros(3,1);
qd = zeros(3,1);
qd4 = zeros(3,1);
q(1) = q4(1);
q(2) = q4(2);
q(3) = q4(3);
qd = 1/2*(q4(4)*omg+cross(q,omg));
qd4(1) = qd(1);
qd4(2) = qd(2);
qd4(3) = qd(3);
qd4(4) = -1/2*dot(omg,q);
end

