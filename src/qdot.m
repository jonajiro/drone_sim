function q_ans = qdot(q,p)
%qdot クオータニオンの掛け算
%   クオータニオンの掛け算
q_ans = zeros(4,1);
q_ans(4) = q(4)*p(4) - q(1)*p(1) - q(2)*p(2) - q(3)*p(3);
q_ans(1) = q(1)*p(4) + q(4)*p(1) - q(3)*p(2) + q(2)*p(3);
q_ans(2) = q(2)*p(4) + q(3)*p(1) + q(4)*p(2) - q(1)*p(3);
q_ans(3) = q(3)*p(4) - q(2)*p(1) + q(1)*p(2) + q(4)*p(3);
end

