function [r1,r2,r3,t] = q2ramt(q4)
%q2ramt �N�I�[�^�j�I�������]�p�A��]�����Z�o
%   �N�I�[�^�j�I�������]�p�A��]�����Z�o
t = 2*acos(q4(4));
r1 = q4(1)/sin(t/2);
r2 = q4(2)/sin(t/2);
r3 = q4(3)/sin(t/2);
end

