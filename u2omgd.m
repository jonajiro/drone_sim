function omgd = u2omgd(u,omg,I)
%u2omgd �O�͂���p�����x���Z�o
%   �O�͂���p�����x���Z�o�i�p���x�Ɗ����\������́j
omgd = inv(I) * (u-cross(omg,I*omg));
end

