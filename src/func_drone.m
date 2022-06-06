function [out,State] = func_drone(t,y,State)
%func_drone この関数の概要をここに記述
%   詳細説明をここに記述

%% 姿勢位置
gpos_offset = 0;%正が重心前方向に移動
v = [y(8);y(9);y(10)];
q4 = [y(4);y(5);y(6);y(7)];
c = q2c(q4);

rpy = c2rpy(q2c(q4));
State.roll = rpy(1);
State.pitch = rpy(2);
State.yaw = rpy(3);

omg = [y(1);y(2);y(3)];
pos = [y(11);y(12);y(13)];

rm1 = [ 0.045; 0.045;-0.01];
rm2 = [ 0.045;-0.045;-0.01];
rm3 = [-0.045;-0.045;-0.01];
rm4 = [-0.045; 0.045;-0.01];

g = 9.8;

Fmg_xyz = c*[0;0;State.m*g];
%%
%制御量
del1 = State.del1;
del2 = State.del2;
del3 = State.del3;
del4 = State.del4;

%%

rm1_xyz = rm1;
norm1 = [0;0;-1];
L1 = State.fk * del1;
L1_xyz = L1*norm1;
M1_xyz = cross(rm1_xyz,L1_xyz);
M1_z = [0;0;State.mk * del1];

rm2_xyz = rm2;
norm2 = [0;0;-1];
L2 = State.fk * del2;
L2_xyz = L2*norm2;
M2_xyz = cross(rm2_xyz,L2_xyz);
M2_z = [0;0;-State.mk * del2];

rm3_xyz = rm3;
norm3 = [0;0;-1];
L3 = State.fk * del3;
L3_xyz = L3*norm3;
M3_xyz = cross(rm3_xyz,L3_xyz);
M3_z = [0;0;State.mk * del3];

rm4_xyz = rm4;
norm4 = [0;0;-1];
L4 = State.fk * del4;
L4_xyz = L4*norm4;
M4_xyz = cross(rm4_xyz,L4_xyz);
M4_z = [0;0;-State.mk * del4];

% M_xyz = Mb_xyz + Mlw_xyz + Mld_xyz*0;
% F_xyz = Db_xyz + Lw_xyz*0 + Ld_xyz*0 + Fmg_xyz;
M_xyz = M1_xyz + M2_xyz + M3_xyz + M4_xyz + M1_z + M2_z + M3_z + M4_z;
F_xyz = L1_xyz + L2_xyz + L3_xyz + L4_xyz + Fmg_xyz;

omgd = State.I\(M_xyz - cross(omg,State.I*omg));
% omgd = [(State.I(3,3)*M_xyz(1)-(State.I(3,3)^2-State.I(2,2)*State.I(3,3))*M_xyz(2)*M_xyz(3))/(State.I(1,1)*State.I(3,3));
%         (M_xyz(2)-(State.I(3,3)-State.I(1,1))*M_xyz(1)*M_xyz(3))/State.I(2,2);
%         (State.I(1,1)*M_xyz(3)-(State.I(1,1)^2-State.I(2,2)*State.I(1,1))*M_xyz(2)*M_xyz(1))/(State.I(1,1)*State.I(3,3))];
qd4 = q2qd(q4,omg);

rpy = c2rpy(q2c(q4));
State.roll = rpy(1);
State.pitch = rpy(2);
State.yaw = rpy(3);

out(1,1) = omgd(1);
out(2,1) = omgd(2);
out(3,1) = omgd(3);
out(4,1) = qd4(1);
out(5,1) = qd4(2);
out(6,1) = qd4(3);
out(7,1) = qd4(4);

Ud = F_xyz/State.m - [omg(2)*v(3) - omg(3)*v(2);
                     omg(3)*v(1) - omg(1)*v(3);
                     omg(1)*v(2) - omg(2)*v(1)];

out(8,1) = Ud(1);
out(9,1) = Ud(2);
out(10,1) =Ud(3);

Xd = c'*v;
out(11,1) = Xd(1);
out(12,1) = Xd(2);
out(13,1) = Xd(3);
end
