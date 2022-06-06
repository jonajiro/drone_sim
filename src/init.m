function [State] = init()
%init この関数の概要をここに記述
%   詳細説明をここに記述
%初期状態量
%% 推力

%推力係数
State.fk = 0.052439*9.8;%[N/duty]
State.mk = 0.0063;%[Nm/duty]
State.mk_b = 0.0006;%[Nm/duty]

%重量[kg]
State.m = 0.091;

%慣性モーメント[]
l = 0.1;
l2 = 0.124;
State.m = 0.091;
m = State.m;
g = 9.8;
T = 0.6025; %roll pitch
T2 = 0.676666667;%yaw
State.Ixx = m*g*(l/2)^2/(4*0.16066*(2*pi/T)^2);
State.Iyy = m*g*(l/2)^2/(4*0.16066*(2*pi/T)^2);
State.Izz = m*g*(l2/2)^2/(4*0.16066*(2*pi/T2)^2);
State.I = [State.Ixx 0          0;
            0        State.Iyy  0;
            0        0          State.Izz];

%% 姿勢位置
%初期姿勢
theta0 = 0/180.0*pi;

q4 = ramt2q(1,0,0,theta0);


%初速
V0 = 0.0/3.6;
V0_xyz = [V0;0;0];

%初期指示値
State.del1 = 0.0/180*pi;
State.del2 = 0.0/180*pi;
State.del3 = 0.0/180*pi;
State.del4 = 0.0/180*pi;

%[omgx,omgy,omgz,q1,q2,q3,q4,x_dot,y_dot,z_dot,x,y,z]
State.output = [0;0;0; q4(1);q4(2);q4(3);q4(4); V0_xyz(1);V0_xyz(2);V0_xyz(3); 0;0;0];

State.roll_ref = 0/180.0*pi;
State.pitch_ref = 0/180.0*pi;
State.yaw_ref = 0/180.0*pi;
State.hight_ref = 0;
State.vel = V0;

rpy = c2rpy(q2c(q4));
State.roll = rpy(1);
State.pitch = rpy(2);
State.yaw = rpy(3);

%モニタ用変数
State.data1 = 0;
State.data2 = 0;
State.data3 = 0; 
end

