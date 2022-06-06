function [out,State] = runge_kutta(f,t,y,State,dt)
%runge_kutta この関数の概要をここに記述
%   詳細説明をここに記述
[k1,State] = f(t,y,State);
[k2,tmp] = f(t+dt/2,y+dt/2*k1,State);
[k3,tmp] = f(t+dt/2,y+dt/2*k2,State);
[k4,tmp] = f(t+dt,y+dt*k3,State);

out = y + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
