clear all;
close all;
warning('off','all');
%%
t = 0;
dt = 0.001;
State = init();
clearvars data;
cnt = 1;
data(cnt,1) = t;
data(cnt,2:14) = State.output';
data(cnt,15) = State.del1;
data(cnt,16) = State.del2;
data(cnt,17) = State.del3;
data(cnt,18) = State.del4;
data(cnt,19) = State.roll_ref;
data(cnt,20) = State.pitch_ref;
data(cnt,21) = State.yaw_ref;
data(cnt,22) = State.vel; 
data(cnt,23) = State.roll;
data(cnt,24) = State.pitch;
data(cnt,25) = State.yaw;
data(cnt,26) = State.data1;
data(cnt,27) = State.data2;
data(cnt,28) = State.data3; 
data(cnt,29) = 0;
data(cnt,30) = 0;
data(cnt,31) = 0;
cnt = cnt + 1;

ct = 0;
ct_cycle = 0.01;

u = [0;0;0];

st_tseq = [1 5.0 10.0 15.0 20.0];

for t=0+dt:dt:25
    t
    %ŒvZ
    [output,State] = runge_kutta(@func_drone,t,State.output,State,dt);
    State.output = output;
    data(cnt,1) = t;
    data(cnt,2:14) = State.output';
    State.vel = norm([data(cnt,9);data(cnt,10);data(cnt,11)]);
    
    
    if t < st_tseq(1)
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = -1 * pi / 180.0;

        State.hight_ref = 0.0;
    elseif (t >= st_tseq(1))&&(t < st_tseq(2))
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = -10 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(2))&&(t < st_tseq(3))
        State.roll_ref =  10 * pi / 180.0;
        State.pitch_ref = 10 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(3))&&(t < st_tseq(4))
        State.roll_ref = -10 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(4))&&(t < st_tseq(5))
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 90 * pi / 180.0;

        State.hight_ref = -1.0;
    else
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = 0.0;
    end
        
    
    %§Œä‘¥
    if ct > ct_cycle
        q4 = data(cnt,5:8)';
        omg = data(cnt,2:4)';
        omgt = [0 0 0]';
        q4t = c2q(rpy2c([State.roll_ref;State.pitch_ref;State.yaw_ref]));

        alp = 0.01;
        beta = 0.002;
        q4a = [-q4(1) -q4(2) -q4(3) q4(4)]';
        q4err = qdot(q4t,q4a);
        qerr = zeros(3,1);
        qerr(1) = q4err(1);
        qerr(2) = q4err(2);
        qerr(3) = q4err(3);

        u = alp*qerr + beta*(omgt-omg);

        del_com_roll = u(1);
        del_com_pitch = u(2);
        del_com_yaw = u(3);

        hp = -1.0;
        hd = -1.0;
        del_com_t = hp * (State.hight_ref - data(cnt,14)) + hd * (0 - data(cnt,11));
        del_com_t = del_com_t / 4;

        State.del1 = -del_com_roll + del_com_pitch + del_com_yaw + del_com_t;
        State.del2 = +del_com_roll + del_com_pitch - del_com_yaw + del_com_t;
        State.del3 = +del_com_roll - del_com_pitch + del_com_yaw + del_com_t;
        State.del4 = -del_com_roll - del_com_pitch - del_com_yaw + del_com_t;

        if State.del1 > 1
            State.del1 = 1;
        elseif State.del1 < 0
            State.del1 = 0;
        end

        if State.del2 > 1
            State.del2 = 1;
        elseif State.del2 < 0
            State.del2 = 0;
        end

        if State.del3 > 1
            State.del3 = 1;
        elseif State.del3 < 0
            State.del3 = 0;
        end

        if State.del4 > 1
            State.del4 = 1;
        elseif State.del4 < 0
            State.del4 = 0;
        end    
        ct = 0;
    else
        ct = ct + dt;
    end

    data(cnt,15) = State.del1;
    data(cnt,16) = State.del2;
    data(cnt,17) = State.del3;
    data(cnt,18) = State.del4;
    data(cnt,19) = State.roll_ref;
    data(cnt,20) = State.pitch_ref;
    data(cnt,21) = State.yaw_ref;
    data(cnt,22) = State.vel; 
    data(cnt,23) = State.roll;
    data(cnt,24) = State.pitch;
    data(cnt,25) = State.yaw;
    data(cnt,26) = State.data1;
    data(cnt,27) = State.data2;
    data(cnt,28) = State.data3; 
    
    data(cnt,29) = u(1);
    data(cnt,30) = u(2);
    data(cnt,31) = u(3);
    if data(cnt,14) > 10.0
        break;
    end
    
    cnt = cnt + 1;
end
data1 = data; 

fig = figure(1);
subplot(3,1,1);
hold on;
plot(data(:,1),data(:,23)*180/pi)%roll
plot(data(:,1),data(:,24)*180/pi)%pitch
plot(data(:,1),data(:,25)*180/pi)%yaw
grid on;
xlabel('ŠÔ[sec]')
ylabel('p¨[deg]')
legend('ƒ[ƒ‹Šp','ƒsƒbƒ`Šp','ƒˆ[Šp')
subplot(3,1,2);
hold on;
plot(data(:,1),data(:,15)*100)%1
plot(data(:,1),data(:,16)*100)%2
plot(data(:,1),data(:,17)*100)%3
plot(data(:,1),data(:,18)*100)%4
grid on;
xlabel('ŠÔ[sec]')
ylabel('„—Í[%]')
legend('mot1','mot2','mot3','mot4')
subplot(3,1,3);
hold on;
plot(data(:,1),-data(:,14))%‹@‘Ì‘¬“x
grid on;
xlabel('ŠÔ[sec]')
ylabel('‚“x[m]')
saveas(fig,'drone1.jpg');

%%
t = 0;
dt = 0.001;
State = init();
clearvars data;
cnt = 1;
data(cnt,1) = t;
data(cnt,2:14) = State.output';
data(cnt,15) = State.del1;
data(cnt,16) = State.del2;
data(cnt,17) = State.del3;
data(cnt,18) = State.del4;
data(cnt,19) = State.roll_ref;
data(cnt,20) = State.pitch_ref;
data(cnt,21) = State.yaw_ref;
data(cnt,22) = State.vel; 
data(cnt,23) = State.roll;
data(cnt,24) = State.pitch;
data(cnt,25) = State.yaw;
data(cnt,26) = State.data1;
data(cnt,27) = State.data2;
data(cnt,28) = State.data3; 
data(cnt,29) = 0;
data(cnt,30) = 0;
data(cnt,31) = 0;
cnt = cnt + 1;

ct = 0;
ct_cycle = dt;

u = [0;0;0];

st_tseq = [1 5.0 10.0 15.0 20.0];

for t=0+dt:dt:25
    t
    %ŒvZ
    [output,State] = runge_kutta(@func_drone,t,State.output,State,dt);
    State.output = output;
    data(cnt,1) = t;
    data(cnt,2:14) = State.output';
    State.vel = norm([data(cnt,9);data(cnt,10);data(cnt,11)]);
    
    if t < st_tseq(1)
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = -1 * pi / 180.0;

        State.hight_ref = 0.0;
    elseif (t >= st_tseq(1))&&(t < st_tseq(2))
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = -10 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(2))&&(t < st_tseq(3))
        State.roll_ref =  10 * pi / 180.0;
        State.pitch_ref = 10 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(3))&&(t < st_tseq(4))
        State.roll_ref = -10 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = -1.0;
    elseif (t >= st_tseq(4))&&(t < st_tseq(5))
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 90 * pi / 180.0;

        State.hight_ref = -1.0;
    else
        State.roll_ref = 0 * pi / 180.0;
        State.pitch_ref = 0 * pi / 180.0;
        State.yaw_ref = 0 * pi / 180.0;

        State.hight_ref = 0.0;
    end
        
    
    %§Œä‘¥
    if ct > ct_cycle
        q4 = data(cnt,5:8)';
        omg = data(cnt,2:4)';
        omgt = [0 0 0]';
        q4t = c2q(rpy2c([State.roll_ref;State.pitch_ref;State.yaw_ref]));

        alp = 0.1;
        beta = 0.02;
        q4a = [-q4(1) -q4(2) -q4(3) q4(4)]';
        q4err = qdot(q4t,q4a);
        qerr = zeros(3,1);
        qerr(1) = q4err(1);
        qerr(2) = q4err(2);
        qerr(3) = q4err(3);

        u = alp*qerr + beta*(omgt-omg);

        del_com_roll = u(1);
        del_com_pitch = u(2);
        del_com_yaw = u(3);

        hp = -10.0;
        hd = -10.0;
        del_com_t = hp * (State.hight_ref - data(cnt,14)) + hd * (0 - data(cnt,11));
        del_com_t = del_com_t / 4;

        State.del1 = -del_com_roll + del_com_pitch + del_com_yaw + del_com_t;
        State.del2 = +del_com_roll + del_com_pitch - del_com_yaw + del_com_t;
        State.del3 = +del_com_roll - del_com_pitch + del_com_yaw + del_com_t;
        State.del4 = -del_com_roll - del_com_pitch - del_com_yaw + del_com_t;

        if State.del1 > 1
            State.del1 = 1;
        elseif State.del1 < 0
            State.del1 = 0;
        end

        if State.del2 > 1
            State.del2 = 1;
        elseif State.del2 < 0
            State.del2 = 0;
        end

        if State.del3 > 1
            State.del3 = 1;
        elseif State.del3 < 0
            State.del3 = 0;
        end

        if State.del4 > 1
            State.del4 = 1;
        elseif State.del4 < 0
            State.del4 = 0;
        end    
        ct = 0;
    else
        ct = ct + dt;
    end

    data(cnt,15) = State.del1;
    data(cnt,16) = State.del2;
    data(cnt,17) = State.del3;
    data(cnt,18) = State.del4;
    data(cnt,19) = State.roll_ref;
    data(cnt,20) = State.pitch_ref;
    data(cnt,21) = State.yaw_ref;
    data(cnt,22) = State.vel; 
    data(cnt,23) = State.roll;
    data(cnt,24) = State.pitch;
    data(cnt,25) = State.yaw;
    data(cnt,26) = State.data1;
    data(cnt,27) = State.data2;
    data(cnt,28) = State.data3; 
    
    data(cnt,29) = u(1);
    data(cnt,30) = u(2);
    data(cnt,31) = u(3);
    if data(cnt,14) > 10.0
        break;
    end
    
    cnt = cnt + 1;
end
data2 = data;

fig = figure(2);
subplot(3,1,1);
hold on;
plot(data(:,1),data(:,23)*180/pi)%roll
plot(data(:,1),data(:,24)*180/pi)%pitch
plot(data(:,1),data(:,25)*180/pi)%yaw
grid on;
xlabel('ŠÔ[sec]')
ylabel('p¨[deg]')
legend('ƒ[ƒ‹Šp','ƒsƒbƒ`Šp','ƒˆ[Šp')
subplot(3,1,2);
hold on;
plot(data(:,1),data(:,15)*100)%1
plot(data(:,1),data(:,16)*100)%2
plot(data(:,1),data(:,17)*100)%3
plot(data(:,1),data(:,18)*100)%4
grid on;
xlabel('ŠÔ[sec]')
ylabel('„—Í[%]')
legend('mot1','mot2','mot3','mot4')
subplot(3,1,3);
hold on;
plot(data(:,1),-data(:,14))%‹@‘Ì‘¬“x
grid on;
xlabel('ŠÔ[sec]')
ylabel('‚“x[m]')
saveas(fig,'drone2.jpg');


target.TARGXI = 0.0;
target.TARGETA = 0.0;
target.TARGALT = 0.0;
target.MAXALT = 0.0;

plotdata(:,:,1) = data1;
plotdata(:,:,2) = data2;
gen_anime( plotdata, target ,'anime.mp4');
