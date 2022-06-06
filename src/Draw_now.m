function Draw_now( tmp ,cflg) % ~ は void と一緒
%%
%tmp(1,n)
%1:time
%2:X
%3:Y
%4:Z
%15:q1
%16:q2
%17:q3
%18:q4

h_f = figure(99);
h_f.OuterPosition = [0,0,1000,500];
if cflg == 1
   clf; 
end

modelc = '';

if mod(cflg,6) == 1
    modelc = 'red';
elseif mod(cflg,6) == 2
    modelc = 'green';
elseif mod(cflg,6) == 3
    modelc = 'blue';
elseif mod(cflg,6) == 4
    modelc = 'cyan';
elseif mod(cflg,6) == 5
    modelc = 'magenta';
else
    modelc = 'yellow';
end

[V_body,F_body] = read_vertices_and_faces_from_obj_file('drone.obj');
modelsize = 0.05;%3dモデル描画倍率
V_body = ( V_body' * modelsize)';%初期姿勢修正
i = 1;

hold on;
  
qm = [tmp(1,15);tmp(1,16);tmp(1,17);tmp(1,18)];
V_body_t = (q2c(qm)'*V_body')';
V_body_t(:,1) = V_body_t(:,1) + tmp(i,2);
V_body_t(:,2) = V_body_t(:,2) + tmp(i,3);
V_body_t(:,3) = V_body_t(:,3) + tmp(i,4);


subplot(1,2,1);%上面
hold on
trisurf(F_body,V_body_t(:,1),V_body_t(:,2),V_body_t(:,3),'FaceColor',modelc,'EdgeColor','none');
plot3(tmp(i,11),tmp(i,12),tmp(i,13),'o');
xlabel('ダウンレンジ[m]')
ylabel('クロスレンジ[m]')
zlabel('高度[m]')
dim = [.2 .5 .2 .3];
str = strcat('Time:', num2str(tmp(i,1)));
annotation('textbox',dim,'String',str,'FitBoxToText','on');
light('Position',[-1.0,-1.0,100.0],'Style','infinite');
lighting phong;
xlim([-10 , 80]);
ylim([-10 , 80]);
zlim([-10 , 80]);
grid on;
az = -30;
el = 30;
view(az,el);
pbaspect([1 1 1])

subplot(1,2,2);
%側面
hold on
trisurf(F_body,V_body_t(:,1),V_body_t(:,2),V_body_t(:,3),'FaceColor',modelc,'EdgeColor','none');
plot3(tmp(i,11),tmp(i,12),tmp(i,13),'o');
xlabel('ダウンレンジ[m]')
ylabel('クロスレンジ[m]')
zlabel('高度[m]')

light('Position',[-1.0,-1.0,100.0],'Style','infinite');
lighting phong;

xlim([-10 , 80]);
ylim([-10 , 80]);
zlim([-10 , 80]);
grid on;
az = 30;
el = 45;

view(az,el);
pbaspect([1 1 1])


drawnow
end