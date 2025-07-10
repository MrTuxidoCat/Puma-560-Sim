clear all
close all
input=([600, 400, 950, 0, 0, 0; 450, 700, 700, 90, 90, 90]); %replace with an input system later
m = 100; %mass in kg
target=zeros(6,100);
points=zeros(6,100);
flag=zeros(4,101);
for a=1:1:6
   target(a,:)=splitter(input(:,a));
end
target = [zeros(6,1) target];
%% Inverse Kinematics
for n=2:101
    if n == 100
        disp(n);
    end
    [points(:,n),flag(:,n)]=InverseKinematics2(target(:,n),points(:,n-1),flag(n-1));
    disp(n);
    disp(flag(:,n));
end
%% Fancy sim
al = [-90 0 90 90 90 0];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
p0 = [0 0 0 1];
p1 = [0 0 0 1;0 0 220 1];
p2 = [0 0 89 1;0 0 0 1];
p3 = [0 0 0 1];
p4 = [0 0 0 1];
p6 = [50 -50 0 1;0 -50 0 1;0 0 0 1;0 50 0 1;50 50 0 1];
p = zeros(4,13);
org1 = [1 0 0;0 1 0;0 0 1];
org = zeros(3,3);
obj = figure;
hold on
grid on
xlim([-1000 1000]);
ylim([-1000 1000]);
zlim([0 1600]);
view(120,30);
gifFile = 'Sim.gif';

yaw = zeros(1,size(target,2));
pitch = zeros(1,size(target,2));
roll = zeros(1,size(target,2));
pos = zeros(3,size(target,2));
for n=2:1:size(points,2)-1
    th = [points(1,n) points(2,n) points(3,n) points(4,n) points(5,n) points(6,n)];
    T = dhMat(th,al,a,d);
    p(:,1) = p0.';
    p(:,2:3) = T{1}*(p1.');
    p(:,4:5) = T{1}*T{2}*(p2.');
    p(:,6) = T{1}*T{2}*T{3}*(p3.');
    p(:,7) = T{1}*T{2}*T{3}*T{4}*(p4.');
    p(:,8:12) = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*(p6.');
    body = plot3(p(1,1:7),p(2,1:7),p(3,1:7),"LineWidth",2,"Color",'b');
    arm = plot3(p(1,8:12),p(2,8:12),p(3,8:12),"LineWidth",2,"Color",'r');
    plot3(p(1,10),p(2,10),p(3,10),"Color",'g',"Marker",".");
    disp(p(:,10));
    pos(1,n) = p(1,10);
    pos(2,n) = p(2,10);
    pos(3,n) = p(3,10);
    Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
    yaw(n)=rad2deg(atan2(Tmat(2,1),Tmat(1,1)));
    pitch(n)=rad2deg(atan2(-Tmat(3,1),sqrt(Tmat(3,2)^2+Tmat(3,3)^2)));
    roll(n)=rad2deg(atan2(Tmat(3,2),Tmat(3,3)));
    %exportgraphics(obj, gifFile, Append=true);
    frame = getframe(obj);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    % Write to GIF file
    if n == 2
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', inf, 'DelayTime', 0.005);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.005);
    end
    delete(body);
    delete(arm);
end
%% Graphs and shit
% graph position and oriantation
figure
time = linspace(0,100,size(pos,2))*0.05;
tiledlayout(3,2)
nexttile
grid on
plot(time(2:100),pos(1,2:100));
xlabel("time")
ylabel("x position [mm]")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),roll(2:100));
xlabel("time")
ylabel("roll [degrees]")
xlim([0 5]);

nexttile
grid on
plot(time(2:100),pos(2,2:100));
xlabel("time")
ylabel("y position [mm]")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),pitch(2:100));
xlabel("time")
ylabel("pitch [degrees]")
xlim([0 5]);

nexttile
grid on
plot(time(2:100),pos(3,2:100));
xlabel("time")
ylabel("z position [mm]")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),yaw(2:100));
xlabel("time")
ylabel("yaw [degrees]")
xlim([0 5]);
%% angles on time
figure
tiledlayout(3,2)
nexttile
grid on
plot(time(2:100),points(1,2:100));
xlabel("time")
ylabel("$\theta_{1} [degrees]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),points(2,2:100));
xlabel("time")
ylabel("$\theta_{2} [degrees]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),points(1,2:100));
xlabel("time")
ylabel("$\theta_{3} [degrees]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),points(4,2:100));
xlabel("time")
ylabel("$\theta_{4} [degrees]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),points(5,2:100));
xlabel("time")
ylabel("$\theta_{5} [degrees]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),points(6,2:100));
xlabel("time")
ylabel("$\theta_{6} [degrees]$",Interpreter="latex")
xlim([0 5]);

%% numerically calculate speed
velocity = zeros(6,100);
for n=3:99
    velocity(1,n) = (pos(1,n-1)-pos(1,n+1))/0.1;
    velocity(2,n) = (pos(2,n-1)-pos(2,n+1))/0.1;
    velocity(3,n) = (pos(3,n-1)-pos(3,n+1))/0.1;
    velocity(4,n) = (roll(n-1)-roll(n+1))/0.1;
    velocity(5,n) = (pitch(n-1)-pitch(n+1))/0.1;
    velocity(6,n) = (yaw(n-1)-yaw(n+1))/0.1;
    velcrow(1:3,n) = norm(velocity(1:3,n));
end
Anglevelocity = zeros(6,100);
for n=3:99
    Anglevelocity(1,n) = (points(1,n-1)-points(1,n+1))/0.1;
    Anglevelocity(2,n) = (points(2,n-1)-points(2,n+1))/0.1;
    Anglevelocity(3,n) = (points(3,n-1)-points(3,n+1))/0.1;
    Anglevelocity(4,n) = (points(4,n-1)-points(4,n+1))/0.1;
    Anglevelocity(5,n) = (points(5,n-1)-points(5,n+1))/0.1;
    Anglevelocity(6,n) = (points(6,n-1)-points(6,n+1))/0.1;
end

%% calculate with jacobian
jacobVel = zeros(6,100);

for n=3:99
    jacob = JAcobian2(points(1,n),points(2,n),points(3,n),points(4,n),points(5,n),points(6,n));
    jacobVel(:,n) = jacob*Anglevelocity(:,n);% why so highhhhhh
end
%% graph that shit
figure
title("Numerically calculated velocity vector")
tiledlayout(3,2)
nexttile
plot(time(3:99),velocity(1,3:99));
xlabel("time")
ylabel("x velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),velocity(4,3:99));
xlabel("time")
ylabel("roll velocity [degrees/s]")
grid on
xlim([0 5])

nexttile
plot(time(3:99),velocity(2,3:99));
xlabel("time")
ylabel("y velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),velocity(5,3:99));
xlabel("time")
ylabel("pitch velocity [degrees/s]")
grid on
xlim([0 5])

nexttile
plot(time(3:99),velocity(3,3:99));
xlabel("time")
ylabel("z velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),velocity(6,3:99));
xlabel("time")
ylabel("yaw velocity [degrees/s]")
grid on
xlim([0 5])

figure

tiledlayout(3,2)
nexttile
plot(time(3:99),Anglevelocity(1,3:99));
xlabel("time")
ylabel("x velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),Anglevelocity(4,3:99));
xlabel("time")
ylabel("roll velocity [degrees/s]")
grid on
xlim([0 5])

nexttile
plot(time(3:99),Anglevelocity(2,3:99));
xlabel("time")
ylabel("y velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),Anglevelocity(5,3:99));
xlabel("time")
ylabel("pitch velocity [degrees/s]")
grid on
xlim([0 5])

nexttile
plot(time(3:99),Anglevelocity(3,3:99));
xlabel("time")
ylabel("z velocity [mm/s]")
grid on
xlim([0 5])
nexttile
plot(time(3:99),Anglevelocity(6,3:99));
xlabel("time")
ylabel("yaw velocity [degrees/s]")
grid on
xlim([0 5])
%% calculate statics
torque = zeros(6,100);

for n=2:100
    torque(:,n) = statics(JAcobian2(points(1,n),points(2,n),points(3,n),points(4,n),points(5,n),points(6,n)),[0 0 m*9.81],[0 0 0]);
end

figure
tiledlayout(3,2)
nexttile
grid on
plot(time(2:100),torque(1,2:100));
xlabel("time")
ylabel("$\tau_{1} [mm*N]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),torque(2,2:100));
xlabel("time")
ylabel("$\tau_{2} [mm*N]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),torque(1,2:100));
xlabel("time")
ylabel("$\tau_{3} [mm*N]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),torque(4,2:100));
xlabel("time")
ylabel("$\tau_{4} [mm*N]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),torque(5,2:100));
xlabel("time")
ylabel("$\tau_{5} [mm*N]$",Interpreter="latex")
xlim([0 5]);
nexttile
grid on
plot(time(2:100),torque(6,2:100));
xlabel("time")
ylabel("$\tau_{6} [mm*N]$",Interpreter="latex")
xlim([0 5]);
