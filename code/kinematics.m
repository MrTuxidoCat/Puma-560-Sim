clear all
close all
syms th1 th2 th3 th4 th5 th6
target = [864 130 660 180 0 0];
[th,flag] = InverseKinematics2(target,[0 0 0 0 0 0].',1);
%th = [0 0 -180 0 0 0];
%th = [th1 th2 th3 th4 th5 th6];
al = [-90 0 90 90 90 0];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
T = dhMat(th.',al,a,d);
Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}
%%coords for visual
p0 = [0 0 0 1];
p1 = [0 0 0 1;0 0 220 1];
p2 = [0 0 89 1;0 0 0 1];
p3 = [0 0 0 1];
p4 = [0 0 0 1];
p6 = [50 -50 0 1;0 -50 0 1;0 0 0 1;0 50 0 1;50 50 0 1];
p = zeros(4,13);
p(:,1) = p0.';
p(:,2:3) = T{1}*(p1.');
p(:,4:5) = T{1}*T{2}*(p2.');
p(:,6) = T{1}*T{2}*T{3}*(p3.');
p(:,7) = T{1}*T{2}*T{3}*T{4}*(p4.');
p(:,8:12) = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*(p6.');

figure
hold on
grid on
plot3(p(1,1:7),p(2,1:7),p(3,1:7),'LineWidth',2);
plot3(p(1,8:12),p(2,8:12),p(3,8:12),'LineWidth',2);
xlim([-1000 1000]);
ylim([-1000 1000]);
zlim([0 1600]);
view(30,30);

pc = zeros(4,24);
pcoords = [0 0 0 1;100 0 0 1; 0 100 0 1; 0 0 100 1];
pc(:,1:4) = T{1}*(pcoords.');
pc(:,5:8) = T{1}*T{2}*(pcoords.');
pc(:,9:12) = T{1}*T{2}*T{3}*(pcoords.');
pc(:,13:16) = T{1}*T{2}*T{3}*T{4}*(pcoords.');
pc(:,17:20) = T{1}*T{2}*T{3}*T{4}*T{5}*(pcoords.');
pc(:,21:24) = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*(pcoords.');

for n=6
    plot3([pc(1,1+4*(n-1)) pc(1,2+4*(n-1))],[pc(2,1+4*(n-1)) pc(2,2+4*(n-1))],[pc(3,1+4*(n-1)) pc(3,2+4*(n-1))],"Color",'r');
    plot3([pc(1,1+4*(n-1)) pc(1,3+4*(n-1))],[pc(2,1+4*(n-1)) pc(2,3+4*(n-1))],[pc(3,1+4*(n-1)) pc(3,3+4*(n-1))],"Color",'g');
    plot3([pc(1,1+4*(n-1)) pc(1,4+4*(n-1))],[pc(2,1+4*(n-1)) pc(2,4+4*(n-1))],[pc(3,1+4*(n-1)) pc(3,4+4*(n-1))],"Color",'b');
end

disp( p(:,10));
yaw=rad2deg(atan2(Tmat(2,1),Tmat(1,1)))
pitch=rad2deg(atan2(-Tmat(3,1),sqrt(Tmat(3,2)^2+Tmat(3,3)^2)))
roll=rad2deg(atan2(Tmat(3,2),Tmat(3,3)))
