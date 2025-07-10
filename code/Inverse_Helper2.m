clear all;close all;
syms th1 th2 th3 th4 th5 th6
syms r11 r12 r13 r21 r22 r23 r31 r32 r33
syms c4 s4 c5 s5 c6 s6;
vars = [r11 r12 r13 r21 r22 r23 r31 r32 r33];
th = [th1 th2 th3 th4 th5 th6];
al = [-90 0 90 180 -90 90];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
T = dhMat(th,al,a,d);
Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
R36 = T{4}*T{5}*T{6}
r13 = R36(1,3);
r23 = R36(2,3);
r33 = R36(3,3);
r32 = R36(3,2);
r31 = R36(3,1);
r11 = R36(1,1);
r22 = R36(2,2);
r21 = R36(2,1);


eq = [c6*c4*c5-s4*s5 == r11,-c4*s5-c5*s4 == r12,s6*c4*c5-s4*s5 == r13,c6*c4*s5+c5*s4 == r21,c4*c5-s4*s5 == r22,s6*c4*s5+c5*s4 == r23,-s6 == r31,0 == r32,c6 == r33];
solve(eq,c4)