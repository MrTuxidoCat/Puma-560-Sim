clear all
close all
syms th1 th2 th3 th4 th5 th6
%th = [0 0 0 0 0 0];
th = [th1 th2 th3 th4 th5 th6];
al = [-90 0 90 0 -90 90];
d = [660 130 0 -432 0 0];
a = [0 432 0 0 0 0];
T = dhMat(th,al,a,d);
R36 = T{4}*T{5}*T{6}