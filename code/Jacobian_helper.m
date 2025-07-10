clear all
close all
syms th1 th2 th3 th4 th5 th6
th = [th1 th2 th3 th4 th5 th6];
al = [-90 0 90 0 -90 90];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
z = [0 0 1 1];
p0 = [0 0 0 1];
T = dhMat(th,al,a,d);
Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
T0{2} = T{1};
T0{3} = T{1}*T{2};
T0{4} = T{1}*T{2}*T{3};
T0{5} = T{1}*T{2}*T{3}*T{4};
T0{6} = T{1}*T{2}*T{3}*T{4}*T{5};
T0{7} = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
p4 = T{1}*T{2}*T{3}*T{4}*p0.';
th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;
for n=1:6
    %J(1,n) = diff(Tmat(1,4),th(n));
    %J(2,n) = diff(Tmat(2,4),th(n));
    %J(3,n) = diff(Tmat(3,4),th(n));
    Jh = T{1};
    if n>1
        r = Tmat*(p0.')-T0{n}*(p0.');
        w = T0{n}(1:3,1:3)*(z(1:3).');
    elseif n==1
        r = Tmat*(p0.');
        w = (z(1:3).');
    end
    w2 = T0{n+1};
    J(1:3,n) = cross(w(1:3),r(1:3));
    J(4:6,n) = w(1:3);
end
disp(J)

subs(p4)