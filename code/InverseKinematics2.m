%close all;clear all;
function [res,flag]=InverseKinematics2(input,guessAngles,flagPrev)
al = [-90 0 90 90 90 0];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
p0 = [0 0 0 1];

target = [input(1) input(2) input(3) 0 0 0];
theta_min = [-160 -225 -225 -110 -100 -266].';
theta_max = [ 160  45  45  170  100  266].';

if guessAngles(1) == 0 || flagPrev ==1
    initState = [432 130 228 180 0 0];
    initAngle = [0 0 0 0 0 0].';
else
    initAngle = guessAngles;
    T = dhMat(guessAngles.',al,a,d);
    Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
    p = Tmat*p0.';
    yaw=atan2(Tmat(2,1),Tmat(1,1));
    pitch=atan2(-Tmat(3,1),sqrt(Tmat(3,2)^2+Tmat(3,3)^2));
    roll=atan2(Tmat(3,2),Tmat(3,3));
    initState = [p(1) p(2) p(3) roll pitch yaw];
end
currentAngle = initAngle;
currentState = initState;
frac = 1/1;
del = target-currentState;
errorValpos = 0.1;
errorValang = 5;
smolDel = del*frac;

stuckIts = 10;
stuck = 0;

maxIt = 10000;
k = 0;
while (~((abs(del(1)) < errorValpos) && (abs(del(2)) < errorValpos) && (abs(del(3)) < errorValpos))) && (k < (maxIt))
    inAngle = currentAngle.';
    J = JAcobian2(currentAngle(1),currentAngle(2),currentAngle(3),currentAngle(4),currentAngle(5),currentAngle(6));
    Jplus = pinv(J);
    delAngle = Jplus*smolDel.';

    currentAngle = delAngle+currentAngle;
    currentAngle = min(max(currentAngle, theta_min), theta_max);

    T = dhMat(inAngle,al,a,d);
    Tmat = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};
    p = Tmat*p0.';

    currentState = [p(1) p(2) p(3) 0 0 0];
    del = target-currentState;
    smolDel = del*frac;
    k=k+1;
end
if ~((abs(del(1)) < errorValpos) && (abs(del(2)) < errorValpos) && (abs(del(3)) < errorValpos))
    flag(1,1) = 1;
else
    flag(1,1) = 0;
end
res = currentAngle;

T = dhMat(currentAngle.',al,a,d);
R30 = (T{1}*T{2}*T{3}).';

RtargetRoll = [1, 0, 0;
               0, cosd(input(4)), -sind(input(4));
               0, sind(input(4)),cosd(input(4))];
RtargetPitch = [cosd(input(5)), 0, sind(input(5));
                0, 1, 0;
                -sind(input(5)), 0, cosd(input(5))];
RtargetYaw = [cosd(input(6)), -sind(input(6)), 0;
              sind(input(6)), cosd(input(6)), 0;
              0, 0, 1];
Rtarget = R30(1:3,1:3)*RtargetYaw*RtargetPitch*RtargetRoll;

r13 = Rtarget(1,3);
r23 = Rtarget(2,3);
r33 = Rtarget(3,3);
r32 = Rtarget(3,2);
r31 = Rtarget(3,1);
r21 = Rtarget(2,1);
r11 = Rtarget(1,1);

res(5)=atan2d(sqrt(r13^2+r23^2),-r33);

if res(5) == 0
    res(4) = 0;
    res(6) = atan2d(r21,r11);
else
    res(4)=atan2d(-r23/sind(res(5)),r13/sind(res(5)));
    res(6)=atan2d(-r32/sind(res(5)),r31/sind(res(5)));
end

if res(5)>100 %check
    res(5)=res(5)-360;
end
if res(5)<-100
    res(5)=res(5)+360;
end



if res(4)>170 %check
    res(4)=res(4)-360;
end
if res(4)<-110
    res(4)=res(4)+360;
end


if res(6)>277
    res(6)=res(6)-360;
end
if res(6)<-277
    res(6)=res(6)+360;
end



if res(5)>100 || res(5)<-100 || res(4)>170 || res(4)<-110 || res(6)>277 || res(6 )<-277
    res(5 )=atan2d(-sqrt(r13^2+r23^2),-r33);
    if res(5 )>100 %check
        res(5 )=res(5)-360;
    end
    if res(5)<-100
        res(5)=res(5)+360;
    end
    if res(5)>100 || res(5)<-100
        flag(2,1) = 1;
    else
        flag(2,1) = 0;
    end

    res(4)=atan2d(-r23/sind(res(5)),r13/sind(res(5)));

    if res(4)>170 %check
        res(4)=res(4)-360;
    end
    if res(4)<-110
        res(4)=res(4)+360;
    end
    if res(4)>170 || res(4)<-110
        flag(3,1) = 1;
    else
        flag(3,1) = 0;
    end

    res(6)=atan2d(-r32/sind(res(5)),r31/sind(res(5)));

    if res(6)>277
        res(6)=res(6)-360;
    end
    if res(6)<-277
        res(6)=res(6)+360;
    end
    if res(6)>277 || res(6)<-277
        flag(4,1) = 1;
    else
        flag(4,1) = 0;
    end
end