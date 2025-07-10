%clearvars;close all;
function [res]=InverseKinematics(target,flag)
res=zeros(6,1450); %results matrix
%flag=[0,0,0,0]; %0: no change,  1:alt
% target2=[400,600,950,0,0,0]; %target coordinates:x,y,z,rotx,roty,rotz
% target=zeros(6,1450);
% for a=1:1:6
% target(a,:)=target2(a);
% end
%old testing stuff
%:1:1450
for ind=1
adjvector=cross([target(1,ind),target(2,ind),0],[0,0,1]);
adjvector=adjvector/norm(adjvector);
if flag(1)==0
adjtarget=[target(1,ind)+adjvector(1)*130,target(2,ind)+adjvector(2)*130];
else
adjtarget=[-target(1,ind)+adjvector(1)*130,-target(2,ind)+adjvector(2)*130];      
%alt version
end
res(1,ind)=atan2d(adjtarget(2),adjtarget(1));
if res(1,ind)>160 %check
    res(1,ind)=res(1,ind)-360;
end
if res(1,ind)<-160
    res(1,ind)=res(1,ind)+360;
end
if res(1,ind)>160 || res(1,ind)<-160
    res(1,1)=999; %this will be used to indicate failure
    return
end
x1tar=sqrt(target(1,ind)^2+target(2,ind)^2);%x of target in system 1
z1tar=target(3,ind)-660;%z of target in system 1
ltarget=sqrt(z1tar^2+x1tar^2);
r2b=atan2d(z1tar,x1tar);
r2a=acosd(ltarget/864);
if flag(2)==0
res(2,ind)=-(r2b+r2a);
else
res(2,ind)=-(r2b-r2a);     
%alt version
end
if flag(1)~=0
res(2,ind)=-180-res(2,ind); %for alt on 1
end
if res(2,ind)>45 %check
    res(2,ind)=res(2,ind)-360;
end
if res(2,ind)<-225
    res(2,ind)=res(2,ind)+360;
end
if res(2,ind)>45 || res(2,ind)<-225
    res(1,1)=999;
    return
end
if flag(1)==flag(2)
res(3,ind)=-acosd((ltarget)^2/-373248+1)+90;
else
res(3,ind)=-(360-acosd((ltarget)^2/-373248+1)-90);    
%alt version
end
if res(3,ind)>45 %check
    res(3,ind)=res(3,ind)-360;
end
if res(3,ind)<-225
    res(3,ind)=res(3,ind)+360;
end
if res(3,ind)>45 || res(3,ind)<-225
    res(1,1)=999;
    return
end
th = [ res(1,ind)  res(2,ind)   res(3,ind)    0   0  0];
al = [-90 0 90 0 -90 90];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
T = dhMat(th,al,a,d);

R30 = inv(T{1}*T{2}*T{3});
RtargetRoll = [1, 0, 0;
               0, cosd(target(4,ind)), -sind(target(4,ind));
               0, sind(target(4,ind)),cosd(target(4,ind))];
RtargetPitch = [cosd(target(5,ind)), 0, sind(target(5,ind));
                0, 1, 0;
                -sind(target(5,ind)), 0, cosd(target(5,ind))];
RtargetYaw = [cosd(target(6,ind)), -sind(target(6,ind)), 0;
              sind(target(6,ind)), cosd(target(6,ind)), 0;
              0, 0, 1];
Rtarget = R30(1:3,1:3)*RtargetYaw*RtargetPitch*RtargetRoll;
% r13=((cosd(res(3))*(cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2))))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((cosd(res(3))*(cosd(res(1))*sind(res(2))+sind(res(1))*cosd(res(2))))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))))+((-sind(res(3)))*(cosd(target(5)*sind(target(4)))));
% r23=((-sind(res(3))*(cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2))))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((-sind(res(3))*(cosd(res(1))*sind(res(2))+sind(res(1))*cosd(res(2))))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))))+((-cosd(res(3)))*(cosd(target(5)*sind(target(4)))));
% r33=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))));
% r32=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*sind(target(5))*sind(target(4))-sind(target(6))*cosd(target(4))))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(6))*sind(target(5))*sind(target(4))+cosd(target(6))*cosd(target(4))));
% r31=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*cosd(target(5)))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(4))*cosd(target(5)))));
%old stuff
r13 = Rtarget(1,3);
r23 = Rtarget(2,3);
r33 = Rtarget(3,3);
r32 = Rtarget(3,2);
r31 = Rtarget(3,1);
if flag(3)==0
res(5,ind)=atan2d(sqrt(r13^2+r23^2),-r33);
else
res(5,ind)=atan2d(-sqrt(r13^2+r23^2),-r33);
end
if res(5,ind)>100 %check
    res(5,ind)=res(5,ind)-360;
end
if res(5,ind)<-100
    res(5,ind)=res(5,ind)+360;
end
if res(5,ind)>100 || res(1,ind)<-100
    res(1,1)=999;
    return
end
res(4,ind)=atan2d(-r23/sind(res(5,ind)),r13/sind(res(5,ind)));
if res(4,ind)>170 %check
    res(4,ind)=res(4,ind)-360;
end
if res(4,ind)<-110
    res(4,ind)=res(4,ind)+360;
end
if res(4,ind)>170 || res(4,ind)<-110
    res(1,1)=999;
    return
end
res(6,ind)=atan2d(-r32/sind(res(5,ind)),r31/sind(res(5,ind)));
end
if res(6,ind)>277 && flag(4)==0 %check and flag
    res(6,ind)=res(6,ind)-360;
end
if res(6,ind)<-277 && flag(4)~=0
    res(6,ind)=res(6,ind)+360;
end
if res(6,ind)>277 || res(6,ind)<-277
    res(1,1)=999;
    return
end
%disp (res);