clearvars;close all;
%flag=zeros(1,4);%0: no change,  1:alt
flag=[0,0,0,0];
res=zeros(1,6);%results
target=[400,100,500,0,0,0];%target coordinates:x,y,z,rotx,roty,rotz
%part of function from here
adjvector=cross([target(1),target(2),0],[0,0,1]);
adjvector=adjvector/norm(adjvector);
if flag(1)==0
adjtarget=[target(1)+adjvector(1)*130,target(2)+adjvector(2)*130];
else
adjtarget=[-target(1)+adjvector(1)*130,-target(2)+adjvector(2)*130];      
alt version
end
res(1)=atan2d(adjtarget(2),adjtarget(1));
x1tar=sqrt(target(1)^2+target(2)^2);%x of target in system 1
z1tar=target(3)-660;%z of target in system 1
ltarget=sqrt(z1tar^2+x1tar^2);
r2b=atan2d(z1tar,x1tar);
r2a=acosd(ltarget/864);
if flag(2)==0
res(2)=-(r2b+r2a);
else
res(2)=-(r2b-r2a);     
%alt version
end
if flag(1)==flag(2)
res(3)=-acosd((ltarget)^2/-373248+1)+90;
else
res(3)=-(360-acosd((ltarget)^2/-373248+1)-90);    
%alt version
end
if flag(1)~=0
res(2)=-180-res(2); %for alt on 1
end
%if alt was used for 1, alt flips for 3 

th = [ res(1)  res(2)   res(3)    0   0  0];
%th = [th1 th2 th3 th4 th5 th6];
al = [-90 0 90 90 90 0];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
T = dhMat(th,al,a,d);

R30 = inv(T{1}*T{2}*T{3});
RtargetRoll = [1, 0, 0;
               0, cosd(target(4)), -sind(target(4));
               0, sind(target(4)),cosd(target(4))];
RtargetPitch = [cosd(target(5)), 0, sind(target(5));
                0, 1, 0;
                -sind(target(5)), 0, cosd(target(5))];
RtargetYaw = [cosd(target(6)), -sind(target(6)), 0;
              sind(target(6)), cosd(target(6)), 0;
              0, 0, 1];
Rtarget = R30(1:3,1:3)*RtargetYaw*RtargetPitch*RtargetRoll;
% r13=((cosd(res(3))*(cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2))))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((cosd(res(3))*(cosd(res(1))*sind(res(2))+sind(res(1))*cosd(res(2))))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))))+((-sind(res(3)))*(cosd(target(5)*sind(target(4)))));
% r23=((-sind(res(3))*(cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2))))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((-sind(res(3))*(cosd(res(1))*sind(res(2))+sind(res(1))*cosd(res(2))))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))))+((-cosd(res(3)))*(cosd(target(5)*sind(target(4)))));
% r33=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*sind(target(5))*cosd(target(4))+sind(target(6))*sind(target(4))))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(6))*sind(target(5))*cosd(target(4))-cosd(target(6))*sind(target(4))));
% r32=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*sind(target(5))*sind(target(4))-sind(target(6))*cosd(target(4))))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(6))*sind(target(5))*sind(target(4))+cosd(target(6))*cosd(target(4))));
% r31=((-cosd(res(1))*sind(res(2))-cosd(res(2))*sind(res(1)))*(cosd(target(6))*cosd(target(5)))+((cosd(res(1))*cosd(res(2))-sind(res(1))*sind(res(2)))*(sind(target(4))*cosd(target(5)))));
r13 = Rtarget(1,3);
r23 = Rtarget(2,3);
r33 = Rtarget(3,3);
r32 = Rtarget(3,2);
r31 = Rtarget(3,1);
if flag(3)==0
res(5)=atan2d(sqrt(r13^2+r23^2),-r33);
else
res(5)=atan2d(-sqrt(r13^2+r23^2),-r33);
end
res(4)=atan2d(-r23/sind(res(5)),r13/sind(res(5)));
res(6)=atan2d(-r32/sind(res(5)),r31/sind(res(5)));
disp (res);