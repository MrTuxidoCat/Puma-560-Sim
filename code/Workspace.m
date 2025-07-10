clear all
close all
th = [0 45 0 0 0 0];
al = [-90 0 90 90 90 0];
d = [660 130 0 -432 0 0];%% change to 56.5 if were not assuming spherical joint
a = [0 432 0 0 0 0];
T = dhMat(th,al,a,d);
p0 = [0 0 0 1];
%WS = zeros(4,25872);
n4 = 1;
for n1=-160:1:160
    for n2=-225:1:45
        for n3=[-90]
            th = [n1 n2 n3 0 0 0];
            T = dhMat(th,al,a,d);
            WS(:,n4) = T{1}*T{2}*T{3}*T{4}*p0.';
            n4 = n4 +1;
        end
    end
end

figure
hold on
grid on

[spx ,spy ,spz] = sphere;
rad = 432*2;
spx = spx*rad;
spy = spy*rad;
spz = spz*rad;
%surf(spx,spy,spz+660)
plot3(WS(1,:),WS(2,:),WS(3,:),"Marker",".",LineStyle="none");
plot3(600,400,950,'Marker','.',Color='r')
view(0,0);
title("X-Z",Interpreter="tex");
xlabel("X[mm]",Interpreter="tex");
zlabel("Z[mm]",Interpreter="tex");
exportgraphics(gcf, 'X-Z.png', 'Resolution', 300);
view(90,0);
title("Y-Z");
ylabel("Y[mm]",Interpreter="tex");
zlabel("Z[mm]",Interpreter="tex");
exportgraphics(gcf, 'Y-Z.png', 'Resolution', 300);
view(90,90);
title("X-Y");
xlabel("X[mm]",Interpreter="tex");
ylabel("Y[mm]",Interpreter="tex");
exportgraphics(gcf, 'X-Y.png', 'Resolution', 300);