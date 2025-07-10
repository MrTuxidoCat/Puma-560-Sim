%clear all
%close all
function [split2]=splitter(tar)
%tar=[100 500];%start and end points
split1=linspace(tar(1),tar(2),550);%initial splitting of the range between target points,dev10 (devided a number by 10 for adjustment)
split2=zeros(1,100);%vector for output of corrected final split,dev10
pt=0;%point in the initial split
pt2=1;%point in final split
for a=1:1:8%acceleration
    for b=1:1:5%dev10
        pt=pt+a;
        split2(pt2)=split1(pt);
        pt2=pt2+1;
    end
end
for a=1:1:19%flat speed,dev10
    pt=pt+10;
    split2(pt2)=split1(pt);
    pt2=pt2+1;
end
for a=8:-1:1%decceleration
    for b=1:1:5%dev10
        pt=pt+a;
        split2(pt2)=split1(pt);
        pt2=pt2+1;
    end
end
%plot(linspace(tar(1),tar(2),pt2-1),split2); %just a quick visaulisation for
%sanity checking,dev10