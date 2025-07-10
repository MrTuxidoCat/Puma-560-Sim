function [T] = dhMat(th,al,a,d)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
T = [];
for n=1:size(th,2)
    matT = [cosd(th(n)) -sind(th(n)).*cosd(al(n)) ...
            sind(th(n)).*sind(al(n)) a(n).*cosd(th(n));
            sind(th(n)) cosd(th(n)).*cosd(al(n)) ...
            -cosd(th(n)).*sind(al(n)) a(n).*sind(th(n));
            0 sind(al(n)) cosd(al(n)) d(n);
            0 0 0 1];
    T = [T; {matT}];
end
end