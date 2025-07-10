function [torque] = statics(J,Fvec,Tvec)
%J = JAcobian2(0,0,0,0,0,0);
%vec = [0 0 10 0 0 0];
vec(1:3) = Fvec;
vec(4:6) = Tvec;
torque = (J.')*vec';
end