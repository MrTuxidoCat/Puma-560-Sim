syms th1 th2 th3 th4 th5 th6 mass
J = Jacobian(th1,th2,th3,th4,th5,th6);
vec = [0; 0; mass*9.81; 0; 0; 0];
torque = (J.')*vec