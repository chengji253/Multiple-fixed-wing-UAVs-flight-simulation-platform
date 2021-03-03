load uav3
abs = states.data;
t = 0;
n = size(abs,1);
A = [];
for i = 1:n
a1 = [t abs(i,1)  abs(i,2) abs(i,3) abs(i,4) abs(i,8) abs(i,7) abs(i,16)];
A = [A;a1];

t = t+0.01;
end

n1 = n;
plot3(A(1:n1,2), A(1:n1,3),A(1:n1,4));
axis equal