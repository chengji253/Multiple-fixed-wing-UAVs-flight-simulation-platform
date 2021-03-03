load uav1u
a1 = A;
load uav2u
a2 = A;
load uav3u
a3 = A;

n1 = 3890;
plot3(a1(1:n1,2), a1(1:n1,3),a1(1:n1,4)); hold on;
plot3(a2(1:n1,2), a2(1:n1,3),a2(1:n1,4)); hold on;
plot3(a3(1:n1,2), a3(1:n1,3),a3(1:n1,4)); hold on;
axis equal