%minicim motor params
free_current = 3;
free_spd_rpm = 5480;
free_spd = free_spd_rpm/60;
G = 12/60;
Jm = .005;
Jw = .002;
stall_torque = 1.4;
stall_current = 89;
R = 12/stall_current;
Kt = stall_torque/stall_current;
Kv = free_spd*2*pi/(12 - R*free_current);
dt = .005;

%State feedback matrices from mechanism model
A = [0 1 0; 0 -2*Kt/(Jm*Kv*G*G*R) 0; 0 0 -2*Kt/(Jw*Kv*G*G*R)];

B = [0 0; Kt/(Jm*G*R) Kt/(Jm*G*R); Kt/(Jw*G*R) -Kt/(Jw*G*R)];

C = [1 0 0; 0 0 1];

%Feed forward magic sauce
Vf = 12.0/(free_spd*G*2*pi);

A_d = expm(A*dt);
B_d = pinv(A)*(A_d - eye(size(A_d)))*B;
%[A_d, B_d] = c2d(A,B,dt);

K = place(A_d,B_d,[.9+.01i, .9-.01i, .6]);

%initialize
x = [0;0;0];
time = 0:dt:.5;
phi = zeros(1,length(time));
alphaprime = zeros(1,length(time));
v1 = zeros(1,length(time));
v2 = zeros(1,length(time));

%simulate!
for t = 1:length(time)
    Rs = [pi/2;x(2);6*pi]; 

    u = K*(Rs-x) + [Vf*Rs(3); -Vf*Rs(3)];%error + velocity feed forward
    %don't exceed max voltage
    u(u>12) = 12;
    u(u<-12) = -12;
    
    phi(t) = x(1);
    alphaprime(t) = x(3);
    v1(t) = u(1);
    v2(t) = u(2);
   
    x = A_d*x + B_d*u;
    y = C*x;
end
%plot stuff
subplot(5,1,1)
plot(time, phi)
ylabel('phi (rad)')
hold on
subplot(5,1,2)
plot(time,alphaprime)
ylabel('alphaprime (rad/s)')
subplot(5,1,3)
plot(time, v1)
ylabel('motor 1 (V)')
subplot(5,1,4)
plot(time,v2)
ylabel('motor 2 (V)')
subplot(5,1,5)
plot(time,v1-v2)
ylabel('motor difference (V)')
xlabel('time (secs)')
