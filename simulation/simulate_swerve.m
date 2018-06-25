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
A = [0 1 0 0; 0 -2*Kt/(Jm*Kv*G*G*R) 0 0; 0 0 0 1; 0 0 0 -2*Kt/(Jw*Kv*G*G*R)];

B = [0 0; Kt/(Jm*G*R) Kt/(Jm*G*R); 0 0; Kt/(Jw*G*R) -Kt/(Jw*G*R)];

C = [1 0 1 0; 1 0 -1 0]; %can only observe motors

% Kalman filter parameters
o_noise = pi/180;

p_noise = pi/180;

Q = eye(size(A))*p_noise; %process noise

R = eye(size(C,1))*o_noise; %observer noise


%Feed forward magic sauce
Vf = 12.0/(free_spd*G*2*pi);

%To run in Octave: Uncomment 36-37, comment out 38
%A_d = expm(A*dt);
%B_d = pinv(A)*(A_d - eye(size(A_d)))*B;
[A_d, B_d] = c2d(A,B,dt);

%place controller poles with discrete LQR
K = lqrd(A,B,eye(4)*5,eye(2),dt);

%initialize
x = [0;0;0;0];
y = C*x;
x_hat = x;
P = eye(size(A));

time = 0:dt:.5;
phi = zeros(1,length(time));
alphaprime = zeros(1,length(time));
v1 = zeros(1,length(time));
v2 = zeros(1,length(time));
state = zeros(4,length(time));
state_est = zeros(4,length(time));
sensor = zeros(2,length(time));

%simulate!
for t = 1:length(time)
    Rs = [pi/2;x_hat(2); x_hat(3); 28*pi]; 

    u = K*(Rs-x_hat) + [Vf*Rs(4); -Vf*Rs(4)];%error + velocity feed forward; error calculation uses estimate
    %don't exceed max voltage
    u(u>12) = 12;
    u(u<-12) = -12;
    
    %log states
    phi(t) = x(1);
    alphaprime(t) = x(4);
    v1(t) = u(1);
    v2(t) = u(2);
    state(:,t) = x;
    state_est(:,t) = x_hat;
    sensor(:,t) = y;
    
    %update system
    x = A_d*x + B_d*u + normrnd(0,p_noise,size(x));%simulate process noise
    y = C*x + normrnd(0,o_noise,size(y));%simualte observation noise
    
    %kalman filter prediction step
    P_ = A_d*P*A_d' + Q;
    x_hat_ = A_d*x_hat + B_d*u;
    
    %kalman filer update step
    Kk = P_*C'*inv(C*P_*C' + R);
    x_hat = x_hat_ + Kk*(y - C*x_hat_);
    P = P_ - Kk*C*P_;
    
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


