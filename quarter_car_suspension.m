%% MODELING OF QUARTER CAR SUSPENSION

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

nump = [(m1 + m2) b2 k2];
denp = [(m1 * m2), (m1 * (b1 + b2)) + (m2 * b1), (m1 * (k1 + k2)) + (m2 * k1) + (b1 * b2), (b1 * k2) + (b2 * k1), k1 * k2];
G1 = tf(nump, denp);

num1 = [-(m1 * b2), -(m1 * k2), 0, 0];
den1 = denp;
G2 = tf(num1, den1);

% Open Loop Feedthrough Function
F = tf(num1, nump);

% Create a single figure window with subplots
figure('Name','Quarter Car Suspension Control Comparisons','NumberTitle','off')
t1 = 0:0.05:5;
t2 = 0:0.01:5;
t3 = 0:0.01:2;

%% Subplot 1: Open Loop Step Response
subplot(3,2,1)
step(0.1 * G2, t1)
title('Open-Loop Response to a 0.1-m Step')

%% Subplot 2: High-Gain PID Controller
Kd = 208025 * 2;
Kp = 832100 * 2;
Ki = 624075 * 2;
C = pid(Kp, Ki, Kd);

sys_cl = F * feedback(G1, C);
subplot(3,2,2)
step(0.1 * sys_cl, t1)
title('Response with High-Gain PID')

%% Subplot 3: 2-Lead Compensator
a = (1 - sin(70/180*pi)) / (1 + sin(70/180*pi));
w = 5;
T = 1 / (w * sqrt(a));
aT = sqrt(a) / w;
numc = 4 * conv([T 1], [T 1]);
denc = conv([aT 1], [aT 1]);
C = tf(numc, denc);
K = 100000;
sys_cl = F * feedback(G1, K * C);

subplot(3,2,3)
step(0.1 * sys_cl, t2)
title('Response with 2-Lead Controller')

%% Subplot 4: Full-State Feedback Controller
A = [0 1 0 0;
    -(b1*b2)/(m1*m2) 0 ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1) -(b1/m1);
     b2/m2 0 -((b1/m1)+(b1/m2)+(b2/m2)) 1;
     k2/m2 0 -((k1/m1)+(k1/m2)+(k2/m2)) 0];
B = [0 0;
     1/m1 (b1*b2)/(m1*m2);
     0 -(b2/m2);
     (1/m1)+(1/m2) -(k2/m2)];
C = [0 0 1 0];
D = [0 0];

Aa = [[A, [0;0;0;0]]; [C, 0]];
Ba = [B; [0 0]];
Ca = [C, 0];
Da = D;
K = [0 2.3e6 5e8 0 8e6];

sys_cl = ss(Aa - Ba(:,1)*K, -0.1*Ba, Ca, Da);
subplot(3,2,4)
step(sys_cl * [0;1], t3)
title('Response with Full-State Feedback')

%% Subplot 5: Digital Controller with Integral Action
T = 0.0005;
sys = ss(A, B, C, D);
d_sys = c2d(sys, T, 'zoh');

Ai = 1; Bi = T; Ci = 1; Di = T/2;
[As,Bs,Cs,Ds] = ssdata(d_sys);
Aa = [As, zeros(4,1); Bi*Cs, Ai];
Ba = [Bs; 0, 0];
Ca = [Cs, 0];
Da = Ds;

d_sys_int = ss(Aa, Ba, Ca, Da, T);
[Ad, Bd, Cd, Dd] = ssdata(d_sys_int);

sys1 = d_sys_int * [1; 0];
[num, den] = tfdata(sys1, 'v');
z = roots(num);

p1 = z(1); p2 = z(2); p3 = z(3); p4 = 0.9992; p5 = 0.5;
K = place(Ad, Bd * [1; 0], [p1 p2 p3 p4 p5]);

d_sys_cl = ss(Ad - Bd * [1; 0] * K, Bd, Cd, Dd, T);
subplot(3,2,5)
step(-0.1 * d_sys_cl * [0;1], 5)
title('Response with Discrete-Time Controller')

%% Subplot 6: Placeholder for future or legend
subplot(3,2,6)
axis off
text(0.1, 0.5, 'Step Responses for Various Controllers', 'FontSize', 12, 'FontWeight', 'bold')
text(0.1, 0.3, sprintf('m₁ = %d kg | m₂ = %d kg\nb₁ = %d Ns/m | b₂ = %d Ns/m\nk₁ = %d N/m | k₂ = %d N/m', m1, m2, b1, b2, k1, k2), 'FontSize', 10)

