%% Define the system parameters
Ra = 1.17;                       % Armature Resistance (Ohm)
La = 0.024;                      % Armature Inductance (H)
b = 0.0737;                      % Shaft Friction (N.m.s)
J = 0.01;                        % Shaft Inertia (Kg.m^2)
K = 0.072;                       % Motor Constant (V/(rad/sec))
%% (A)
% Numerator and Denominator of G(s)
num = [K];
den = [La*J, (La*b + Ra*J), (Ra*b + K^2), 0];

% Create the transfer function
G = tf(num, den);

% Display transfer function
disp('Open-Loop Transfer Function G(s):');
disp(G);
%% (B)
% Plot root locus
figure;
rlocus(G);
title('Root Locus of the System');
xlim([-5, 3]);
ylim([-20, 20]);
%% (C)
% Optionally, use rlocfind to select gain and see the poles
[K_stabil, poles] = rlocfind(G);

% Display the critical gain
disp(['Range of the controller gain to ensure system stability: K < ', num2str(K_stabil)]);
disp('Poles:');
disp(poles);
%% (D) 
% Damping ratio = 0.7
zgrid(0.7, []);
[K_Damping, poles] = rlocfind(G);
disp(['Controller gain for Damping Ratio = 0.7: K = ', num2str(K_Damping)]);

% Settling time < 0.75 sec
%zgrid([], [0.75]);
%[K_Ts1, poles] = rlocfind(G);
%disp(['Controller gain for T_s < 0.75 sec: K = ', num2str(K_Ts1)]);

% Settling time < 2 sec
zgrid([], [2]);
[K_Ts2, poles] = rlocfind(G);
disp(['Controller gain for T_s < 2 sec: K = ', num2str(K_Ts2)]);

% Natural frequency = 5 rad/sec
zgrid([], [], 5);
[K_wn, poles] = rlocfind(G);
disp(['Controller gain for ?_n = 5 rad/sec: K = ', num2str(K_wn)]);
%% e
figure;
rlocus(G);
xlim([-5, 3]);
ylim([-25, 25]);
zgrid(1, []);                       %  critically damped line (zeta = 1)
[K_crit_damping, poles] = rlocfind(G);
disp(['Controller Gain for Critically Damped Response: K = ', num2str(K_crit_damping)]);

% Plot the step response
T = feedback(K_crit_damping * G, 1);
figure
step(T);
title('Step Response of the State-Space Model');
xlabel('Time (s)');
ylabel('Output');
%% (f)
% Steady-state error for unit step
K_p = dcgain(K_crit_damping * G);           % Position error constant
ess_step = 1 / (1 + K_p);
disp(['Steady-State Error for Unit Step Input: e_ss = ', num2str(ess_step)]);

% Steady-state error for unit ramp
syms s;
G_ramp = K_crit_damping*G;
K_v = limit(s * G_ramp, s, 0);              % Velocity error constant
ess_ramp = 1 / K_v;
disp(['Steady-State Error for Unit Ramp Input: e_ss = ', num2str(ess_ramp)]);
%% (g)
numC = [1 15];
denC = [1 25];
C=7*tf(numC,denC);                          % Controller

T = feedback(C * G, 1);                     % Closed-loop transfer function
figure;
step(T);
title('Cascaded controller');
xlabel('Time (s)');
ylabel('Output y(t)');
grid on;

%figure;
%rlocus(G*C);
%figure
%step(GC*C);
%title('Cascaded controller');

% Use stepinfo to get performance metrics like overshoot, settling time, etc.
%info = stepinfo(GC*C);
info = stepinfo(T);

% Extract the overshoot from the stepinfo structure
overshoot = info.Overshoot;
disp(['Overshoot: ', num2str(overshoot), '%']);

% Extract the settling time and overshoot from the stepinfo structure
settling_time = info.SettlingTime;
disp(['Settling Time: ', num2str(settling_time), ' seconds']);
%------------------------------------------------------------------------
% Calculate steady-state error for a unit step input
% The steady-state value for a unit step input can be found using the final value theorem
% or just evaluating the final value of the step response.

% Final value theorem: ess = 1 - steady-state value
%[y, t] = step(GC*C);  % Get the step response data
%ess = 1 - y(end);  % Steady-state error (assumes unit step input with reference value = 1)

% Display the steady-state error
%disp(['Steady-State Error: ', num2str(ess)]);
%-------------------------------------------------------------------------
final_value = dcgain(T); % Steady-state output
error_ss = 1 - final_value; % Steady-state error
disp(['Steady-State Error (e_ss): ', num2str(error_ss)]);

% Control Action: u(t)
U = feedback(C, G); % Control action transfer function
figure;
step(U);
title('Control Action u(t)');
xlabel('Time (s)');
ylabel('Control Signal u(t)');
grid on;
