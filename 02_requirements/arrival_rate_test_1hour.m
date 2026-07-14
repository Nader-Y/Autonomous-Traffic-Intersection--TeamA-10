clc;
clear;
close all;

%% Simulation Parameters
T = 60;                  % 60 minutes
time = 1:T;

%% Arrival Rates (vehicles/minute)

% S1 Normal Operation
lambda_S1 = 15;

% S2 Non-Conflicting Movement
lambda_S2 = 12.5;

% S3 Conflicting Movement
lambda_S3 = 20;

% S4 Emergency Vehicle
lambda_S4 = 1/60;        % 1 emergency vehicle per hour

%% Generate Poisson Arrivals

arrivals_S1 = poissrnd(lambda_S1, T, 1);
arrivals_S2 = poissrnd(lambda_S2, T, 1);
arrivals_S3 = poissrnd(lambda_S3, T, 1);
arrivals_S4 = poissrnd(lambda_S4, T, 1);

%% Plot Traffic Scenarios

figure;

subplot(4,1,1)
plot(time, arrivals_S1,'LineWidth',1.5)
title('S1 Normal Operation')
ylabel('Vehicles/min')
grid on

subplot(4,1,2)
plot(time, arrivals_S2,'LineWidth',1.5)
title('S2 Non-Conflicting Movement')
ylabel('Vehicles/min')
grid on

subplot(4,1,3)
plot(time, arrivals_S3,'LineWidth',1.5)
title('S3 Conflicting Movement')
ylabel('Vehicles/min')
grid on

subplot(4,1,4)
stem(time, arrivals_S4,'filled')
title('S4 Emergency Vehicle')
ylabel('Vehicles/min')
xlabel('Time (minutes)')
grid on

%% Statistics

fprintf('\nArrival Statistics\n');
fprintf('-------------------\n');

fprintf('S1 Average Arrival Rate = %.2f vehicles/min\n',mean(arrivals_S1));
fprintf('S2 Average Arrival Rate = %.2f vehicles/min\n',mean(arrivals_S2));
fprintf('S3 Average Arrival Rate = %.2f vehicles/min\n',mean(arrivals_S3));
fprintf('S4 Average Arrival Rate = %.4f vehicles/min\n',mean(arrivals_S4));