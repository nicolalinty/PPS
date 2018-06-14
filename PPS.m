% PPS: Polito Positioning system
% Script to compute a 2D position solution using Least Squares (LS) method.

% This code has been shown during the seminar on GNSS, at the class of
% Digital and connected world, 01PRQLP, June 13, 2018.
% Intended to be used with the Polito map attached.

% Author: Nicola Linty
% Last revision: June 14, 2018

%% Clean up the environment first
clear
close all
clc

%% Define some settings
%-- Number of LS iteration:
numIterations = 10;

%-- Transmitters position in [X Y] coordinates
TX1 = [3.8 2.5];
TX2 = [11 17];
TX3 = [26.5 10];
%-- Coordinates of the linearization point
P = [0 0];

%% Measured pseudoranges
%-- modify here!
rho1 = 21;
rho2 = 10.5;
rho3 = 6.7;

%% Run the LS iterations
for iter = 1:numIterations
    %-- Save the [X Y] values of all iterations
    Pls(iter, :) = P;
    
    %-- Compute the distances linearization point - satellites
    r1 = sqrt( (TX1(1)-P(1))^2 + (TX1(2) - P(2))^2 );
    r2 = sqrt( (TX2(1)-P(1))^2 + (TX2(2) - P(2))^2 );
    r3 = sqrt( (TX3(1)-P(1))^2 + (TX3(2) - P(2))^2 );
    
    %-- Compute the values of the H matrix coefficients
    a1x = (TX1(1)-P(1)) / r1;
    a1y = (TX1(2)-P(2)) / r1;
    
    a2x = (TX2(1)-P(1)) / r2;
    a2y = (TX2(2)-P(2)) / r2;
    
    a3x = (TX3(1)-P(1)) / r3;
    a3y = (TX3(2)-P(2)) / r3;
    
    %-- Compute the H matrix
    H = [a1x a1y; ...
         a2x a2y; ...
         a3x a3y];

    %-- Compute delta pseudoranges
    drho1 = r1 - rho1;
    drho2 = r2 - rho2;
    drho3 = r3 - rho3;
    rho = [drho1; ...
           drho2; ...
           drho3];
    
    %-- Update the position estimate
    P = P + (inv(H'*H)*H' * rho)';
end
%-- add the last point
Pls(iter+1, :) = P;

%% Display the results
fprintf('Position: x = %d; y = %d.\n', round(P(1)), round(P(2)));

%% Plot the results for all iterations
figure,
plot(Pls(:,1), Pls(:,2), '.-', 'MarkerSize', 14),
grid minor
axis square
axis equal
xlabel('x (cm)')
ylabel('y (cm)')
title('Position solution')
labels = cellstr(num2str((1:numIterations+1)'));
dx = 0.2; dy = 0.2; % displacement so the text does not overlay the data points
text(Pls(:,1)+dx, Pls(:,2)+dy, labels);
set(gca, 'FontSize', 12)