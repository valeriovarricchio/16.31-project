%% Examples of flat output trajectory assignment
% The state trajectory and the controls are computed symbolically into the
% assignFlatOutputTraj function. Hence, computations might take some time!

%% Create Car Object (skipped if C is already available on the workspace)
if(~exist('C'))
    % takes approx. 40 sec 
    C = CarWithNTrailers(2, [1, 2, 3]);
end
disp('Car object is created!')
syms t;

%% Sine wave example
disp('Running the sine wave example, should take around 1 min')
fo_sine = [t; sin(t)];
[traj_sine, u_sine] = assignFlatOutputTraj(C, fo_sine, 20);
traj_sine.playback(1:3); % Re-run this line to watch again w/o recomputing! 

%% Ellipse example
disp('Running the ellipse example, should take around 2 min')
fo_ellipse = [3*cos(t); sin(t)];
[traj_ellipse, u_ellipse] = assignFlatOutputTraj(C, fo_ellipse, 2*pi);
traj_ellipse.playback(1:3); % Re-run this line to watch again w/o recomputing! 

%% Hypotrochoid example
disp('Running the hypotrochoid example, should take around 10 min')
R=5; r=3; d=5;
fo_star = [(R-r)*cos(t)+d*cos((R-r)/r*t); (R-r)*sin(t)-d*sin((R-r)/r*t)]; 
[traj_star, u_star] = assignFlatOutputTraj(C, fo_star, 5);
traj_star.playback(1:3); % Re-run this line to watch again w/o recomputing! 