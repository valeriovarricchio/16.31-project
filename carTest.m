% Create a car with 2 trailers
C = CarWithNTrailers(2, [1, 2, 3]);

% Plot states of the car
q1 = [1;0;0;pi/2;-pi/8; pi*0.45];
C.draw(q1);
%hold on;
%q2 = [10;3;-pi/8;pi/6;0; -0.3];
%C.draw(q2);

% Defining an elementary control law
ts = 0:0.01:10;
u1 = 3*ones(size(ts));  % Forward velocity
u2 = 0.4*cos(ts); % Steering rate 
u = [u1;u2];

% Old code testing a dummy trajectory
% nT = size(ts, 2);
% qs = bsxfun(@times,q1,linspace(1,0,nT)) 
%      + bsxfun(@times,q2,linspace(0,1,nT));
% dummyTraj = Trajectory(C, ts, qs);
% dummyTraj.playback();

% Simulating a forward driving
driveForward = ControlLaw(C, ts, u);
traj = C.simulate(q1, driveForward);
traj.playback();