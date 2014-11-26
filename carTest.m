% Create a car with 2 trailers
C = CarWithNTrailers(2, [1, 2, 3]);

% Plot states of the car
q_init = [1;0;pi/20;0;-pi/8; pi*0.45];
C.draw(q_init);
%hold on;
%q2 = [10;3;-pi/8;pi/6;0; -0.3];
%C.draw(q2);

% Defining an elementary control law
ts = 0:0.01:40;
u1 = 2*ones(size(ts));  % Forward velocity
u2 = 0.2*cos(ts); % Steering rate 
%u2 = zeros(size(ts));
u = [u1;u2];

% Old code testing a dummy trajectory
% nT = size(ts, 2);
% qs = bsxfun(@times,q1,linspace(1,0,nT)) 
%      + bsxfun(@times,q2,linspace(0,1,nT));
% dummyTraj = Trajectory(C, ts, qs);
% dummyTraj.playback();

% Simulate system
controls = ControlLaw(C, ts, u);
traj = C.simulate(q_init, controls);
traj.playback(1:3);