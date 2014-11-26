% Create a car with 2 trailers
C = CarWithNTrailers(2, [1, 2, 4]);

% Plot two states of the car
q1 = [1;0;pi/8;-pi/4;-pi/8; 0.5];
C.draw(q1);
%hold off;
q2 = [10;3;-pi/8;pi/6;0; -0.3];
C.draw(q2);

% Defining an elementary control law
ts = 0:0.01:3;
u1 = ones(size(ts));  % Forward velocity
u2 = zeros(size(ts)); % Steering rate 
u = [u1;u2];


driveForward = ControlLaw(C, ts, u);

nT = size(ts, 2);
qs = bsxfun(@times,q1,linspace(1,0,nT)) + bsxfun(@times,q2,linspace(0,1,nT));
dummyTraj = Trajectory(C, ts, qs);

dummyTraj.playback();