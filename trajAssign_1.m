syms s
%p1 = [cos(s); sin(s)]; % Circle
%p1 = [s; sin(s)]; % Sinusoid
%p1 = [3*cos(s); sin(s)]; % Ellipse
%p1 = 0.2*[16*sin(s)^3; 13*cos(s)-5*cos(2*s)-2*cos(3*s)-cos(4*s)]; % Heart
R=5; r=3; d=5;
p1 = [(R-r)*cos(s)+d*cos((R-r)/r*s); (R-r)*sin(s)-d*sin((R-r)/r*s)]; % star (hypotrochoid)

d0 = 1;
d1 = 2;
C = CarWithNTrailers(1, [d0, d1]);

t1 = diff(p1)/sum(diff(p1).^2)^.5;
p0 = p1+t1*d1;
t0 = diff(p0)/sum(diff(p0).^2)^.5;

theta1 = atan2(t1(2), t1(1));
theta0 = atan2(t0(2), t0(1));

u1 = sum(diff(p0).^2)^.5;
theta0dot = diff(theta0);
phi = atan(d0*theta0dot/u1);

% We have backed up the state and control vectors!
x = [p0; phi; theta0; theta1];
u = [u1; diff(phi)];

% plot the two trajectories
%fp0 = matlabFunction(p0);
%fp1 = matlabFunction(p1);
%path = fp0(ss);
%plot(path(1,:), path(2,:))

%%
% converting to functions for fast evaluation
% (this part is quite time consuming)
fx = matlabFunction(x);
fu = matlabFunction(u);

%% 
% simulate
ss = linspace(0,20, 1000);

% The following are superslow (and not guaranteed to work as expected)
x_nom = fx(ss);
u_nom = fu(ss);

% Just evaluating the expressions point by point
%x_nom = [];
%u_nom = [];
%for v=ss
%    x_nom = [x_nom fx(v)];
%    u_nom = [u_nom fu(v)];
%end

% Playing back the nominal computed trajectory
nom_traj = Trajectory(C, ss, x_nom);
nom_traj.playback(1:2);

% Actually simulate the nominal controls
% (sensitive to simulation timestep)
%controls = ControlLaw(C, ss, u_nom);
%traj = C.simulate(x_nom(:,1), controls);
%traj.playback(1:2);