classdef CarWithNTrailers

%  Class to describe a car with N trailers.
%  A state of the car is intended as a column vector of N+4 elements:
%
%        /   x_0   \
%       |    y_0    |
%       |    phi    |
%   x = |  theta_0  | ,
%       |  theta_1  |
%       |    ...    |
%        \ theta_N /
%
%  with x0, y0 coords of the truck rear axle midpoint
%  phi steering angle
%  theta_i yaw of i-th trailer (i=0 means truck)

   properties (SetAccess = private)
      N;
      D;
      StateDimension;
      InputDimension;

      % F is a function handle to a function that computes state from
      % flat output and its derivatives, i.e. x = F(y, y', y'', ... )
      F;
      flatDim; % Dimension of the vector to recover state
               % (includes flat output and derivatives)
   end
   methods
      function Car = CarWithNTrailers(N,D)
        % N Number of trailers
        % D lengths of trailers
        % (length(D) = N+1, firts element = length of the truck)
        Car.N = N;
        Car.D = D;
        Car.StateDimension = N+4;
        Car.InputDimension = 2;

        % Construct function F
        [Car.F, Car.flatDim] = constructF(Car); % Can be very slow
      end

      function [F, flatDim] = constructF(Car)
         % F takes the derivatives of the flat output as inputs
         % and outputs the corresponding state
         % input is in the form x, x', x'' ... , y, y', y'' ...
         disp('Building the flatness map...');

         syms t x(t) y(t);
         flatDim = 2*(Car.N+3);

         t = cell(1, Car.N+1); % Tangent vectors
         theta = cell(1, Car.N+1); % Headings
         p = cell(1, Car.N+1); % Trajectories

         p{Car.N+1} = [x;y];
         for i=Car.N+1:-1:1
             t{i} = diff(p{i})/sum(diff(p{i}).^2)^.5;
             theta{i} = angle([1 1i]*t{i});
             if(i>1)
                p{i-1} = p{i}+t{i}*Car.D(i);
             end
         end

         u1 = sum(diff(p{1}).^2)^.5;
         theta0dot = diff(theta{1});
         phi = atan(Car.D(1)*theta0dot/u1);

         % We have backed up the state and control vectors!
         q = [p{1}; phi; theta{1};];
         for i=2:Car.N+1
             q = [q; theta{i}];
         end

         %u = [u1; diff(phi)];
         % derivatives
         xd = sym('x', [Car.N+3 1]);
         yd = sym('y', [Car.N+3 1]);

         f = q(0);

         for i=Car.N+3:-1:1
             string = [repmat('D(', [1, i-1]) 'x' repmat(')', [1, i-1]) '(0)'];
             f = subs(f, string, xd(i));
             string = [repmat('D(', [1, i-1]) 'y' repmat(')', [1, i-1]) '(0)'];
             f = subs(f, string, yd(i));
         end

         F = matlabFunction(f, 'vars', {[xd; yd]});
      end

      function y = getFlatOutputDerivatives(Car, xdes)
        % Finds **A** set of flat outputs + derivatives that correspond to
        % a given state (map is not invertible)
        F0 = @(in) Car.F(in)-xdes;
        y = fsolve(F0, rand((Car.N+3)*2,1)-.5);
      end

    function traj = steerFlatOutput(Car, yi, yf, T)
        [as, bs] = Car.findPolynomials(yi, yf, T);

        fod = zeros(2*(Car.N+3), 2*(Car.N+3));
        fod([1;Car.N+4], :) = [as'; bs'];

        for i=2:Car.N+3
           fod(i, i:end) = polyder(fod(i-1,:));
           fod(i+Car.N+3, i:end) = polyder(fod(i+Car.N+2,:));
        end

        %[traj, u] = assignFlatOutputTraj(Car, y, T); takes forever
        ts = linspace(0, T, 1000);

        yds = fod*bsxfun(@power, ts, flipud((0:2*(Car.N+3)-1)'));

        xs = Car.F(yds);
        traj = Trajectory(Car, ts, xs);
        %traj.playback(1:(Car.N+1));

     end

     function traj = steerState2state(Car, xi, xf, T)
        yi = Car.getFlatOutputDerivatives(xi);
        yf = Car.getFlatOutputDerivatives(xf);

        traj = Car.steerFlatOuput(Car, yi, yf, T);

     end

     function [as, bs] = findPolynomials(Car, yi, yf, T)
        % Takes the desired initial and final flat output values and derivatives
        % and returns polynomials that satisfy the provided boundary conditions
        % yi and yf variables in yi and yf are arranged as follows
        x0 = yi(1:Car.N+3);
        y0 = yi(Car.N+4:end);

        xT = yf(1:Car.N+3);
        yT = yf(Car.N+4:end);

        M0 = spdiags(factorial(0:Car.N+2)', 0, Car.N+3, 2*(Car.N+3));

        v = T.^(0:2*(Car.N+3)-1); % first row
        MT = v;

        for i=1:Car.N+2
            v = v(2:end)/T;
            v = v.*(1:size(v, 2));
            MT = [MT; zeros(1, i) v];
        end

        as = [M0; MT]\[x0; xT];
        bs = [M0; MT]\[y0; yT];

        as = flipud(as);
        bs = flipud(bs);
      end

      function checkState(Car, x)
        if(size(x, 1)~=Car.StateDimension || size(x, 2)~=1)
            error(['Invalid Car state, should be ' num2str(Car.StateDimension) ' x 1']);
        end % TODO probably add more checks?
      end

      function traj = simulate(Car, x0, u)
        %   x0 is the initial state of the car
        %   u is a ControlLaw
        %   traj is a Trajectory with the result of the simulation
        %   using ode45 to solve the problem
        Car.checkState(x0);
        [ts, xs] = ode45(@(t, x) Car.dynamics(x, u.evalAt(t)), u.ts, x0);
        traj = Trajectory(Car, ts', xs');
      end

      function xdot = dynamics(Car, x, u)
        xdot = x*0;
        xdot(1) = cos(x(4))*u(1);
        xdot(2) = sin(x(4))*u(1);
        xdot(3) = u(2);
        xdot(4) = 1/Car.D(1)*tan(x(3))*u(1);

        % Dynamics of the trailers
        prod = 1;
        for i=1:Car.N
           xdot(i+4) = u(1)/Car.D(i+1)*sin(x(i+3)-x(i+4))*prod;
           prod = prod*cos(x(i+3)-x(i+4));
        end
      end

      function [A, B] = linearizeAt(Car, x0, u0, epsilon)
        if nargin < 4
            epsilon = 1e-5;
        end
        A = zeros(Car.StateDimension);
        for i=1:Car.StateDimension
            sel = (1:Car.StateDimension)'==i;
            A(:,i) = (Car.dynamics(x0+epsilon*sel, u0)-Car.dynamics(x0+epsilon*sel, u0))/2/epsilon;
        end

        B = zeros(2, Car.StateDimension);
        B(1,:) = (Car.dynamics(x0, u0+epsilon*[1;0])-Car.dynamics(x0, u0-epsilon*[1;0]))'/2/epsilon;
        B(2,:) = (Car.dynamics(x0, u0+epsilon*[0;1])-Car.dynamics(x0, u0-epsilon*[0;1]))'/2/epsilon;
      end



      % -------------------   Drawing functions

      function handle = draw(Car, x, handle)
        % Takes a state x as input and plots the car at state x
        [Xb, Yb, Xa, Ya] = getPlotData(Car, x);
        if nargin < 3
            handle = plot(Xb,Yb, 'black-', Xa, Ya, 'black-.');
            axis equal;
        else
            set(handle(1), 'XData',  Xb, 'YData', Yb);
            set(handle(2), 'XData',  Xa, 'YData', Ya);
        end
      end

      function [Xb, Yb, Xa, Ya] = getPlotData(Car, x)
        % Takes a state x as input and output the vector to plot
        checkState(Car, x);

        % Draw the truck
        [truckPts, axisPts] = truckPoints(Car, x(3));
        R = [cos(x(4)) -sin(x(4)); sin(x(4)) cos(x(4))];
        truckPts = R*truckPts;
        axisPts = R*axisPts;
        Xb = truckPts(1,:)+x(1);
        Yb = truckPts(2,:)+x(2);
        Xa = axisPts(1,:)+x(1);
        Ya = axisPts(2,:)+x(2);

        origin = x(1:2);

        % Draw the trailers
        for i=1:Car.N
          [trailerPts, axisPts]= trailerPoints(Car, i+1);

          % Rotate trailer
          theta = x(i+4);
          R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
          Tt = R*trailerPts; % trailer transformed
          At = R*axisPts;
          Xb = [Xb nan Tt(1,:)+origin(1)];
          Yb = [Yb nan Tt(2,:)+origin(2)];
          Xa = [Xa nan At(1,:)+origin(1)];
          Ya = [Ya nan At(2,:)+origin(2)];
          origin = origin - Car.D(i+1)*R(:,1);
        end
      end

      function p = getRearAxleMidpoint(Car, trailerIdx, x)
          % returns the position of the rear axle midpoint of the specified
          % trailer (truck is 1) in state x. x can be a matrix (i.e. more
          % than one state).
          p = x(1:2,:);
          for i=2:trailerIdx
            p = p-Car.D(i)*[cos(x(3+i,:)); sin(x(3+i,:))];
          end
      end

      function [bodyPts, axisPts] = truckPoints(Car, phi)
         [bodyPts, axisPts] = trailerPoints(Car, 1, -0.5);
         X = bodyPts(1,:)+Car.D(1);
         Y = bodyPts(2,:);

         % Wheels
         L = Car.D(1);
         [xw, yw] = wheelPoints(Car);
         Wr = [cos(phi) -sin(phi); sin(phi) cos(phi)]*[xw;yw];
         xw = Wr(1,:);
         yw = Wr(2,:);
         X = [X nan xw+L nan xw+L];
         Y = [Y nan yw+1.2*0.7/2 nan yw-1.2*0.7/2]; % TODO remove hardcoded (these should agree with the ones in trailerPoints)

         bodyPts = [X;Y];
         axisPts(1,:) = axisPts(1,:)+Car.D(1);
      end

      function [bodyPts, axisPts] = trailerPoints(Car, i, foh)
        % Outputs the set of Xs and Ys to draw the trailer
        W = 1.2; % Width
        L = Car.D(i); % Length
        roh = 0.3; % rear overhang
        if(nargin < 3)
            foh = 0.8; % front overhang
        end
        fee = 0.5; % front ellipse eccentricity
        wd = W*0.7;

        % Main body
        ang = (.5:-0.02:-.5)*pi;
        X = [-L-roh -L-roh -foh+fee*W/2*(cos(ang)-1) -L-roh];
        Y = [-W/2 W/2 W/2*sin(ang) -W/2];

        % Wheels
        [xw, yw] = wheelPoints(Car);
        X = [X nan xw-L nan xw-L];
        Y = [Y nan yw+wd/2 nan yw-wd/2];

        bodyPts = [X;Y];

        % Axis
        axisPts = [-L 0 nan -L -L;
                   0  0 nan -wd/2 wd/2];
      end

      function [X, Y] = wheelPoints(Car)
        ww = 0.05; % Wheel width
        wr = 0.3; % Wheel radius
        X = [-.5 -.5 .5 .5 -.5]*wr;
        Y = [-.5 .5 .5 -.5 -.5]*ww;
      end

   end
end
