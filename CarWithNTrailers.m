classdef CarWithNTrailers

%  Class to describe a car with N trailers.
%  A state of the car is inended as a column vector of N+4 elements:
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
%  theta_i yaw of ith trailer (i=0 means truck)

   properties (SetAccess = private)
      N;
      D;
      StateDimension;
      InputDimension;
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
      end
      
      function checkState(Car, x)
        if(size(x, 1)~=Car.StateDimension || size(x, 2)~=1)
            error('Invalid Car state');
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
      
      % Drawing functions
      
      function draw(Car, x)
        % Takes a state x as input and plots the car at state x
        [Xb, Yb, Xa, Ya] = getPlotData(Car, x);
        plot(Xb,Yb, 'black-', Xa, Ya, 'b--');
        axis equal;
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