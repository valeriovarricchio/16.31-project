classdef CarWithNTrailers
   properties (SetAccess = private)
      N;
      D;
   end
   methods
      function Car = CarWithNTrailers(N,D)
        % N Number of trailers
        % D lengths of trailers 
        % (length(D) = N+1, firts element = length of the truck)
        Car.N = N;
        Car.D = D; 
      end
      
      
      % Drawing functions
      function figh = draw(Car, x)
        % Takes a state x and a figure handle as inputs and plots the car
        
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
        hold on;
        
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
        
        figh = plot(Xb,Yb, 'black-');
        hold on;
        plot(Xa, Ya, 'b--');
        axis equal;
        
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
         Y = [Y nan yw+1.2*0.7/2 nan yw-1.2*0.7/2]; % TODO remove hardcoded

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