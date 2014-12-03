classdef Trajectory < TimedVector

    % Class to describe a zero order hold state trajectory of the car
   
   properties (SetAccess = private)
      Car;
   end

   methods
      function obj = Trajectory(Car, ts, u)
        obj = obj@TimedVector(Car.StateDimension, ts, u);
        obj.Car = Car;
      end
      
      function playback(traj, trailerTraces, axisBnds, permanence_interval)
          if(nargin<2)
            trailerTraces = [];
          end
              
          if(nargin<3 || isempty(axisBnds))
            % Try to figure out axisBnds automatically
            xmin = Inf;
            xmax = -Inf;
            ymin = Inf;
            ymax = -Inf;
            blowup = 0.5;
            % just sample a bunch of points (faster)
            %T = traj.ts(end);
            %N = 100;
            %for t=linspace(0,1,10)*T
            for t=traj.ts % consider all steps
                [Xb, Yb] = traj.Car.getPlotData(traj.evalAt(t));
                xmin = min(min(Xb), xmin);
                xmax = max(max(Xb), xmax);
                ymin = min(min(Yb), ymin);
                ymax = max(max(Yb), ymax);
            end
            axisBnds = [xmin-blowup xmax+blowup ymin-blowup ymax+blowup];
          end
          
          tracesX = nan(length(trailerTraces), length(traj.ts));
          tracesY = nan(length(trailerTraces), length(traj.ts));
          
          last_t=0;
          ti=1;
          h_car = 0; 
          h_traces = 0;
          hold on
          for t=traj.ts
            t0=tic;
            x = traj.evalAt(t);

            hold on;
            for i=1:length(trailerTraces)
                ram =  traj.Car.getRearAxleMidpoint(trailerTraces(i),x);
                tracesX(i,ti) = ram(1);
                tracesY(i,ti) = ram(2);
            end
            axis(axisBnds)
            if ti==1
                h_car = traj.Car.draw(x); 
                h_traces = plot(tracesX',  tracesY', '--');
            else
                traj.Car.draw(x, h_car);
                for i=1:length(trailerTraces)
                    set(h_traces(i), 'XData', tracesX(i,:)', 'YData', tracesY(i,:)');
                end
            end
            
            if nargin>=4
                if mod(ti, permanence_interval) == 0
                    c = 1-t/traj.ts(end);
                    car_h = traj.Car.draw(x);
                    set(car_h, 'Color', [c, c, c]);
                end
            end
            
            pause(t-last_t-toc(t0));
            drawnow;
            last_t = t;
            ti=ti+1;
          end
      end
   end 
end