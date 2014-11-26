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
      
      function playback(traj, axisBnds)
          if(nargin<2)
            % Try to figure out axisBnds automatically
            T = traj.ts(end);
            N = 100;
            xmin = Inf;
            xmax = -Inf;
            ymin = Inf;
            ymax = -Inf;
            for t=linspace(0,1,10)*T
                [Xb, Yb] = traj.Car.getPlotData(traj.evalAt(t));
                xmin = min(min(Xb), xmin);
                xmax = max(max(Xb), xmax);
                ymin = min(min(Yb), ymin);
                ymax = max(max(Yb), ymax);
            end
            axisBnds = [xmin xmax ymin ymax];
          end
          
          last_t=0;
          for t=traj.ts
            t0 = tic;
            set(gca,'NextPlot','replacechildren') ;
            traj.Car.draw(traj.evalAt(t));
            axis(axisBnds)
            pause(t-last_t-toc(t0));
            drawnow;
            last_t = t;
          end
      end
   end 
end