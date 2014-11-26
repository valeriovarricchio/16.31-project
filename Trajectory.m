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
            % TODO figure out axisBnds automatically
            % to show the whole trajectory
            axisBnds = [-8 15 -8 16];
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