classdef Planner
   % Quick implementation of an RRT planner
   % for a car with trailers and a world with obstacles
   
   properties (SetAccess = private)
    car
    world
    start
    goal
    goalTolerance
    samplAmpl
    samplAvg
    tree
    K
    bestDest
    bestCost
   end

   methods
      function obj = Planner(car, world)
        obj.car = car;
        obj.world = world;
        
        % Generate sampling region
        % Account for worlds span
        obj.samplAmpl = 2*ones(obj.car.flatDim, 1);
        obj.samplAmpl(1) = obj.world.span(2) - obj.world.span(1);
        obj.samplAmpl(obj.car.flatDim/2+1) = obj.world.span(4) - obj.world.span(3);
        obj.samplAvg = zeros(obj.car.flatDim, 1);
        obj.samplAvg(1) = (obj.world.span(1) + obj.world.span(2))/2;
        obj.samplAvg(obj.car.flatDim/2+1) = (obj.world.span(3) + obj.world.span(4))/2;
        
        % Tree
        obj.tree = Tree();
        obj.bestCost = Inf;
        obj.bestDest = 0;

        % 
        obj.K = 10;
      end

      function obj = setStart(obj, start)
        obj.start = start;
        obj.tree = obj.tree.addVertex(start);
      end

      function obj = setGoal(obj, goal, tolerance)
        obj.goal = goal;
        obj.goalTolerance = tolerance; % quick n dirty euclidean ball around goal
      end

      function idx = getNearest(obj, coords)
        % Return the vertex index of closest neighbor
        idx=knnsearch(coords',obj.tree.vertices(1:obj.tree.numVertices, :), 1);
      end
      
      %function neighbors = getNeighbors(obj, coords)
        % Return the vertex indices of neighbors
      %  neighbors=knnsearch(coords',obj.tree.vertices(1:obj.tree.numVertices, :), obj.K);
      %end

      function obj = iteration(obj)
        % One planning iteration
        newsample = obj.sample();
        
        % Get nearest
        nearest = obj.getNearest(newsample);
        
        % Extend to nearest
        traj = obj.car.steerFlatOutput(obj.tree.vertices(nearest, :)', newsample, 5);
        %traj.playback(1:(obj.car.N+1), obj.world.span);
        
        % ~~~~~~~~~~~  TODO Check trajectory for collisions  ~~~~~~~~~~~~~~
        
        % Add vertex
        obj.tree = obj.tree.addVertex(newsample, nearest, traj);
        
        % Check if new vertex is in goal region
        if norm(newsample-obj.goal)<obj.goalTolerance
            disp('Sample in goal!');
            obj.bestDest = obj.tree.numVertices;
        end
      end
      
      function traj = getPath(obj)
          segments = [];
          parent = obj.bestDest;
          
          while parent~=0
              segments = [segments; obj.tree.edgesFromParent(parent)];
              parent = obj.tree.parents(parent);
          end
          
          ts = [];
          val = [];
          segments = flipud(segments);
          lastt = 0;
          
          for i=2:length(segments)
               ts = [ts lastt+segments(i).ts];
               val = [val segments(i).val];
               lastt = ts(end);
          end
          traj = Trajectory(obj.car, ts, val);
      end

      function new_sample = sample(obj)
        % Account for world span
        new_sample = (rand(obj.car.flatDim, 1)-0.5).*obj.samplAmpl+obj.samplAvg;
      end
      
      function c = getTrajCost(traj)
        % Control effort on steering wheel
        c = sum(traj.val(3,:).^2);
      end
      
   end
end
