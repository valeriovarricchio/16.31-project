classdef World
    % Quick implementation of a class to describe a 2D world with obstacles
   
   properties (SetAccess = private)
      span % 2 x 2 matrix, first column is minima of x and y, second column is maxima
      obstacles % a set of polygons (cell array with 2xN matrices)
   end

   methods
      function obj = World(span)
        obj.span = span;
        obj.obstacles = cell(0);
      end
      
      function isfree = isFreePoly(obj, polygon)
          isfree = 1;
          % Check if polygon is in free space
          for i=1:length(obj.obstacles)
              int = polybool('&', obj.obstacles{i}(1,:), obj.obstacles{i}(2,:), polygon(1,:), polygon(2,:));
              if(~isempty(int))
                  isfree = 0;
                break;
              end
          end
      end
      
      % function isfree = test_isFreePoly(obj)
      %     obj.draw;
      %     [xv, yv] =  World.polygonFromUI();
      %     isfree = obj.isFreePoly([xv;yv]);
      % end
      
      function draw(obj)
          axis(reshape(obj.span, 1, []));
          % plot obstacles
          hold on
          for i=1:length(obj.obstacles)
            fill(obj.obstacles{i}(1,:), obj.obstacles{i}(2,:), 'r');
          end
      end
      
      function obj = addObstacle(obj, xs, ys)
          % add an obstacle
          obj.obstacles{length(obj.obstacles)+1} = [xs;ys];
      end
      
      function obj = addObstacleUI(obj)
        obj.draw

        [xv, yv] = World.polygonFromUI();
        xv = [xv xv(end)];
        yv = [yv yv(end)];
        obj = addObstacle(obj, xv, yv);
      end
   end
   
   methods (Static = true)
      function [xv, yv] = polygonFromUI()
        hit = 1;
        [xv(1),yv(1)]=ginput(1);
        plot(xv,yv,'ro');
        while hit==1
            [xi,yi,hit]=ginput(1);
                plot(xi,yi,'ro', [xv(end) xi], [yv(end) yi], 'b-');
            xv=[xv xi];
            yv=[yv yi];
        end
        plot([xv(1) xi], [yv(1) yi], 'b-');
        hold on;
      end
   end
end