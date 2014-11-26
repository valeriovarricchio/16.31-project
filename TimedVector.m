classdef TimedVector

% Class to describe a time varying vector quantity
% considered as a zero order hold between some specified 
% time instants (ts)

   properties (SetAccess = private)
      dim;
   end
   
   properties (SetAccess = public)
      ts; % Time steps
      val; % value at knot points
   end
   
   methods 
      function obj = TimedVector(dim, ts, val)
        obj.dim = dim;
        obj.ts = ts;
        if(nargin<3)
            obj.val = zeros(dim, size(ts, 2));
        else
            obj.val = val;
        end
        
        if(size(val, 1)~=obj.dim)
            error(['Values should have the specified dimension (' num2str(obj.dim) ')']);
        end
        
        if(size(val, 2)~=length(ts))
            error('The length of the timeseries must be the same as the number of time istants');
        end
      end

      function value = evalAt(obj, t)
          idx = find(obj.ts>=t, 1);
          if(isempty(idx))
              error(['No data at time ' num2str(t)])
          end
          value = obj.val(:, idx);
      end
      
   end 
end