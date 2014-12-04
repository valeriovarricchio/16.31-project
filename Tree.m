classdef Tree
   % Quick implementation of a Tree for motion planning

   properties (SetAccess = private)
    allocateEvery
    numVertices
    
    % V minimum multiple of allocateEvery > numVertices
    % N dimension of sampling space
    vertices % Matrix V x N
    edgesFromParent % Matrix V x N
    parents % vector of indices
   end

   methods
    function obj = Tree()
      obj.allocateEvery = 100;
      obj.numVertices = 0;
      %obj = addVertex(root, 0);
    end

    function obj = addVertex(obj, newVertex, parentIdx, edge)
        if nargin < 3
            parentIdx = 0;
        end
        % newVertex is 1 x N row vector!!
        obj.numVertices = obj.numVertices +1;
        if(obj.numVertices > length(obj.vertices))
            obj.vertices = [obj.vertices; zeros(obj.allocateEvery, size(newVertex, 1))];
            obj.parents = [obj.parents; zeros(obj.allocateEvery, 1)];
            newlyAllocated(1:obj.allocateEvery) = Trajectory();
            obj.edgesFromParent = [obj.edgesFromParent; newlyAllocated'];
        end
        
        obj.vertices(obj.numVertices, :) = newVertex';
        obj.parents(obj.numVertices) = parentIdx;
        if(parentIdx~=0)
            obj.edgesFromParent(obj.numVertices) = edge;
        end
    end
    
    function pos = getVertex(obj, idx)
        pos = obj.vertices(idx, :);
    end
        
    function draw
        % TODO
    end
   end
end
