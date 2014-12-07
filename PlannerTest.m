%C = CarWithNTrailers(2, [1, 2, 3]);
C = CarWithNTrailers(1, [1 2]);
%W = World([-10 10 -10 10]);

warning off;
data = load('matfiles/World.mat', 'W');
W = data.W;

W.draw;
P = Planner(C, W);

start = zeros(C.flatDim, 1);
start(1) = 0;
start(C.flatDim/2+1) = -6;
goal = zeros(C.flatDim, 1);
goal(1) = 25;
goal(C.flatDim/2+1) = 15;

P = P.setStart(start);
P = P.setGoal(goal, 0.5);

for i=1:1000
    if mod(i, 10)==0
        disp(i);
    end
    P = P.iteration();
    % Get First Solution
    if(P.bestDest~=0)
        break
    end
end

traj = P.getPath();
traj.playback(1:(C.N+1), W.span);
