%% RRT planner example
% Set vizMode to:
%  - 0 to only visualize the solution after plnning is finished
%  - 1 to visualize the tree as it is built
%  - 2 to animate the whole planning procedure (takes long time)
vizMode = 0;
NumIterations = 100; % Number of iterations when the algorithm should be stopped

%% Create Car Object (skipped if C is already available on the workspace)
if(~exist('C'))
    % takes approx. 40 sec 
    C = CarWithNTrailers(2, [1, 2, 3]);
end
%% Loading World 
warning off;
data = load('matfiles/World.mat', 'W');
W = data.W;

W.draw; % Draw the world
P = Planner(C, W);

%% Setting initial and final states
start = zeros(C.flatDim, 1);
start(1) = 0;
start(C.flatDim/2+1) = -6;
goal = zeros(C.flatDim, 1);
goal(1) = 25;
goal(C.flatDim/2+1) = 15;

P = P.setStart(start);
P = P.setGoal(goal, 0.5);

%% Start Planning
foundone = false;
t = tic;
for i=1:NumIterations
    if mod(i, 10)==0
        disp(['Iteration ' num2str(i) ', t = ' num2str(toc(t))]);
    end
    P = P.iteration(vizMode);
    % Get First Solution
    if(P.bestDest~=0 && ~foundone)
        foundone = true;
        disp(['FIRST SOLUTION! Iteration ' num2str(i) ', t = ' num2str(toc(t))]);
    end
end

%% Extract solution and animate
traj = P.getPath();
traj.playback(1:(C.N+1), W.span);
