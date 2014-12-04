%C = CarWithNTrailers(2, [1, 2, 3]);
C = CarWithNTrailers(1, [1 2]);
W = World([-10 10 -10 10]);

P = Planner(C, W);

start = P.sample;
goal = P.sample;

P = P.setStart(start);
P = P.setGoal(goal, 2);

for i=1:500
    if mod(i, 10)==0
        disp(i);
    end
    P = P.iteration();
end

traj = P.getPath();
traj.playback(1:(C.N+1));
