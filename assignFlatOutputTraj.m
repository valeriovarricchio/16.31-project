function [nom_traj, u_nom] = assignFlatOutputTraj(C, y, T)

    N = C.N;
    D = C.D;
    
    t = cell(1, N+1); % Tangent vectors
    theta = cell(1, N+1); % Headings
    p = cell(1, N+1); % Trajectories

    p{N+1} = y;
    for i=N+1:-1:1
        t{i} = diff(p{i})/sum(diff(p{i}).^2)^.5;
        theta{i} = atan2(t{i}(2), t{i}(1));
        if(i>1)
            p{i-1} = p{i}+t{i}*D(i);
        end
    end

    u1 = sum(diff(p{1}).^2)^.5;
    theta0dot = diff(theta{1});
    phi = atan(D(1)*theta0dot/u1);

    % We have backed up the state and control vectors!
    x = [p{1}; phi; theta{1};];
    for i=2:N+1
        x = [x; theta{i}];
    end
    u = [u1; diff(phi)];

    % plot the two trajectories
    %fp0 = matlabFunction(p0);
    %fp1 = matlabFunction(p1);
    %path = fp0(ss);
    %plot(path(1,:), path(2,:))

    %%
    % converting to functions for fast evaluation
    % (this part is quite time consuming)
    fx = matlabFunction(x);
    fu = matlabFunction(u);

    %% 
    ss = linspace(0,T, 1000);

    % Just evaluating the expressions point by point
    x_nom = fx(ss);
    u_nom = fu(ss);
    
    % Playing back the nominal computed trajectory
    nom_traj = Trajectory(C, ss, x_nom);
    %nom_traj.playback(1:N+1);

    % Actually simulate the nominal controls
    % % (sensitive to simulation timestep)
    % controls = ControlLaw(C, ss, u_nom);
    % traj = C.simulate(x_nom(:,1), controls);
    % traj.playback(1:N+1);
    
end