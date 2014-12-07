function F = getF(C)
    syms t x(t) y(t)
    
    N = C.N;
    D = C.D;
    
    t = cell(1, N+1); % Tangent vectors
    theta = cell(1, N+1); % Headings
    p = cell(1, N+1); % Trajectories

    p{N+1} = [x;y];
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

    % We have backed out the state and control vectors!
    q = [p{1}; phi; theta{1};];
    for i=2:N+1
        q = [q; theta{i}];
    end
    %u = [u1; diff(phi)];

    % derivatives 
    xd = sym('x', [N+3 1]);
    yd = sym('y', [N+3 1]);

    f = q(0);
    for i=N+3:-1:1
        string = [repmat('D(', [1, i-1]) 'x' repmat(')', [1, i-1]) '(0)'];
        f = subs(f, string, xd(N));
    end

    % F takes the derivatives of the flat output as inputs
    % and outputs the corresponding state
    F = matlabFunction(f, 'vars', {[xd; yd]});

end