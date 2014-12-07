T = 3;

D = [1 2 3]; % lengths of truck and trailers
N = length(D)-1;
C = CarWithNTrailers(N, D);

xi = zeros(N+4, 1);
xf = ones(N+4, 1);

% Build the polynomia
a = sym('a', [1 N+1]); 
b = sym('b', [1 N+1]);
y = [poly2sym(a, 't'); poly2sym(b, 't')];

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

% initial and final states
x0 = subs(x, 't', 0);
xT = subs(x, 't', T);

% now you should compute as and bs

