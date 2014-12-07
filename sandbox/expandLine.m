% Nice (and time consuming) chunk of code, but useless in the end ;( 

t = 0.2;

ps=10*rand(2, 5);

plot(ps(1,:), ps(2,:));

% Get tangent vectors
ts = diff(ps')';
ts = bsxfun(@rdivide, ts, (sum(ts.^2)).^.5); % normalize

% Get bisector vectors
bs = diff(ts')';
bs = bsxfun(@rdivide, bs, (sum(bs.^2)).^.5); % normalize
bs = [[ts(2, 1); -ts(1, 1)] bs [ts(2, end); -ts(1, end)]];

cprod = cross([bs(:,2:end-1); zeros(1, length(bs)-2)], [ts(:,1:end-1); zeros(1, length(ts)-1)]);
sins = [1 cprod(3,:) 1];

right = ps+t*bsxfun(@rdivide, bs, sins); 
left = ps-t*bsxfun(@rdivide, bs, sins); 

hold on
plot(right(1,:), right(2,:), 'r-');
plot(left(1,:), left(2,:), 'r-');
axis equal
%
