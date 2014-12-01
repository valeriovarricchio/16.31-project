function [C, F] = getCarAndF()
    syms t x(t) y(t)

    p1 = [x; y];

    d0 = 1;
    d1 = 2;
    C = CarWithNTrailers(1, [d0, d1]);

    t1 = diff(p1, t)/sum(diff(p1, t).^2)^.5;
    p0 = p1+t1*d1;
    t0 = diff(p0, t)/sum(diff(p0, t).^2)^.5;

    theta0 = angle([1 1i]*t0);
    theta1 = angle([1 1i]*t1);

    u1 = sum(diff(p0, t).^2)^.5;
    theta0dot = diff(theta0, t);
    phi = atan(d0*theta0dot/u1);

    % We have backed up the state and control vectors!
    q = [p0; phi; theta0; theta1];
    %u = [u1; diff(phi, t)];

    % derivatives 
    xd = sym('x', [4 1]);
    yd = sym('y', [4 1]);

    j = subs(q(0), 'D(D(D(x)))(0)', xd(4));
    j = subs(j, 'D(D(x))(0)', xd(3));
    j = subs(j, 'D(x)(0)', xd(2));
    j = subs(j, 'x(0)', xd(1));

    j = subs(j, 'D(D(D(y)))(0)', yd(4));
    j = subs(j, 'D(D(y))(0)', yd(3));
    j = subs(j, 'D(y)(0)', yd(2));
    j = subs(j, 'y(0)', yd(1));

    % F takes the derivatives of the flat output as inputs
    % and outputs the corresponding state
    F = matlabFunction(j, 'vars', {[xd; yd]});

end