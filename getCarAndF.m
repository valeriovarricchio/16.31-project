function [C, F] = getCarAndF()
    syms t x(t) x1(t) x2(t) x3(t)
    syms y(t) y1(t) y2(t) y3(t)
   
    
    p1 = [x; y];

    d0 = 1;
    d1 = 2;
    C = CarWithNTrailers(1, [d0, d1]);

    t1 = diff(p1, t)/sum(diff(p1, t).^2)^.5;
    t1 = subs(t1, 'D(x)(t)', x1);
    t1 = subs(t1, 'D(y)(t)', y1);
    p0 = p1+t1*d1;
    
    t0 = diff(p0, t)/sum(diff(p0, t).^2)^.5;

    t0 = subs(t0, 'D(x)(t)', x1);
    t0 = subs(t0, 'D(y)(t)', y1);
    t0 = subs(t0, 'D(x1)(t)', x2);
    t0 = subs(t0, 'D(y1)(t)', y2);
    
    theta0 = angle([1 1i]*t0);
    theta1 = angle([1 1i]*t1);
% 
    u1 = sum(diff(p0, t).^2)^.5;
    u1 = subs(u1, 'D(x)(t)', x1);
    u1 = subs(u1, 'D(y)(t)', y1);
    u1 = subs(u1, 'D(x1)(t)', x2);
    u1 = subs(u1, 'D(y1)(t)', y2);
    u1 = subs(u1, 'D(x2)(t)', x3);
    u1 = subs(u1, 'D(y2)(t)', y3);
    
    theta0dot = diff(theta0, t);
    theta0dot = subs(theta0dot, 'D(x)(t)', x1);
    theta0dot = subs(theta0dot, 'D(y)(t)', y1);
    theta0dot = subs(theta0dot, 'D(x1)(t)', x2);
    theta0dot = subs(theta0dot, 'D(y1)(t)', y2);
    theta0dot = subs(theta0dot, 'D(x2)(t)', x3);
    theta0dot = subs(theta0dot, 'D(y2)(t)', y3);
    
    phi = atan(d0*theta0dot/u1);
    
    q = [p0; phi; theta0; theta1];

    xd = sym('x', [1 4]);
    yd = sym('y', [1 4]);

    q = subs(q(0), 'x(0)', xd(1));
    q = subs(q, 'x1(0)', xd(2));
    q = subs(q, 'x2(0)', xd(3));
    q = subs(q, 'x3(0)', xd(4));
    
    q = subs(q, 'y(0)', yd(1));
    q = subs(q, 'y1(0)', yd(2));
    q = subs(q, 'y2(0)', yd(3));
    q = subs(q, 'y3(0)', yd(4));
    
    F = matlabFunction(q, 'vars', {[xd; yd]});
    
   % keyboard
end


% function [C, F] = getCarAndF()
%     syms t x(t) y(t)
% 
%     p1 = [x; y];
% 
%     d0 = 1;
%     d1 = 2;
%     C = CarWithNTrailers(1, [d0, d1]);
% 
%     t1 = diff(p1, t)/sum(diff(p1, t).^2)^.5;
%     p0 = p1+t1*d1;
%     t0 = diff(p0, t)/sum(diff(p0, t).^2)^.5;
% 
%     %theta1 = atan2([0 1]*t1, [1 0]*t1);
%     %theta0 = atan2([0 1]*t0, [1 0]*t0);
% 
%     theta0 = angle([1 1i]*t0);
%     theta1 = angle([1 1i]*t1);
% 
%     u1 = sum(diff(p0, t).^2)^.5;
%     theta0dot = diff(theta0, t);
%     phi = atan(d0*theta0dot/u1);
% 
%     % We have backed up the state and control vectors!
%     q = [p0; phi; theta0; theta1];
%     u = [u1; diff(phi, t)];
% 
%     % derivatives 
%     xd = sym('x', [1 4]);
%     yd = sym('y', [1 4]);
% 
%     j = subs(q(0), 'D(D(D(x)))(0)', xd(4));
%     j = subs(j, 'D(D(x))(0)', xd(3));
%     j = subs(j, 'D(x)(0)', xd(2));
%     j = subs(j, 'x(0)', xd(1));
% 
%     j = subs(j, 'D(D(D(y)))(0)', yd(4));
%     j = subs(j, 'D(D(y))(0)', yd(3));
%     j = subs(j, 'D(y)(0)', yd(2));
%     j = subs(j, 'y(0)', yd(1));
% 
%     % F takes the derivatives of the flat output as inputs
%     % and outputs the corresponding state
%     F = matlabFunction(j, 'vars', {[xd; yd]});
% 
%     keyboard
% end