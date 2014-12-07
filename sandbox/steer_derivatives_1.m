function [traj, u] = steer_derivatives_1(yi, yf, T)
    % To try it...
    % steer_derivatives_1([0 1 0 0 3 0 0 0]', [10 1 0 0 -3 0 0 0]', 10)
    
    [C, F] = getCarAndF();
    %yi = getDerivatives(F, xi)
    %yf = getDerivatives(F, xf)
    
    [as, bs] = findPolynomials(yi, yf, T);
    
    y = [poly2sym(as, 't'); poly2sym(bs, 't')];
    
    [traj, u] = assignFlatOutputTraj(C, y, T);
    
    keyboard
end

function [as, bs] = findPolynomials(yi, yf, T)
    % Takes the desired initial and final flat output values and derivatives
    % and returns polynomials that satisfy the provided boundary conditions
    % yi and yf variables in yi and yf are arranged as follows
    x0 = yi(1:4);
    y0 = yi(5:end);
    
    xT = yf(1:4);
    yT = yf(5:end); 
    
    M0 = [1 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0; 0 0 2 0 0 0 0 0; 0 0 0 6 0 0 0 0];
    MT = [1 T T^2 T^3 T^4 T^5 T^6 T^7; 
          0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5 7*T^6; 
          0 0 2 3*2*T 4*3*T^2 5*4*T^3 6*5*T^4 7*6*T^5;
          0 0 0 3*2*1 4*3*2*T 5*4*3*T^2 6*5*4*T^3 7*6*5*T^4];
    
    as = [M0; MT]\[x0; xT];
    bs = [M0; MT]\[y0; yT];
    
    as = flipud(as);
    bs = flipud(bs);
end

function y = getDerivatives(F, xdes)
    F0 = @(in) F(in) -xdes;
    y = fsolve(F0, 10*(rand(8,1)-.5));
end

