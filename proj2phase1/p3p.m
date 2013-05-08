function [ lambda ] = p3p( p, P )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    c12 = p(:, 1)' * p(:, 2) / (norm(p(:, 1)) * norm(p(:, 2)));
    c13 = p(:, 1)' * p(:, 3) / (norm(p(:, 1)) * norm(p(:, 3)));
    c23 = p(:, 2)' * p(:, 3) / (norm(p(:, 2)) * norm(p(:, 3)));
    
    d12 = norm(P(:, 1) - P(:, 2));
    d13 = norm(P(:, 1) - P(:, 3));
    d23 = norm(P(:, 2) - P(:, 3));
    
    % lambda1 = x*
    ka = d23^2/d13^2;
    kb = d23^2/d12^2;
    ca = c12;
    cb = c13;
    cc = c23;
    a1 = (ka*kb + ka - kb)^2 - 4*ka^2*kb*(cb)^2;
    a2 = 4*(ka*kb + ka - kb)*kb*(1 - ka)*ca + 4*ka*((ka*kb - ka + kb)*cb*cc + 2*ka*kb*ca*cb^2);
    a3 = ((2*kb*(1 - ka)*ca))^2 + 2*(ka*kb + ka - kb)*(ka*kb - ka - kb) + 4*ka*((ka - kb)*cc^2 + (1 - kb)*ka*cb^2 - 2*kb*(1 + ka)*ca*cb*cc);
    a4 = 4*(ka*kb - ka - kb)*kb*(1 - ka)*ca + 4*ka*cc*((ka*kb + kb - ka)*cb + 2*kb*ca*cc);
    a5 = (ka*kb - ka - kb)^2 - 4*ka*kb*cc^2;
    C = [a5 a4 a3 a2 a1];
    x = roots(C);
    
    % get rid of the imaginary numbers
    x = x(conj(x) == x);
    % compute lambdas
    lambda1 = d12./sqrt(x.^2 - 2*x*ca + 1);
    lambda2 = lambda1.*x;
    m1 = 1-ka;
    m2 = 1;
    p1 = 2*(ka*cb - x*cc);
    p2 = 2*(-x*cc);
    q1 = x.^2 - ka;
    q2 = x.^2*(1 - kb) + 2*x*kb*ca - kb;
    y = (p2.*q1 - p1.*q2)./(m1.*q2 - m2.*q1);
    lambda3 = y.*lambda1;
    lambda = [lambda1, lambda2, lambda3];
end

