function [ R T ] = RT( Q, P )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Q_bar = mean(Q, 2);
    P_bar = mean(P, 2);

    A = bsxfun(@minus, Q, Q_bar);
    B = bsxfun(@minus, P, P_bar);

    [U S V] = svd(B*A');
    R = V*[1 0 0; 0 1 0; 0 0 det(V'*U)]*U';
    T = Q_bar - R*P_bar;
end