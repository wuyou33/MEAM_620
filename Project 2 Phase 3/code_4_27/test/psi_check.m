function [ psi ] = psi_check( R, phi, psi )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    x = R(2,2)/cos(phi);
    y = -R(2,1)/cos(phi);
    if x > 0
        return;
    elseif y >= 0 && x < 0
        psi = psi + pi;
    elseif y < 0 && x < 0
        psi = psi - pi;
    end

end

