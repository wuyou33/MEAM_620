function [ features, points ] = cornerDetection( I )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    pts = detectSURFFeatures(I);
    [features, points] = extractFeatures(I, pts);
%     pts = corner(I, 'SensitivityFactor', 0.04);
%     [features, points] = extractFeatures(I, pts);
end
