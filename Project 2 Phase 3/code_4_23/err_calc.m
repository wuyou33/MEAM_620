function  error = err_calc(est, vicon, t, tVic, posLog)
% ERR_CALC calculates the standard deviation error of estimated data against
% the ground truth provided by the vicon.

% INPUT:
% est -- The estimated data;
% vicon -- The ground truth data;
% t -- The estimation time;
% tVic -- The vicon time.

% OUTPUT:
% error(1x6) -- The standard deviation between estimation and ground truth.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% April 9th, 2013
dim = size(est,1);
error = zeros(1,dim);
for n = 1 : dim
    error(n) = std(est(n,posLog)...
    -interp1(tVic,vicon((n>3)*(n-3)+(n<4)*(3+n),:),t(posLog),'spline'),1,2);
end
end