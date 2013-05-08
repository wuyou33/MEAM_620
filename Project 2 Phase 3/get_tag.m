function [tagPos, tagSub] = get_tag(tag,tagID)
% GET_TAG Gets the position and subscript of a tag.
%   [tagPos, tagSub] = GET_TAG(tag,tagID). Finds the index of a tag in the
%   matrix tagID, then transform index into subscript and calculate corres-
%   -ponding actual position of the tag.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Mar.13th, 2013

[x y] = ind2sub([12,9],find(tagID == tag));
X = 0.152 * (2*x-1);
Y = 0.152 * (2*y-2)+(y>3)*0.026+(y>6)*0.026;
tagPos = [X;Y];
tagSub = [x y];
end
