function [test_suite] = collide_test
initTestSuite;
end

function assertNoCollision(map, points)
c = collide(map, points);
assertEqual(size(points, 1), length(c(:)));
assertFalse(any(c));
end

function assertCollision(map, points)
c = collide(map, points);
assertTrue(isvector(c));
assertEqual(size(points, 1), length(c(:)));
assertTrue(all(c));
end

function [xyzs] = grid(x, y, z)
[x, y, z] = meshgrid(x, y, z);
xyzs = [x(:), y(:), z(:)];
end

function testEmptyMap
map = load_map('emptyMap.txt', 0.2, 2.0, 0.0);
points = grid(0:0.1:10, 0:0.1:10, 0:1.0:6.0);
assertNoCollision(map, points);
end

function testSingleCube
map = load_map('singleCube.txt', 0.1, 1.0, 0.0);

top_points    = grid(0:0.1:10, 0:0.1:10, 4.1:0.4:6);
left_points   = grid(0:0.2:4.4, 0:0.2:10, 0:0.4:6);
right_points  = grid(5.7:0.2:10, 0:0.2:10, 0:0.4:6);
bottom_points = grid(0:0.2:10, 0:0.2:10, 0:0.4:1.6);
square_points = grid(4.6:0.2:5.4, 4.6:0.2:5.4, 2.5:0.2:3.5);

assertNoCollision(map, top_points);
assertNoCollision(map, bottom_points);
assertNoCollision(map, left_points);
assertNoCollision(map, right_points);
assertCollision(map, square_points);
end

function testMap1
map = load_map('map1.txt', 0.2, 0.5, 0.2);

valid = [0.0  -1.0 2.0; 
         3.0  17.0 4.0; 
         0.0  -5.0 0.5];
collision = [0.0 2.1 1.0; 
             3.0 18.5 4.6];
assertNoCollision(map, valid);
assertCollision(map, collision);
end
