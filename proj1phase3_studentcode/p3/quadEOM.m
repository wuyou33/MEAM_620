function sdot = quadEOM(t, s, controlhandle, trajhandle, params)

%convert state to quad stuct for control
[qd] = stateToQd(s);

%get control outputs 
[F, M, trpy, drpy] = feval(controlhandle, qd, t, trajhandle, params);

%compute derivative
sdot = quadEOM_readonly(t, s, F, M, params);
