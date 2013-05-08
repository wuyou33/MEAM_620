The file to run is 'test_trajectory.m'

The functions from the Robotics Toolbox that I use to smooth my trajectory
are 'jtraj.m', 'mstraj.m', 'numcols.m', 'numrows.m', and 'tb_optparse.m'.

Any function or m file that has a name similar to one we would use in the 
script, but with something changed or appended about it (i.e. dijkstra_keeps,
test_trajectory_old, init_cript_old, etc.) are depricated versions and are 
not to be used.

You may turn on dijkstra plotting by going into djikstra and uncommenting all
of the lines that are followed by "Progress Plotting". NOTE: this greatly slows
the code.

When using trajectory generator, to increase the max velocity in any direction
change the appropriate ( [x y z] ) element of max_vel, to reduce smoothing lower
t_acc (I know it can go as low as zero, but it may be able to go below that),
and to reduce the number of points in the trajectory increase dt.

NOTE: i have changed the plot_path() call in my run_trajectory_readonly to plot
the trajectory and not the dijkstra path.