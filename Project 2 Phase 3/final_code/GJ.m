function GJ = GJ(dt,phi,psi,svx,svy,svz,swx,swy,swz,theta,vx,vy,vz,wx,wy,wz)
%GJ
%    GJ = GJ(DT,PHI,PSI,SVX,SVY,SVZ,SWX,SWY,SWZ,THETA,VX,VY,VZ,WX,WY,WZ)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    28-Apr-2013 21:55:30

t438 = sin(theta);
t439 = cos(phi);
t440 = svx+vx;
t441 = cos(psi);
t442 = sin(phi);
t443 = sin(psi);
t444 = svy+vy;
t445 = cos(theta);
t446 = svz+vz;
t447 = t441.*t445;
t473 = t438.*t442.*t443;
t448 = t447-t473;
t449 = t443.*t445;
t450 = t438.*t441.*t442;
t451 = t449+t450;
t452 = t438.*t443;
t472 = t441.*t442.*t445;
t453 = t452-t472;
t454 = t438.*t441;
t455 = t442.*t443.*t445;
t456 = t454+t455;
t457 = swx+wx;
t458 = dt.*t457;
t459 = cos(t458);
t460 = swy+wy;
t461 = dt.*t460;
t463 = cos(t461);
t464 = sin(t458);
t465 = sin(t461);
t466 = t442.*t459.*t463;
t467 = t439.*t441.*t464;
t468 = t439.*t443.*t459.*t465;
t462 = t466+t467+t468;
t469 = t462.^2;
t470 = -t469+1.0;
t471 = 1.0./sqrt(t470);
t480 = t451.*t464;
t481 = t448.*t459.*t465;
t482 = t438.*t439.*t459.*t463;
t474 = -t480+t481+t482;
t475 = t453.*t464;
t476 = t439.*t445.*t459.*t463;
t478 = t456.*t459.*t465;
t477 = t475+t476-t478;
t479 = 1.0./t477.^2;
t483 = 1.0./t477;
t484 = t474.^2;
t485 = t479.*t484;
t486 = t485+1.0;
t487 = 1.0./t486;
t488 = swz+wz;
t489 = dt.*t488;
t490 = sin(t489);
t491 = cos(t489);
t492 = t465.*t490;
t500 = t463.*t464.*t491;
t493 = t492-t500;
t494 = t442.*t493;
t495 = t463.*t490;
t496 = t464.*t465.*t491;
t497 = t495+t496;
t498 = t439.*t441.*t459.*t491;
t506 = t439.*t443.*t497;
t499 = t494+t498-t506;
t501 = t465.*t491;
t502 = t463.*t464.*t490;
t503 = t501+t502;
t504 = t463.*t491;
t508 = t464.*t465.*t490;
t505 = t504-t508;
t507 = 1.0./t499.^2;
t509 = t439.*t443.*t505;
t510 = t439.*t441.*t459.*t490;
t513 = t442.*t503;
t511 = t509+t510-t513;
t512 = 1.0./t499;
t514 = t511.^2;
t515 = t507.*t514;
t516 = t515+1.0;
t517 = 1.0./t516;
GJ = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt.*(t438.*t442.*t446-t438.*t439.*t440.*t443+t438.*t439.*t441.*t444),dt.*(t439.*t446+t440.*t442.*t443-t441.*t442.*t444),-dt.*(t442.*t445.*t446-t439.*t440.*t443.*t445+t439.*t441.*t444.*t445),-t471.*(t441.*t442.*t464-t439.*t459.*t463+t442.*t443.*t459.*t465),-t487.*(t483.*(t438.*t439.*t441.*t464+t438.*t442.*t459.*t463+t438.*t439.*t443.*t459.*t465)-t474.*t479.*(t439.*t441.*t445.*t464+t442.*t445.*t459.*t463+t439.*t443.*t445.*t459.*t465)),-t517.*(t512.*(t439.*t503+t442.*t443.*t505+t441.*t442.*t459.*t490)+t507.*t511.*(t439.*t493+t442.*t443.*t497-t441.*t442.*t459.*t491)),-dt.*(t440.*t456+t444.*t453+t439.*t445.*t446),0.0,dt.*(t440.*t448+t444.*t451-t438.*t439.*t446),0.0,1.0,0.0,dt.*(t444.*t448-t440.*t451),-dt.*(t439.*t440.*t441+t439.*t443.*t444),-dt.*(t440.*t453-t444.*t456),-t471.*(t439.*t443.*t464-t439.*t441.*t459.*t465),-t487.*(t483.*(t448.*t464+t451.*t459.*t465)+t474.*t479.*(t456.*t464+t453.*t459.*t465)),t517.*(t512.*(t439.*t441.*t505-t439.*t443.*t459.*t490)+t507.*t511.*(t439.*t441.*t497+t439.*t443.*t459.*t491))],[6, 6]);
