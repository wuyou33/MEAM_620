function GJ = GJb(ax,ay,az,bax,bay,baz,dt,phi,psi,sax,say,saz,swx,swy,swz,theta,wx,wy,wz)
%GJB
%    GJ = GJB(AX,AY,AZ,BAX,BAY,BAZ,DT,PHI,PSI,SAX,SAY,SAZ,SWX,SWY,SWZ,THETA,WX,WY,WZ)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    28-Apr-2013 22:15:16

t650 = dt.^2;
t651 = sin(theta);
t652 = cos(phi);
t653 = cos(psi);
t654 = sin(phi);
t655 = sin(psi);
t656 = -ax+bax+sax;
t657 = cos(theta);
t658 = -ay+bay+say;
t659 = -az+baz+saz;
t660 = t653.*t657;
t665 = t651.*t654.*t655;
t661 = t660-t665;
t662 = t655.*t657;
t663 = t651.*t653.*t654;
t664 = t662+t663;
t666 = t651.*t655;
t671 = t653.*t654.*t657;
t667 = t666-t671;
t668 = t651.*t653;
t669 = t654.*t655.*t657;
t670 = t668+t669;
t672 = t652.*t659;
t673 = t654.*t655.*t656;
t674 = t672+t673-t653.*t654.*t658;
t675 = t652.*t655.*t658;
t676 = t652.*t653.*t656;
t677 = t675+t676;
t678 = t654.*t657.*t659;
t679 = t652.*t653.*t657.*t658;
t680 = t678+t679-t652.*t655.*t656.*t657;
t681 = t656.*t661;
t682 = t658.*t664;
t683 = t681+t682-t651.*t652.*t659;
t684 = t656.*t667;
t685 = t684-t658.*t670;
t686 = swx+wx;
t687 = dt.*t686;
t688 = cos(t687);
t689 = swy+wy;
t690 = dt.*t689;
t692 = cos(t690);
t693 = sin(t687);
t694 = sin(t690);
t695 = t654.*t688.*t692;
t696 = t652.*t653.*t693;
t697 = t652.*t655.*t688.*t694;
t691 = t695+t696+t697;
t698 = t691.^2;
t699 = -t698+1.0;
t700 = 1.0./sqrt(t699);
t707 = t664.*t693;
t708 = t661.*t688.*t694;
t709 = t651.*t652.*t688.*t692;
t701 = -t707+t708+t709;
t702 = t667.*t693;
t703 = t652.*t657.*t688.*t692;
t705 = t670.*t688.*t694;
t704 = t702+t703-t705;
t706 = 1.0./t704.^2;
t710 = 1.0./t704;
t711 = t701.^2;
t712 = t706.*t711;
t713 = t712+1.0;
t714 = 1.0./t713;
t715 = swz+wz;
t716 = dt.*t715;
t717 = sin(t716);
t718 = cos(t716);
t719 = t694.*t717;
t727 = t692.*t693.*t718;
t720 = t719-t727;
t721 = t654.*t720;
t722 = t692.*t717;
t723 = t693.*t694.*t718;
t724 = t722+t723;
t725 = t652.*t653.*t688.*t718;
t733 = t652.*t655.*t724;
t726 = t721+t725-t733;
t728 = t694.*t718;
t729 = t692.*t693.*t717;
t730 = t728+t729;
t731 = t692.*t718;
t735 = t693.*t694.*t717;
t732 = t731-t735;
t734 = 1.0./t726.^2;
t736 = t652.*t655.*t732;
t737 = t652.*t653.*t688.*t717;
t740 = t654.*t730;
t738 = t736+t737-t740;
t739 = 1.0./t726;
t741 = t738.^2;
t742 = t734.*t741;
t743 = t742+1.0;
t744 = 1.0./t743;
GJ = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t650.*t651.*t654.*t659.*(-1.0./2.0)-t650.*t651.*t652.*t653.*t658.*(1.0./2.0)+t650.*t651.*t652.*t655.*t656.*(1.0./2.0),t650.*t674.*(-1.0./2.0),t650.*t680.*(1.0./2.0),-dt.*(t651.*t654.*t659+t651.*t652.*t653.*t658-t651.*t652.*t655.*t656),-dt.*t674,dt.*t680,-t700.*(t653.*t654.*t693-t652.*t688.*t692+t654.*t655.*t688.*t694),-t714.*(t710.*(t651.*t652.*t653.*t693+t651.*t654.*t688.*t692+t651.*t652.*t655.*t688.*t694)-t701.*t706.*(t652.*t653.*t657.*t693+t654.*t657.*t688.*t692+t652.*t655.*t657.*t688.*t694)),-t744.*(t739.*(t652.*t730+t654.*t655.*t732+t653.*t654.*t688.*t717)+t734.*t738.*(t652.*t720+t654.*t655.*t724-t653.*t654.*t688.*t718)),0.0,0.0,0.0,0.0,0.0,t650.*t658.*t667.*(1.0./2.0)+t650.*t656.*t670.*(1.0./2.0)+t650.*t652.*t657.*t659.*(1.0./2.0),0.0,t650.*t683.*(-1.0./2.0),dt.*(t658.*t667+t656.*t670+t652.*t657.*t659),0.0,-dt.*t683,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t650.*t658.*t661.*(-1.0./2.0)+t650.*t656.*t664.*(1.0./2.0),t650.*t677.*(1.0./2.0),t650.*t685.*(1.0./2.0),-dt.*(t658.*t661-t656.*t664),dt.*t677,dt.*t685,-t700.*(t652.*t655.*t693-t652.*t653.*t688.*t694),-t714.*(t710.*(t661.*t693+t664.*t688.*t694)+t701.*t706.*(t670.*t693+t667.*t688.*t694)),t744.*(t739.*(t652.*t653.*t732-t652.*t655.*t688.*t717)+t734.*t738.*(t652.*t653.*t724+t652.*t655.*t688.*t718)),0.0,0.0,0.0,0.0,0.0,t650.*t661.*(-1.0./2.0),t650.*t652.*t655.*(1.0./2.0),t650.*t670.*(-1.0./2.0),-dt.*t661,dt.*t652.*t655,-dt.*t670,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,t650.*t664.*(-1.0./2.0),t650.*t652.*t653.*(-1.0./2.0),t650.*t667.*(-1.0./2.0),-dt.*t664,-dt.*t652.*t653,-dt.*t667,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t650.*t651.*t652.*(1.0./2.0),t650.*t654.*(-1.0./2.0),t650.*t652.*t657.*(-1.0./2.0),dt.*t651.*t652,-dt.*t654,-dt.*t652.*t657,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0],[14, 14]);
