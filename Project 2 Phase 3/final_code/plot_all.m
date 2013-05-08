function plot_all(EKF, est, vicon, t, posLog, t0, tf)
% PLOT_ALL Plots figures to compare the estimation with the ground truth.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Mar.13th, 2013

% Position Comparision.
figure()
subplot(3,2,1); 
plot(t(posLog),vicon(4,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(1,posLog),'g','LineWidth',1.5);
plot(t(posLog),EKF(1,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('x/m');
obj1= title('Plot of x');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,3); 
plot(t(posLog),vicon(5,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(2,posLog),'g','LineWidth',1.5);
plot(t(posLog),EKF(2,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('y/m');
obj1= title('Plot of y');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,5); 
plot(t(posLog),vicon(6,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(3,posLog),'g','LineWidth',1.5);
plot(t(posLog),EKF(3,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('z/m');
obj1= title('Plot of z');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

% Angle Comparison.
% figure()
subplot(3,2,2); 
plot(t(posLog),vicon(1,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(4,posLog),'g','LineWidth',2);
plot(t(posLog),EKF(4,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('roll/rad');
obj1= title('Plot of roll-angle');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,4); 
plot(t(posLog),vicon(2,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(5,posLog),'g','LineWidth',2);
plot(t(posLog),EKF(5,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('pitch/rad');
obj1= title('Plot of pitch-angle');obj2 = legend('$Vicon$','$EstRob$','EFK'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,6); 
plot(t(posLog),vicon(3,posLog),'r','LineWidth',1.5);hold on;
plot(t(posLog),est(6,posLog),'g','LineWidth',2);
plot(t(posLog),EKF(6,posLog),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('yaw/rad');
obj1= title('Plot of yaw-angle');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

% 3D Plot.
figure()
plot3(vicon(4,posLog),vicon(5,posLog),vicon(6,posLog),'r','LineWidth',1.5); grid on; hold on;
plot3(est(1,posLog),est(2,posLog),est(3,posLog),'g','LineWidth',2);
plot3(EKF(1,posLog),EKF(2,posLog),EKF(3,posLog),'b','LineWidth',2);
obj1= title('Plot of Position Esitimation in 3D');
obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2;
xlabel('x/m');ylabel('y/m');zlabel('z/m')
end
