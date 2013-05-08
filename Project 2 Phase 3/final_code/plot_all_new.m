function plot_all_new(EKF, est, vicon, t, t0, tf)
% PLOT_ALL Plots figures to compare the estimation with the ground truth.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Mar.13th, 2013

% Position Comparision.
figure()
subplot(3,2,1); 
plot(t(:),vicon(4,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(1,:),'g','LineWidth',1.5);
plot(t(:),EKF(1,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('x/m');
obj1= title('Plot of x');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,3); 
plot(t(:),vicon(5,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(2,:),'g','LineWidth',1.5);
plot(t(:),EKF(2,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('y/m');
obj1= title('Plot of y');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,5); 
plot(t(:),vicon(6,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(3,:),'g','LineWidth',1.5);
plot(t(:),EKF(3,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('z/m');
obj1= title('Plot of z');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

% Angle Comparison.
% figure()
subplot(3,2,2); 
plot(t(:),vicon(1,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(4,:),'g','LineWidth',2);
plot(t(:),EKF(4,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('roll/rad');
obj1= title('Plot of roll-angle');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,4); 
plot(t(:),vicon(2,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(5,:),'g','LineWidth',2);
plot(t(:),EKF(5,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('pitch/rad');
obj1= title('Plot of pitch-angle');obj2 = legend('$Vicon$','$EstRob$','EFK'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

subplot(3,2,6); 
plot(t(:),vicon(3,:),'r','LineWidth',1.5);hold on;
plot(t(:),est(6,:),'g','LineWidth',2);
plot(t(:),EKF(6,:),'b','LineWidth',1.5);
xlim([t0 tf]);xlabel('t/s');ylabel('yaw/rad');
obj1= title('Plot of yaw-angle');obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2; grid on;

% 3D Plot.
figure()
plot3(vicon(4,:),vicon(5,:),vicon(6,:),'r','LineWidth',1.5); grid on; hold on;
plot3(est(1,:),est(2,:),est(3,:),'g','LineWidth',2);
plot3(EKF(1,:),EKF(2,:),EKF(3,:),'b','LineWidth',2);
obj1= title('Plot of Position Esitimation in 3D');
obj2 = legend('$Vicon$','$EstRob$','EKF'); 
set(obj1,'Interpreter','Latex');set(obj2,'Interpreter','Latex');
clear obj1 obj2;
xlabel('x/m');ylabel('y/m');zlabel('z/m')
end
