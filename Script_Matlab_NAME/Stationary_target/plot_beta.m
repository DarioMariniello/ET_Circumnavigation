%% PLOT COUNTERCLOCKWISE ANGLES
figure(1)
%title('Counterclocwise angles')
plot(time_datab1,beta1,'b')
hold on
plot(time_datab2,beta2,'r')
hold on
plot(time_datab3,beta3,'g')
hold on
plot(time_datab4,beta4,'k')
hold on
plot(time_datab5,beta5,'m')
grid on
xlabel('time [s]')
legend('\beta_1(t)','\beta_2(t)','\beta_3(t)','\beta_4(t)','\beta_5(t)');

