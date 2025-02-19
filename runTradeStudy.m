clear all;
run setup.m

mass = 1.25;
diameter = .076;
leng = .6;
alt0 = 15;


motor = E31Motor();
controller = stateSpace();
models = getModels();

rocket = getRocketTradeStudy1(motor, mass, alt0, leng, diameter);
result = simulate(motor,rocket,controller, models);
%%

figure()
grid on
hold on

plot((result.traj.velI(3,:)),result.traj.posI(3,:),'linewidth',3);
% plot((result.traj.time),result.traj.posI(3,:),'linewidth',3);
% igniteTime = find(result.ctrl.igniteMotor == 1,1,'first');
% scatter(result.traj.time(igniteTime),result.traj.posI(3,igniteTime),'filled');

xlabel('Planet-relative Velocity, m/s');
ylabel('Altitude, m');

set(gca, 'FontName', 'Times New Roman','fontsize',30)

set(gcf,'paperorientation','portrait');
set(gcf,'paperunits','normalized')
set(gcf,'paperposition',[0 0 1 1]);
s_dir = "\\ad.uillinois.edu\engr-ews\dfawley2\documents\Classes\AE 442\Trade Study 2";
fig_name = strcat('mass=',num2str(mass),'length=',num2str(leng),...
    'diameter=',num2str(diameter),'.png');
print(gcf,'-dpng',fullfile(s_dir,fig_name));

% drawRocket(result, rocket)
