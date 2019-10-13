clear all;
run setup.m

mass = 1;
alt0 = 20;
leng = .5;
diameter = .08;

motor = F15Motor();
controller = stateSpace();
models = getModels();

rocket = getRocketTradeStudy1(motor, mass, alt0, leng, diameter);
result = simulate(motor,rocket,controller, models);

figure()
grid on
hold on

plot((result.traj.velI(3,:)),result.traj.posI(3,:),'linewidth',3);

xlabel('Planet-relative Velocity, m/s');
ylabel('Altitude, m');

set(gca, 'FontName', 'Times New Roman','fontsize',30)

set(gcf,'paperorientation','portrait');
set(gcf,'paperunits','normalized')
set(gcf,'paperposition',[0 0 1 1]);
s_dir = '\\ad.uillinois.edu\engr-ews\dfawley2\documents\Classes\AE 442\TradeStudy1';
fig_name = strcat('mass=',num2str(mass),'length=',num2str(leng),...
    'diameter=',num2str(diameter),'.png');
print(gcf,'-dpng',fullfile(s_dir,fig_name));


