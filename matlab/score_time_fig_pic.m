clear;
clc;

level = [2 3 4 5 6 7 8 9 10 ...
        11 12 13 14];
time = [477.029 147.726 72.3284 16.2717 6.04284 3.83979 2.69635 2.01338 1.40164 1.00389 0.793515 0.704042 0.684701];
score = [0.0155494 0.0106792 0.0364238 1.17868 4.68893 11.2745 32.1579 82.9319 223.168 290.539 454.703 497.734 594.929];
   
%% 误差-时间关于层级的双轴图
yyaxis left; % 激活右边的轴
time_str=num2str(time');
plot(level,time,'LineWidth',2.5, 'Color',[255/255,128/255,0/255],'LineStyle','-');
text(level, time, cellstr(time_str));
hold on;
set(gca,'FontSize',20);
xlabel('Octree Level','fontsize',26);
ylabel('Time/s','fontsize',26); % 给右y轴添加轴标签

%设置x，y轴颜色，不然两个轴matlab会自动改色
set(gca,'Xcolor',[0 0 0]);
set(gca,'Ycolor',[0 0 0]);


yyaxis right; % 激活左边的轴
score_str=num2str(score');
plot(level,score,'Color',[0/255,96/255,156/255],'LineStyle','--','LineWidth',1.5);
text(level,score,cellstr(score_str));
ylabel('Score/m^2','fontsize',26); % 给左y轴添加轴标签
set(gca,'Ycolor',[0 0 0]);%设置x，y轴颜色

legend({'Time','Score'},'Orientation','horizontal');
set(legend,'Location','NorthOutside');



set(gcf,'PaperUnits','centimeters')
set(gcf,'PaperSize',[60,20])
set(gcf,'PaperPositionMode','manual')
set(gcf,'PaperPosition',[0,0,60,20]);
set(gcf,'Renderer','painters');
print('time-score /w level','-dpng')
%print 4_2.jpg -djpeg -r600
%print 4_2.eps -depsc2 -r600