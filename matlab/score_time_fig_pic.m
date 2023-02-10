clear;
clc;

level = [6 7 8 9 10 11];
time = [1138.47 3492.04 7072.71 21388.9 39621.8 82378.2];
score = [6.8886 4.27474 3.19272 2.31843 1.65238 1.20095];
   
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