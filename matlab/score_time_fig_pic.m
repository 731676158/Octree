clear;
clc;

level = [4 5 6 7 8 9 10 ...
        11 12 13 14 15 16];
time = [1.9524 5.23515 17.2523 25.4684 ...
       85.3767 174.773 454.976 1403.98 ...
       4480.85 3696.16 1970.02 2639.28 ...
       2158.43];
score = [292.335 238.954 222.781 183.591 ...
       142.478 174.773 454.976 20.5645 ...
       10.2858 7.37504 6.89826 6.84535 ...
       6.8394];
   
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