clear;
clc;

% indep gicp
% level = [5 6 7 8 9 10 11 12];
% time = [149.332 1138.47 3492.04 7072.71 21348.9 39621.8 82378.2 88430];
% score = [18.5336 6.8886 4.27474 3.19272 2.31843 1.65238 1.20095 1.07576];

% indep vgicp
% level = [5 6 7 8 9 10 11 12];
% time = [66.2227 164.824 316.462 717.274 3463.23 8052.05 9798.91 17307.9];
% score = [16.5512 6.7113 4.35569 3.1957 2.31906 1.65388 1.20251 1.07548];

% rec gicp
% level = [5 6 7 8 9 10 11];
% time = [63.8802 2004.79 2593.56 3691.66 4607.16 9487.68 7861.84];
% score = [33.0049 11.6586 7.08617 5.0079 3.46368 2.31871 1.66245];

% rec vgicp
level = [5 6 7 8 9 10 11];
time = [61.7852 127.413 348.331 493.818 926.099 1455.51 1776.85];
score = [16.5512 7.00186 4.38643 3.43151 2.31607 1.65392 1.20252];

%% 误差-时间关于层级的双轴图
yyaxis left; % 激活右边的轴
time_str=num2str(time');
plot(level,time,'LineWidth',1.0, 'Color',[182/255,194/255,154/255],'LineStyle',':',...
    'Marker','*');
% text(level, time, cellstr(time_str));
hold on;
set(gca,'FontSize',15);
xlabel('Octree-based Registration','FontName','Times New Roman','fontsize',15);
ylabel('Time/ms','FontName','Times New Roman','fontsize',15); % 给右y轴添加轴标签

%设置x，y轴颜色，不然两个轴matlab会自动改色
set(gca,'Xcolor',[0 0 0]);
set(gca,'Ycolor',[0 0 0]);


yyaxis right; % 激活左边的轴
score_str=num2str(score');
plot(level,score,'Color',[230/255,180/255,80/255],'LineStyle',':','LineWidth',1.0,...
    'Marker','x');
% text(level,score,cellstr(score_str));
ylabel('Score/m^2','FontName','Times New Roman','fontsize',15); % 给左y轴添加轴标签
set(gca,'Ycolor',[0 0 0]);%设置x，y轴颜色

legend({'Time','Score'},'Orientation','horizontal','FontName','Times New Roman');
set(legend,'Location','NorthOutside');



set(gcf,'PaperUnits','centimeters')
set(gcf,'PaperSize',[60,20])
set(gcf,'PaperPositionMode','manual')
set(gcf,'PaperPosition',[0,0,60,20]);
set(gcf,'Renderer','painters');
print('time-score /w level','-dpng')
%print 4_2.jpg -djpeg -r600
%print 4_2.eps -depsc2 -r600