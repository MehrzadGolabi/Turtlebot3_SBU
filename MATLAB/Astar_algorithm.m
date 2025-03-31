% خواندن تصویر نقشه
mapImage = imread('myMap.pgm');

% تبدیل تصویر به نقشه باینری
binaryMap = mapImage < 128;

% تنظیم رزولوشن جدید برای نقشه
resolution = 0.02; % افزایش دقت نقشه

% ایجاد شیء binaryOccupancyMap با رزولوشن جدید
map = binaryOccupancyMap(binaryMap, 1/resolution);
% مقدار دهی اولیه شعاع ربات
robotRadius = 0.02;  % به متر

% گسترش موانع نقشه با شعاع ربات
inflate(map, robotRadius);
% نمایش نقشه برای انتخاب نقاط شروع و هدف
figure;
show(map);
title('نقشه با رزولوشن بالاتر برای انتخاب نقاط شروع و هدف');

% دریافت نقاط شروع و هدف از کاربر
[x, y] = ginput(2);
startWorld = [x(1), y(1)];
goalWorld  = [x(2), y(2)];

% تبدیل مختصات دنیای واقعی به مختصات گرید
startGrid = world2grid(map, startWorld);
goalGrid  = world2grid(map, goalWorld);

% بررسی آزاد بودن نقاط شروع و هدف
if checkOccupancy(map, startWorld)
    error('نقطه شروع انتخاب‌شده اشغال شده است. لطفاً نقطه دیگری انتخاب کنید.');
end
if checkOccupancy(map, goalWorld)
    error('نقطه هدف انتخاب‌شده اشغال شده است. لطفاً نقطه دیگری انتخاب کنید.');
end

% برنامه‌ریزی مسیر با استفاده از الگوریتم A*
planner = plannerAStarGrid(map);
[path, pathCost] = plan(planner, startGrid, goalGrid);

% تبدیل مسیر به مختصات دنیای واقعی برای نمایش
worldPath = grid2world(map, path);

% نمایش مسیر نهایی
figure;
show(map);
hold on;
plot(worldPath(:,1), worldPath(:,2), 'r-', 'LineWidth', 2);
plot(startWorld(1), startWorld(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goalWorld(1), goalWorld(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
%title(sprintf('مسیر برنامه‌ریزی شده با A* (هزینه = %.2f)', pathCost));
legend('مسیر', 'شروع', 'هدف');
hold off;


