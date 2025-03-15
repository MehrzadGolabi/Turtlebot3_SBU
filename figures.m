function figures(path,poses,realposes,estPose,map1,map2,particles,idx,difrence)
    figure(2);
        %hold on;
        plot(difrence(1,1:idx),difrence(2,1:idx), '-');
        %hold off;

    figure(1);  % plot
    subplot(1,3,1);% naghshe asli
    show(map1);
        title(' ');
        hold on;
        plot(path(:,1), path(:,2), 'o');
        plot(poses(1,1:idx), poses(2,1:idx), 'blue-','LineWidth',2);
        plot(realposes(1,1:idx), realposes(2,1:idx), 'g-','LineWidth',2);
        plot(estPose(1,1:idx), estPose(2,1:idx), 'black.', 'MarkerSize',4);
        hold off;
    subplot(1,3,2); % neghshe lidar
        show(map2);
        title(' ');
        hold on;
        plot(poses(1,idx)', poses(2,idx)', 'ko');
        hold off;
    subplot(1,3,3); % particle
        title(' ');
        hold on
        plot(estPose(1), estPose(2), 'k.','MarkerSize',5); % Estimated position
        plot(particles(1,:), particles(2,:), '.'); % Particles
        hold off
    a = {'','   خط سبز : مسیر مرجع            ','خط آبی: مسیر واقعیت طی شده          ','نقطات مشکی : موقعیت تخمینی      ',' ',' ','نقشه ایحاد شده با لیدار           ',' ',' ',' ذره های  دوباره رسم شده در هر مرحله '};
    annotation('textbox', [.16 .02 .15 .15],'String', a(1:4), 'EdgeColor', 'blue','BackgroundColor','white', 'FontSize', 12);
    annotation("textbox", [.44 .02 .15 .15],'String', a(5:7) , 'EdgeColor', 'blue','BackgroundColor','white','FontSize', 12)
    annotation("textbox", [.72 .02 .15 .15],'String', a(8:10) , 'EdgeColor', 'blue','BackgroundColor','white','FontSize', 12)
end