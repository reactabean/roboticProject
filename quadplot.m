function [] = quadplot( quadInput, idealPath, time, limits, plotheight, inputName )
% Plots 4 values in a subplot figure
% Plot exceeeds limits by a factor of plotheight

f1 = figure;

for i = 1:4
    p1 = subplot(2, 2, i);
    plot(time(:), quadInput(:, i), 'LineWidth', 2);
    if limits(2, i) > 0
        axis(p1, [0 time(end) limits(1, i)*plotheight limits(2, i)*plotheight]);
    else
        axis(p1, [0 time(end) limits(1, i)*plotheight limits(2, i)/plotheight]);
    end
    vec = ones(size(quadInput(:, i), 1), 2);
    vec(:, 1) = limits(1, i);
    vec(:, 2) = limits(2, i);
    hold on;
    plot(time, 0, 'color', 'green'); % make ideal path?
    plot(time, vec, 'color', 'red');
    title(['Values for ', inputName]);

    if (quadInput(end) > 0)
        legend(['Current ', inputName], 'Ideal Path', [inputName, ' Limits'], 'Location', 'southeast');
    else
        legend(['Current ', inputName], 'Ideal Path', [inputName, ' Limits'], 'Location', 'northeast');
    end

    xlabel('Time (ms)');
    ylabel(inputName);
%     plot(timeValues(findBreaks(2)-findBreaks(1)),theta1values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
%     plot(timeValues(findBreaks(3)-findBreaks(1)),theta1values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
%     plot(timeValues(findBreaks(4)-findBreaks(1)),theta1values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
%     plot(timeValues(end),theta1values(size(theta1values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
% 
%     plot(0,theta1spline(1, 1),'r.','MarkerSize',15, 'color', 'green')
%     plot(timeValues(size(theta1values, 1)/4),theta1spline(2, 1),'r.','MarkerSize',15, 'color', 'green')
%     plot(timeValues(size(theta1values, 1)*2/4),theta1spline(3, 1),'r.','MarkerSize',15, 'color', 'green')
%     plot(timeValues(size(theta1values, 1)*3/4),theta1spline(4, 1),'r.','MarkerSize',15, 'color', 'green')
%     plot(timeValues(size(theta1values, 1)),sum(theta1spline(4, :)),'r.','MarkerSize',15, 'color', 'green')
end
set (f1, 'Units', 'normalized', 'Position', [0,0,1,1]);
end

