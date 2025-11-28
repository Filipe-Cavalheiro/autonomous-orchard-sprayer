function plot_figures(animate, Tree_pos,ref, state_history, actuation_history, linkage_dim)
% --- Plot Trees ---
figure('Color', 'w');
mkr = '+';
color_brown = [0.6 0.3 0];
scatter(Tree_pos(:,1), Tree_pos(:,2), [], color_brown, mkr, 'HandleVisibility','on');
hold on;
viscircles(Tree_pos, 2,'Color', [0 0.3 0]);
plot(ref(:,1),ref(:,2), 'HandleVisibility','on') 

if animate == 1
    tractor_path = animatedline('Color',[220 220 220]/255,'HandleVisibility','off');
    
    % Create a circle plot object (handles)
    tractor_radius = 1;                      % radius values for the tractor
    theta = linspace(0, 2*pi, 100);         % Angle values for the tractor
    x_circle = tractor_radius * cos(theta);  % X coordinates of the tractor
    y_circle = tractor_radius * sin(theta);  % Y coordinates of the tractor
    tractor= plot(NaN, NaN, 'k');           % Initialize a placeholder for the tractor
    tractor_linkage = plot(NaN, NaN, 'k');  % Initialize a placeholder for the linkage
    tractor_trailer = plot(NaN, NaN, 'k');  % Initialize a placeholder for the trailer
    
    % --- Trailer rectangle parameters ---
    rect_length = 0.3;    % 1 meter
    rect_width  = 1;  % 0.3 meter
    % Rectangle corner offsets before rotation (centered at origin)
    rect_x = 0.5 * [-rect_length,  rect_length,  rect_length, -rect_length];
    rect_y = 0.5 * [-rect_width,  -rect_width,   rect_width,   rect_width];
    
    for i = 1:size(state_history, 2)
        addpoints(tractor_path,state_history(1,i),state_history(3,i));
        % Update the position of the circle
        set(tractor, 'XData', state_history(1,i) + x_circle, 'YData', state_history(3,i) + y_circle);
        
        % Compute direction of movement (the angle)
        if i > 1
            dx = state_history(1,i) - state_history(1,i-1);
            dy = state_history(3,i) - state_history(3,i-1);
            angle = atan2(dy, dx);  % Angle of movement in radians
        else
            angle = 0;  % Initial angle (you could adjust based on initial direction)
        end
        
        % Position of the linkage (1m behind the tractor)
        linkage_x = state_history(1,i) - linkage_dim*cos(angle);
        linkage_y = state_history(3,i) - linkage_dim*sin(angle);
        set(tractor_linkage, 'XData', [state_history(1,i), linkage_x], 'YData', [state_history(3,i), linkage_y]);
        
        % Rotation matrix for angle + pi (trailer faces opposite direction of tractor)
        R = [cos(angle+pi), -sin(angle+pi);
             sin(angle+pi),  cos(angle+pi)];
        
        rotated = R * [rect_x; rect_y];
        
        % Shift rectangle to trailer position
        rect_x_world = rotated(1,:) + linkage_x;
        rect_y_world = rotated(2,:) + linkage_y;
        
        % Update patch data
        set(tractor_trailer, 'XData', rect_x_world, 'YData', rect_y_world);
        drawnow
        pause(0.01)
    end
end

% --- Plot Tractor Path ---
plot(state_history(1,:), state_history(2,:), 'r-', 'LineWidth', 1.5);
scatter(state_history(1,1), state_history(2,1), 80, 'bo', 'filled');
scatter(state_history(1,end), state_history(2,end), 80, 'ro', 'filled');

% Formatting
xlabel('X Position [m]', 'FontSize', 12);
ylabel('Y Position [m]', 'FontSize', 12);
title('Tractor Path Among Olive Trees', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
axis([-1 20 -1 20])
set(gca, 'GridLineStyle', '--', 'FontSize', 11);
legend('Trees', 'Reference', 'Tractor Path', 'Start', 'End', 'Location', 'northwest');

% % plot results
% t = 1:size(state_history,2)-1;
% figure('Name','Backstepping control - trajectory','NumberTitle','off'); 
% subplot(2,1,1); hold on; grid on;
% plot(t, actuation_history(1,:), 'LineWidth', 1.5);
% xlabel('steps'); ylabel('actuation (x)');
% 
% subplot(2,1,2); hold on; grid on;
% plot(t, actuation_history(2,:), 'LineWidth', 1.5);
% xlabel('steps'); ylabel('actuation (y)');
end

