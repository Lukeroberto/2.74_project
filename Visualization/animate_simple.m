function animate_simple(t,z,p, speed)
    hold on
    axis([-.2 .2 -.2 3])
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
    h_leg    = plot([0],[0],'-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 

    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint,p,h_leg);
    end
    draw_lines(z(:,end),p,h_leg);
end

function draw_lines(z,p, h_leg)
    keypoints = keypoints_jumping_leg(z,p);
    h_leg.XData = keypoints(1,:);
    h_leg.YData = keypoints(2,:);
    drawnow
end