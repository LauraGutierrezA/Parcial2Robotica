function animar_robot(SlnSetVer, capa, aij, alphaij, sj, guardar_video, varargin)
% guardar_video: true/false — si true, exporta un .mp4 al final
% varargin{1}: duracion del pause entre cuadros (default 0.5 s)

    if nargin < 2, capa = 1; end
    if nargin < 6, guardar_video = false; end

    % *** NUEVO: leer el pause desde varargin, con default 0.5 ***
    if ~isempty(varargin)
        t_pause = varargin{1};
    else
        t_pause = 0.5;
    end

    n = size(SlnSetVer,1);
    figure; hold on; grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45,30)
    all_pts = [];
    for k = 1:n
        thetaj = squeeze(SlnSetVer(k,:,capa));
        pts = forward_points(thetaj, aij, alphaij, sj);
        all_pts = [all_pts; pts]; %#ok<AGROW>
    end
    margin = 50;
    xlim([min(all_pts(:,1))-margin, max(all_pts(:,1))+margin])
    ylim([min(all_pts(:,2))-margin, max(all_pts(:,2))+margin])
    zlim([min(all_pts(:,3))-margin, max(all_pts(:,3))+margin])
    h_robot = plot3(nan,nan,nan,'-o','LineWidth',3,'MarkerSize',6,...
        'MarkerFaceColor','b','Color',[0.2 0.2 0.8]);
    h_traj  = animatedline('Color','r','LineStyle','none','Marker','.','MarkerSize',10);

    if guardar_video
        vid = VideoWriter(sprintf('animacion_IK_capa%d.mp4', capa), 'MPEG-4');
        vid.FrameRate = 10;
        open(vid);
    end

    for k = 1:n
        thetaj = squeeze(SlnSetVer(k,:,capa));
        pts = forward_points(thetaj, aij, alphaij, sj);
        set(h_robot, 'XData', pts(:,1), 'YData', pts(:,2), 'ZData', pts(:,3));
        addpoints(h_traj, pts(end,1), pts(end,2), pts(end,3));
        title(sprintf('Punto %d de %d — capa %d', k, n, capa));
        drawnow;
        if guardar_video
            frame = getframe(gcf);
            writeVideo(vid, frame);
        end
        pause(t_pause);   % *** ahora usa el parametro, no un valor fijo ***
    end

    if guardar_video
        close(vid);
        fprintf('Video guardado como animacion_IK_capa%d.mp4\n', capa);
    end
end

function pts = forward_points(thetaj, aij, alphaij, sj)
    frame = numel(aij);
    DH = [aij(:), alphaij(:), sj(:), thetaj(:)];
    T = eye(4);
    pts = zeros(frame+1, 3);
    pts(1,:) = T(1:3,4)';
    for i = 1:frame
        Ti = T_DH(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        T = T * Ti;
        pts(i+1,:) = T(1:3,4)';
    end
end