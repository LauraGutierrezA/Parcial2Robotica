function comparar_capa(SlnSet, theta_moveit, k)
% Compara la capa k de SlnSet contra la solución de MoveIt,
% aplicando la conversión de convención de signo/offset encontrada
% entre el modelo DH propio y el URDF/MoveIt:
%   joint_1_real =  -theta_DH_1
%   joint_2_real =   theta_DH_2
%   joint_3_real =   theta_DH_3 - 90°   (deg2rad(-90))
%   joint_4_real =   theta_DH_4
%   joint_5_real =   theta_DH_5
%   joint_6_real =   180° - theta_DH_6  (pi - theta_DH_6)
%
% Entradas:
%   SlnSet       -> tensor de tu IK_try, n x 6 x 8 (usa SlnSet(1,:,:) si es un solo punto)
%   theta_moveit -> vector 1x6 con los ángulos de /compute_ik, en RADIANES
%   k            -> índice de la capa a comparar (1 a 8)

    theta_DH = squeeze(SlnSet(1,:,k));   % ajusta el índice de fila si tu estructura es distinta

    % Vector de signos por junta (+1 o -1)
    signo = [-1, 1, 1, 1, 1, -1];

    % Offset por junta, en radianes
    offset = deg2rad([0, 0, -90, 0, 0, 180]);

    % Aplicar transformación junta por junta
    % (joint_6 usa la forma "offset - theta", las demás "signo*theta + offset")
    theta_convertido = signo .* theta_DH + offset;
    theta_convertido(6) = offset(6) - theta_DH(6);   % caso especial de joint_6

    % Comparar contra MoveIt, con wrap-around de ángulos
    diff = theta_convertido - theta_moveit;
    diff_wrapped = mod(diff + pi, 2*pi) - pi;
    err = norm(diff_wrapped);

    fprintf('--- Capa %d ---\n', k);
    fprintf('Theta DH crudo      (rad): %s\n', mat2str(round(theta_DH,4)));
    fprintf('Theta convertido    (rad): %s\n', mat2str(round(theta_convertido,4)));
    fprintf('Theta MoveIt        (rad): %s\n', mat2str(round(theta_moveit,4)));
    fprintf('Diferencia por junta(rad): %s\n', mat2str(round(diff_wrapped,4)));
    fprintf('Error total (norma)      : %.4f rad (%.2f°)\n\n', err, rad2deg(err));
end
