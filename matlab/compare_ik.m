% Ángulos devueltos por MoveIt para 'pick' (en radianes)
theta_moveit_pick = [-1.197153898, 0.646602806, -1.077764789, ...
                       3.159085258, -1.890033045, 3.585575272];

% SlnSet_pick(1,:,k) se asume en RADIANES; si tu IK_try devuelve en
% GRADOS, descomenta la siguiente línea y comenta la de radianes:
% theta_moveit_pick = rad2deg(theta_moveit_pick);

n_layers = size(SlnSet_pick, 3);
errores = nan(n_layers, 1);

fprintf('%-8s %-10s %s\n', 'Capa', 'Error(rad)', 'Ángulos de la capa');
for k = 1:n_layers
    sol_k = squeeze(SlnSet_pick(1,:,k));   % [1x6], ajusta el índice de fila si aplica

    % Si la capa está vacía/NaN (solución no válida), sáltala
    if any(isnan(sol_k))
        continue
    end

    diff = sol_k - theta_moveit_pick;
    diff_wrapped = mod(diff + pi, 2*pi) - pi;   % corrige el wrap-around de ángulos
    err = norm(diff_wrapped);
    errores(k) = err;

    fprintf('%-8d %-10.4f %s\n', k, err, mat2str(round(sol_k,4)));
end

[min_err, best_k] = min(errores);
fprintf('\n>>> Mejor coincidencia: capa %d, con error total = %.4f rad (%.2f°)\n', ...
    best_k, min_err, rad2deg(min_err));
disp('Ángulos de esa capa (rad):')
disp(squeeze(SlnSet_pick(1,:,best_k)))
disp('Ángulos de esa capa (grados):')
disp(rad2deg(squeeze(SlnSet_pick(1,:,best_k))))
