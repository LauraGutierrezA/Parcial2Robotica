%% jacobiano_completo.m
% Parte 5 - Jacobiano analitico (formula de Velocidad.pdf / AnalisisVeloc.pdf)
% + comparacion de velocidades en los tramos 4B y 4D, con el MISMO
% formato de tabla que el script de Python que corre con RViz.
%
% Formula usada (juntas revolutas, todas las del KR6 lo son):
%   Jvi = z_(i-1) x (On - O_(i-1))      -- velocidad lineal
%   Jwi = z_(i-1)                       -- velocidad angular
% donde z_(i-1) y O_(i-1) salen de la transformacion acumulada T_(i-1)
% (columna 3 = eje z, columna 4 = origen), y On es la posicion final.
%
% Requiere en el mismo path: T_DH.m, trayectoria_4b.json, trayectoria_4d.json

clear; clc;

% --- Tus mismos vectores DH de siempre ---
% aij = [...]; alphaij = [...]; sj = [...];

%% ==================== JACOBIANO ANALITICO (con ciclo for) ====================
syms th1 th2 th3 th4 th5 th6 real
thetaj_syms = [th1 th2 th3 th4 th5 th6];
DH_syms = [aij(:), alphaij(:), sj(:), thetaj_syms'];
n = size(DH_syms, 1);   % n = 6 juntas

% T{i+1} = transformacion ACUMULADA hasta el frame i (T{1} = base = identidad)
T = cell(n+1, 1);
T{1} = eye(4);
for i = 1:n
    Ti = T_DH(DH_syms(i,1), DH_syms(i,2), DH_syms(i,3), DH_syms(i,4));
    T{i+1} = simplify(T{i} * Ti);
end

On = T{n+1}(1:3,4);   % posicion del efector final

J_DH = sym(zeros(6, n));
for i = 1:n
    % IMPORTANTE: se usa T{i+1} (frame DESPUES de aplicar la junta i),
    % no T{i} -- esta es la convencion real de la clase (ver
    % AnalisisVeloc.pdf: Jv1 usa T(:,:,1), el frame ya transformado
    % por la junta 1, no la identidad/base).
    z_i = T{i+1}(1:3,3);
    O_i = T{i+1}(1:3,4);

    Jvi = cross(z_i, On - O_i);   % junta revoluta: formula del producto cruz
    Jwi = z_i;                    % junta revoluta: Jwi = z_i

    J_DH(:, i) = simplify([Jvi; Jwi]);
end

fprintf('Jacobiano J_DH calculado (6x%d), columna i = junta i.\n', n);

% --- Correccion de signos: DH -> angulos reales (ver relacion de la Parte 3) ---
%   joint1_real = -th1_DH -> -1 | joint2_real = th2_DH -> +1
%   joint3_real = th3_DH+90 -> +1 (offset no afecta) | joint4_real = -th4_DH -> -1
%   (CORREGIDO: se encontro por verificacion cruzada contra el Jacobiano
%   numerico via /compute_fk que joint4 tambien invierte signo, no +1
%   como se penso originalmente en la Parte 3)
%   joint5_real = th5_DH -> +1 | joint6_real = 180-th6_DH -> -1
J_real = J_DH * diag([-1, 1, 1, -1, 1, -1]);
fprintf('Jacobiano J_real (corregido, comparable con MoveIt2/KDL) listo.\n\n');


%% ==================== TABLA: mismo formato que el script de Python ====================
real_a_DH = @(r) [-r(1), r(2), r(3) - pi/2, -r(4), r(5), pi - r(6)];

D_4B = 0.1; VMAX_4B = 0.200; AMAX_4B = 0.300;
D_4D = 0.1; VMAX_4D = 0.100; AMAX_4D = 0.020;

procesar_tramo('trayectoria_4b.json', '4B (pre-pick -> pick)', ...
    D_4B, VMAX_4B, AMAX_4B, J_real, real_a_DH, thetaj_syms);

procesar_tramo('trayectoria_4d.json', '4D (pre-place -> place)', ...
    D_4D, VMAX_4D, AMAX_4D, J_real, real_a_DH, thetaj_syms);


function procesar_tramo(archivo, nombre, d, vmax, amax, J_real, real_a_DH, th_syms)
    fprintf('======================================================================\n');
    fprintf('TRAMO: %s\n', nombre);
    fprintf('======================================================================\n');

    if ~isfile(archivo)
        fprintf('  No se encontro %s.\n', archivo);
        return;
    end

    datos = jsondecode(fileread(archivo));
    tiempos = datos.tiempos(:);
    posiciones = datos.posiciones;
    n = length(tiempos);
    if n < 2
        fprintf('  Muy pocos puntos guardados.\n');
        return;
    end

    T_total = tiempo_total_quintico(d, vmax, amax);

    theta_dot = zeros(n, 6);
    theta_dot(1,:) = (posiciones(2,:) - posiciones(1,:)) / (tiempos(2) - tiempos(1));
    theta_dot(end,:) = (posiciones(end,:) - posiciones(end-1,:)) / (tiempos(end) - tiempos(end-1));
    for i = 2:n-1
        theta_dot(i,:) = (posiciones(i+1,:) - posiciones(i-1,:)) / (tiempos(i+1) - tiempos(i-1));
    end

    % --- Imprimir la matriz Jacobiana COMPLETA (6x6, J_real) en cada punto ---
    fprintf('\n--- Matrices Jacobianas (J_real, 6x6) en cada punto de %s ---\n', nombre);
    J_num_todos = cell(n, 1);
    for i = 1:n
        angulos_DH = real_a_DH(posiciones(i,:));
        J_num_todos{i} = double(subs(J_real, th_syms, angulos_DH));
        fprintf('\nJ en t=%.4f s (angulos reales=[%s]):\n', tiempos(i), ...
            strjoin(arrayfun(@(x) sprintf('%.4f', x), posiciones(i,:), 'UniformOutput', false), ', '));
        disp(J_num_todos{i});
    end

    % --- Tabla resumen (mismo formato que verificar_jacobiano_4b_4d.py) ---
    fprintf('\n--- Tabla resumen: velocidad teorica vs. analitica ---\n');
    fprintf('%-10s%-18s%-22s%s\n', 't (s)', 'v_teorica (m/s)', 'v_real |J*qdot| (m/s)', 'error (%)');
    for i = 1:n
        Jv = J_num_todos{i}(1:3, :);

        v_real = norm(Jv * theta_dot(i,:)');
        v_teo = velocidad_teorica_quintico(d, T_total, tiempos(i));
        if v_teo > 1e-6
            error_pct = abs(v_real - v_teo) / v_teo * 100;
        else
            error_pct = 0;
        end

        fprintf('%-10.4f%-18.4f%-22.4f%.2f%%\n', tiempos(i), v_teo, v_real, error_pct);
    end
    fprintf('\n');
end

function T = tiempo_total_quintico(d, vmax, amax)
    T = max(1.875 * d / vmax, sqrt((10.0 / sqrt(3.0)) * d / amax));
end

function v = velocidad_teorica_quintico(d, T, t)
    u = min(max(t / T, 0.0), 1.0);
    v = (d / T) * (30 * u^2 - 60 * u^3 + 30 * u^4);
end
