%% =========================================================
%% PUNTO 5 - VERIFICACION DEL JACOBIANO EN 4B Y 4D
%% =========================================================

clear
clc
close all

addpath('/home/david-londono-e/MATLAB/')

format long g

%% =========================================================
%% PARAMETROS DH DEL KUKA KR6 R700
%% =========================================================

aij = [0 25 335 25 0 0];

alphaij = [0 -pi/2 0 -pi/2 pi/2 -pi/2];

sj = [400 0 0 365 0 90];

%% =========================================================
%% ARCHIVOS DE LAS TRAYECTORIAS
%% =========================================================

archivo_4B = 'trayectoria_4b(1).json';

archivo_4D = 'trayectoria_4d(1).json';

%% =========================================================
%% RESTRICCIONES DE LOS PERFILES
%% =========================================================

% Tramo 4B: perfil rojo de aproximacion.

D_4B = 0.1;

VMAX_4B = 0.200;

AMAX_4B = 0.300;

% Tramo 4D: valores usados por verificar_jacobiano_4b_4d.py.

D_4D = 0.1;

VMAX_4D = 0.100;

AMAX_4D = 0.020;

%% =========================================================
%% VERIFICAR TRAMO 4B
%% =========================================================

resultados_4B = verificar_tramo( ...
    '4B: PRE-PICK A PICK', ...
    archivo_4B, ...
    D_4B, ...
    VMAX_4B, ...
    AMAX_4B, ...
    aij, ...
    alphaij, ...
    sj);

%% =========================================================
%% VERIFICAR TRAMO 4D
%% =========================================================

resultados_4D = verificar_tramo( ...
    '4D: PRE-PLACE A PLACE', ...
    archivo_4D, ...
    D_4D, ...
    VMAX_4D, ...
    AMAX_4D, ...
    aij, ...
    alphaij, ...
    sj);

%% =========================================================
%% RESUMEN FINAL
%% =========================================================

disp(' ')
disp('============================================================')
disp('RESUMEN DEL PUNTO 5')
disp('============================================================')

if ~isempty(resultados_4B)

    fprintf('TRAMO 4B\n');

    fprintf('Velocidad teorica maxima: %.6f m/s\n', ...
        max(resultados_4B.velocidad_teorica));

    fprintf('Velocidad obtenida maxima: %.6f m/s\n', ...
        max(resultados_4B.velocidad_real));

    fprintf('Error absoluto medio: %.6f m/s\n', ...
        mean(resultados_4B.error_absoluto));

    errores_validos_4B = resultados_4B.error_porcentual( ...
        resultados_4B.velocidad_teorica > 1e-6);

    if ~isempty(errores_validos_4B)

        fprintf('Error porcentual medio: %.4f %%\n', ...
            mean(errores_validos_4B));

    end

end

disp(' ')

if ~isempty(resultados_4D)

    fprintf('TRAMO 4D\n');

    fprintf('Velocidad teorica maxima: %.6f m/s\n', ...
        max(resultados_4D.velocidad_teorica));

    fprintf('Velocidad obtenida maxima: %.6f m/s\n', ...
        max(resultados_4D.velocidad_real));

    fprintf('Error absoluto medio: %.6f m/s\n', ...
        mean(resultados_4D.error_absoluto));

    errores_validos_4D = resultados_4D.error_porcentual( ...
        resultados_4D.velocidad_teorica > 1e-6);

    if ~isempty(errores_validos_4D)

        fprintf('Error porcentual medio: %.4f %%\n', ...
            mean(errores_validos_4D));

    end

end

%% =========================================================
%% GRAFICAS
%% =========================================================

if ~isempty(resultados_4B)

    figure

    plot( ...
        resultados_4B.tiempos, ...
        resultados_4B.velocidad_teorica, ...
        'LineWidth', 2)

    hold on

    plot( ...
        resultados_4B.tiempos, ...
        resultados_4B.velocidad_real, ...
        'LineWidth', 2)

    grid on

    xlabel('Tiempo [s]')

    ylabel('Velocidad lineal [m/s]')

    title('Verificacion de velocidad del TCP en 4B')

    legend( ...
        'Velocidad teorica', ...
        'Velocidad obtenida con J_v qdot', ...
        'Location', ...
        'best')

end

if ~isempty(resultados_4D)

    figure

    plot( ...
        resultados_4D.tiempos, ...
        resultados_4D.velocidad_teorica, ...
        'LineWidth', 2)

    hold on

    plot( ...
        resultados_4D.tiempos, ...
        resultados_4D.velocidad_real, ...
        'LineWidth', 2)

    grid on

    xlabel('Tiempo [s]')

    ylabel('Velocidad lineal [m/s]')

    title('Verificacion de velocidad del TCP en 4D')

    legend( ...
        'Velocidad teorica', ...
        'Velocidad obtenida con J_v qdot', ...
        'Location', ...
        'best')

end

%% =========================================================
%% FUNCION PARA VERIFICAR CADA TRAMO
%% =========================================================

function resultados = verificar_tramo( ...
    nombre, archivo, distancia, vmax, amax, ...
    aij, alphaij, sj)

resultados = [];

disp(' ')
disp('============================================================')
fprintf('%s\n', nombre)
disp('============================================================')

if ~isfile(archivo)

    fprintf('No se encontro el archivo:\n');

    fprintf('%s\n', archivo);

    fprintf(['Ejecuta primero el programa de Python correspondiente ' ...
        'para generar el archivo JSON.\n']);

    return

end

%% Leer archivo JSON

texto_json = fileread(archivo);

datos = jsondecode(texto_json);

if ~isfield(datos, 'tiempos')

    error('El archivo %s no contiene el campo tiempos.', archivo)

end

if ~isfield(datos, 'posiciones')

    error('El archivo %s no contiene el campo posiciones.', archivo)

end

tiempos = double(datos.tiempos(:));

posiciones_moveit = double(datos.posiciones);

%% Verificar dimensiones

if size(posiciones_moveit,2) ~= 6

    if size(posiciones_moveit,1) == 6

        posiciones_moveit = posiciones_moveit.';

    else

        error(['La matriz posiciones debe tener seis columnas, ' ...
            'una por articulacion.'])

    end

end

numero_puntos = length(tiempos);

if size(posiciones_moveit,1) ~= numero_puntos

    error(['El numero de tiempos no coincide con el numero ' ...
        'de posiciones articulares.'])

end

if numero_puntos < 2

    error('La trayectoria contiene muy pocos puntos.')

end

if any(diff(tiempos) <= 0)

    error('Los tiempos deben ser estrictamente crecientes.')

end

fprintf('Archivo cargado: %s\n', archivo);

fprintf('Numero de puntos: %d\n', numero_puntos);

fprintf('Tiempo inicial: %.6f s\n', tiempos(1));

fprintf('Tiempo final: %.6f s\n', tiempos(end));

%% =========================================================
%% VELOCIDADES ARTICULARES EN CONVENCION MOVEIT
%% =========================================================

qdot_moveit = derivada_numerica( ...
    posiciones_moveit, ...
    tiempos);

%% =========================================================
%% CONVERSION DE Q Y QDOT DE MOVEIT A DH
%% =========================================================

posiciones_DH = zeros(size(posiciones_moveit));

qdot_DH = zeros(size(qdot_moveit));

for punto = 1:numero_puntos

    posiciones_DH(punto,:) = convertir_MoveIt_a_DH( ...
        posiciones_moveit(punto,:));

    qdot_DH(punto,:) = convertir_qdot_MoveIt_a_DH( ...
        qdot_moveit(punto,:));

end

%% =========================================================
%% VELOCIDAD TEORICA DEL PERFIL QUINTICO
%% =========================================================

T_teorico = tiempo_total_quintico( ...
    distancia, ...
    vmax, ...
    amax);

velocidad_teorica = velocidad_teorica_quintico( ...
    distancia, ...
    T_teorico, ...
    tiempos);

%% =========================================================
%% EVALUAR EL JACOBIANO EN CADA PUNTO
%% =========================================================

velocidad_cartesiana = zeros(numero_puntos,6);

velocidad_real = zeros(numero_puntos,1);

error_absoluto = zeros(numero_puntos,1);

error_porcentual = zeros(numero_puntos,1);

jacobianos = zeros(6,6,numero_puntos);

fprintf('\n');

fprintf('%-10s %-18s %-22s %-14s\n', ...
    't [s]', ...
    'v_teorica [m/s]', ...
    'v_real [m/s]', ...
    'error [%]');

for punto = 1:numero_puntos

    theta_DH = posiciones_DH(punto,:);

    [J, Jv, ~, ~] = calcular_jacobiano_DH( ...
        theta_DH, ...
        aij, ...
        alphaij, ...
        sj);

    jacobianos(:,:,punto) = J;

    qdot_columna = qdot_moveit(punto,:).';
    velocidad_cartesiana(punto,:) = (J*qdot_columna).';
    velocidad_real(punto) = norm(Jv*qdot_columna);

    error_absoluto(punto) = abs( ...
        velocidad_real(punto) - ...
        velocidad_teorica(punto));

    if velocidad_teorica(punto) > 1e-6

        error_porcentual(punto) = ...
            100 * error_absoluto(punto) / ...
            velocidad_teorica(punto);

    else

        error_porcentual(punto) = 0;

    end

    fprintf('%-10.4f %-18.6f %-22.6f %-14.4f\n', ...
        tiempos(punto), ...
        velocidad_teorica(punto), ...
        velocidad_real(punto), ...
        error_porcentual(punto));

end

%% =========================================================
%% MOSTRAR JACOBIANOS DE LOS EXTREMOS
%% =========================================================

disp(' ')
fprintf('\n--- Matrices Jacobianas (lineal, 3x6, ya en convencion REAL) ---\n');
    for punto = 1:numero_puntos
        fprintf('\nJ en t=%.4f s (angulos reales=[%s]):\n', ...
            tiempos(punto), ...
            strjoin(arrayfun(@(x) sprintf('%.4f', x), posiciones_moveit(punto,:), ...
                'UniformOutput', false), ', '));
        disp(jacobianos(1:3, :, punto));   % Jv, ya con signos corregidos
    end
%% =========================================================
%% MOSTRAR Q INICIAL Y FINAL
%% =========================================================

disp('Angulos iniciales MoveIt ')

disp(posiciones_moveit(1,:))

disp('Angulos finales MoveIt ')

disp(posiciones_moveit(end,:))

disp('Angulos iniciales MoveIt ')

disp(rad2deg(posiciones_moveit(1,:)))

disp('Angulos finales MoveIt ')

disp(rad2deg(posiciones_moveit(end,:)))

disp('Angulos iniciales DH ')

disp(posiciones_DH(1,:))

disp('Angulos finales DH ')

disp(posiciones_DH(end,:))

%% =========================================================
%% GUARDAR RESULTADOS
%% =========================================================

resultados.nombre = nombre;

resultados.tiempos = tiempos;

resultados.posiciones_moveit = posiciones_moveit;

resultados.posiciones_DH = posiciones_DH;

resultados.qdot_moveit = qdot_moveit;

resultados.qdot_DH = qdot_DH;

resultados.jacobianos = jacobianos;

resultados.velocidad_cartesiana = velocidad_cartesiana;

resultados.velocidad_teorica = velocidad_teorica;

resultados.velocidad_real = velocidad_real;

resultados.error_absoluto = error_absoluto;

resultados.error_porcentual = error_porcentual;

resultados.T_teorico = T_teorico;

end

%% =========================================================
%% JACOBIANO ANALITICO MEDIANTE DH 
%% =========================================================


function [J, Jv, Jw, T06] = calcular_jacobiano_DH(thetaj, aij, alphaij, sj)

numero_articulaciones = 6;

T_acumulada = eye(4);

origenes = zeros(3, numero_articulaciones);
ejes_z   = zeros(3, numero_articulaciones);

for articulacion = 1:numero_articulaciones

    T_local = T_DH( ...
        aij(articulacion), alphaij(articulacion), ...
        sj(articulacion), thetaj(articulacion));

    T_acumulada = T_acumulada * T_local;

    origenes(:,articulacion) = T_acumulada(1:3,4);
    ejes_z(:,articulacion)   = T_acumulada(1:3,3);

end

T06 = T_acumulada;
origen_efector = T06(1:3,4);

Jv = zeros(3, numero_articulaciones);
Jw = zeros(3, numero_articulaciones);

for articulacion = 1:numero_articulaciones
    Jv(:,articulacion) = cross(ejes_z(:,articulacion), origen_efector - origenes(:,articulacion));
    Jw(:,articulacion) = ejes_z(:,articulacion);
end

Jv = Jv / 1000;   % mm -> m

J_DH = [Jv; Jw];

% --- Correccion de signos DH -> real (CORREGIDA: joint4 tambien invierte) ---
factores_signo = diag([-1, 1, 1, -1, 1, -1]);

J  = J_DH * factores_signo;
Jv = J(1:3, :);
Jw = J(4:6, :);

end



%% =========================================================
%% CONVERSION DE ANGULOS MOVEIT A DH
%% =========================================================
function theta_DH = convertir_MoveIt_a_DH(q_moveit)

theta_DH = zeros(1,6);

theta_DH(1) = -q_moveit(1);
theta_DH(2) = q_moveit(2);
theta_DH(3) = q_moveit(3) - pi/2;
theta_DH(4) = -q_moveit(4);   % <-- CORREGIDO: antes era +q_moveit(4)
theta_DH(5) = q_moveit(5);
theta_DH(6) = pi - q_moveit(6);

end

%% =========================================================
%% CONVERSION DE VELOCIDADES MOVEIT A DH
%% =========================================================



% Los offsets constantes desaparecen al derivar.
%
% theta_DH_1 = -joint_1
% theta_DH_2 =  joint_2
% theta_DH_3 =  joint_3 - pi/2
% theta_DH_4 =  joint_4
% theta_DH_5 =  joint_5
% theta_DH_6 =  pi - joint_6
function qdot_DH = convertir_qdot_MoveIt_a_DH(qdot_moveit)

qdot_DH = zeros(1,6);

qdot_DH(1) = -qdot_moveit(1);
qdot_DH(2) = qdot_moveit(2);
qdot_DH(3) = qdot_moveit(3);
qdot_DH(4) = -qdot_moveit(4);   % <-- CORREGIDO: antes era +qdot_moveit(4)
qdot_DH(5) = qdot_moveit(5);
qdot_DH(6) = -qdot_moveit(6);

end

%% =========================================================
%% DERIVADA NUMERICA DE LAS POSICIONES ARTICULARES
%% =========================================================

function derivada = derivada_numerica(valores, tiempos)

numero_puntos = size(valores,1);

numero_variables = size(valores,2);

derivada = zeros(numero_puntos,numero_variables);

%% Primer punto: diferencia hacia adelante

delta_t_inicial = tiempos(2) - tiempos(1);

derivada(1,:) = ...
    (valores(2,:) - valores(1,:)) / delta_t_inicial;

%% Puntos interiores: misma formulacion de numpy.gradient
% para tiempos no uniformemente espaciados

for punto = 2:numero_puntos-1

    h_anterior = tiempos(punto) - tiempos(punto-1);

    h_siguiente = tiempos(punto+1) - tiempos(punto);

    coef_anterior = ...
        -h_siguiente / ...
        (h_anterior * (h_anterior + h_siguiente));

    coef_actual = ...
        (h_siguiente - h_anterior) / ...
        (h_anterior * h_siguiente);

    coef_siguiente = ...
        h_anterior / ...
        (h_siguiente * (h_anterior + h_siguiente));

    derivada(punto,:) = ...
        coef_anterior * valores(punto-1,:) + ...
        coef_actual * valores(punto,:) + ...
        coef_siguiente * valores(punto+1,:);

end

%% Ultimo punto: diferencia hacia atras

delta_t_final = tiempos(end) - tiempos(end-1);

derivada(end,:) = ...
    (valores(end,:) - valores(end-1,:)) / delta_t_final;

end

%% =========================================================
%% TIEMPO TOTAL DEL PERFIL QUINTICO
%% =========================================================

function T = tiempo_total_quintico(distancia, vmax, amax)

T_velocidad = 1.875*distancia/vmax;

T_aceleracion = sqrt( ...
    (10/sqrt(3))*distancia/amax);

T = max(T_velocidad,T_aceleracion);

end

%% =========================================================
%% VELOCIDAD TEORICA DEL PERFIL QUINTICO
%% =========================================================

function velocidad = velocidad_teorica_quintico( ...
    distancia, T, tiempos)

u = tiempos/T;

u = max(0,min(1,u));

velocidad = (distancia/T).*( ...
    30*u.^2 ...
    - 60*u.^3 ...
    + 30*u.^4);

end

function T = T_DH(a, alpha, d, theta)

T = [
    cos(theta),             -sin(theta),             0,           a;
    sin(theta)*cos(alpha),   cos(theta)*cos(alpha),  -sin(alpha), -sin(alpha)*d;
    sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),  cos(alpha)*d;
    0,                       0,                       0,           1
];

end
