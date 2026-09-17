%% jacobiano_a_mano.m
% Parte 5 - Jacobiano analitico, calculado EXPLICITAMENTE columna por
% columna (igual que en AnalisisVeloc.pdf y las diapositivas de
% Velocidad: Jvi = z_(i-1) x (On - O_(i-1)), Jwi = z_(i-1), para
% articulaciones revolutas -- las 6 juntas del KR6 son revolutas).
%
% Requiere: T_DH.m en el mismo path, y tus vectores DH de siempre.

clear; clc;

% --- Tus mismos vectores DH de siempre (columnas: a, alpha, d) ---
% aij = [...]; alphaij = [...]; sj = [...];

syms th1 th2 th3 th4 th5 th6 real

%% ==================== PASO 1: Cadena de transformaciones ====================
% T0 = base (identidad). T1..T6 = transformaciones ACUMULADAS hasta cada
% frame (igual que el patron de AnalisisVeloc.pdf: T(:,:,i+1)=T(:,:,i)*T_ij).

T0 = eye(4);   % frame base

T_01 = T_DH(aij(1), alphaij(1), sj(1), th1);
T1 = T_01;

T_12 = T_DH(aij(2), alphaij(2), sj(2), th2);
T2 = T1 * T_12;

T_23 = T_DH(aij(3), alphaij(3), sj(3), th3);
T3 = T2 * T_23;

T_34 = T_DH(aij(4), alphaij(4), sj(4), th4);
T4 = T3 * T_34;

T_45 = T_DH(aij(5), alphaij(5), sj(5), th5);
T5 = T4 * T_45;

T_56 = T_DH(aij(6), alphaij(6), sj(6), th6);
T6 = T5 * T_56;

%% ==================== PASO 2: Extraer z_(i-1) y O_(i-1) de cada frame ====================
% z_(i-1) = tercera columna de la matriz de rotacion del frame (i-1)
% O_(i-1) = cuarta columna (traslacion) del frame (i-1)

z0 = T0(1:3,3);   O0 = T0(1:3,4);   % frame base
z1 = T1(1:3,3);   O1 = T1(1:3,4);
z2 = T2(1:3,3);   O2 = T2(1:3,4);
z3 = T3(1:3,3);   O3 = T3(1:3,4);
z4 = T4(1:3,3);   O4 = T4(1:3,4);
z5 = T5(1:3,3);   O5 = T5(1:3,4);

On = T6(1:3,4);   % posicion del efector final (frame 6)

%% ==================== PASO 3: Jacobiano lineal, columna por columna ====================
% Formula (todas las juntas son REVOLUTAS): Jvi = z_(i-1) x (On - O_(i-1))

Jv1 = simplify(cross(z0, On - O0));
Jv2 = simplify(cross(z1, On - O1));
Jv3 = simplify(cross(z2, On - O2));
Jv4 = simplify(cross(z3, On - O3));
Jv5 = simplify(cross(z4, On - O4));
Jv6 = simplify(cross(z5, On - O5));

Jv = [Jv1, Jv2, Jv3, Jv4, Jv5, Jv6];

%% ==================== PASO 4: Jacobiano angular, columna por columna ====================
% Formula (todas las juntas son REVOLUTAS): Jwi = z_(i-1)

Jw1 = z0;
Jw2 = z1;
Jw3 = z2;
Jw4 = z3;
Jw5 = z4;
Jw6 = z5;

Jw = [Jw1, Jw2, Jw3, Jw4, Jw5, Jw6];

%% ==================== PASO 5: Jacobiano completo ====================
J_DH = [Jv; Jw];   % 6x6, derivado respecto a los angulos DH puros (th1..th6)

fprintf('Jacobiano J_DH (respecto a angulos DH) calculado, columna por columna.\n');

%% ==================== PASO 6: Correccion DH -> angulos reales ====================
% (necesaria para comparar contra MoveIt2/KDL, que trabajan en angulos
% reales del URDF -- ver relacion de signos de la Parte 3)
%   joint1_real = -th1_DH        -> factor -1
%   joint2_real =  th2_DH        -> factor +1
%   joint3_real =  th3_DH + 90   -> factor +1 (offset constante no afecta)
%   joint4_real =  th4_DH        -> factor +1
%   joint5_real =  th5_DH        -> factor +1
%   joint6_real = 180 - th6_DH   -> factor -1
factores_signo = diag([-1, 1, 1, 1, 1, -1]);
J_real = J_DH * factores_signo;

fprintf('Jacobiano J_real (corregido por signos, comparable con MoveIt2/KDL) listo.\n');
