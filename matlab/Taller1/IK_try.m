function [SlnSet, SlnSetVer] = IK_try(Q, S, A67, aij, alphaij, sj)
% Q, S, A67: matrices de nx3 donde n es el número de puntos de la trayectoria
% aij, alphaij, sj: son los vectores del DH original
    n = size(Q,1);
    SlnSet = NaN(n, 6, 8);
    tol = 1e-6;

    syms th1 th2 th3 th4 th5 th6 real
    px = sym('px','real');
    py = sym('py','real');
    pz = sym('pz','real');
    syms wx wy wz ux uy uz real
    qx = sym('qx','real');
    qy = sym('qy','real');
    qz = sym('qz','real');

    thetaj_syms = [th1 th2 th3 th4 th5 th6];
    DH_syms = [aij', alphaij', sj', thetaj_syms'];
    DH = [aij', alphaij', sj', zeros(6,1)];

    frame = size(DH_syms, 1);

    for i=1:frame
        T_ij_syms(:,:,i) = T_DH(DH_syms(i,1), DH_syms(i,2), DH_syms(i,3), DH_syms(i,4));
    end
    T_syms(:,:,1) = T_ij_syms(:,:,1);
    for i=1:frame-1
        T_syms(:,:,i+1) = T_syms(:,:,i)*T_ij_syms(:,:,i+1);
    end

    %% ---- LHS / RHS (desacople) ----
    Pc = [px,py,pz,1]';

    RHS = simplify(expand(inv(T_ij_syms(:,:,1))*Pc));
    RHS(1) = RHS(1) - DH(2,1);

    Pc3 = [DH_syms(4,1);
           -DH_syms(4,3)*sin(DH_syms(4,2));
            DH_syms(4,3)*cos(DH_syms(4,2))];

    LHS = simplify(expand(T_ij_syms(:,:,2)*T_ij_syms(:,:,3)*[Pc3;1]));
    LHS(1) = LHS(1) - DH(2,1);

    %% ---- THETA1 (2 ramas) ----
    th1_a = atan2(Pc(2), Pc(1));
    th1_b = th1_a + pi;

    %% ---- THETA3: coeficientes del triángulo auxiliar (SIN sustituir aún th1) ----
    LHS_ss = expand(LHS(1)^2 + LHS(3)^2);
    RHS_ss = expand(RHS(1)^2 + RHS(3)^2);     % <-- sto sigue teniendo th1 adentro
    LHS_ss = simplify(LHS_ss);

    syms cos_th3 sin_th3 real
    LHS_ss_mod = subs(LHS_ss, [cos(th3), sin(th3)], [cos_th3, sin_th3]);
    coeffs3 = jacobian(LHS_ss_mod, [cos_th3, sin_th3]);

    A = coeffs3(1);
    B = coeffs3(2);
    LHS_ss_indep = simplify(LHS_ss_mod - dot(coeffs3, [cos_th3, sin_th3])); % Saca C, solo depende de los parámetros

    D = simplify(LHS_ss_indep - RHS_ss);      % <-- D todavía depende de th1, px, py, pz
    h = sqrt(A^2 + B^2);
    alpha_aux = atan2(B, A); % Para tener todo en términos de A y B directamente

    % th3_a y th3_b quedan como funciones simbólicas de th1 (además de px,py,pz)
    th3_a =  acos(-D/h) + alpha_aux;
    th3_b = -acos(-D/h) + alpha_aux;

    %% ---- THETA2 (sistema lineal, Cramer) ----
    syms cos_th2 sin_th2 real
    
    % Expandir antes de sustituír
    LHS_th2_full = expand(T_ij_syms(:,:,2)*T_ij_syms(:,:,3)*[Pc3;1]);
    LHS_th2_full(1) = LHS_th2_full(1) - DH(2,1);
    
    eqs_lhs = [LHS_th2_full(1); LHS_th2_full(3)];
    eqs_lhs_mod = subs(eqs_lhs, [cos(th2), sin(th2)], [cos_th2, sin_th2]);
    Amat2 = jacobian(eqs_lhs_mod, [cos_th2, sin_th2]);   % ahora SÍ depende de th3
    
    eqs_rhs = [RHS(1); RHS(3)];
    
    det_A2 = simplify(det(Amat2));
    cos_th2_sol = simplify((eqs_rhs(1)*Amat2(2,2) - eqs_rhs(2)*Amat2(1,2)) / det_A2);
    sin_th2_sol = simplify((Amat2(1,1)*eqs_rhs(2) - Amat2(2,1)*eqs_rhs(1)) / det_A2);
    
    th2_formula = atan2(sin_th2_sol, cos_th2_sol);
    %% ---- THETA4, THETA5, THETA6 (orientación) ----
    R_f1 = T_ij_syms(1:3,1:3,1);
    R_12 = T_ij_syms(1:3,1:3,2);
    R_23 = T_ij_syms(1:3,1:3,3);
    R_f3 = R_f1*R_12*R_23; 

    q_f6  = [qx; qy; qz];
    S_vec = [wx; wy; wz];
    a67_vec = [ux; uy; uz];
    n_vec   = cross(a67_vec, S_vec);

    R_deseado = [n_vec, a67_vec, S_vec];
    Pc_sym = q_f6 - abs(DH(6,3))*S_vec;
    R_36_lhs = simplify(R_f3.' * R_deseado); % Numéricamente conocida

    %% ==================== LOOP POR PUNTO DE TRAYECTORIA ====================
    for k = 1:n
        subs_vars = [qx,qy,qz, wx,wy,wz, ux,uy,uz];
        subs_vals = [Q(k,1),Q(k,2),Q(k,3), S(k,1),S(k,2),S(k,3), A67(k,1),A67(k,2),A67(k,3)];

        Pc_num = double(subs(Pc_sym, subs_vars, subs_vals));
        subs_pos = [px,py,pz]; % Expresiones
        vals_pos = Pc_num(:)'; % números

        th1_vals = [double(subs(th1_a, subs_pos, vals_pos)), ... % rama 1
                    double(subs(th1_b, subs_pos, vals_pos))]; % rama 2

        col = 0;
        for i1 = 1:2
            th1_now = th1_vals(i1);

            % th3 se evalúa acá, ya con th1 conocido
            th3_vals = [double(subs(th3_a, [th1, subs_pos], [th1_now, vals_pos])), ... % codo arriba
                        double(subs(th3_b, [th1, subs_pos], [th1_now, vals_pos]))]; % codo abajo

            for i3 = 1:2 
                col_th5a = col + 1; % Capa de la primera theta5
                col_th5b = col + 2; % Capa de la segunda theta5
                col = col + 2; % va reservando de a 2

                th3_now = th3_vals(i3);

                % --- filtro: theta3 complejo ---
                if abs(imag(th3_now)) > tol % si tiene componente imaginaria
                    SlnSet(k,:,col_th5a) = NaN(1,6);
                    SlnSet(k,:,col_th5b) = NaN(1,6);
                    continue;
                end
                th3_now = real(th3_now); % limpiar

                th2_raw = double(subs(th2_formula, [th1, th3, subs_pos], ... % Reemplaza th2
                            [th1_now, th3_now, vals_pos]));

                % --- filtro: theta2 complejo ---
                if abs(imag(th2_raw)) > tol
                    SlnSet(k,:,col_th5a) = NaN(1,6);
                    SlnSet(k,:,col_th5b) = NaN(1,6);
                    continue;
                end
                th2_num = real(th2_raw);

                R36_num = double(subs(R_36_lhs, [th1, th2, th3, subs_vars], ...
                            [th1_now, th2_num, th3_now, subs_vals]));

                c5 = R36_num(2,3); % coseno de theta 5

                % --- filtro: theta5 fuera de dominio de acos ---
                if abs(imag(c5)) > tol || abs(real(c5)) > 1 + tol % imaginario o inviable (no acotado)
                    SlnSet(k,:,col_th5a) = NaN(1,6);
                    SlnSet(k,:,col_th5b) = NaN(1,6);
                    continue;
                end
                c5 = min(max(real(c5), -1), 1); % clamping para extremos

                th5_vals = [acos(c5), -acos(c5)]; % despeje

                for i5 = 1:2
                    this_col = col - 2 + i5; % para saber si col_th5a o col_th5b
                    s5 = sin(th5_vals(i5));

                    if abs(s5) < tol
                        SlnSet(k,:,this_col) = NaN(1,6);
                        continue;
                    end

                    th6_num = atan2(-R36_num(2,2)/s5,  R36_num(2,1)/s5);
                    th4_num = atan2( R36_num(3,3)/s5, -R36_num(1,3)/s5);

                    SlnSet(k,:,this_col) = [th1_now; th2_num; th3_now; th4_num; th5_vals(i5); th6_num];
                end
            end
        end
    end

    %% ==================== SlnSetVer: filtrado por capa completa ====================
    capas_validas = [];
    for c = 1:8
        if ~any(isnan(SlnSet(:,:,c)), 'all')
            capas_validas(end+1) = c; %#ok<AGROW>
        end
    end
    SlnSetVer = SlnSet(:, :, capas_validas);
end