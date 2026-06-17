Pac = HoosierR20_2024();
% 
Fz = movmean([Fz_fl,Fz_fr,Fz_rl,Fz_rr],50).';
kap = movmean(SR,50)';
alp=movmean(SA,50).';
camb=0*Fz;
[Fx,Fy,Mz,Fxmax,Fymax] = TireForcePac2002(Fz,kap,alp,camb,Pac);

%%
% 1. Inicializar el modelo con tus parámetros
Pac = HoosierR20_2024();

% 2. Definir los rangos de barrido (Ejes X e Y de las superficies)
kappa_vec  = linspace(-0.3, 0.3, 100);       % Deslizamiento longitudinal (SR)
alpha_vec  = linspace(-15, 15, 100) * pi/180; % Ángulo de deriva (SA) en radianes
Fz_vec     = linspace(100, 1100, 100);       % Rango de carga vertical (N) desde ligera a máxima

% 3. Configurar condiciones del "otro" eje para fricción combinada/pura
% Modifica estos valores para ver el efecto del deslizamiento combinado
alpha_fijo = 0 * pi/180;  % SA fijo (0 deg = Tracción pura para Fx)
kappa_fijo = 0;           % SR fijo (0 = Viraje puro para Fy)
camb_virtual = 0;         % Caída constante (rad)

% 4. Crear las mallas bidimensionales (Grid: Slip vs Fz)
[KAPPA_MESH, FZ_MESH_X] = meshgrid(kappa_vec, Fz_vec);
[ALPHA_MESH, FZ_MESH_Y] = meshgrid(alpha_vec, Fz_vec);

% 5. Preasignar matrices para guardar las fuerzas resultantes
Fx_surf = zeros(size(KAPPA_MESH));
Fy_surf = zeros(size(ALPHA_MESH));

% 6. Evaluar el modelo Pacejka recorriendo las mallas
% Barrido para Superficie Fx (Longitudinal)
for i = 1:size(KAPPA_MESH, 1)
    for j = 1:size(KAPPA_MESH, 2)
        [Fx, ~, ~, ~, ~] = TireForcePac2002(...
            FZ_MESH_X(i,j), ...
            KAPPA_MESH(i,j), ...
            alpha_fijo, ...
            camb_virtual, ...
            Pac);
        Fx_surf(i,j) = Fx;
    end
end

% Barrido para Superficie Fy (Lateral)
for i = 1:size(ALPHA_MESH, 1)
    for j = 1:size(ALPHA_MESH, 2)
        [~, Fy, ~, ~, ~] = TireForcePac2002(...
            FZ_MESH_Y(i,j), ...
            kappa_fijo, ...
            ALPHA_MESH(i,j), ...
            camb_virtual, ...
            Pac);
        Fy_surf(i,j) = Fy;
    end
end

% 7. Graficar las superficies resultantes (F vs Slip vs Fz)
% figure('Name', 'Superficies de Neumático: Fuerza vs Slip vs Fz', 'NumberTitle', 'off');

% % Subplot 1: Fx vs Longitudinal Slip vs Fz
% subplot(1, 2, 1);
% surf(KAPPA_MESH, FZ_MESH_X, Fx_surf, 'EdgeColor', 'none');
% colormap jet; colorbar; shading interp;
% title(sprintf('Fuerza Longitudinal $F_x(\\kappa, F_z)$ \\\\ ($\\alpha = %g^\\circ$)', alpha_fijo*180/pi), 'Interpreter', 'latex');
% xlabel('Deslizamiento $\kappa$ (SR)', 'Interpreter', 'latex');
% ylabel('Carga Vertical $F_z$ (N)', 'Interpreter', 'latex');
% zlabel('$F_x$ (N)', 'Interpreter', 'latex');
% view(3); grid on;

% Subplot 2: Fy vs Side Slip Angle vs Fz
% subplot(1, 2, 2);
surf(ALPHA_MESH, FZ_MESH_Y, Fy_surf);
colorbar;
title(sprintf('Fuerza Lateral $F_y(\\alpha, F_z)$ \\\\ ($\\kappa = %g$)', kappa_fijo), 'Interpreter', 'latex');
xlabel('Ángulo de deriva $\alpha$ (deg)', 'Interpreter', 'latex');
ylabel('Carga Vertical $F_z$ (N)', 'Interpreter', 'latex');
zlabel('$F_y$ (N)', 'Interpreter', 'latex');
view(3); grid on;
alpha(0.5)