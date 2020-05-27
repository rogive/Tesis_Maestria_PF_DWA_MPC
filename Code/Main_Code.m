%% CONDICIONES INICIALES
clear; clc; clear all; %clf;
L = 0.29; %L Distancia entre las ruedas
r = 0.05; %r Radio de las ruedas > 1
[Pos0,Ang0,Resolucion,Mapa] = Condiciones_Iniciales(); %Requiere iniciar el entorno en V-Rep/CoppeliaSim
PuntoFinal = [400 450]; %[x y] %[550 250];
AnguloFinal = 120; %rad %0
%% CREAR MAPA
Figura = figure(1);
[Fig1,obstaculos,fondo,Obs] = Crear_Mapa(Mapa,Resolucion,Figura);
smap=size(obstaculos);

%% Creación gráfica del modelo del robot móvil diferencial
plataforma = hgtransform('parent',Fig1);
grosor = 2;
%[handle,handle] = CrearRobot(...
[line_plataforma,quiver_orientacion,quiver_eje] = Crear_Robot(smap(1,1),L,grosor,plataforma,Figura);
lastTRob = makehgtform('translate',[Pos0 0])*makehgtform('zrotate',Ang0);
set(plataforma,'Matrix',lastTRob);
%% Planificacion Global de la Ruta
PuntoInicial=Pos0;
AnguloInicial=Ang0;
figure(Figura); delete(findobj('color','green'))
delta_i = 0.6; %0.4
aten_delta = 0.85; %0.99 cuanto se acortan los pasos al llegar a la meta
aten_time = 5/1500; %5/cuantarde aparece F_goal %max([min(size(obstaculos)) norm(PuntoFinal-PuntoInicial)])
tol_goal = 50;%cuanlejos empieza a disminuir delta
tol_rho_goal = 0.001; %cuando salgo del ciclo while
aten_ini = 0*5/750;%5/cuanlejos desaparece F_init
aten_obs = 5/200;%5/cuanlejos desaparece F_init    0*5/750
k_obs = 200;
exponente = 0.95; %0.99
[Ruta,iteracion,F_tot1] = RutaGlobal(Obs,PuntoInicial,AnguloInicial,PuntoFinal,...
                        AnguloFinal,delta_i,aten_ini,aten_obs,k_obs,aten_time,tol_rho_goal,aten_delta,tol_goal,exponente);
figure(Figura); hold on;
plot([PuntoInicial(1) PuntoFinal(1)],[PuntoInicial(2) PuntoFinal(2)],'*g',...
                            'LineWidth',1.4,'MarkerSize',6,'parent',Fig1)
plot(Ruta(1,:),Ruta(2,:),'g','LineWidth',1.2,'parent',Fig1)

%% Creación de la grilla de trayectoria global
% factor = 3.5;
% grilla = CrearGrilla(Ruta,size(obstaculos),L,factor);
