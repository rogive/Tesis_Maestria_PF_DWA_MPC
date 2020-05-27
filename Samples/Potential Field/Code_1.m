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
%% Compute distance transform

d = bwdist(obstaculos);

% Rescale and transform distances

d2 = (d/100) + 1;

d0 = 2;
d_0= 2;
d_1= d_0/2;
nu = 200*(d_0.^2);

repulsive = nu*((1./(d_1*d2) - 1/d_0).^2);

repulsive (d2 > d0) = 0;   %campo maximo de repulsión

%% Display repulsive potential
figure;
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;

title ('Repulsive Potential');

%% 



%% Compute attractive force
[x, y] = meshgrid (1:smap(1), 1:smap(2));

goal = PuntoFinal;

xi = 1/700;

attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );

figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;

title ('Attractive Potential');

%% Display 2D configuration space

figure;
imshow(~obstaculos);

hold on;
plot (goal(1), goal(2), 'g.', 'MarkerSize', 25);
hold off;

hold on;
plot (Pos0(1), Pos0(2), 'r.', 'MarkerSize', 25);
hold off;

axis ([0 smap(2) 0 smap(1)]);
axis xy;
axis on;

xlabel ('x');
ylabel ('y');

title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;

figure;
m = mesh (f);
m.FaceLighting = 'phong';
axis equal;

title ('Total Potential');

%% Plan route
% start = [Pos0(1), Pos0(2)];
start = [round(Pos0(1)), round(Pos0(2))];

route = GradientBasedPlanner (f, start, goal, 1000);

%% Plot the energy surface

figure;
m = mesh (f);
axis equal;

%% Plot ball sliding down hill




[sx, sy, sz] = sphere(10);

scale = 10;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
p = mesh(sx, sy, sz);
p.FaceColor = 'red';
p.EdgeColor = 'none';
p.FaceLighting = 'phong';
hold off;


%Revisar error Index exceeds the number of array elements (1).

for i = 1:size(route,1)
    P = round(route(i,:));
    z = f(P(2), P(1));
    
    p.XData = sx + P(1);
    p.YData = sy + P(2);
    p.ZData = sz + f(P(2), P(1));
    
    drawnow;
    
    drawnow;
    
end

%% quiver plot
[gx, gy] = gradient (-f);
skip = 20;

figure;

xidx = 1:skip:smap(2);
yidx = 1:skip:smap(1);

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 smap(2) 1 smap(1)]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);

