function [line_plataforma,quiver_orientacion,quiver_eje] = Crear_Robot(sm,L,grosor,plataforma,Figura)%plataforma es un htransform
%%Creación gráfica del modelo del robot móvil diferencial
figure(Figura);
L=L*sm*0.38;
delete(findobj('Color','r','-or','Color','b'));
theta = -pi:0.04:pi;
x = 0.5*L*cos(theta);   y = 0.5*L*sin(theta);
line_plataforma = line(x,y,'color','b','LineWidth',grosor,'Parent',plataforma);
hold on;
quiver_orientacion = quiver(0,0,1,0);
set(quiver_orientacion,'color','r','Parent',plataforma,'LineWidth',grosor,'MaxHeadSize',L);
set(quiver_orientacion,'AutoScaleFactor',0.5*L);
quiver_eje = quiver([0 0],[0 0],[0 0],[1 -1]);
set(quiver_eje,'color','b','Parent',plataforma,'ShowArrowHead','off');
set(quiver_eje,'AutoScaleFactor',0.5*L);
end