function [Fig1,imagen,fondo,Obs] = Crear_Mapa(Mapa,Resolucion,Figura)
%%Creación del Mapa3D en que debe moverse el robot - Espacio con obstáculos
figure(Figura); cla; %Poner alfrente la figura y resetear eje 0-1
delete(findobj(imhandles(gca),'-not',{'Color','r','-or','Color','b','-or','Color','m','-or','Color','g','-or','Color','c','-or','Color','y'}))
Mapa1=Mapa;
Mapa2=false(size(Mapa1));
d=10;
m=102;
Mapa2((Mapa1>=m-d)&(Mapa1<=m+d))=true;
% Mapa2(1:500,1:13)=true;Mapa2(1:13,1:500)=true;Mapa2(487:500,1:500)=true;Mapa2(1:500,487:500)=true;
Mapa3 = flipdim(Mapa2,1);

imagen=Mapa3;
%     h=0;
%     for i = 5:495
%         for j = 4:495
%             if (imagen(i,j)==1)
%             h=h+1;
%             Obs(h,1) = i;
%             Obs(h,2) = j;
%             end
%         end
%     end
    
% endpoints=[2 2 498 2; 498 2 498 498; 498 498 2 498; 2 498 2 2;...
%            100 400 300 400; 200 300 200 400; 300 300 300 400; 300 300 400 300; 0 300 50 300; 450 200 498 200;...
%             279 84 327 167; 109 163 132 123; 210 210 211];
% a1=linspace(0,500,11);
Obs = [0 0; 0 50; 0 100; 0 150; 0 200; 0 250; 0 300; 0 350; 0 400; 0 450; 0 500;...
               50 0; 100  0; 150 0; 200 0; 250 0; 300 0; 350 0; 400 0; 450 0; 500 0;...
               50 500; 100 500; 150 500; 200 500; 250 500; 300 500; 350 500; 400 500; 450 500;...
               500 50; 500 100; 500 150; 500 200; 500 250; 500 300; 500 350; 500 400; 500 450; 500 500;...
           100 400; 150 400; 200 400; 250 400; 300 400;200 300; 200 350; 300 300; 300 350; 350 300; 400 300;...
           50 300; 450 200; 279 84; 303 125; 327 167; 109 163; 120 143; 132 123; 210 210];
           
           
Fig1 = gca; set(Fig1,'Parent',Figura); %Crea el nuevo objeto Fig1
fondo = imshow(~imagen,'Parent',Fig1); %para que invierta en blanco y el negro
set(Figura,'Position',get(0,'ScreenSize'),'Name','Mapa 2D','NumberTitle','off'); %Configura la posicion y el tamaño de la ventana, ademas del nombre
set(Fig1,'YDir','normal'), grid on; %Establece el eje Y Hacia arriba (invierte)
xlabel('Eje X'), ylabel('Eje Y'); %Coloca los nombres junto a los ejes
%csvwrite('Obstaculos.csv',imagen);
%La 'imagen' tiene el origen en la esquina superior-izquierda, x es columnas, y es filas
%csvwrite('Imagen_2.csv',obstaculos);
%imagen2 = rgb2gray(imagen); %Convierte la imagen RGB (3 Matrices) a Escala de Grises (1 Matriz)
% Mapa4 = imclearborder(Mapa3);
% Mapa4(1:500,1:13)=true;Mapa4(1:13,1:500)=true;Mapa4(487:500,1:500)=true;Mapa4(1:500,487:500)=true;
% imshow(Mapa4);
% imagen2 = edge(imagen1,'Sobel',0.027); % Sobel 0.027 Encuentra los bordes y los coloca en blanco, el resto en negro Metodos: Canny/Prewitt/_Roberts/Log/Sobel
% %imagen2(2:499,2)=true;imagen2(499,2:499)=true;imagen2(2:499,499)=true;imagen2(2,2:499)=true;
% imagen3 = flipdim(imagen2,1);%Invierte las columnas, ahora el origen esta en la esquina inferior-izquierda
% imagen4 = imfill(imagen3,4,'holes');%Rellena todo lo que tenga bordes cerrados ,'holes'
% imagen5 = imfill(imagen3,[200 200],8);
% imagen6 = imclearborder(imagen5);%Elimina cualquierobjeto que se encuentre con
% imagen = imagen6;
%70-215;
% imshow(Mapa2);
% imshow(Mapa3);
% imshow(Mapa4);
% figure
% subplot(1,2,1); imshow(Mapa1);
% subplot(1,2,2), imshow(Mapa4);
% subplot(2,2,3), imshow(imagen3);
% subplot(2,2,4), imshow(imagen4);
% subplot(2,3,5), imshow(imagen5);
% subplot(2,3,6), imshow(imagen6);
% imshow(imagen1);
end