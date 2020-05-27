%% LIMPIAR TODO
% clear; clc; clf;
% ADQUISICION MAPA Y POSICION ROBOT
clear; clc;
clear all
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected');
    
    [returnCode,Motor1]=vrep.simxGetObjectHandle(clientID,'OmniWheel',vrep.simx_opmode_blocking);
    [returnCode,Motor2]=vrep.simxGetObjectHandle(clientID,'OmniWheel#0',vrep.simx_opmode_blocking);
    [returnCode,Motor3]=vrep.simxGetObjectHandle(clientID,'OmniWheel#1',vrep.simx_opmode_blocking);
    
    [returnCode,camera1]=vrep.simxGetObjectHandle(clientID,'Vision_sensor1',vrep.simx_opmode_blocking);
    [returnCode,camera2]=vrep.simxGetObjectHandle(clientID,'Vision_sensor2',vrep.simx_opmode_blocking);
    [returnCode,ref1]=vrep.simxGetObjectHandle(clientID,'XYZCameraProxy0',vrep.simx_opmode_blocking);
    
    [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_streaming);
    [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_streaming);   
    [returnCode,resolution,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_streaming);
%   [returnCode,resolution,Map]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_streaming);
   
    pause(0.25)
    
    [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_buffer);
    [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
    [returnCode,resolution,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_buffer);
    [returnCode,resolution,Map]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_buffer);

% CONVERSION MAPA A BINARIO
Mapa1=Mapa(1:500,1:500);
Mapa2=false(size(Mapa1));
final=[400 100];
d=25;
m=215;
Mapa2((Mapa1>=m-d)&(Mapa1<=m+d))=true;
    
% CONDICIONES INICIALES PARA EL PLANIFICADOR
%Convertir a BinaryOccupancy
Mapa6 = robotics.BinaryOccupancyGrid(Mapa2,100);
Mapa7 = robotics.BinaryOccupancyGrid(Mapa2,100);
%Inflar Mapa
inflate(Mapa7, 0.35)

% Posicion Actual Robot
x=double(robotpos(1)+2.5);
y=double(robotpos(2)+2.5);
    
% PLANIFICADOR

planner = robotics.PRM(Mapa7,500)

startLocation = [x y];
endLocation = [4 4];
path = findpath(planner,startLocation,endLocation);
    
    
% CONTROLADOR
    posicionx=robotpos(1);
    posiciony=robotpos(2);
    angle=robotori(3);
    posx(1)=posicionx;
    posy(1)=posiciony;
    ab=0;
    %  GRAFICAS


tam=size(path);
    j2=1;
    for k=1:1:tam(1,1)
    ab=ab+1;
    %puntofinal=[53 100 0];
    puntofinal=[path(k,1)-2.5 path(k,2)-2.5 0];
    
    xf=puntofinal(1);
    yf=puntofinal(2);
    thetaf=puntofinal(3);
        [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
        [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_buffer);
        posicionx=robotpos(1);
        posiciony=robotpos(2);
        theta1=robotori(3);
    errorx=xf-posicionx;
    errory=yf-posiciony;
    erroro=thetaf-theta1;
    j=1;
    t(1)=0;

%      || abs(erroro)>=0.001
    rad=sqrt((errorx^2)+(errory^2));
%     abs(errorx)>=0.05 || abs(errory)>=0.05 || abs(erroro)>=0.2
    while rad>=0.05 || abs(erroro)>=0.2
        [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
        [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_buffer);
        posicionx=robotpos(1);
        posiciony=robotpos(2);
        theta1=robotori(3);
        erroro=thetaf-theta1;
        errorx=xf-posicionx;
        errory=yf-posiciony;
        rad=sqrt((errorx^2)+(errory^2));
        Vxr=errorx*10;
        Vyr=errory*10;
        Wr=erroro*5;
        [Vr1,Vr2,Vr3]=cin(Vxr,Vyr,Wr);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor1,Vr1,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor2,Vr2,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor3,Vr3,vrep.simx_opmode_blocking);
        posx(j2)=posicionx+2.5;
        posy(j2)=posiciony+2.5;
        j=j+1;
        j2=j2+1;
        t(j2)=t(j2-1)+0.025;

        Bandera=[rad errorx errory erroro xf yf thetaf posicionx posiciony theta1 tam(1,1) k j Vr1 Vr2 Vr3 Vxr Vyr Wr]
    end
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor2,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor3,0,vrep.simx_opmode_blocking);
    

end
    
    
    
    
    
    
vrep.simxFinish(-1);
end
vrep.delete();

figure
subplot(2,2,1);
show(Mapa6);
subplot(2,2,2), show(Mapa7);
subplot(2,2,3), show(planner);
subplot(2,2,4);
plot(posx,posy);
axis([0 5 0 5]);
grid on;

