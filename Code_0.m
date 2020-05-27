%% ADQUISICION MAPA Y POSICION ROBOT
clear; clc;
clear all
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected');
    
    [returnCode,camera1]=vrep.simxGetObjectHandle(clientID,'Vision_sensor1',vrep.simx_opmode_blocking);
    [returnCode,camera2]=vrep.simxGetObjectHandle(clientID,'Vision_sensor2',vrep.simx_opmode_blocking);
    [returnCode,ref1]=vrep.simxGetObjectHandle(clientID,'XYZCameraProxy0',vrep.simx_opmode_blocking);
    
    [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_streaming);
    [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_streaming);   
    [returnCode,resolution,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_streaming);
    [returnCode,resolution,Map]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_streaming);
   
    pause(0.25)
    
    [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_buffer);
    [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
    [returnCode,resolution,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_buffer);
    [returnCode,resolution,Map]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_buffer);


    
vrep.simxFinish(-1);
end
vrep.delete();

%% CONVERSION MAPA A BINARIO
Mapa1=Mapa(1:500,1:500);
Mapa2=false(size(Mapa1));
final=[400 100];
d=25;
m=215;
Mapa2((Mapa1>=m-d)&(Mapa1<=m+d))=true;
%% CONDICIONES INICIALES PARA EL PLANIFICADOR
%Convertir a BinaryOccupancy
Mapa6 = robotics.BinaryOccupancyGrid(Mapa2,100);
Mapa7 = robotics.BinaryOccupancyGrid(Mapa2,100);
%Inflar Mapa
inflate(Mapa7, 0.28)

% Posicion Actual Robot
x=double(robotpos(1)+2.5);
y=double(2.5+robotpos(2));


%% PLANIFICADOR

planner = robotics.PRM(Mapa7,500)

startLocation = [x y];
endLocation = [4 4];
path = findpath(planner,startLocation,endLocation);

%% CONTROLADOR

    posicionx=robotpos(1);
    posiciony=robotpos(2);
    posx(1)=posicionx;
    posy(1)=posiciony;
    
    puntofinal=[0 0 0];
    xf=puntofinal(1);
    yf=puntofinal(2);
    thetaf=puntofinal(3);
    j=1;
    t(1)=0;
    while abs(errorx)>=0.001 || abs(errory)>=0.001 || abs(erroro)>=0.001
        [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
        [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,Ref1,-1,vrep.simx_opmode_buffer);
        posicionx=robotpos(1);
        posiciony=robotpos(2);
        theta1=robotori(3);
        erroro=thetaf-theta1
        errorx=xf-posicionx;
        errory=yf-posiciony;
        Vxr=errorx*10;
        Vyr=errory*10;
        Wr=erroro*10;
        [Vr1,Vr2,Vr3]=cin(Vxr,Vyr,Wr);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor1,Vr1,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor2,Vr2,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor3,Vr3,vrep.simx_opmode_blocking);
        j=j+1;
        t(j)=t(j-1)+0.025;
        posx(j)=posicionx;
        posy(j)=posiciony;
    end
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor2,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Motor3,0,vrep.simx_opmode_blocking);
    



%%  GRAFICAS

figure
subplot(2,2,1), imshow(Mapa1)
subplot(2,2,2), show(Mapa6)
subplot(2,2,3), show(Mapa7)
subplot(2,2,4), show(planner)


