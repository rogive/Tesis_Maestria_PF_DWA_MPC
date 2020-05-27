function [Pos0,Ang0,Resolucion,Mapa] = Condiciones_Iniciales()

    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if (clientID>-1)
    disp('Connected');
    
        [returnCode,ref1]=vrep.simxGetObjectHandle(clientID,'XYZCameraProxy0',vrep.simx_opmode_blocking);
        %[returnCode,camera1]=vrep.simxGetObjectHandle(clientID,'Vision_sensor1',vrep.simx_opmode_blocking);
        [returnCode,camera2]=vrep.simxGetObjectHandle(clientID,'Vision_sensor2',vrep.simx_opmode_blocking);
        
        [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_streaming);
        [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_streaming);
        %[returnCode,Resolucion,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_streaming);
        [returnCode,Resolucion,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_streaming);
        
        pause(0.1)
        
        [returnCode,robotpos]=vrep.simxGetObjectPosition(clientID,ref1,-1,vrep.simx_opmode_buffer);
        [returnCode,robotori]=vrep.simxGetObjectOrientation(clientID,ref1,-1,vrep.simx_opmode_buffer);
        %[returnCode,resolution,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera1,1,vrep.simx_opmode_buffer);
        [returnCode,Resolucion,Mapa]=vrep.simxGetVisionSensorImage2(clientID,camera2,1,vrep.simx_opmode_buffer);
        Pos0=[(robotpos(1)+2.5)*100 (robotpos(2)+2.5)*100];
        Ang0=robotori(3);

    vrep.simxFinish(-1);
    end
    vrep.delete();
end