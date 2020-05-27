clc; clear all;
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected');
    %Asignar los objetos a las variables
    [returnCode,Motor1]=vrep.simxGetObjectHandle(clientID,'OmniWheel',vrep.simx_opmode_blocking);
    [returnCode,Motor2]=vrep.simxGetObjectHandle(clientID,'OmniWheel#0',vrep.simx_opmode_blocking);
    [returnCode,Motor3]=vrep.simxGetObjectHandle(clientID,'OmniWheel#1',vrep.simx_opmode_blocking);
    [returnCode,Ref1]=vrep.simxGetObjectHandle(clientID,'XYZCameraProxy0',vrep.simx_opmode_blocking);
    
    [returnCode,angulos]=vrep.simxGetObjectOrientation(clientID,Ref1,-1,vrep.simx_opmode_streaming);
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,Ref1,-1,vrep.simx_opmode_streaming);
    
   
    tf=3;
    h=0.025;      
    t=h:h:tf;
   %Subrutina orientacion
    Vxr=0;
    Vyr=0;
    for i=1:2
        [returnCode,angulos]=vrep.simxGetObjectOrientation(clientID,Ref1,-1,vrep.simx_opmode_buffer);
        angle(i)=angulos(3);
        disp(angulos(3))
        theta1=angulos(3)
        
        %[theta,Wr,ac] = lspb(theta1, 0, t);
        pause(h)
    end
    disp(theta1)
    erroro=0-theta1;
    [returnCode,angulos]=vrep.simxGetObjectOrientation(clientID,Ref1,-1,vrep.simx_opmode_buffer);
    disp(angulos(3))
     %Subrutina Posicion
   puntofinal=[0 0];
   xf=puntofinal(1);
   yf=puntofinal(2);
   
   [returnCode,position]=vrep.simxGetObjectPosition(clientID,Ref1,-1,vrep.simx_opmode_buffer);
   pause(h);
   [returnCode,position]=vrep.simxGetObjectPosition(clientID,Ref1,-1,vrep.simx_opmode_buffer);
    posicionx=position(1);
    posiciony=position(2);
    errorx=xf-posicionx;
    errory=yf-posiciony;
    t(1)=0;
    j=1;
    posx(1)=posicionx;
    posy(1)=posiciony;
    
    while abs(errorx)>=0.001 || abs(errory)>=0.001 || abs(erroro)>=0.001
        [returnCode,angulos]=vrep.simxGetObjectOrientation(clientID,Ref1,-1,vrep.simx_opmode_buffer);
        theta1=angulos(3);
        erroro=0-theta1
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Ref1,-1,vrep.simx_opmode_buffer);
        posicionx=position(1);
        posiciony=position(2);
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
    
    plot(posx,posy)
    axis([-2 2 -2 2])
    grid on
 

 vrep.simxFinish(-1);
 end
 vrep.delete();