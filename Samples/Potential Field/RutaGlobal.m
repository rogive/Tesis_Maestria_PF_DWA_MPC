function [Ruta,iteracion,F_tot1] = RutaGlobal(Obs,PuntoInicial,AnguloInicial,PuntoFinal,...
                        AnguloFinal,delta_i,aten_ini,aten_obs,k_obs,aten_time,tol_rho_goal,aten_delta,...
                        tol_goal,exponente)%,aten_delta,tol_F,tol_goal

%%Planificacion Global de la Ruta       %Puntos = [x y] %Angulos = rad
    q_init = PuntoInicial;      phi_init = AnguloInicial; %q_init repele
    q_goal = PuntoFinal;        phi_goal = AnguloFinal*pi/180; %q_goal atrae
    q = q_init;
    Ruta = [q'; phi_init];
    iteracion = 0;

    vectTOgoal = q - q_goal;
    rho_goal = norm(vectTOgoal);
%     q=[150 100];
    d1=size(Obs);
    for a = 1:d1(1)
    vectTOObs(a,1:2) = q - Obs(a,1:2);
    rho_obs(a) = norm(vectTOObs(a,1:2));
    end
    
    F_1=[0 0 0];
    F_2=[0 0 0];
    F_3=[0 0 0];
    K_1=[0];
    K_2=[0];
    K_3=[0];
    K_4=[0];
    K_5=[0];
    F4=0;
%     h=0;
%     for i = 100:400
%         for j = 100:400
%             if (obstaculos(i,j)==1)
%             h=h+1;
%             Obs(h,1) = i;
%             Obs(h,2) = j;
%             end
%         end
%     end
while rho_goal > tol_rho_goal
    angle_vectTOgoal = atan2(vectTOgoal(2),vectTOgoal(1)) - phi_goal/2 +pi/4;
    %if rho_goal > tol_F_goal, F2 = 0; else F2 = [-sin(2*angle_vectTOgoal) cos(2*angle_vectTOgoal)]; end
    F1 = -vectTOgoal/rho_goal^exponente; %Fuerza att al objetivo con enfoque a la posicion / aumenta con la distancia al target
    F2 = [-sin(2*angle_vectTOgoal) cos(2*angle_vectTOgoal)]; %fuerza atrayente al objetivo con enfoque a la orientacion
    if rho_goal < tol_goal, F2=1.3*F2; F1=F1/1.3; end
    %if rho_goal > tol_F_goal, F2 = 0; else F2 = [-sin(2*angle_vectTOgoal) cos(2*angle_vectTOgoal)]; end
    %%%%%%%%
    vectTOinit = q - q_init; %vectTOgoal = q - q_goal;
    rho_init = norm(vectTOinit); %rho_goal = norm(vectTOgoal);
    angle_vectTOinit = atan2(vectTOinit(2),vectTOinit(1)) ...
                                    - phi_init/2 -pi/4;
    F3 = [-sin(2*angle_vectTOinit) cos(2*angle_vectTOinit)];
    F3 = exp(-aten_ini*rho_init)*F3; %decrece con la distancia
    if rho_init == 0 
        F3 = 0.5*[cos(phi_init) sin(phi_init)];
    end
%   Fuerzas Repulsivas de los obstaculos
%     vectTOobs=[1000 1000];
%     rho_obs=[1000];
%     for z = 1:h
%         q_obs = q - Obs(h,1:2);
%         vectTOobs=[vectTOobs; q_obs];
%         qrho_obs = norm(vectTOobs);
%         rho_obs = [rho_obs;qrho_obs];
%     end
%     for z = 1:d1(1)
%     F4 = -1/(vectTOObs(z,1:2)/((rho_Obs(z,1:2))^exponente));
% %     F4 = exp(-aten_obs*rho_Obs)*F4; %decrece con la distancia
%     end


%     F4 = -1/(vectTOObs(1,1:2)/((rho_obs(1))^exponente));
%     F4 = [1/(vectTOObs(1,1)/((rho_obs(1))^exponente)) +1/(vectTOObs(1,2)/((rho_obs(1))^exponente))];
%     F4 = exp(-aten_obs*rho_Obs)*F4; %decrece con la distancia
%     F4=k_obs*(1/rho_obs(48));
%     F4 = exp(-aten_obs*rho_obs(48))*F4;
    for a=1:d1(1)
        F4 = F4 + exp(-aten_obs*rho_obs(a))*(k_obs*(-1/rho_obs(a)));
    end
%     Datos de las Fuerzas
    K_1 = [K_1; angle_vectTOinit];
    K_2 = [K_2; atan2(vectTOinit(2),vectTOinit(1))];
    K_3 = [K_3; phi_init/2];
    K_4 = [K_4; rho_goal];
    K_5 = [K_5; rho_goal];
    F_1=[F_1;F1 norm(F1)];
    F_2=[F_2;F2 norm(F2)];
    F_3=[F_3;F3 norm(F3)];
    
    %%%%%%%%
    F_tot = (1-exp(-aten_time*iteracion))*(F1+F2*1)+...
             (exp(-aten_time*iteracion))*(F3*1)+F4*0;
% 
%     F_tot = (1-exp(-aten_time*iteracion))*(F1+F2)+...
%              (exp(-aten_time*iteracion))*F3;
    %si me acerco al blanco delta_i se acorta
    if rho_goal < tol_goal,  delta_i = delta_i*aten_delta;  end
    incremento = delta_i*F_tot;
    %reviso que no aumente mucho el ángulo
    aux = -(vectTOgoal + incremento); %vector del goal al punto donde llega el incremento
    if norm(incremento) > rho_goal || (abs(atan2(aux(2),aux(1))-atan2(incremento(2),incremento(1))) > pi/4 && rho_goal<tol_goal)
        incremento = -vectTOgoal;
    end
    q = q + incremento;    
    Ruta = [Ruta [q';atan2(incremento(2),incremento(1))]];
    %la orientación es la orientación del campo
    vectTOgoal = q - q_goal;
    rho_goal = norm(vectTOgoal);
        for a = 1:d1(1)
        vectTOObs(a,1:2) = q - Obs(a,1:2);
        rho_obs(a) = norm(vectTOObs(a,1:2));
        end
    iteracion = iteracion + 1;
    disp(['iteración: ', num2str(iteracion)]);
end
Ruta = [Ruta [q_goal';phi_goal]];
F_tot1=[F_1 F_2 F_3 K_1 K_2 K_3 K_4 K_5];
end

