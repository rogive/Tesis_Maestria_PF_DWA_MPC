[X,Y] = meshgrid(-5:0.1:5);
Kobj=0.08;
Kobs=0.7;
obj=[0 0];
obs=[-2 0; 1 -2; 0 3];
m=size(obs);
Uobj = Kobj*((sqrt((X-obj(1)).^2+(Y-obj(2)).^2))).^2;
% Uobj2 = Kobj*(((X-obj(1)).^2+(Y-obj(2)).^2));
% Uobs=0;
% for k=1:1:m(1,1)
%       Uobs = Kobs*((sqrt((X-obs(k,1)).^2+(Y-obs(k,2)).^2))).^(-1);
% end
Uobs1 = Kobs*((sqrt((X-obs(1,1)).^2+(Y-obs(1,2)).^2))).^(-1);
Uobs2 = Kobs*((sqrt((X-obs(2,1)).^2+(Y-obs(2,2)).^2))).^(-1);
Uobs3 = Kobs*((sqrt((X-obs(3,1)).^2+(Y-obs(3,2)).^2))).^(-1);
Uobs=Uobs1+Uobs2+Uobs3;
Uesp=Uobj+Uobs;
% mesh(X,Y,Uobs);
surf(X,Y,Uesp);

%% Variables

% syms a b c
% d=[a*b*c, b^2, a + c];
% jacobian([a*b*c, b^2, a + c], [a, b, c]);
% gradient(2*a^2 + 3*b + 4*c, [a, b, c]);
% 
% 
% %% Fuerza Activa
% 
% Fact= 
