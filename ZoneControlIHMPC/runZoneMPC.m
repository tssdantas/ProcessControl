
%Criando modelo discreto MIMO em SS
%A = [1 0 0 0 0 0 0;0 0.8981 0 0 0 0 0;0 0 0.8632 0 0 0 0;0 0 0 0.9174 0 0 0;0 0 0 0 0.9057 0 0;0 0 0 0 0 0.8594 0;0 0 0 0 0 0 0.9219];
%B = [2.3 -0.007 0.2; 0.4792 0 0; 0 -0.00019 0; 0 0 0.03305; 0.1791 0 0; 0 0.008537 0; 0 0 0.01566];
%%C = [1 0 0 0 0 0 0;0 1 1 1 0 0 0;0 0 0 0 1 1 1];
%D = zeros(3);
%ssmimod_mao = ss(A,B,C,D,1);
%ssmimod_mao.Inputname = {'steamrate', 'refluxrate', 'feedtemp'};
%ssmimod_mao.Outputname = {'lvltopdrum','temptray68','%flooding'};
%ssmimod_mao.IODelay = [0 0 0;7 2 3;1 3 3];

%------ Transformando um model MIMO em TF para SS
Numalltf = {2.3 0.2;4.7 0.4;1.9 0.2}; %Definindo todos os elementos dos numeradores de G(s) em um cell array
%Denalltf = {[1 0] [1 0];[9.3 1] [11.6 1];[10.1 1] [12.3 1]}; %Definindo os denominadores de G(s) em um cell array
Denalltf = {[1 1] [1 1];[9.3 1] [11.6 1];[10.1 1] [12.3 1]}; %Definindo os denominadores de G(s) em um cell array
tfmimo = tf(Numalltf,Denalltf); %Criando a funcao de transferencia MIMO
tfmimo.Inputname = {'steamrate', 'feedtemp'};
tfmimo.Outputname = {'lvltopdrum','temptray68','%flooding'};
%tfmimo.IODelay = [0 0;7 3;1 3]
setmpcsignals(tfmimo, 'MD', 2);
ssmimo_matlab = ss(tfmimo); %Estimando as matrizes A,B,C e D.
ssmimo_minimal = ss(tfmimo,'minimal'); %verificando a forma minima do modelo SS, sem estados nao-observaveis e nao-controlaveis. Sera usada apenas para comparacao com o metodo que sera aplicado
ssmimod = c2d(ssmimo_matlab,1,'zoh')

[mA,nA] = size(ssmimod.A);
[mb,nb] = size(ssmimod.B);

%Calculando as matrizes Aincr e Bincr do modelo SS na forma incremental
Aincr = [ssmimod.A ssmimod.B;zeros(nb,mb) eye(nb)]
Bincr = [ssmimod.B;eye(nb)]
Cincr = [ssmimod.C zeros(3,nb)] %!!!!!!!!!!!! 3 saidas e nb entradas


p = 100; %  p - Optimization horizon
m = 10; %  m    - Control horizon
nu = 2; %  nu   - Number of inputs
ny = 3; %  ny   - Number of outputs
nx = mA+nu; %  outro tipo de forma incremental
nsim = 1000; %  nsim - Simulation time
q = [1 1 1]; %  q  - Output weights (dimension: 1 x ny)
r = [1 1]; %  r -  Input weights (dimension: 1 x nu)
%Ap = A; Bp = B; Cp = C; %igualando planta e o modelo
res = 100; 
dures = 25;
umax = [res; res]; %  umax,umin - Max and min values for the inputs (dimension: ny x 1)
umin = [-res; -res]; %  umax,umin - Max and min values for the inputs (dimension: ny x 1)
dumax = [dures; dures]; %  dumax - Max input change (dimension: ny x 1)

% umax=[1.5 1.0]';  %input restriction 
% umin=[-1.5 -1.0]';
% dumax=[.5 .1]'; %delta_input restriction

ys = [1;1;1]; %  ys   - Set-points for the outputs (dimension: ny x 1)
y0 = [0;0;0];
ys_bound_max = 1.0;
ys_bound_min = -1.0;
ys_max = [ys_bound_max; ys_bound_max; ys_bound_max];
ys_min = [ys_bound_min; ys_bound_min; ys_bound_min];
u0 = [0;0];

[ma,na] = size(ssmimod.A);
[mb,nb] = size(ssmimod.B);

%Construindo o modelo na forma incremental
ssmimod_incr_dt = ss(Aincr,Bincr,Cincr,ssmimod.D);

Ap = Aincr; Bp = Bincr; Cp = Cincr;
[ur2,yr2] = MPC_SSZONE(p,m,nu,ny,nx,nsim,q,r,Aincr,Bincr,Cincr,Ap,Bp,Cp,umax,umin,dumax,u0,y0, ys_max, ys_min);

 t = 0:1:(nsim-1);
 ysetplot = [];
for i = 1:length(t)
    ysetplot = [ysetplot 1];
end

figure
subplot(2,3,1)
plot(t,yr2(1,:),'b',t,ysetplot,'c--')
title(ssmimod.Outputname(1))
ylabel('yr')
xlabel('tempo')
subplot(2,3,4)
plot(t,ur2(1,:),'k')
title(ssmimod.Inputname(1))
ylabel('ur')
xlabel('tempo')
subplot(2,3,2)
plot(t,yr2(2,:),'b',t,ysetplot,'c--')
title(ssmimod.Outputname(2))
ylabel('yr')
xlabel('tempo')
subplot(2,3,5)
plot(t,ur2(2,:),'k')
title(ssmimod.Inputname(2))
ylabel('ur')
xlabel('tempo')
subplot(2,3,3)
plot(t,yr2(3,:),'b',t,ysetplot,'c--')
title(ssmimod.Outputname(3))
ylabel('yr')
xlabel('tempo')
% subplot(2,3,6)
% plot(t,ur2(3,:),'k')
% title(ssmimod_mao.Inputname(3))
% ylabel('ur')
% xlabel('tempo')


