function [ur,yr]=MPC_SSZONE(p,m,nu,ny,nx,nsim,q,r,A,B,C,Ap,Bp,Cp,umax,umin,dumax,u0,y0,ys_max, ys_min)
%  Simulates the closed-loop system with MPC with state-space model in the
%  positional form
%  ur,yr  - Input and output responses (dimension: ny x nsim)
%  p    - Optimization horizon
%  m    - Control horizon
%  nu   - Number of inputs
%  ny   - Number of outputs
%  nx   - Dimension of the state vector
%  nsim - Simulation time
%  q  - Output weights (dimension: 1 x ny)
%  r -  Input weights (dimension: 1 x nu)
%  A,B,C - State, input and output matrices of the state-space model used in the MPC controller
%  Ap,Bp,Cp - State, input and output matrices of the state-space model used to represent the true plant
%  umax,umin - Max and min values for the inputs (dimension: ny x 1)
%  dumax - Max input change (dimension: ny x 1)
%  ys_max - setpoint maximum value/bound (Zone upperbound)
%  ys_min - setpoint minimum value/bound (Zone upperbound)
%  //ys   - Set-points for the outputs (dimension: ny x 1)
%  //yss  - Steady-state output (dimension: ny x 1)
%  uss  - Steady-state input  (dimension: nu x 1)

%  Defining the initial conditions
xmk=zeros(nx,1);  %cria a matriz de estados com todos os elementos igula a zero
% xmk=ones(nx,1);
ymk=C*xmk;  
xpk=zeros(nx,1);
ypk=Cp*xpk;
ysp=[];

% for i=1:p;
%   ysp=[ysp;ys]; %cria o vetor com o setpoint
% end

uk_1=u0;

Phi=[];ThA=[];
for in=1:p;
    Phi=[Phi;C*A^in]; % "Psi"
    ThA=[ThA;C*A^(in-1)*B]; % "Theta"
end

sizePhi = size(Phi) %Phi(p*ny, nr_states)


% Creating the Dynamic Matrix 
% As colunas da matriz Phi_lit sao identicas ao caso posicional.
a=ThA;
Dm=[a];
if m >= 2
    for iu=1:m-2;
        a=[zeros(ny,nu);a(1:(p-1)*ny,:)];
        Dm=[Dm a];
    end
    b=C*B;
    Ai=eye(nx);
    for in=1:p-m;
        Ai=Ai+A^in;
        b=[b;C*Ai*B];
    end
    Theta=[Dm [zeros(ny*(m-1),nu);b]];
end
if m==1
    b=C*B;
    Ai=eye(nx);
    for in=1:p-m;
        Ai=Ai+A^in;
        b=[b;C*Ai*B];
    end
    Theta=b;
end

sizeTheta = size(Theta); %Theta(p*ny, m*nu)

%Matrices Qbar and Rbar
aux=[];
for in=1:p;
  aux=[aux q];
end
Qbar=diag(aux); %Qbar(diag(p*ny))

clear aux; aux=[];
for in=1:m;
  aux=[aux r];
end
Rbar=diag(aux); %Rbar(m*nu)
% ----- Qbar,Rbar
%Criando as variaveis M_til e I_til
I_til = [];M_til = [];
for im = 1:m
    I_til = [I_til; eye(nu)];
    M_til = [M_til; eye(nu)]; %primeiras nu colunas de M_til
end
M_col = M_til;
M_aux = []; %expandido as colunas da matriz M_til ate dimensao (m*nu)x(m*nu)
if m >= 2
    for j = 1:m-1;
        M_col = [zeros(j*nu,nu)];
        for i = (j+1):m
            M_col = [M_col; eye(nu)];
        end
        M_aux = [M_aux M_col];
    end
end
M_til = [M_til M_aux];

if m <= 2
    error('m muito pequeno')
end

%M_til = zeros(m*nu,m*nu);

%Ibar = eye(p*ny);
Ibar = [];
for i = 1:p 
    Ibar = [Ibar;eye(ny)];
end

%Matrix H
H = [Theta'*Qbar*Theta+Rbar, Theta'*Qbar*(-Ibar); -Ibar'*Qbar*Theta, Ibar'*Qbar*Ibar]; %Alterada para a forma nova
%H(m*nu x m*nu, m*nu x ny*p; ny*p x m*nu, ny*p x ny*p)
sizeH = size(H)


% State observer
% Kf=zeros(nx,ny);
Kf = FKalman(ny,A,C,300)

%Auxiliary constraint matrix

Dumax=dumax;
Dumin=-dumax; % nova
Umax=umax; 
Umin=umin;
for i=1:m-1;  %(nu*m)x1
 Umax=[Umax;umax];
 Umin=[Umin;umin];
 Dumax=[Dumax;dumax];
 Dumin=[Dumin;-dumax];
end

% ys_max_nuxm = zeros(m*nu,ny);  ys_min_nuxm = zeros(m*nu,ny);
% for i = 1:m
%     ys_max_nuxm = [ys_max_nuxm; diag(ys_max')]; %usa-se transposta aqui !
%     ys_min_nuxm = [ys_min_nuxm; diag(ys_min')];
% end

ys_max_nuxm = [];  ys_min_nuxm = [];
for i = 1:nu*m
    ys_max_nuxm = [ys_max_nuxm; ys_max']; %usa-se transposta aqui !
    ys_min_nuxm = [ys_min_nuxm; ys_min'];
end

sizeys_max_nuxm = size(ys_max_nuxm)

%  Defining the initial conditions
x0=zeros(nx,1) ;
ymk=y0;
xpk=x0;
ypk=y0;
uk_1=u0;

%----------------------------------------------
% Starting simulation
for in=1:nsim;
    
  ur(:,in)=uk_1(:);
  yr(:,in)=ymk(1:ny); 
  
  
%   sizePhixmkT = size((Phi*xmk)')
%   sizeQbar = size(Qbar)
%   sizeTheta = size(Theta)
%   sizeIbar = size(-Ibar)
  
  sizect1 = size(Theta'*Qbar*(Phi*xmk))
  sizect2 = size((-Ibar)'*Qbar*(Phi*xmk)) %elements in f = 20 + 3

  %ct(1,nu*m)
  ct = [Theta'*Qbar*(Phi*xmk); (-Ibar)'*Qbar*(Phi*xmk)];
  sizect = size(ct)

%Including constraints on the input changes

%     %Including constraints on the input changes
%     A1 = [Mtil;-Mtil];
%     b1 = [Umax-Itil*uk_1;-Umin+Itil*uk_1];
% 
%     Ac = [A1 zeros(2*m*nu,nu)];
%     bc = b1;
%
%            deltaU  slacks
%     Aeq = [-Dotil,eye(ny)];
%     beq = [es];
 
 %Restricoes em funcao da forma incremental du, Ac*x < bc:

    %Ac = [M_til ys_max_nuxm; -M_til ys_min_nuxm]; %precisa de +3 colunas !!   2x(nu*p + ny)
    %Ac = [M_til eye(nu*m,ny); -M_til eye(nu*m,ny)];
    Ac = [M_til zeros(nu*m,ny); ...
           zeros(ny,m*nu+ny);...
          -M_til eye(nu*m,ny); zeros(ny,m*nu+ny); ];
    %Number of columns in "Ac" must be the same as elements in "cf"
    sizeH = size(H)
    sizeAc = size(Ac)
    
    bc = [Umax - I_til*uk_1; zeros(ny,1); Umax + I_til*uk_1; zeros(ny,1)];
    sizebc = size(bc)

%  size(H),size(ct),size(A),size(b)
  %ukk=quadprog(H,ct,Ac,bc,[],[],Umin,Umax);
  opt = optimoptions(@quadprog,'Display','off');
  dukk=quadprog(H,ct,Ac,bc,[],[],[Dumin; ys_min],[Dumax; ys_max]);
  sizedukk = size(dukk)
  duk = dukk(1:nu,1);
  %ysp = ukk(nu+1:nu+ny)
  
%Correction of the last control input
  %xmk=A*xmk+B*uk;
  xmk=A*xmk+B*duk;
  ymk=C*xmk;
  
  d=0;
  if in == 100
      d = 20;
  else 
      d = 0;
  end
  % xpk=Ap*xpk+Bp*uk;
  xpk=Ap*xpk+Bp*(duk+[0 d]');
  ypk=Cp*xpk;
  
%Correction of the last measurement

  de=ypk-ymk;
  xmk=xmk+Kf*(de);
  ymk=C*xmk; %nova estimacao de ymk
  
  uk_1=duk;
  %uk_1=uk_1 + duk;
  
end










