clc
clear all
% -----------------------------------------------------------------------------------------------
% Simulation of a Infinite Horizon Model Predictive Controller (IHMPC) with stability guarantees. 
% Author: T. S. S. Dantas (tssdantas@gmail.com)
%
% Based on "Problem P1" in the paper: Odloak, D., 2004. Extended robust model predictive control. AIChE Journal, 50(8), pp.1824-1836.
% https://aiche.onlinelibrary.wiley.com/doi/full/10.1002/aic.10175?casa_token=wXzw8rgrQJ8AAAAA%3AcS-uWqK4_OpxudIuCFL09zSrI3xZFD-lS5rrJgN5VGrEoKQONnQdz6zbHJ-J6t-sVHOVfqVdHvE180h_
% ----------------------------------------------------------------------------------------------

% A generic MIMO Tranfer Function Model
nu=3;% Number of manipulated inputs
ny=3;% Number of controlled outputs
nd = nu*1*ny;

% Some gains are zero because there is no dynamics associated with
% respective pair on inputs and outputs
numl1 = [5 8 0];  % [Kp1 Kp2 Kp3]
numl2 = [0 3.3 7];
numl3 = [6 0 20];
den11 = [10 1]; den12 = [8 1]; den13 = [1 1];
den21 = [1 1]; den22 = [15 1]; den23 = [10 1];
den31 = [15 1]; den32 = [1 1]; den33 = [20 1];

%-------------------------------------------------------------------------
% Augmented Space-State model

% Exponential of the roots (r_1(1:nu),...,r_ny(1:nu)) for marix F
r1 = [exp(roots(den11)) exp(roots(den12)) 0];
r2 = [0 exp(roots(den22)) exp(roots(den23))];
r3 = [exp(roots(den31)) 0 exp(roots(den33))];
Phi = ones(1,nu);

% Do, Dd, F, N e Psi matrices Ref: Odloak, 2004, Eq. (3-4)
Do = [numl1; numl2; numl3];
Dd = diag([-numl1 -numl2 -numl3]);
F = diag([r1 r2 r3]);
psi = [Phi zeros(1,nu) zeros(1,nu);
       zeros(1,nu) Phi zeros(1,nu);
       zeros(1,nu) zeros(1,nu) Phi];
N = [eye(nu); eye(nu); eye(nu)];

% Augmented state matrices A,B e C Ref: Odloak, 2004, Eq. (3-4)
A = [eye(ny) zeros(ny,nd);
     zeros(nd,ny) F];
B = [Do; Dd*F*N];
C = [eye(ny) psi];
Ap = A;
Bp = B;
Cp = C;

% Input Restrictions
umax=[.5 .5 .1]';  %input restriction 
umin=[-.5 -.5 -.1]';
dumax=[.05 .05 .01]'; %delta_input restriction

%-------------------------------------------------------------
% Simulation parameters, initial conditions, setpoint
% Matrices for the extended IHMPC optimization problem

%Simulation
T=1;% Sampling time (min)
m=3; %input horizon
nsim=300; 

% Initial conditions
xs0 = ones(ny,1);
xd0 = ones(nd,1);
x0 = [xs0;xd0];
u0 = [0 0 0]';
y0 = C*x0;
duk0 = [0 0 0]';

%set point
ysp = [1 1 1]';

%Weigth Matrices (for DeltaU and Slack), Eq. (6), Odloak (2004) (Funcao Objetivo)
R = diag([1 1 1]);
S = diag([10 10 10]);

%Terminal weigth matrix Qbar,Odloak, 2004, Eq. (10)
Q = diag([1 1 1]);
Qaux1 = F'*psi'*Q*psi*F;
Qbar = dlyap(F',Qaux1); %

%matrix Ibar, Ref: Odloak, 2004, Eq. (15)
Ibar = [];
for i = 1:m
    Ibar = [Ibar;eye(ny)];
end

%matrix Dmo,  Ref: Odloak, 2004, Eq. (15)
aux = [];
for i = 1:m;
    aux = [aux; Do];
end
Dmo = [aux];
for i = 1:m-1;
    aux = [zeros(ny);aux(1:(m-1)*ny,:)];
    Dmo = [Dmo aux];
end

%matrix Fx,  Ref: Odloak, 2004, Eq. (16)
Fx = [];
for i = 1:m
    Fx = [Fx;F^m];
end

%matrix Fu, Ref: Odloak, 2004, Eq. (16)
aux = [eye(nd)];
for i = 1:m-1;
    aux = [aux; F^m];
end
Fu_1 = [aux];
for i = 1:m-1;
    aux = [zeros(nd);aux(1:(m-1)*nd,:)];
    Fu_1 = [Fu_1 aux];
end

aux = Dd*F*N;
sz_aux = size(aux);
for i = 1:m-1;
    aux = [aux; zeros(sz_aux)];
end
Fu_2 = [aux];
for i = 1:m-1;
    aux = [zeros(sz_aux);aux(1:(m-1)*nd,:)];
    Fu_2 = [Fu_2 aux];
end

Fu = Fu_1*Fu_2;

%matrix Q1, Ref: Odloak, 2004, Eq. (18-20)
aux = [Q];
for i = 1:m-1;
    aux = [aux; zeros(ny)];
end
Q1 = [aux];
for i = 1:m-1;
    aux = [zeros(ny);aux(1:(m-1)*ny,:)];
    Q1 = [Q1 aux];
end
  
%matrix Q2, Ref: Odloak, 2004, Eq. (18-20)
aux = zeros((m-1)*nd,m*nd);
Q2 = [aux; zeros(nd,(m-1)*nd) Qbar];

%matrix R1, Ref: Odloak, 2004, Eq. (18)
aux = [R];
for i = 1:m-1;
    aux = [aux; zeros(nu)];
end
R1 = [aux];
for i = 1:m-1;
    aux = [zeros(nu);aux(1:(m-1)*nu,:)];
    R1 = [R1 aux];
end
  
%matrix Psi1, Ref: Odloak, 2004, close to Eq. 20
aux = [psi];
for i = 1:m-1;
    aux = [aux; zeros(ny,nd)];
end
psi1 = [aux];
for i = 1:m-1;
    aux = [zeros(ny,nd);aux(1:(m-1)*ny,:)];
    psi1 = [psi1 aux];
end

%matrix Dotil, Ref: Odloak, 2004, Eq. (21)
Dotil = [];
for i = 1:m
    Dotil = [Dotil Do];
end

%matrix H,  Ref: Odloak, 2004, Eq. (18)
H = [((Dmo + psi1*Fu)'*Q1*(Dmo + psi1*Fu) + Fu'*Q2*Fu + R1), -(Dmo + psi1*Fu)'*Q1*Ibar;
     -Ibar'*Q1*(Dmo + psi1*Fu), S + Ibar'*Q1*Ibar + Q];

% Matrices Mtil and Itil. 
% Used to convert input from velocity form (deltaU(k)) to positional form (U(k))
% Ref:  Odloak. Class Notes p. 33, (Remark for Eq. VII.5)
%       Odloak, 2004, Eq. (23) (Implicit)
Mt=[];Itil=[];
for in=1:m
    Mt=[Mt;eye(nu)];
    Itil=[Itil;eye(nu)];
end
Mtil=Mt;
for in=1:m-1
    Mt=[zeros(nu);Mt(1:nu*(m-1),:)];
    Mtil=[Mtil,Mt];
end

%Auxiliary constrain matrices for dynamic implementation
Dumax=dumax;
Umax=umax;
Umin=umin;
for i=1:m-1;
    Umax=[Umax;umax];
    Umin=[Umin;umin];
    Dumax=[Dumax;dumax];
end

slack_inf_upperbound = [];
slack_inf_lowerbound = [];
for i = 1:(ny)
    slack_inf_upperbound = [slack_inf_upperbound; +Inf];
    slack_inf_lowerbound = [slack_inf_lowerbound; -Inf];
end

%  Defining the initial conditions
xmk=x0;
ymk=y0;
xpk=x0;
ypk=y0;
uk_1=u0;
duk_1 = u0;

dukk = [];
for i=1:m;
    dukk=[dukk;duk0];
end

%Incluing slacks
slack = (xs0 - ysp) + Dotil*dukk; %Ref: Odloak, 2004, Eq. (21)
dukk = [dukk; slack];

%State Observer
Kf = KalmanFilter(ny,A,C,300)

% Starting simulation
for in=1:nsim;
    in
    ur(:,in) = uk_1(:);
    dur(:,in) = duk_1(:);
    yr(:,in) = ypk(1:ny);
    slackr(:,in) = dukk((m*nu+1):((m*nu)+nu));
    
    xsmk = xmk(1:ny);
    xdmk = xmk(ny+1:ny+nd);
    es = xsmk - ysp;
    e = ymk - ysp;
        
    rest(:,in) = es + Dotil*dukk(1:m*nu,1) - slack;
    
    % -------
    % Ref: Odloak, 2004, Eq. (19)
    cf = [(Dmo + psi1*Fu)'*Q1*(Ibar*es + psi1*Fx*xdmk) + Fu'*Q2*(Fx*xdmk);
                    -Ibar'*Q1*(Ibar*es + psi1*Fx*xdmk) - Q*e];

    % Ref: Odloak, 2004, Eq. (20)
    c = e'*Q*e+(Ibar*es + psi1*Fx*xdmk)'*Q1*(Ibar*es + psi1*Fx*xdmk) + (Fx*xdmk)'*Q2*(Fx*xdmk);
    % -------
    
    % Objective function output value
    Jk = dukk'*H*dukk + 2*cf'*dukk + c;
    Jkr(:,in) = Jk;  

    %Inequality constraints Ref: Odloak, 2004, Eq. (23)
    A1 = [Mtil;-Mtil];
    b1 = [Umax-Itil*uk_1;-Umin+Itil*uk_1];
    
    Ac = [A1 zeros(2*m*nu,nu)];
    bc = b1;

    %Equality constraint  Ref: Odloak, 2004, Eq. (21)
    Aeq = [-Dotil, eye(ny)];
    beq = es;

    % Solving the optimization problem with QP
    dukk = quadprog(H,cf,Ac,bc,Aeq,beq,[-Dumax; slack_inf_lowerbound],[Dumax; slack_inf_upperbound ]);
    %sizedukk = size(dukk)
    duk = dukk(1:nu,1);
    slack = dukk((m*nu+1):((m*nu)+nu));

    %Predicting state and output vectors
    xmk = A*xmk+B*duk;
    ymk = C*xmk;
    
    % Disturbance
    d=0;
    if in == 150
      d=.1;
    else
      d = 0;
    end
    
    xpk=Ap*xpk+Bp*(duk+[0 0 d]');
    ypk=Cp*xpk;
    
    % 1. For the case with no state observer
        xmk = xpk;
        ymk = ypk;
   
    % 2. Using state observer to correct the last measurement
%     de=ypk-ymk;
%     xmk=xmk+Kf*(de);
%     ymk=C*xmk; % New estimation of ym(k)
    
    uk_1 = uk_1+duk;
    duk_1 = duk;
    
    %Checking if slack equality constrain is statisfied 
    aux = xsmk - ysp + Dotil*dukk(1:m*nu,1) - slack;
    prova(:,in) = aux;
 
end

%Plotting results
figure(1)
subplot(2,3,1),hold on,plot(yr(1,:)),plot([0 nsim],[ysp(1) ysp(1)],'r'),ylabel('y1')
subplot(2,3,2),hold on,plot(yr(2,:)),plot([0 nsim],[ysp(2) ysp(2)],'r'),ylabel('y2')
subplot(2,3,3),hold on,plot(yr(3,:)),plot([0 nsim],[ysp(3) ysp(3)],'r'),ylabel('y3')
subplot(2,3,4),hold on,plot(dur(1,:)),ylabel('u1')
subplot(2,3,5),hold on,plot(dur(2,:)),ylabel('u2')
subplot(2,3,6),hold on,plot(dur(3,:)),ylabel('u3')

figure(2)
ylabel('Jk'),plot(Jkr(1,:)),ylabel('Jk')

function [Kalman] = KalmanFilter(ny,A,C,it)

    V=.5;
    W=.5;
    sM=size(A);
    PP=eye(sM(1));
    VV=eye(ny)*V; 
    WW=eye(sM(1))*W;
    for j=1:it;
        PP = A*PP*A'-A*PP*C'*inv(VV+C*PP*C')*C*PP*A'+ WW;
    end
    Kalman = A*PP*C'*inv(VV+C*PP*C');

end