% Dynamic Matrix Controller without restrictions
% Author: Tarcisio S. S. Dantas, email: tssdantas@gmail.com
% Licenced under GNU GPL v3.0
clc;clear;

%MIMO in Transfer function model of process 
Kp11 = 4; Kp12 = 7; % Kp = static gain
Kp21 = 2; Kp22 = 6;
den11 = [15 1]; den12 = [10 1];  % [Ts 1], Ts = static gain
den21 = [20 1]; den22 = [15 1];

G11 = tf(Kp11,den11);
G12 = tf(Kp12,den12);
G21 = tf(Kp21,den21);
G22 = tf(Kp22,den22);

G_plant = [G11 G12; G21 G22];

%Generating step response model from TF model
[Y_plant,T_plant] = step(G_plant);
n_S = length(T_plant);
%Coefficient matrix of the step response model 
S_plant = Y_plant(2:round(0.70*n_S),:,:); 
%Number of states
n_S = length(S_plant);

% Defining simulation and controller parameters
nu = 2; %  nu - Number of inputs
ny = 2; %  ny - Number of outputs
nx = ny*n_S; % nx - number of states
p = nx; %  p - Prediction horizon
m = 20; %  m - Control horizon
nsim = 1000; %  nsim - Total simulation time
q = [1 1]; %  q  - Output weights (dimension: 1 x ny)
r = [1 1]; %  r -  Input weights (dimension: 1 x nu)
res = 5; % scalar value of restriction for umax,umin 
dures = 2; % scalar value of restriction for umax,umin 
umax = []; %  umax,umin - Max and min values for the inputs (dimension: ny x 1)
umin = []; %  umax,umin - Max and min values for the inputs (dimension: ny x 1)
dumax = [] %  dumax - Max input change (dimension: ny x 1)
for i = 1:nu
    umax = [umax; res];
    umin = [umin; res];
    dumax = [dumax; res];
end
ys = [1 1]; %  ys - Set-points(dimension: ny x 1)
y0 = [0 0]; % Initial output (dimension: ny x 1)
u0 = [0 0]; % Initial input (dimension: nu x 1)
x0 = zeros(nx,1);% Initial state

% runs the controller simulation
[ur, dur, yr, Jkk] = DMCMIMO(p,m,nu,ny,nx,nsim,q,r,S_plant,umax,umin,dumax,u0,y0,ys);

% Ploting results
t = 1:1:length(yr);

figure
subplot(2,2,1)
plot(t,yr(1,:),'b')%,t,ysetplot,'c--')
ylabel('yr1')
xlabel('time')
subplot(2,2,2)
plot(t,ur(1,:),'k',t,dur(1,:),'b')
legend('ur1','dur1')
ylabel('ur1 & dur1')
xlabel('time')
subplot(2,2,3)
plot(t,yr(2,:),'b')%,t,ysetplot,'c--')
ylabel('yr2')
xlabel('time')
subplot(2,2,4)
plot(t,ur(2,:),'k',t,dur(2,:),'b')
legend('ur2','dur2')
ylabel('ur2 & dur2')
xlabel('time')


function [ur, dur, yr, Jkk] = DMCMIMO(p,m,nu,ny,nx,nsim,q,r,S,umax,umin,dumax,u0,y0,ys)

    %  Simulates the closed-loop system with MPC with Quadratic Dynamic matrix
    %  controller, with restrictions on the Inputs 
    
    %  ur,yr  - Input and output responses (dimension: ny x nsim)
    %  p    - Prediction/Optimization horizon
    %  m    - Control horizon
    %  nu   - Number of inputs
    %  ny   - Number of outputs
    %  nx   - Dimension of the state vector
    %  nsim - Simulation time
    %  q  - Output weights (dimension: 1 x ny)
    %  r -  Input weights (dimension: 1 x nu)
    %  umax,umin - Max and min values for the inputs (dimension: ny x 1)
    %  dumax - Max input change (dimension: ny x 1)
    %  ys   - Set-points for the outputs (dimension: ny x 1)
    %  yss  - Steady-state output (dimension: ny x 1)
    %  uss  - Steady-state input  (dimension: nu x 1)

    %  Defining the initial conditions
    xmk=[]; %state
    for i = 1:(p/ny)
        xmk = [xmk; y0'] 
    end
    ymk = xmk; %output
    uk_1 = u0'; %input
    duk_1 = zeros(nu,1); %delta input
    
    ysp=[]; %set-point values along the prediction horizon
    for i=1:(p/ny)
       ysp=[ysp;ys']; 
    end
    
    %Dynamic matrix - Dm
    Dm = [];
    if (m >= 2)
        for i = 1:(p/ny)
            Gi = []; 
            S_tmp = S(i,1:ny,1:nu)
            Gi_permuted = permute(S_tmp,[2 3 1])
            Gi = reshape(Gi_permuted,ny,nu)
            Dm = [Dm; Gi]
        end
        
        for j = 1:m-1
            aux = zeros(ny*nu*j,nu);

            for i = 1:((p/ny)-ny*j)
                S_tmp = S(i,1:ny,1:nu);
                Gi_permuted = permute(S_tmp,[2 3 1]);
                Gi = reshape(Gi_permuted,ny,nu);
                aux = [aux; Gi];
            end

            Dm = [Dm aux];
            
        end
    else
        disp(' m < 2 !');
    end

    %N matrix
    N = [];
    for i = 1:(p/ny)
        N = [N; [eye(ny) zeros(ny,p-ny)] ];
    end
    
    %N1 matrix
    N1 = eye(nu);
    for i = 1:m-1
        N1 = [N1 zeros(nu)];
    end

    % Input and output weigths, R and Q
    aux = [];  
    for i = 1:(p/ny) aux = [aux q]; end
    Qbar = diag(aux);
    aux = [];
    for i = 1:m aux = [aux r]; end
    Rbar = diag(aux);
    
    %  Hessian matrix (for computation of objective function only)
    Hbar = Dm'*Qbar*Dm + Rbar;
    
    % % ----------------------------------------------
    % % Starting simulation
    ur = []; dur = []; yr = [];  Jkk = [];
  
    for in=1:nsim
        % stores variables for ploting 
        dur(:,in) = duk_1(:);
        ur(:,in)=uk_1(:);
        yr(:,in)=ymk(1:ny);
        
        % Setpoint change at 0.33*nsim
        if in >= round(0.33*nsim)
            ysp = [];
            for i=1:(p/ny)
                ysp = [ysp; 0.5*ys']; %cria o vetor com o setpoint
            end
        end

        % deltaU for the unrestricted case 
        % ..estimated for comparison only, its not implemented
        duk = -N1*(Hbar^(-1))*Dm'*Qbar*(N*xmk-ysp); %dimension: nu
        dukk = -1*(Hbar^(-1))*Dm'*Qbar*(N*xmk-ysp); %dimension: nu*m
        
        %Simulation of disturbance at 0.66*nsim
        if in == round(0.66*nsim)
            dukk(1,1) = dukk(1,1)+ 1.2;
        end

        % Updating state & output 
        % for the unrestricted case  
        xmk = N*xmk + Dm*(dukk);
        ymk = xmk;
        
        % Computation of objective function 
        % gradient vector
        ct = Dm'*Qbar*(N*xmk-ysp);
        % QP bias term
        c = (N*xmk-ysp)'*Qbar*(N*xmk-ysp); 
        Jkk(:,in) = dukk'*Hbar*dukk + 2*ct'*dukk + c;

        % storing current input and its delta input....
        % ... for the next iteration
        duk_1 = duk;
        uk_1 = duk + uk_1;
        
    end
    
end

