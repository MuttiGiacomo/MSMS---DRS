% Matlab code for the Course: Modelling and Simulation Mechatronics System (2020)
% by Enrico Bertolazzi and Francesco Biral
% Dipartimento di Ingegneria Industriale
% Università degli Studi di Trento
% email: enrico.bertolazzi@unitn.it

%% Initialize by cleaning memory and widows
clc;
clear all;
close all;

%% Initialize parameters of the model and instantiate model class
xA = 0.172675;
yA = 0.130000;
xD = 0.335000;
yD = 0.154000;
L1 = 0.083446;
L2 = 0.070000;
L3 = 0.057000;
H3 = 0.01082898;
L__wing = 0.120;
d__tip = 0.0100;
m__pist = 0.0800;
m__link = 0.0375;
m__wing = 2.0000;
F__drag = 98.41972500;
F__down = 782.3705250;
s__desired = @(t) 12873.67971*t.^5 - 3218.419926*t.^4 + 214.5613284*t.^3 + 0.023856;
%  ttt = linspace(0,0.1,20);
%  plot(ttt,s__desired(ttt));

g = 9.81;
k = 1000000;
ode     = PodPullEqnsLagr(xA,yA,xD,yD,L1,L2,L3,H3,L__wing,d__tip,m__pist,m__link,m__wing,F__drag,F__down,s__desired,g,k );

%% Initialize the solver class
% names =[Collatz,CrankNicolson,ExplicitEuler,GaussLegendre4,GaussLegendre6,Heun,Heun3,ImplicitEuler,LobattoIIIA,LobattoIIIB,LobattoIIIC,LobattoIIIC_star,Midpoint,RadauIA,RadauIIA,Ralston,Ralston3,Ralston4,RK3,RK3_8,RK4,SSPRK3];
% CrankNicolson
% LobattoIIIC
solver_1 = RK4();
solver_2 = ImplicitEuler();
solver_3 = Collatz();

% LEGEND = {'ExplicitEuler', 'ImplicitEuler', 'Heun'};

solver_1.setODE(ode);
solver_2.setODE(ode);
solver_3.setODE(ode);

%% Select the range and the sampling point for the numerical solution
Tmax = 0.1;
h    = 0.00001;
tt   = 0:h:Tmax;

%% Set consistent initial conditions
s_0 = 0.02385614723;
psi2_0 = -0.3110662763;
psi3_0 = 1.087104656;
u_0 = 0;
v_0 = 0;
w_0 = 0;
lambda1_0 = 329.8028538;
lambda2_0 = -296.501350;
ini = [s_0,psi2_0,psi3_0,u_0,v_0,w_0,lambda1_0,lambda2_0];

%% Compute numerical solution
sol_1 = solver_1.advance( tt,ini,true,true );
sol_2 = solver_2.advance( tt, ini,true,true );
sol_3 = solver_3.advance( tt, ini,true,true );

%% Extract solution
s_1    = sol_1(1,:);
psi2_1    = sol_1(2,:);
psi3_1    = sol_1(3,:);

s_2    = sol_2(1,:);
psi2_2    = sol_2(2,:);
psi3_2    = sol_2(3,:);

s_3    = sol_3(1,:);
psi2_3    = sol_3(2,:);
psi3_3    = sol_3(3,:);


%% Plot the variable s obtained by the different solutions
h = figure();
plot(tt,s__desired(tt),"Color",'g')
hold on
plot( tt(1,1:500:end),s_1(1,1:500:end),  '-o', 'MarkerSize', 6, 'Linewidth', 2,"Color",'#77AC30')
title('Base profile vs Actual profile (RK4)')
xlabel(['Time [s]'])
ylabel(['Piston position [m]'])
legend({'Base Profile','Numerical integration (RK4)'},'FontSize',7,'location','southwest');
% print('./output/RK4','-dpng')

h = figure();
plot(tt,s__desired(tt),"Color",'g')
hold on
plot( tt(1,1:500:end),s_2(1,1:500:end),  '-o', 'MarkerSize', 6, 'Linewidth', 2, "Color",'red')
title('Base profile vs Actual profile (ImplicitEuler)')
xlabel(['Time [s]'])
ylabel(['Piston position [m]'])
legend({'Base Profile','Numerical integration (ImplicitEuler)'},'FontSize',7,'location','southwest');
% print('./output/ImplicitEuler','-dpng')

h = figure();
plot(tt,s__desired(tt),"Color",'g')
hold on
plot( tt(1,1:500:end),s_3(1,1:500:end),  '-o', 'MarkerSize', 6, 'Linewidth', 2,"Color",'blue')
title('Base profile vs Actual profile (Collatz)')
xlabel(['Time [s]'])
ylabel(['Piston position [m]'])
legend({'Base Profile','Numerical integration (Collatz)'},'FontSize',7,'location','southwest');
% print('./output/Collatz','-dpng')

h = figure();
plot( tt(1,1:1:end),s_3(1,1:1:end), '-o', 'MarkerSize', 6, 'Linewidth', 2, "Color", 'blue')
hold on
plot( tt(1,1:1:end),s_2(1,1:1:end), '-', 'MarkerSize', 6, 'Linewidth', 2, "Color", '#77AC30')
plot(tt,k*(s__desired(tt)-s_1))
title('Numerical integrations comparison')
legend({'Collatz','RK4','ImplicitEuler'},'FontSize',7,'location','southeast')
% print('./output/matlabComparison','-dpng')

%% Plot the variable psi2 obtained by the different solutions
% h = figure();
%plot( psi2_1,'-o', 'MarkerSize', 6, 'Linewidth', 2 );
% hold on;
% plot( psi2_2, '-o', 'MarkerSize', 6, 'Linewidth', 2 );
% plot( psi2_3, '-o', 'MarkerSize', 6, 'Linewidth', 2 );
% title(LEGEND);

%% Plot the variable psi3 obtained by the different solutions
% h = figure();
%plot( psi3_1,'-o', 'MarkerSize', 6, 'Linewidth', 2 );
% hold on;
% plot( psi3_2, '-o', 'MarkerSize', 6, 'Linewidth', 2 );
% plot( psi3_3, '-o', 'MarkerSize', 6, 'Linewidth', 2 );
% title(LEGEND);

%% Make animation of the solution
%figure('Position', [1 1 1200 400]);
%hold on;
%ode.plot(tt(2),sol_1(:,2))
% while true
%   for k=1:length(tt)
%     subplot(1,3,1);
%     ode.plot( tt(k), sol_1(:,k));
%     title(LEGEND{1});
% %     subplot(1,3,2);
% %     ode.plot( tt(k), sol_2(:,k));
% %     title(LEGEND{2});
% %     subplot(1,3,3);
% %     ode.plot( tt(k), sol_3(:,k));
% %     title(LEGEND{3});
%     drawnow limitrate;
%     pause(0.03);
%   end
% end