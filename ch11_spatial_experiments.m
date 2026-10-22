%% CHAPTER 11 - Spatial cart-pole control experiments (no GUI)
% State order: z = [x; dx; alpha; dalpha; y; dy; beta; dbeta].
% All angles and rates are in radians and radians/second.
% The nonlinear model is the one derived in Chapter 10.
% Requires base MATLAB (ode45), no Control System Toolbox for the supplied K.
% Optional lqr commands are shown but commented out.
clear; clc; close all;

p.M = 1; p.m = 0.2; p.L = 0.5; p.g = 9.81;
p.lc = p.L/2; p.J = p.m*p.L^2/3;
p.a = p.M + p.m; p.b = p.m*p.lc; p.k = p.m*p.g*p.lc;
D = p.a*p.J - p.b^2;
Ap = [0 1 0 0; 0 0 -p.b*p.k/D 0; 0 0 0 1; 0 0 p.a*p.k/D 0];
bp = [0; p.J/D; 0; -p.b/D];
A = blkdiag(Ap,Ap);
B = [bp zeros(4,1); zeros(4,1) bp];
Q = diag([10 1 100 1 10 1 100 1]);
R = diag([1 1]);
% With Control System Toolbox, obtain the gain from K = lqr(A,B,Q,R).
k = [-3.162277660 -4.276333370 -38.585859200 -6.658354530];
K = [k zeros(1,4); zeros(1,4) k];
% CARE check if Toolbox is available:
% K_check = lqr(A,B,Q,R); assert(norm(K-K_check,inf)<1e-5);

z0 = zeros(8,1); z0(3) = deg2rad(5); z0(7) = deg2rad(3);
tspan = (0:0.01:12)';

cases = {
    'Nominal', K, [25 25], [0 0];
    'Y-force saturated', K, [25 1.5], [0 0];
    'Both forces saturated', K, [1 1], [0 0]
};

for i = 1:size(cases,1)
    name = cases{i,1}; Ki = cases{i,2}; caps = cases{i,3}; pulse = cases{i,4};
    [t,Z,U] = simulatePlant(p,Ki,z0,tspan,caps,pulse);
    results = summarizeTrial(t,Z,U);
    fprintf('%-23s: |x|max=%7.3f m, |y|max=%7.3f m, ', ...
        name,results.xmax,results.ymax);
    fprintf('|Fx|max=%6.3f N, |Fy|max=%6.3f N, integral F^2 dt=%7.3f N^2 s\n', ...
        results.fxmax,results.fymax,results.forceEnergy);
    if i==1
        tNom=t; ZNom=Z; UNom=U;
    end
end

figure('Name','Chapter 11: spatial cart-pole baseline','Color','w');
tiledlayout(2,1,'Padding','compact');
nexttile;
plot(tNom,rad2deg(ZNom(:,3)),'LineWidth',1.8); hold on;
plot(tNom,rad2deg(ZNom(:,7)),'LineWidth',1.8);
yline(0,':'); xlabel('Time (s)'); ylabel('Tilt (deg)');
legend('alpha: X tilt','beta: Y tilt','Location','northeast'); grid on;
nexttile;
plot(tNom,UNom(:,1),'LineWidth',1.8); hold on;
plot(tNom,UNom(:,2),'LineWidth',1.8);
yline(0,':'); xlabel('Time (s)'); ylabel('Actuator force (N)');
legend('F_x','F_y','Location','northeast'); grid on;

% Optional asymmetric LQR study when Control System Toolbox is installed:
% for ry = [1 10 100]
%     Ky = lqr(A,B,Q,diag([1 ry]));
%     [t,Z,U] = simulatePlant(p,Ky,z0,tspan,[25 25],[0 0]);
%     r = summarizeTrial(t,Z,U);
%     fprintf('R_y=%g, |y|max=%.3f m, |Fy|max=%.3f N\n', ...
%         ry,r.ymax,r.fymax);
% end

function [t,Z,U] = simulatePlant(p,K,z0,ts,limits,pulse)
    limits = limits(:);
    fun = @(t,z) rhs(t,z,p,K,limits,pulse);
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
    [t,Z] = ode45(fun,ts,z0,opts);
    U = zeros(numel(t),2);
    for j = 1:numel(t)
        cmd = -K*Z(j,:)';
        U(j,:) = max(-limits,min(limits,cmd))';
    end
end

function dz = rhs(t,z,p,K,limits,pulse)
    al=z(3); da=z(4); be=z(7); db=z(8);
    sa=sin(al); ca=cos(al); sb=sin(be); cb=cos(be);
    a=p.a; b=p.b; J=p.J; k=p.k;
    MM=[a,0,b*ca*cb,-b*sa*sb; ...
        0,a,0,b*cb; ...
        b*ca*cb,0,J*cb^2,0; ...
        -b*sa*sb,b*cb,0,J];
    h=[-b*(sa*cb*(da^2+db^2)+2*ca*sb*da*db); ...
       -b*sb*db^2; ...
       -2*J*sb*cb*da*db-k*sa*cb; ...
       J*sb*cb*da^2-k*ca*sb];
    cmd=-K*z;
    u=max(-limits,min(limits,cmd));
    % Optional external force pulse, not included in actuator clipping.
    if t>=1 && t<1.2
        u=u+pulse(:);
    end
    acc=MM\([u;0;0]-h);
    dz=[z(2);acc(1);z(4);acc(3);z(6);acc(2);z(8);acc(4)];
end

function out = summarizeTrial(t,Z,U)
    out.xmax=max(abs(Z(:,1)));
    out.ymax=max(abs(Z(:,5)));
    out.alphamax=max(abs(rad2deg(Z(:,3))));
    out.betamax=max(abs(rad2deg(Z(:,7))));
    out.fxmax=max(abs(U(:,1)));
    out.fymax=max(abs(U(:,2)));
    out.forceEnergy=trapz(t,sum(U.^2,2));
end
