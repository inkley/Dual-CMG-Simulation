% AUV_SIM.m
% This is the main simulation script for the Autonomous Underwater Vehicle
% (AUV) with active gyroscopic roll control. The script initializes the
% vehicle's state, selects the simulation scenario, and sets up the
% gyroscope and control parameters. It then runs the simulation using an
% ODE solver to simulate the vehicle's dynamics over a specified control
% cycle.
% 
% The script also handles the post-processing of simulation results,
% including plotting various state variables, Euler angles, and
% CMG-generated torques. It provides options for different simulation
% cases, such as pure roll from rest, nominal forward surge velocity,
% standard turn maneuvers, and simple path following.
%
% Author:           Tyler J. Inkley
% Last Updated:     11/12/2024

clc; clear; close all; 
%% STATE SPACE  
tic         % Start simulaiton timer

% SELECT SIM CASE
ii = 0;     % 0 VFR, 1 FSV, 2 STM, 3 SPF                          

% Change result designation based on sim case
manpath = '/Users/Tyler/Desktop/RESEARCH/4 Dissertation/Manuscripts/CMG/Working Results/';  % Work
%manpath = '/Users/Tyler/Desktop/RESEARCH/4 Dissertation/Manuscripts/CMG/MDPI_Work/';        % MDPI

    %%%%% PURE ROLL FROM VEHICLE AT REST %%%%%
    if      ii == 0                     
        % Case String
        mancase     = 'VFR';
        manpathcase = append(manpath,mancase);
        simcase     = '../Results/Case1/VFR';
    
        % Inertial states
        state.x     = 0;
        state.y     = 0;
        state.z     = 0;
        state.phi   = 0;
        state.theta = 0;
        state.psi   = 0;
        
        % Body-fixed states
        state.u     = 0;
        state.v     = 0;
        state.w     = 0;
        state.p     = 0;
        state.q     = 0;
        state.r     = 0; 
    end

    % %%%%% NOMINAL FORWARD SURGE VELOCITY %%%%%
    % elseif  ii == 1                    
    %     % Case String
    %     mancase     = 'FSV';
    %     manpathcase = append(manpath,mancase);
    %     simcase     = '../Results/Case2/FSV';
    % 
    %     % Inertial states
    %     state.x     = 0;
    %     state.y     = 0;
    %     state.z     = 0;
    %     state.phi   = 0;
    %     state.theta = 0;
    %     state.psi   = 0;
    % 
    %     % Body-fixed states
    %     state.u     = 3;
    %     state.v     = 0;
    %     state.w     = 0;
    %     state.p     = 0;
    %     state.q     = 0;
    %     state.r     = 0; 
    % 
    % %%%%% STANDARD TURN MANEVUER %%%%%    
    % elseif  ii == 2                    
    %     % Case String
    %     mancase     = 'STM';
    %     manpathcase = append(manpath,mancase);
    %     simcase     = '../Results/Case3/STM';
    % 
    %     % Inertial states
    %     state.x     = 0;
    %     state.y     = 0;
    %     state.z     = 0;
    %     state.phi   = 0;
    %     state.theta = 0;
    %     state.psi   = 0;
    % 
    %     % Body-fixed states
    %     state.u     = 3;
    %     state.v     = 0;
    %     state.w     = 0;
    %     state.p     = 0;
    %     state.q     = 0;
    %     state.r     = 1.5;
    % 
    % %%%%% SIMPLE PATH FOLLOWING (2-D SINE WAVE, 3-D HELIX) %%%%%
    % else                               
    %     % Case String
    %     mancase     = 'SPF';
    %     manpathcase = append(manpath,mancase);
    %     simcase     = '../Results/Case4/SPF';
    % 
    %     path.sigma      = 4;            % 3D Trajectory (sigma)          
    %     path.LAD        = 5;            % Look Ahead Distance (LAD)
    % 
    %     % Inertial states
    %     state.x     = 0;
    %     state.y     = 0;
    %     state.z     = 0;
    %     state.phi   = 0;
    %     state.theta = 0;
    %     state.psi   = 0;
    % 
    %     % Body-fixed states
    %     state.u     = 3;
    %     state.v     = 0;
    %     state.w     = 0;
    %     state.p     = 0;
    %     state.q     = 0;
    %     state.r     = 0; 
    % 
    % end

%% GYROSCOPE STATES
% AFT CMG (CMG #1)
state.alpha1    = 0;                        % CMG deflection angle, rad (about zg)
state.Omega1    = 10*pi;                    % Flywheel angular velocity, rad/s (300rpm)
gyro1.r         = 0.0508;                   % m, (2") 
gyro1.t         = 0.0127;                   % m, (1/2")
gyro1.v         = pi*gyro1.r^2*gyro1.t;     % m^3
gyro1.rho       = 7750;                     % Steel, kg/m^3
gyro1.m         = gyro1.v *gyro1.rho;       % kg    
gyro1.I         = 0.5*gyro1.m*gyro1.r^2;    % CMG flywheel inertia (uniform thin disc)  --> HOLDS [1/3x - 1/2x FW D]

% FWD CMG (CMG #2)
state.alpha2    = 0;                        % CMG deflection angle, rad (about zg)
state.Omega2    = 10*pi;                    % Flywheel angular velocity, rad/s (300rpm)
gyro2.r         = 0.0508;                   % m, (2") 
gyro2.t         = 0.0127;                   % m, (1/2")
gyro2.v         = pi*gyro2.r^2*gyro2.t;     % m^3
gyro2.rho       = 7750;                     % Steel, kg/m^3
gyro2.m         = gyro2.v *gyro2.rho;       % kg    
gyro2.I         = 0.5*gyro2.m*gyro2.r^2;    % CMG flywheel inertia (uniform thin disc)  --> HOLDS [1/3x - 1/2x FW D]

%% STATE VECTOR 
%state_cell =
%{'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r','\alpha_1','\Omega_1','\alpha_2','\Omega_2'};
state_vec   =   [state.x;state.y;state.z;state.phi;state.theta;state.psi;
                state.u;state.v;state.w;state.p;state.q;state.r;state.alpha1;
                state.Omega1;state.alpha2;state.Omega2];

%% INPUTS
% Desired Euler Angles
d.phi       = pi/2;                 % Desired Phi   - Roll Euler Angle, rad
d.theta     = 0;                    % Desired Theta - Pitch Euler Angle, rad
d.psi       = 0;                    % Desired Psi   - Yaw Euler Angle, rad

% AUV properties
auv.W       = 2.99E02;              % Vehicle weight (REMUS 100), N
auv.g       = 9.81;                 % m/s^2
auv.m       = auv.W/auv.g;          % kg, results converged with m = 3kg!
auv.D       = 0.14732;              % m, IVER 3EP Outer Tube Diameter (5.8")
auv.d       = auv.D - 0.0127;       % m, (assumed) Inner Tube Diameter (quarter inch thickness, -0.5")

% PD gains 
gains.Kpu   = 4;                    % Proportional gain for surge (forward/backward) control
gains.Kdu   = 1;                    % Derivative gain for surge control
gains.Kpp   = 5;                    % Proportional gain for roll control
gains.Kdp   = 1.5;                  % Derivative gain for roll control
gains.Kpr   = 30;                   % Proportional gain for yaw control
gains.Kdr   = 14;                   % Derivative gain for yaw control

% Loop frequencies 
loop.cycleT = 3;                    % s
loop.fc     = 1/loop.cycleT;        % fc = 0.33 Hz 

% Params
params.m    = auv.m;                % Vehicle dry mass      (kg)
params.Ix   = 1.77E-01;             % Vehicle Ixx           (kgm^2)
params.Iy   = 3.45;                 % Vehicle Iyy           (kgm^2)
params.Iz   = 3.45;                 % Vehicle Izz           (kgm^2)
params.xg   = 0;                    % Cg x-position         (m)
params.yg   = 0;                    % Cg y-position         (m)
params.zg   = 0;                    % Cg z-position         (m)

%%  AUV SIMULATION
%[Etadot] = CONTROL(0,state_vec,gains,gyro1,gyro2,auv,params,d,loop);  % Debug fn. 
[T_OUT, Y_OUT] = ode45(@CONTROL, [0 loop.cycleT], state_vec, [], gains, gyro1, gyro2, auv, params, d, loop);      

% CMG TORQUE CALCS
[tau_cmg1,tau_cmg2] = TORQUE(gyro1,gyro2,T_OUT,Y_OUT);
t                   = T_OUT(1:length(T_OUT)-1,:);
toc

%% PLOTS
state_cell = {'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r','\alpha_1','\Omega_1','\alpha_2','\Omega_2'};

% All states  
    figure 
    for kk = 1:width(Y_OUT)
        plot(T_OUT,Y_OUT(:,kk),'LineWidth',2,'DisplayName',state_cell{1,kk})    
        hold on; 
        legend('-DynamicLegend','Location','eastoutside')
    end
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Magnitude')
        title('All Simulation States')
        set(gca,'FontSize',16,'LineWidth',1.0)

        resultcase = 'ARC_ALL';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpath,resultcase);
        print(gcf,'-depsc',strNameMAN)

% Generalized Position Vector, eta
    figure 
    for kk = 1:6
        plot(T_OUT,Y_OUT(:,kk),'LineWidth',2,'DisplayName',state_cell{1,kk})    
        hold on; 
        legend('-DynamicLegend','Location','eastoutside')
    end
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Magnitude (m,rad)')
        title('Generalized Position Vector, \eta')
        set(gca,'FontSize',16,'LineWidth',1.0)

        resultcase = 'ARC_ETA';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpath,resultcase);
        print(gcf,'-depsc',strNameMAN)

% Generalized Velocity Vector, nu
    figure 
    for kk = 7:12
        plot(T_OUT,Y_OUT(:,kk),'LineWidth',2,'DisplayName',state_cell{1,kk})    
        hold on; 
        legend('-DynamicLegend','Location','eastoutside')
    end
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Magnitude (m/s, rad/s)')
        title('Generalized Velocity Vector, \nu')
        set(gca,'FontSize',16,'LineWidth',1.0)

        resultcase = 'ARC_NU';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpath,resultcase);
        print(gcf,'-depsc',strNameMAN)

% Gyroscope states
    % overlay
    figure 
    for kk = 13:16
        plot(T_OUT,Y_OUT(:,kk),'LineWidth',2,'DisplayName',state_cell{1,kk})    
        hold on; 
        legend('-DynamicLegend','Location','northeast')
    end
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Magnitude (rad, rad/s)')
        title('Gyroscope States')
        set(gca,'FontSize',16,'LineWidth',1.0)

        resultcase = 'ARC_GYRO1';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)

    % subplots
    figure 
    subplot(2,1,1)      % alpha
    plot(T_OUT,Y_OUT(:,13),'LineWidth',2) 
    hold on
    plot(T_OUT,Y_OUT(:,15),'LineWidth',2)        
    %plot(T_OUT,Y_OUT(:,13),'Color',[0 0.4470 0.7410],'LineWidth',2)         % blue  
    %plot(T_OUT,Y_OUT(:,13),'Color',[0.9290 0.6940 0.1250],'LineWidth',2)    % yellow  
    %plot(T_OUT,Y_OUT(:,13),'Color',[0.4660 0.6740 0.1880],'LineWidth',2)    % green  
        grid on; grid minor
        title('Gyroscope States throughout Control Cycle')
        ylabel('\alpha, (rad)')
        %legend('Al','Ti','Steel','Location','southeast')
        ylim([-pi/2 0])
        set(gca,'FontSize',16,'LineWidth',1.0)

        % Convert y-axis to pi units
        yticks(-pi/2:pi/4:0)  % Set y-ticks at intervals of pi/4
        yticklabels({'-\pi/2', '-\pi/4', '0'})  % Set corresponding labels

    subplot(2,1,2)      % Omega
    plot(T_OUT,Y_OUT(:,14),'LineWidth',2)  
    hold on
    plot(T_OUT,Y_OUT(:,16),'LineWidth',2) 
    %plot(T_OUT,Y_OUT(:,14),'Color',[0.8500 0.3250 0.0980],'LineWidth',2)    % red (Single Gyroscope State Plot)
    %plot(T_OUT,Y_OUT(:,14),'LineWidth',2)        
    %plot(T_OUT,Y_OUT(:,14),'Color',[0.4940 0.1840 0.5560],'LineWidth',2)   % purple
    %plot(T_OUT,Y_OUT(:,14),'Color',[0.3010 0.7450 0.9330],'LineWidth',2)   % light blue
        grid on; grid minor
        ylabel('\Omega, (rad/s)')
        %legend('Al','Ti','Steel','Location','northeast')
        xlabel('Control Cycle Time (s)')
        set(gca,'FontSize',16,'LineWidth',1.0)

        resultcase = 'ARC_GYRO2';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)

% Roll convergence over single control cycle
    figure
    plot(T_OUT,Y_OUT(:,4),'LineWidth',2)                        % phi
    hold on
    plot([0 loop.cycleT],[d.phi, d.phi], '--k','LineWidth',2)   % phi_d
        xlabel('Control Cycle Time (s)')
        ylabel('Angle (rad)')
        title(' Roll Convergence over Single Control Cycle')
        grid on; grid minor
        xlim([0 loop.cycleT])
        ylim([0 pi])
        legend('\phi','\phi_d')
        set(gca,'FontSize',16,'LineWidth',1.0)
        
        % Convert y-axis to pi units
        yticks(0:pi/4:pi)  % Set y-ticks at intervals of pi/4
        yticklabels({'0', '\pi/4', '\pi/2', '3\pi/4', '\pi'})  % Set corresponding labels
    
        resultcase = 'ARC_ROLL';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)
    
% Euler Angles
    % overlay
    figure
    plot(T_OUT,Y_OUT(:,4),'LineWidth',2)    % phi
    hold on
    plot(T_OUT,Y_OUT(:,5),'LineWidth',2)    % theta
    plot(T_OUT,Y_OUT(:,6),'LineWidth',2)    % psi
        xlabel('Control Cycle Time (s)')
        ylabel('Angle (rad)')
        title('Euler Angles throughout Control Cycle')
        grid on; grid minor
        xlim([0 loop.cycleT])
        ylim([-0.01 pi])
        legend('\phi','\theta', '\psi')
        set(gca,'FontSize',16,'LineWidth',1.0)
        
        % Convert y-axis to pi units
        yticks(0:pi/4:pi)  % Set y-ticks at intervals of pi/4
        yticklabels({'0', '\pi/4', '\pi/2', '3\pi/4', '\pi'})  % Set corresponding labels

        resultcase = 'ARC_EULER1';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)
    
    % subplots
    figure 
    subplot(3,1,1)    % phi
    plot(T_OUT,Y_OUT(:,4),'LineWidth',2)                                    
    hold on
        grid on; grid minor
        title('Euler Angles throughout Control Cycle (rad)')
        ylabel('\phi')
        %legend('Al','Ti','Steel','Location','southeast')
        ylim([-0.1 pi/2])
        set(gca,'FontSize',16,'LineWidth',1.0)

        % Convert y-axis to pi units
        yticks(0:pi/4:pi/2)  % Set y-ticks at intervals of pi/4
        yticklabels({'0', '\pi/4', '\pi/2'})  % Set corresponding labels
    subplot(3,1,2)    % theta
    %plot(T_OUT,Y_OUT(:,5),'LineWidth',2)                                    
    plot(T_OUT,Y_OUT(:,5),'Color',[0.8500 0.3250 0.0980],'LineWidth',2)    % Single Gryoscope State Plot  
    hold on
        grid on; grid minor
        ylabel('\theta')
        set(gca,'FontSize',16,'LineWidth',1.0)
    subplot(3,1,3)    % psi
    %plot(T_OUT,Y_OUT(:,6),'LineWidth',2)              
    plot(T_OUT,Y_OUT(:,6),'Color',[0.9290 0.6940 0.1250],'LineWidth',2)    % Single Gryoscope State Plot     
    hold on
        grid on; grid minor
        ylabel('\psi')
        xlabel('Control Cycle Time (s)')
        set(gca,'FontSize',16,'LineWidth',1.0)
    
        resultcase = 'ARC_EULER2';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)
% AFT CMG Torques
    % overlay
    figure 
    plot(t,tau_cmg1.K,'LineWidth',2)     % K
    hold on
    plot(t,tau_cmg1.M,'LineWidth',2)     % M
    plot(t,tau_cmg1.N,'LineWidth',2)     % N
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Torque (N•m)')
        title('All CMG Torques')
        legend('K','M','N')
        set(gca,'FontSize',16,'LineWidth',1.0)
    
        resultcase = 'ARC_CMG1';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)
   
    % subplots
    figure 
    subplot(3,1,1)  % K
    plot(t,tau_cmg1.K,'LineWidth',2)   
    hold on
        grid on; grid minor
        title('All CMG Torques (N•m)')
        ylabel('K')
        set(gca,'FontSize',16,'LineWidth',1.0)
    subplot(3,1,2)  % M
    %plot(t,tau_cmg.M,'LineWidth',2)                                     
    plot(t,tau_cmg1.M,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  % (Single Gyroscope State)
    hold on
        grid on; grid minor
        ylabel('M')
        set(gca,'FontSize',16,'LineWidth',1.0)
    subplot(3,1,3)  % N             
    %plot(t,tau_cmg.N,'LineWidth',2)                                     
    plot(t,tau_cmg1.N,'Color',[0.9290 0.6940 0.1250],'LineWidth',2)  % (Single Gyroscope State)  
    hold on
        grid on; grid minor
        ylabel('N')
        xlabel('Control Cycle Time (s)')
        set(gca,'FontSize',16,'LineWidth',1.0)
    
        resultcase = 'ARC_CMG2';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN) 

% FWD CMG Torques
    % overlay
    figure 
    plot(t,tau_cmg2.K,'LineWidth',2)     % K
    hold on
    plot(t,tau_cmg2.M,'LineWidth',2)     % M
    plot(t,tau_cmg2.N,'LineWidth',2)     % N
        grid on; grid minor
        xlabel('Control Cycle Time (s)')
        ylabel('Torque (N•m)')
        title('All CMG Torques')
        legend('K','M','N')
        set(gca,'FontSize',16,'LineWidth',1.0)
    
        resultcase = 'ARC_CMG3';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN)
   
    % subplots
    figure 
    subplot(3,1,1)  % K
    plot(t,tau_cmg2.K,'LineWidth',2)   
    hold on
        grid on; grid minor
        title('All CMG Torques (N•m)')
        ylabel('K')
        set(gca,'FontSize',16,'LineWidth',1.0)
    subplot(3,1,2)  % M
    %plot(t,tau_cmg.M,'LineWidth',2)                                     
    plot(t,tau_cmg2.M,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)  % (Single Gyroscope State)
    hold on
        grid on; grid minor
        ylabel('M')
        set(gca,'FontSize',16,'LineWidth',1.0)
    subplot(3,1,3)  % N             
    %plot(t,tau_cmg.N,'LineWidth',2)                                     
    plot(t,tau_cmg2.N,'Color',[0.9290 0.6940 0.1250],'LineWidth',2)  % (Single Gyroscope State)  
    hold on
        grid on; grid minor
        ylabel('N')
        xlabel('Control Cycle Time (s)')
        set(gca,'FontSize',16,'LineWidth',1.0)
    
        resultcase = 'ARC_CMG4';
        strNameARC = append(simcase,resultcase);
        print(gcf,'-depsc',strNameARC)
        print(gcf,'-dpng',strNameARC)
        %savefig(strNameARC)
        strNameMAN = append(manpathcase,resultcase);
        print(gcf,'-depsc',strNameMAN) 
        
% %% CMG PROPERTIES
% % Define the range of radius and thickness
% radii       = linspace(0.0508, 0.0762, 20);     % 2 inches to 3 inches (in meters)
% thicknesses = linspace(0.00635, 0.0508, 20);    % 1/4 inch to 2 inches (in meters)
% 
% % Preallocate matrices to store max Δɑ, ΔΩ, and input/output energies
% max_delta_alpha                 = zeros(length(radii), length(thicknesses));
% max_delta_omega                 = zeros(length(radii), length(thicknesses));
% energy_input_matrix             = zeros(length(radii), length(thicknesses));
% output_energy_roll_matrix       = zeros(length(radii), length(thicknesses));
% peak_output_energy_roll_matrix  = zeros(length(radii), length(thicknesses));
% 
% % Loop over the range of radii and thicknesses
% for i = 1:length(radii)
%     for j = 1:length(thicknesses)
% 
%         % Set the gyroscope parameters for this iteration
%         gyro.r = radii(i);
%         gyro.t = thicknesses(j);
% 
%         % Recalculate the mass and inertia of the gyroscope
%         gyro.v = pi * gyro.r^2 * gyro.t;   % Volume of flywheel
%         gyro.m = gyro.v * gyro.rho;        % Mass of flywheel
%         gyro.I = 0.5 * gyro.m * gyro.r^2;  % Moment of inertia (thin disc)
% 
%         % Run the simulation
%         [T_OUT, Y_OUT] = ode45(@CONTROL, [0 loop.cycleT], state_vec, [], gains, gyro, auv, params, d, loop);
%         [tau_cmg, tau_Omega, tau_alpha] = TORQUE(gyro, T_OUT, Y_OUT);
% 
%         % Extract Δɑ and ΔΩ
%         delta_alpha = diff(Y_OUT(:, 13));  % Deflection angle changes
%         delta_omega = diff(Y_OUT(:, 14));  % Angular velocity changes
% 
%         % Store the max values in the matrices
%         max_delta_alpha(i, j) = max(abs(delta_alpha));  % Max change in alpha
%         max_delta_omega(i, j) = max(abs(delta_omega));  % Max change in Omega
% 
%         % Extract necessary parameters for energy calculations
%         Omega       = Y_OUT(1:end-1, 14);                   % Angular velocity
%         alpha_dot   = diff(Y_OUT(:, 13)) ./ diff(T_OUT);    % Deflection velocity        
%         K           = tau_cmg.K;                            % Roll torque (from TORQUE.m)
%         p           = Y_OUT(1:end-1, 10);                   % Roll velocity
% 
%         % Compute instantaneous roll power
%         P_roll = K .* p;
% 
%         % Integrate over time to find total output energy in roll
%         E_out_roll = trapz(T_OUT(1:end-1), abs(P_roll));
% 
%         % Find the maximum values of K and p
%         max_K = max(abs(K));
%         max_p = max(abs(p));        
% 
%         % Calculate peak output energy based on maximum K and p
%         peak_output_energy_roll = max_K * max_p;
% 
%         % Compute instantaneous powers for input energy calculation
%         P1 = Omega .* tau_Omega;               % Power from gyroscope angular velocity
%         P2 = alpha_dot .* tau_alpha;           % Power from deflection velocity
% 
%         % Integrate the absolute power to calculate the total energy input
%         E_in = trapz(T_OUT(1:end-1), abs(P1)) + trapz(T_OUT(1:end-1), abs(P2));
% 
%         % Store the energy input in the matrix
%         energy_input_matrix(i, j) = E_in;        
% 
%         % Store the total output energy in roll in the matrix
%         output_energy_roll_matrix(i, j) = E_out_roll;
% 
%         % Store the peak output energy in roll in a separate matrix
%         peak_output_energy_roll_matrix(i, j) = peak_output_energy_roll;
%     end
% end
% 
% % Contour plot for energy input
% figure
% contourf(radii, thicknesses, energy_input_matrix, 20) 
% xlabel('Gyroscope Radius (m)')
% ylabel('Gyroscope Thickness (m)')
% title('CMG Energy Input (J)')
% colorbar
% set(gca, 'FontSize', 16, 'LineWidth', 1.0)
% 
% % Plotting the total output energy in roll as a contour plot
% figure
% contourf(radii, thicknesses, output_energy_roll_matrix, 20)
% xlabel('Gyroscope Radius (m)')
% ylabel('Gyroscope Thickness (m)')
% title('CMG Total Output Energy in Roll (J)')
% colorbar
% set(gca, 'FontSize', 16, 'LineWidth', 1.0)
% 
% % Plotting the peak output energy in roll as a contour plot
% figure
% contourf(radii, thicknesses, peak_output_energy_roll_matrix, 20)
% xlabel('Gyroscope Radius (m)')
% ylabel('Gyroscope Thickness (m)')
% title('CMG Peak Output Energy in Roll (J)')
% colorbar
% set(gca, 'FontSize', 16, 'LineWidth', 1.0)