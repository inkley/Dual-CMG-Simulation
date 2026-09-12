% THRUSTER_ALLOCATION_ANALYSIS.m
% Plot the attainable body sway/yaw envelope of the fixed thruster pair.

clearvars;
scriptDir = fileparts(mfilename('fullpath'));
baseline = load(fullfile(scriptDir,'Working Results','dual', ...
    'symmetric_spin','VFR','simulation_result.mat'),'cmgConfig');
config = baseline.cmgConfig.thruster;
B = [1,1;config.positionBody(1,:)];
forceCorners = config.maxForce*[-1,-1,1,1,-1; -1,1,1,-1,-1];
envelope = B*forceCorners;

result.allocationMatrix = B;
result.conditionNumber = cond(B);
result.maximumSwayForce = 2*config.maxForce;
result.maximumYawMoment = abs(diff(config.positionBody(1,:))) ...
    *config.maxForce;
result.envelopeYN = envelope;

fig = figure('Name','Thruster sway-yaw allocation envelope');
fill(envelope(1,:),envelope(2,:),[0.80,0.90,1.00], ...
    'EdgeColor',[0,0.35,0.70],'LineWidth',2);
hold on;
plot(0,0,'ok','MarkerFaceColor','k');
hold off; grid on; axis equal;
xlabel('Body sway force Y (N)');
ylabel('Body yaw moment N (N m)');
title('Fixed Vortex-Ring Thruster Allocation Envelope');

outputDir = fullfile(scriptDir,'Working Results');
exportgraphics(fig,fullfile(outputDir,'THRUSTER_ALLOCATION_ENVELOPE.png'), ...
    'Resolution',300);
save(fullfile(outputDir,'thruster_allocation_envelope.mat'),'result');
fprintf(['Thruster envelope: |Y| <= %.1f N at N=0, |N| <= %.2f N m ' ...
    'at Y=0, allocation condition %.3f.\n'],result.maximumSwayForce, ...
    result.maximumYawMoment,result.conditionNumber);
