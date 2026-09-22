function results=COMPARE_CMG_MECHANICAL_ENERGY
% Re-account saved single/dual 0--5s baseline trajectories without changing
% controller tuning. Uses a common dense interpolation grid; not an ODE rerun.
root=CMG_ROOT();
folders={fullfile('single','VFR'),fullfile('dual','symmetric_spin','VFR')};
current=load(fullfile(root,'Working Results',folders{2},'simulation_result.mat'));
rows=struct([]); audits=cell(2,2); inputs=cell(2,1);
for k=1:2
    b=load(fullfile(root,'Working Results',folders{k},'simulation_result.mat')); inputs{k}=b;
    assert(b.T_OUT(1)<=0 && b.T_OUT(end)>=5,'Need full 0--5s baseline.');
    config=b.cmgConfig;
    config.thruster=current.cmgConfig.thruster; config.thruster.enabled=false;
    config.thruster.commandMode='direct_force'; config.thruster.commandForce=[0;0];
    config.hybrid=current.cmgConfig.hybrid; config.hybrid.enabled=false;
    config.propulsion=AFT_PROPULSION_DEFAULTS(); config.propulsion.enabled=false;
    loop.cycleT=5; loop.fc=.2; loop.controlEndTime=inf;
    for grid=1:2
        t=(0:.001/grid:5).'; x=interp1(b.T_OUT,b.Y_OUT,t,'pchip');
        [q1,q2,c]=TORQUE(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,b.d,loop,config);
        a=CMG_MECHANICAL_ACCOUNTING(t,x,q1,q2,c,b.gyro1,b.gyro2,config);
        audits{k,grid}=a;
    end
    s.mode=string(config.mode); s.duration=a.duration;
    s.vehicleNetJ=a.vehicle.net; s.vehiclePositiveJ=a.vehicle.positive;
    s.vehicleNegativeJ=a.vehicle.negativeMagnitude; s.vehicleGrossJ=a.vehicle.gross;
    s.rollGrossJ=a.roll.gross;
    s.initialSpinJ=sum(a.spinInitial); s.finalSpinJ=sum(a.spinFinal);
    s.spinPositiveJ=sum(a.spinAxialWork.positive); s.spinNegativeJ=sum(a.spinAxialWork.negativeMagnitude);
    s.gimbalInertialPositiveJ=sum(a.gimbalInertialWork.positive);
    s.gimbalInertialNegativeJ=sum(a.gimbalInertialWork.negativeMagnitude);
    s.spinBalanceResidualJ=max(abs(a.spinIntegralResidual));
    s.gimbalBalanceResidualJ=max(abs(a.gimbalIntegralResidual));
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
end
results=struct2table(rows);
out=fullfile(root,'Working Results','mechanical_energy_audit');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'comparison.mat'),'results','audits','inputs');
writetable(results,fullfile(out,'comparison.csv')); disp(results);
end
