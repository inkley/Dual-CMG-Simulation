function a=CMG_MECHANICAL_ACCOUNTING(t,x,tau1,tau2,c,g1,g2,config)
% Consistent separate mechanical boundaries; NOT complete motor input.
% Positive/negative integrals are reported without assumed regeneration.
t=t(:); assert(numel(t)==size(x,1) && all(diff(t)>0));
body=x(:,10:12);
moments1=[tau1.K,tau1.M,tau1.N]; moments2=[tau2.K,tau2.M,tau2.N];
ports=[sum(moments1.*body,2),sum(moments2.*body,2)];
a.vehicle=integrals(t,sum(ports,2));
a.vehiclePerModule=integrals(t,ports);
a.roll=integrals(t,(tau1.K+tau2.K).*x(:,10));
spinI=[g1.I,g2.I]; omega=x(:,[14,16]);
spinEnergy=.5*omega.^2.*spinI;
a.spinInitial=spinEnergy(1,:); a.spinFinal=spinEnergy(end,:);
a.spinChange=a.spinFinal-a.spinInitial;
a.spinAxialWork=integrals(t,omega.*c.Omegadot.*spinI);
a.spinIntegralResidual=a.spinAxialWork.net-a.spinChange;
% Inertial component about gimbal coordinate only. Gyroscopic loads, moving
% base coupling and friction are not included; do not add vehicle transfer
% as a substitute motor torque or call their sum motor consumption.
J=config.gimbal.assemblyInertia(:).';
rate=c.alphadot; accel=c.gimbalAccel;
gimbalEnergy=.5*rate.^2.*J;
a.gimbalInertialWork=integrals(t,rate.*accel.*J);
a.gimbalInitial=gimbalEnergy(1,:); a.gimbalFinal=gimbalEnergy(end,:);
a.gimbalChange=a.gimbalFinal-a.gimbalInitial;
a.gimbalIntegralResidual=a.gimbalInertialWork.net-a.gimbalChange;
a.completeMotorInputSupported=false; a.electricalEfficiencySupported=false;
a.duration=t(end)-t(1);
end
function z=integrals(t,p)
z.net=trapz(t,p,1);
z.positive=trapz(t,max(p,0),1);
z.negativeMagnitude=trapz(t,max(-p,0),1);
z.gross=trapz(t,abs(p),1);
end
