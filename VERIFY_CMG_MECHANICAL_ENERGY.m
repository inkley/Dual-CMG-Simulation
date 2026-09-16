function VERIFY_CMG_MECHANICAL_ENERGY
t=(0:.001:1).'; x=zeros(numel(t),18); x(:,10)=1;
x(:,14)=2+t; x(:,16)=3-t;
q.K=ones(size(t)); q.M=zeros(size(t)); q.N=q.M;
z=q; z.K=-q.K; g1.I=2; g2.I=1;
c.Omegadot=[ones(size(t)),-ones(size(t))];
c.alphadot=[t,2*t]; c.gimbalAccel=[ones(size(t)),2*ones(size(t))];
cfg.gimbal.assemblyInertia=[1,2];
a=CMG_MECHANICAL_ACCOUNTING(t,x,q,z,c,g1,g2,cfg);
assert(abs(a.vehicle.gross)<1e-12 && abs(sum(a.vehiclePerModule.gross)-2)<1e-12);
assert(max(abs(a.spinIntegralResidual))<1e-10);
assert(max(abs(a.gimbalIntegralResidual))<1e-10);
assert(abs(sum(a.spinChange)-2.5)<1e-10);
root=fileparts(mfilename('fullpath'));
b=load(fullfile(root,'Working Results','mechanical_energy_audit','comparison.mat'));
for k=1:2
    coarse=b.audits{k,1}; fine=b.audits{k,2};
    assert(abs(coarse.roll.gross-fine.roll.gross)<1e-4);
    assert(max(abs(coarse.spinAxialWork.gross-fine.spinAxialWork.gross))<1e-3);
    fprintf('%s spin/gimbal integral residual %.6g / %.6g J\n', ...
        b.results.mode(k),max(abs(fine.spinIntegralResidual)),max(abs(fine.gimbalIntegralResidual)));
end
disp('Mechanical accounting signs, storage identities, cancellation and quadrature checks passed.');
end
