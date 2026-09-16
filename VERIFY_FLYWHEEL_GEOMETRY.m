function VERIFY_FLYWHEEL_GEOMETRY
g=FLYWHEEL_GEOMETRY(.0508,.0127,7750,5.8*.0254);
large=FLYWHEEL_GEOMETRY(.0508*1.2,.0127,7750,5.8*.0254);
assert(abs(large.m/g.m-1.2^2)<1e-12 && abs(large.I/g.I-1.2^4)<1e-12);
assert(g.sweptDiameter>2*g.r && g.rotorOnlyEnvelopeFits);
bad=FLYWHEEL_GEOMETRY(.1,.0127,7750,5.8*.0254); assert(~bad.rotorOnlyEnvelopeFits);
root=fileparts(mfilename('fullpath')); out=fullfile(root,'Working Results','flywheel_geometry');
a=load(fullfile(out,'sweep_0.01.mat')); b=load(fullfile(out,'sweep_0.005.mat'));
assert(height(a.results)==5 && isequal(a.results.screenPass,b.results.screenPass));
assert(max(abs(a.results.finalErrorDeg-b.results.finalErrorDeg))<.01);
assert(max(abs(a.results.peakGimbalDeg-b.results.peakGimbalDeg))<.01);
for k=1:5
    r=a.runs{k}; assert(abs(r.auv.m-a.b.baseMassProperties.m-2*r.gyro.m)<1e-10);
end
disp('Geometry scaling, envelope, installed mass and half-step checks passed.');
end
