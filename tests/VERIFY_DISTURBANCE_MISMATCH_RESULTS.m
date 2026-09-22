function VERIFY_DISTURBANCE_MISMATCH_RESULTS
root=CMG_ROOT();
out=fullfile(root,'Working Results','roll_disturbance_envelope');
m=load(fullfile(out,'mismatch_summary.mat'));
b=load(fullfile(root,'Working Results','roll_disturbance_tests','zero_reference.mat'));
assert(height(m.results)==12 && numel(m.cases)==6);
for k=1:numel(m.cases)
    for j=1:2
        a=load(fullfile(out,char(m.cases(k).name),sprintf('case_%d.mat',j)));
        assert(all(isfinite(a.x),'all'));
        assert(isequal(a.a.b.gains,b.b.gains));
        assert(isequal(a.c.estimation.rotorInertia,[b.b.gyro1.I,b.b.gyro2.I]));
        assert(abs(a.a.b.gyro1.I-b.b.gyro1.I*m.cases(k).inertiaScale(1))<eps);
        assert(abs(a.a.b.gyro2.I-b.b.gyro2.I*m.cases(k).inertiaScale(2))<eps);
        plant=ACTUATOR_PLANT_CONFIG(a.c);
        assert(abs(plant.limits.maxGimbalAccel-b.c.limits.maxGimbalAccel*m.cases(k).gimbalAccel)<1e-10);
        assert(all(a.h.externalDumpMoment==0));
        expected=a.c.external.rollDisturbance+SINUSOIDAL_ROLL_LOAD(a.t,a.c.external.sinusoidalRoll);
        assert(max(abs(a.h.rollDisturbance-expected))<1e-12);
        if k==1
            original=load(fullfile(out,sprintf('case_%d.mat',j+2)));
            assert(abs(a.s.peakErrorDeg-original.s.peakErrorDeg)<1e-6);
        end
    end
end
disp(m.results);
fprintf('Metadata, fixed gains, nominal allocator inertia, plant limits, applied loads and nominal-repeat checks passed.\n');
end
