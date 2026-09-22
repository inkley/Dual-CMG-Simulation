function results=REPORT_INERTIA_ESTIMATION_BOUNDS
% Summarize every saved tested point; do not infer monotonicity or a continuum.
root=CMG_ROOT();
out=fullfile(root,'Working Results','inertia_estimation_bounds');
files=dir(fullfile(out,'e1_*.mat')); assert(~isempty(files));
rows=struct([]);
for k=1:numel(files)
    a=load(fullfile(out,files(k).name));
    expected=a.trueInertia.*(1+[a.s.error1Percent,a.s.error2Percent]/100);
    assert(max(abs(expected-a.c.estimation.rotorInertia))<1e-14);
    assert(isequal(a.trueInertia,[a.a.a.b.gyro1.I,a.a.a.b.gyro2.I]));
    assert(all(isfinite(a.x),'all'));
    assert(all(a.h.externalDumpMoment==0));
    assert(max(abs(a.h.rollDisturbance-(a.s.bias+SINUSOIDAL_ROLL_LOAD(a.t,a.c.external.sinusoidalRoll))))<1e-12);
    if a.s.maxStep<.01
        coarseTag=sprintf('e1_%+.6f_e2_%+.6f_bias_%+.6f_step_%g.mat', ...
            a.s.error1Percent/100,a.s.error2Percent/100,a.s.bias,.01);
        coarse=load(fullfile(out,coarseTag));
        assert(isequal(a.t,coarse.t));
        difference=max(abs(rad2deg(a.x(:,4)-coarse.x(:,4))));
        fprintf('Refinement [%+.2f,%+.2f]%%: max roll change %.9g deg\n',a.s.error1Percent,a.s.error2Percent,difference);
        assert(difference<.001 && a.s.pass==coarse.s.pass,'Refinement changed the conclusion.');
    end
    if isempty(rows), rows=a.s; else, rows(end+1)=a.s; end %#ok<AGROW>
end
results=sortrows(struct2table(rows),{'bias','error2Percent','error1Percent','maxStep'});
writetable(results,fullfile(out,'all_tested_points.csv'));
disp(results);
fprintf('Verified %d saved points; %d passed unchanged roll/momentum criteria.\n',height(results),nnz(results.pass));
end
