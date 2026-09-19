function results=DISTURBANCE_MISMATCH_SWEEP
% Deterministic screen at the nominally worst tracking/momentum period.
% Rotor inertia changes isolate momentum-model uncertainty at fixed installed
% mass/inertia; they are not alternative physical rotor geometries.
v=struct('name',"nominal_repeat",'inertiaScale',[1,1], ...
    'speedBias',[0,0],'gimbalLag',[1,1],'gimbalAccel',1);
cases=repmat(v,6,1);
cases(2).name="speed_bias"; cases(2).speedBias=[.05,-.05];
cases(3).name="unequal_inertia"; cases(3).inertiaScale=[1.1,.9];
cases(4).name="unequal_servo"; cases(4).gimbalLag=[.8,1.2]; cases(4).gimbalAccel=.8;
cases(5)=cases(4); cases(5).name="combined";
cases(5).speedBias=[.05,-.05]; cases(5).inertiaScale=[1.1,.9];
cases(6)=cases(5); cases(6).name="combined_reversed";
cases(6).speedBias=-cases(5).speedBias;
cases(6).inertiaScale=fliplr(cases(5).inertiaScale);
cases(6).gimbalLag=fliplr(cases(5).gimbalLag);
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'Working Results','roll_disturbance_envelope');
results=table;
for k=1:numel(cases)
    fprintf('Mismatch configuration: %s\n',cases(k).name);
    r=VERIFY_ROLL_DISTURBANCE_ENVELOPE(cases(k));
    r.variant=repmat(cases(k).name,height(r),1);
    results=[results;r]; %#ok<AGROW>
    writetable(results,fullfile(out,'mismatch_summary.csv'));
end
save(fullfile(out,'mismatch_summary.mat'),'results','cases');
fprintf('Roll/momentum screen: %d/%d cases passed. Cross-axis motion reported separately.\n',nnz(results.pass),height(results));
end
