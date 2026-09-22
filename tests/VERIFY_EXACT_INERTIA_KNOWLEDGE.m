function VERIFY_EXACT_INERTIA_KNOWLEDGE
root=CMG_ROOT(); cases=MISSION_ESTIMATION_CASES();
for k=[12,13]
    folder=['estimation_',char(cases(k).name)];
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15',folder,'exact_inertia','mission.mat'));
    f=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.025_timeout_15',folder,'exact_inertia','mission.mat'));
    assert(a.result.terminal==f.result.terminal && a.result.passes==f.result.passes);
    assert(abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg)<.02);
    assert(all(a.history.propulsionForce==0) && ~any(a.history.phase=="SURGE"));
    % Exact knowledge makes predicted moments match true moments at the same
    % physical gimbal rates; this is not proof of requested-moment tracking.
    worst=0;
    for i=1:20:height(a.history)
        x=a.history.state(i,:).';
        cp.alphadot1=a.history.gimbalRate(i,1); cp.alphadot2=a.history.gimbalRate(i,2);
        cp.Omegadot1=0; cp.Omegadot2=0;
        [p,q]=CMG(a.b.gyro1,a.b.gyro2,cp,x);
        [y,g1,g2]=CMG_CONTROLLER_ESTIMATE(x,a.b.gyro1,a.b.gyro2,a.cfg);
        [u,v]=CMG(g1,g2,cp,y);
        worst=max(worst,norm([p.K+q.K-u.K-v.K,p.M+q.M-u.M-v.M,p.N+q.N-u.N-v.N]));
    end
    assert(worst<1e-12);
    fprintf('Case%d: exact moment-model error %.3g Nm; refined heading difference %.6f deg; outcome %s\n', ...
        k,worst,abs(a.result.finalHeadingErrorDeg-f.result.finalHeadingErrorDeg),a.result.terminal);
end
disp('Exact-inertia moment identity, safe inhibit and refinement checks passed.');
end
