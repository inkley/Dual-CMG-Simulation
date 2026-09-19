function results=MISSION_ESTIMATION_ABLATION(runNew)
% Complete opposite-error factorial grid: nominal, 4 corners, 4 isolated edges.
% Existing corner trajectories are reused, not silently rerun/retuned.
if nargin<1, runNew=true; end
root=fileparts(mfilename('fullpath')); cases=MISSION_ESTIMATION_CASES();
if runNew
    for k=10:13, RUN_ROLL_TURN_SURGE(struct('estimationCase',k)); end
end
rows=struct([]); ids=[1,6:13];
for k=ids
    a=load(fullfile(root,'Working Results','roll_turn_surge', ...
        'plane_45_heading_45_dt_0.05_timeout_15', ...
        ['estimation_',char(cases(k).name)],'mission.mat'));
    h=a.history; inTurn=h.phase=="TURN"; j=find(inTurn,1,'last');
    s.caseIndex=k; s.name=cases(k).name; s.bias1=cases(k).bias(1);
    s.inertiaError1=cases(k).inertiaError(1); s.terminal=a.result.terminal;
    s.passes=a.result.passes;
    s.lastTurnHeadingDeg=rad2deg(h.headingError(j));
    s.lastTurnPlaneDeg=rad2deg(h.planeError(j));
    s.lastTurnRateDeg=rad2deg(h.angularRate(j));
    s.lastTurnTime=h.time(j); s.limitSamples=sum(a.result.limitedSamples);
    s.rollInfeasible=a.result.rollInfeasibleSamples;
    idsTurn=find(inTurn); mismatch=zeros(numel(idsTurn),3);
    for z=1:numel(idsTurn)
        i=idsTurn(z); x=h.state(i,:).';
        cp.alphadot1=h.gimbalRate(i,1); cp.alphadot2=h.gimbalRate(i,2);
        cp.Omegadot1=0; cp.Omegadot2=0;
        [p,q]=CMG(a.b.gyro1,a.b.gyro2,cp,x);
        [measured,g1,g2]=CMG_CONTROLLER_ESTIMATE(x,a.b.gyro1,a.b.gyro2,a.cfg);
        [u,v]=CMG(g1,g2,cp,measured);
        mismatch(z,:)=[p.K+q.K-u.K-v.K,p.M+q.M-u.M-v.M,p.N+q.N-u.N-v.N];
    end
    tt=h.time(inTurn);
    rms=sqrt(trapz(tt,mismatch.^2)/(tt(end)-tt(1)));
    s.rollModelRMSE=rms(1); s.pitchModelRMSE=rms(2); s.yawModelRMSE=rms(3);
    if isempty(rows), rows=s; else, rows(end+1)=s; end %#ok<AGROW>
end
results=struct2table(rows);
out=fullfile(root,'Working Results','mission_estimation_ablation');
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'ablation.mat'),'results','cases');
writetable(results,fullfile(out,'summary.csv')); disp(results);
end
