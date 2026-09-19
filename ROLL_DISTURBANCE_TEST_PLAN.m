function plan=ROLL_DISTURBANCE_TEST_PLAN
% Fixed before nonzero-load experiments. Provisional development criteria.
plan.holdAngle=pi/4;
plan.load=struct('amplitude',0,'period',5,'startTime',10,'duration',60,'rampTime',2);
plan.endTime=80; plan.sampleTime=.01; plan.maxStep=.01;
plan.peakRollErrorDeg=2; plan.rmsRollErrorDeg=1;
plan.finalRecoveryErrorDeg=.5;
plan.notes="Synthetic loads; no hardware or sea-state validation.";
% Zero control plus four active/passive pairs; no momentum unloading.
plan.cases=struct('name',"zero_reference",'amplitude',0,'period',5,'feedback',true);
for amplitude=[.01,.03]
    for period=[5,15]
        for feedback=[true,false]
            s.name=string(sprintf('A%g_T%g_feedback%d',amplitude,period,feedback));
            s.amplitude=amplitude; s.period=period; s.feedback=feedback;
            plan.cases(end+1)=s; %#ok<AGROW>
        end
    end
end
end
