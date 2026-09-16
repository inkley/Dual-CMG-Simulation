function cases=ACTUATOR_UNCERTAINTY_CASES
u=struct('gimbalLag',[1,1],'gimbalAccel',1,'vrtLag',[1,1], ...
    'vrtSlew',1,'vrtGain',[1,1],'aftLag',1,'aftSlew',1);
cases=repmat(struct('name',"nominal",'parameters',u),9,1);
cases(2).name="gimbal_fast"; cases(2).parameters.gimbalLag=[.8,.8];
cases(3).name="gimbal_slow"; cases(3).parameters.gimbalLag=[1.2,1.2]; cases(3).parameters.gimbalAccel=.8;
cases(4).name="gimbal_unequal"; cases(4).parameters.gimbalLag=[.8,1.2];
cases(5).name="vrt_slow"; cases(5).parameters.vrtLag=[1.2,1.2]; cases(5).parameters.vrtSlew=.8;
cases(6).name="vrt_low_gain"; cases(6).parameters.vrtGain=[.9,.9];
cases(7).name="vrt_unequal"; cases(7).parameters.vrtLag=[.8,1.2]; cases(7).parameters.vrtGain=[.9,1.1];
cases(8).name="aft_slow"; cases(8).parameters.aftLag=1.2; cases(8).parameters.aftSlew=.8;
cases(9).name="combined"; cases(9).parameters=cases(7).parameters;
cases(9).parameters.gimbalLag=[.8,1.2]; cases(9).parameters.gimbalAccel=.8;
cases(9).parameters.vrtSlew=.8; cases(9).parameters.aftLag=1.2; cases(9).parameters.aftSlew=.8;
end
