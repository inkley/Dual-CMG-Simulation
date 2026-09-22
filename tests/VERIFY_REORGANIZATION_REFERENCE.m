function VERIFY_REORGANIZATION_REFERENCE(referenceFile)
% Optional integration regression against archived pre-reorganization inputs.
% Not in the fast suite; requires the separately archived 11 MB MAT fixture.
if nargin<1
    referenceFile=fullfile(CMG_ROOT,'Working Results','reorganization_validation','numerical_reference.mat');
end
a=load(referenceFile,'inputs','results');
for k=1:numel(a.inputs)
    z=a.inputs{k}; b=z.b;
    [t,x]=ode45(@(t,x) CONTROL(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,z.d,z.loop,z.c), ...
        z.times,z.x0,odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',.01));
    [~,~,h]=TORQUE(t,x,b.gains,b.gyro1,b.gyro2,b.auv,b.params,z.d,z.loop,z.c);
    assert(isequaln(a.results{k},struct('t',t,'x',x,'h',h)), ...
        'Pre/post trajectory or control-history difference for case %d.',k);
    fprintf('Exact pre/post equivalence: archived case %d\n',k);
end
end
