function cases=MISSION_ESTIMATION_CASES
cases=struct('name',"nominal",'bias',[0,0],'inertiaError',[0,0]);
for pattern=[1,2]
    weights=[1,1]; if pattern==2, weights=[1,-1]; end
    for bias=[-.05,.05]
        for inertia=[-.1,.1]
            s.name=string(sprintf('pattern%d_bias%g_inertia%g',pattern,bias*100,inertia*100));
            s.bias=bias*weights; s.inertiaError=inertia*weights;
            cases(end+1)=s; %#ok<AGROW>
        end
    end
end
% Complete the opposite-error 3x3 grid with single-factor edges; retain 1:9 IDs.
for signValue=[-1,1]
    s.name=string(sprintf('opposite_speed_only_%g',signValue*5));
    s.bias=signValue*.05*[1,-1]; s.inertiaError=[0,0]; cases(end+1)=s;
end
for signValue=[-1,1]
    s.name=string(sprintf('opposite_inertia_only_%g',signValue*10));
    s.bias=[0,0]; s.inertiaError=signValue*.1*[1,-1]; cases(end+1)=s;
end
end
