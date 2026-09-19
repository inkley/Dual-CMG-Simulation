function d=ROLL_FEASIBILITY_DIAGNOSTIC(B,rhs,limit)
% Static two-axis bounded least squares; not finite-servo reachability.
% Include interior minimum-norm solution and all four box edges. At rank
% deficiency an edge candidate also covers feasible null-space solutions.
u=pinv(B)*rhs; candidates=min(max(u,-limit),limit);
for fixed=1:2
    free=3-fixed;
    for signValue=[-1,1]
        v=zeros(2,1); v(fixed)=signValue*limit;
        column=B(:,free); denom=column.'*column;
        if denom>0, v(free)=column.'*(rhs-B(:,fixed)*v(fixed))/denom; end
        v(free)=min(max(v(free),-limit),limit);
        candidates(:,end+1)=v; %#ok<AGROW>
    end
end
[residual,j]=min(vecnorm(B*candidates-rhs));
d.exactRates=u; d.exactResidual=norm(B*u-rhs);
d.boundedRates=candidates(:,j); d.boundedResidual=residual;
d.momentError=B*d.boundedRates-rhs;
d.condition=cond(B); sigma=svd(B); d.sigmaMin=sigma(end);
d.legacyFeasible=d.exactResidual<1e-7 && max(abs(u))<=limit+1e-8;
d.boundedToleranceFeasible=residual<1e-7;
% Roll-only interval for the bias-subtracted demand. Pitch may be nonzero.
d.rollCapacity=limit*sum(abs(B(1,:)));
d.rollMargin=d.rollCapacity-abs(rhs(1));
d.rollOnlyFeasible=d.rollMargin>=-1e-10;
d.rollPriorityRates=[NaN;NaN]; d.rollPriorityPitchResidual=NaN;
if d.rollOnlyFeasible
    % Intersections of exact-roll line with the rate box. Minimize |pitch|
    % on this segment, preserving exact roll instead of trading K against M.
    vertices=[];
    for fixed=1:2
        free=3-fixed;
        if abs(B(1,free))>eps
            for signValue=[-1,1]
                v=zeros(2,1); v(fixed)=signValue*limit;
                v(free)=(rhs(1)-B(1,fixed)*v(fixed))/B(1,free);
                if abs(v(free))<=limit+1e-9, vertices(:,end+1)=v; end %#ok<AGROW>
            end
        end
    end
    if isempty(vertices) && abs(rhs(1))<1e-10
        vertices=candidates; % degenerate zero roll row
    end
    if ~isempty(vertices)
        pitch=B(2,:)*vertices-rhs(2);
        [lo,i]=min(pitch); [hi,j]=max(pitch);
        if lo<=0 && hi>=0 && hi>lo
            v=vertices(:,i)+(vertices(:,j)-vertices(:,i))*(-lo)/(hi-lo);
        else
            [~,i]=min(abs(pitch)); v=vertices(:,i);
        end
        d.rollPriorityRates=v;
        d.rollPriorityPitchResidual=B(2,:)*v-rhs(2);
    end
end
end
