function VERIFY_ROLL_TURN_SURGE()
% Unit checks for dwell reset, ordering, turn qualification, inhibit, timeout,
% and forward-only speed-control anti-windup. No external state/persistent ODE memory.
m=ROLL_TURN_SURGE_DEFAULTS(); x=zeros(21,1); x(4)=pi/4;
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(0,x,[],m);
assert(s.phase=="ROLL" && ~c.propulsionEnabled);
[s,~]=ROLL_TURN_SURGE_SUPERVISOR(.25,x,s,m); assert(s.phase=="ROLL");
bad=x; bad(4)=0; [s,~]=ROLL_TURN_SURGE_SUPERVISOR(.4,bad,s,m);
assert(isnan(s.qualifiedSince));
[s,~]=ROLL_TURN_SURGE_SUPERVISOR(.5,x,s,m);
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(1,x,s,m);
assert(s.phase=="TURN" && ~c.propulsionEnabled && norm(c.heading-[1;0;0])<eps);
% Exact target attitude: R=Rx(plane)*Rz(in-plane turn).
p=pi/4; a=pi/4;
R=[1 0 0;0 cos(p) -sin(p);0 sin(p) cos(p)]*[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
x(4:6)=[atan2(R(3,2),R(3,3));asin(-R(3,1));atan2(R(2,1),R(1,1))];
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(24,x,s,m);
assert(s.phase=="TURN" && ~c.propulsionEnabled); % reference not yet complete
[s,~]=ROLL_TURN_SURGE_SUPERVISOR(25,x,s,m);
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(25.5,x,s,m); assert(~c.propulsionEnabled);
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(26,x,s,m);
assert(s.phase=="SURGE" && c.propulsionEnabled && c.speedReference==0 && s.hasSurged);
[completed,cc]=ROLL_TURN_SURGE_SUPERVISOR(56,x,s,m);
assert(completed.phase=="COMPLETE" && cc.propulsionEnabled && cc.speedReference==m.surgeSpeed);
bad=x; bad(6)=bad(6)+deg2rad(10);
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(27,bad,s,m);
assert(s.phase=="ABORT" && ~c.propulsionEnabled && s.hasSurged);
m.rollTimeout=.1;
[s,~]=ROLL_TURN_SURGE_SUPERVISOR(0,zeros(21,1),[],m);
[s,c]=ROLL_TURN_SURGE_SUPERVISOR(.2,zeros(21,1),s,m);
assert(s.phase=="ABORT" && ~c.propulsionEnabled);
speed=m.speed; speed.effectiveMass=33;
[f,I,d]=SURGE_SPEED_CONTROL(0,3,0,0,.05,speed,10);
assert(f==10 && I==0 && d.saturated);
[f,I,d]=SURGE_SPEED_CONTROL(2,0,0,0,.05,speed,10);
assert(f==0 && I==0 && d.saturated);
[f,I,d]=SURGE_SPEED_CONTROL(0,.5,0,0,.05,speed,10);
assert(f>0 && f<10 && I>0 && ~d.saturated);
fprintf('Mission supervisor and surge PI unit checks passed.\n');
end
