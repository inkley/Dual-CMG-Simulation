function g=FLYWHEEL_GEOMETRY(radius,thickness,density,tubeDiameter)
% Solid cylindrical rotor, SI units. Rotor-only conservative swept envelope.
assert(all(isfinite([radius,thickness,density,tubeDiameter])) ...
    && all([radius,thickness,density,tubeDiameter]>0));
g.r=radius; g.t=thickness; g.rho=density;
g.v=pi*radius^2*thickness; g.m=g.v*density;
g.I=.5*g.m*radius^2;
g.Itransverse=g.m*(3*radius^2+thickness^2)/12;
% Bounding sphere accommodates all rotor orientations about its centered
% gimbal pivot. It does not include shaft/frame/motor/housing or wall thickness.
g.sweptDiameter=2*sqrt(radius^2+(thickness/2)^2);
g.radialEnvelopeRemainder=(tubeDiameter-g.sweptDiameter)/2;
g.rotorOnlyEnvelopeFits=g.sweptDiameter<=tubeDiameter;
end
