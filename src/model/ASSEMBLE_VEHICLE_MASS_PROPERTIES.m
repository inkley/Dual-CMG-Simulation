function [totalMass, totalCG, totalInertia] = ...
        ASSEMBLE_VEHICLE_MASS_PROPERTIES(baseMass, baseCG, baseInertia, ...
        moduleMasses, modulePositions, moduleInertias)
%ASSEMBLE_VEHICLE_MASS_PROPERTIES Add installed modules consistently.
% baseInertia and each moduleInertias(:,:,i) are centroidal body-axis
% tensors. modulePositions are body-axis locations relative to the original
% vehicle origin. The returned tensor is about the combined center of mass.

moduleMasses = moduleMasses(:);
if size(modulePositions,1) ~= numel(moduleMasses) ...
        || size(modulePositions,2) ~= 3 ...
        || size(moduleInertias,3) ~= numel(moduleMasses)
    error('CMG:InvalidMassPropertyInput', ...
        'Module masses, positions, and inertia tensors must have equal counts.');
end

totalMass = baseMass+sum(moduleMasses);
totalCG = (baseMass*baseCG(:) ...
    + modulePositions.'*moduleMasses)/totalMass;
totalInertia = shiftInertia(baseInertia, baseMass, baseCG(:)-totalCG);
for module = 1:numel(moduleMasses)
    offset = modulePositions(module,:).'-totalCG;
    totalInertia = totalInertia+shiftInertia( ...
        moduleInertias(:,:,module), moduleMasses(module), offset);
end
totalInertia = 0.5*(totalInertia+totalInertia.');
end

function shifted = shiftInertia(centroidal, mass, offset)
    shifted = centroidal+mass*((offset.'*offset)*eye(3)-offset*offset.');
end
