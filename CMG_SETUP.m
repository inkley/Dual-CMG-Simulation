function root=CMG_SETUP
%CMG_SETUP Add only project code directories; no results/archive dependencies.
% Run once after changing to this folder in an existing MATLAB session.
% AUV_SIM invokes this automatically. No settings or saved path are modified.
root=fileparts(mfilename('fullpath'));
addpath(root);
folders={'config','src','studies','tests'};
for k=1:numel(folders)
    addpath(genpath(fullfile(root,folders{k})));
end
end
