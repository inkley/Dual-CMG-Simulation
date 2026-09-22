function VERIFY_REPOSITORY_LAYOUT
root=CMG_ROOT;
assert(isfolder(fullfile(root,'Working Results')));
inventory=jsondecode(fileread(fullfile(root,'docs','file-map.json')));
for k=1:numel(inventory)
    if iscell(inventory), item=inventory{k}; else, item=inventory(k); end
    target=fullfile(root,item.new);
    assert(isfile(target),'Missing moved file: %s',target);
    [~,name,extension]=fileparts(target);
    if strcmp(extension,'.m')
        assert(strcmp(which(name),target),'Wrong MATLAB path resolution for %s',name);
    end
end
original=pwd; cleanup=onCleanup(@() cd(original));
cd(fullfile(root,'studies','disturbance'));
assert(strcmp(CMG_ROOT,root));
assert(isfile(fullfile(CMG_ROOT,'Working Results','dual','symmetric_spin','VFR','simulation_result.mat')));
fprintf('All moved files resolve; nested-folder results lookup is unchanged.\n');
end
