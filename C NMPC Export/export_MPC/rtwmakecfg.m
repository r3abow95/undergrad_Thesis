function makeInfo = rtwmakecfg()
%RTWMAKECFG adds include and source directories to the make files.
%    makeInfo=
%    Simulink version    : 8.9 (R2017a) 16-Feb-2017
%    MATLAB file generated on : 07-Nov-2017 09:31:31

% Verify the Simulink version
verify_simulink_version();

% Get the current directory
currDir = pwd;

% Get the MATLAB search path, remove the toolbox sub-directories except LCT demos
matlabPath = legacycode.lct.util.getSearchPath();

% Declare cell arrays for storing the paths
allIncPaths = {};
allSrcPaths = {};

% Get the build type
isSimTarget = is_simulation_target();
% Get the serialized paths information
info = get_serialized_info();

% Get all S-Function's name in the current model
sfunNames = {};
if ~isempty(bdroot)
    sfunBlks = find_system(bdroot,...
        'LookUnderMasks', 'all',...
        'FollowLinks', 'on',...
        'BlockType', 'S-Function'...
    );
    sfunNames = get_param(sfunBlks, 'FunctionName');
end

for ii = 1:numel(info)
    sfunInfo = info(ii);
    % If the S-Function isn't part of the current build then skip its path info
    if ~any(strcmp(sfunInfo.SFunctionName, sfunNames))
        continue
    end

    % Call the helper
    pathInfo = legacycode.lct.util.resolvePaths(sfunInfo, isSimTarget==1, currDir, matlabPath);

    % Concatenate known include and source directories
    allIncPaths = RTW.uniquePath([allIncPaths pathInfo.IncPaths(:)']);
    allSrcPaths = RTW.uniquePath([allSrcPaths pathInfo.SrcPaths(:)']);
end

% Additional include directories
makeInfo.includePath = correct_path_name(allIncPaths);

% Additional source directories
makeInfo.sourcePath = correct_path_name(allSrcPaths);

%% ------------------------------------------------------------------------
function info = get_serialized_info()

% Allocate the output structure array
info(1:1) = struct(...
    'SFunctionName', '',...
    'IncPaths', {{}},...
    'SrcPaths', {{}},...
    'LibPaths', {{}},...
    'SourceFiles', {{}},...
    'HostLibFiles', {{}},...
    'TargetLibFiles', {{}},...
    'singleCPPMexFile', false,...
    'Language', ''...
    );

info(1).SFunctionName = 'acado_solver_sfun';
info(1).IncPaths = {'qpoases3', 'qpoases3/include', 'qpoases3/src'};
info(1).SourceFiles = {'acado_solver.c', 'acado_integrator.c', 'acado_auxiliary_functions.c', 'acado_solver_sfunction.c', 'acado_qpoases3_interface.c', 'qpoases3/src/Bounds.c', 'qpoases3/src/Constraints.c', 'qpoases3/src/Indexlist.c', 'qpoases3/src/Matrices.c', 'qpoases3/src/MessageHandling.c', 'qpoases3/src/Options.c', 'qpoases3/src/Flipper.c', 'qpoases3/src/QProblem.c', 'qpoases3/src/QProblemB.c', 'qpoases3/src/Utils.c'};
info(1).Language = 'C';

%% ------------------------------------------------------------------------
function verify_simulink_version()

% Retrieve Simulink version
slVer = ver('simulink');
factor = 1.0;
thisVer = 0.0;
for ii = 1:numel(slVer.Version)
    if slVer.Version(ii)=='.'
        factor = factor/10.0;
    else
        thisVer = thisVer + sscanf(slVer.Version(ii), '%d')*factor;
    end
end

% Verify that the actual platform supports the function used
if thisVer < 8.7
    DAStudio.error('Simulink:tools:LCTErrorBadSimulinkVersion', sprintf('%g',thisVer))
end

%% ------------------------------------------------------------------------
function pathSet = correct_path_name(pathSet)

for ii = 1:numel(pathSet)
    pathSet{ii} = RTW.transformPaths(pathSet{ii});
end
pathSet = RTW.uniquePath(pathSet);

%% ------------------------------------------------------------------------
function isSimTarget = is_simulation_target()

% Default output value
isSimTarget = 0;

% Get the current model and the code generation type to decide
% if we must link with the host libraries or with the target libraries
try
    modelName = get_param(0, 'CurrentSystem');
    if ~isempty(modelName)
        modelName = bdroot(modelName);
        sysTarget = get_param(modelName, 'RTWSystemTargetFile');
        isSimTarget = ~isempty([strfind(sysTarget, 'rtwsfcn') strfind(sysTarget, 'accel')]);

        mdlRefSimTarget = get_param(modelName,'ModelReferenceTargetType');
        isSimTarget = strcmpi(mdlRefSimTarget, 'SIM') || isSimTarget;

        % Verify again it's not Accelerator
        if ~isSimTarget
            simMode = get_param(modelName, 'SimulationMode');
            simStat = get_param(modelName, 'SimulationStatus');
            isSimTarget = strcmp(simStat, 'initializing') & strcmp(simMode, 'accelerator');
        end
    end
catch
end
