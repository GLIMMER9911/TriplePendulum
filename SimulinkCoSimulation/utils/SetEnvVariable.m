% Copyright 2018 The MathWorks, Inc.

%% Add the ZMQ DLL to the system path:
p = simulinkproject;
ud_Tri_parameter;
if strcmp(mexext,'mexw64')
    disp(p.RootFolder);
    if exist(fullfile(p.RootFolder,'libzmq','bin','libzmq-v142-mt-4_3_4.dll'),'file')
        
        pathEnvVar = getenv('PATH');
        setenv('PATH',[pathEnvVar ';' fullfile(p.RootFolder,'lib')]);
        
        disp('libzmq-v142-mt-4_3_4.dll added to system PATH')
    else
        disp('libzmq-v142-mt-4_3_4.dll not found, build libzmq.dll before adding to the system path. See readme.mlx for more details.')
    end
else
    error('Non-Windows OS, please add the ZMQ library to your PATH environment variable manually');
end