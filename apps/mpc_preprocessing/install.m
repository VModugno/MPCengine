%% add path to matlab path
disp('installing mpcEngine toolbox');
mpcEnginePath = fileparts( mfilename('fullpath') );
addpath(mpcEnginePath);
savepath;
disp('installion mpcEngine toolbox complete!');

