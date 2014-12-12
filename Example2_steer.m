%% Examples of single calls to steer
% We hereby show steering from state to state, which requires the "extraction"
% of flat outputs and derivatives from the initial and final states. 
% Since the flatness map (Psi_x) is not invertible, we use a numerical procedure
% started with random intial guesses, and the results may differ from the 
% ones shown in the writeup.

%% Create Car Object 
% (skipped if a good C is already available on the workspace)
if(exist('C'))
    if(C.N~=2)
        disp('This example requires a car with 2 trailers, deleting old C.')
        clear C;
    end
end
if(~exist('C'))
    % takes approx. 40 sec 
    C = CarWithNTrailers(2, [1, 2, 3]);
end

%% Example of steer rotating 90°
traj_rot = C.steerState2state([0,0.5,0,0,0,0]', [4,0.5,0,pi/2,pi/2, pi/2]', 1);
traj_rot.playback(1:3, [], 20);

%% Example of steering to symmetric configuration
traj_sym = C.steerState2state([0,0,0,pi/8,-pi/4,0]', [[0,0,0,-pi/8,pi/4,0]'], 1);
traj_sym.playback(1:3, [], 20);