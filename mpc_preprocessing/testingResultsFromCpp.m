clear variables 
close all 
clc


% open file inside @log

state  = load('state.mat','-ascii');
action = load('action.mat','-ascii');

% plot using the enviroment 
delta_t    = 0.1;
init_state = [0; 0; pi/10; 0];
reward     = @(x,u)(norm(x)); % dummy reward
env        = Env.CartPole(init_state,delta_t,reward);

env.Render();

for ii=1:size(state,1)
    
    env.UpdateRender(state(ii,:)');
    pause(30/1000)
    
end