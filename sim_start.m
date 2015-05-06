function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 1;%3.6;

% initiate drone
position = [0; 0; 0]; % x, y, theta (in radians)

defaultaction = [15; 0; 1]; % v, u, length of time (in seconds)


minposition=0;
maxposition=0;

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;



    % take measurement
    p = cloudsamp(cloud,position(1),position(2),t);
    
    gpsposition = checkPosition(position);
    action = behaviour(t, p, gpsposition);
    
    %%%%%%%%%%%%%%%%%%%%output%%%%%%%%%%%%%%%%%%%%%%%%%%
    % apply any movement
    position = move(dt, position, action);
    
    if position(1) > maxposition
        maxposition = position(1);
    end
    
    if position(1) < minposition
        minposition = position(1);
    end
    
    maxmin = maxposition - minposition;
    
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t, position(1),position(2),p))
        
    % plot robot location
    plot(position(1),position(2),'o')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function action = behaviour(t, p, gpsposition)
% simulate behaviour of uav

action = [];

if p < 0.1
    action = logspiral(t, gpsposition);
else
    action = trackcloud(p, gpsposition);
end
% if gpsposition(2) > 200
%     actionqueue = [actionqueue [15; 90; 5]];
% end
% 
% if gpsposition(2) < - 200
%     actionqueue = [actionqueue [15; 90; 5]];
% end

% if mod(t,8) == 0
%     actionqueue = [actionqueue [15; 90; 5]];
% end





if isempty(action)
    action = [10; 0.1];
end


function action = trackcloud(p, gpsposition)
% logarithmic spiral behaviour

if p > 1
    action = [20;-.01]
else
    action = [20;.01]
end





function action = logspiral(t, gpsposition)
% logarithmic spiral behaviour

action = [10;.05 - log(t)*.01]

% if t>100
%     action = [10;.05 - log(t)*.01 + .001]
% end





function gpsposition = checkPosition(position)
% simulate measurement of agent position from gps with error margin +/- 3m

% position plus error
gpsposition = position(1:2) + randi([-3 3], 2, 1);


function moved = move(dt, position, action)
% simulate movement and restrictions
v = action(1);
u = action(2);
theta = position(3);

% restricts speed to uav limits
if v < 10
    v = 10;
elseif v > 20
    v = 20;
end
% restricts turn rate to uav limits
if u < -0.1
    u = -0.1;
elseif u > 0.1
    u = 0.1;
end

positiondot = [v * sin(theta) * dt; v * cos(theta) * dt; v * u * dt];



moved = position + positiondot;

