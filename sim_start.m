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
dt = 3.6;%3.6;

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
    actionqueue = behaviour(t, gpsposition);
    
    %%%%%%%%%%%%%%%%%%%%output%%%%%%%%%%%%%%%%%%%%%%%%%%
    % apply any movement
    position = move(dt, position, actionqueue);
    
    if position(1) > maxposition
        maxposition = position(1);
    end
    
    if position(1) < minposition
        minposition = position(1);
    end
    
    maxmin = maxposition - minposition
    
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

function actionqueue = behaviour(t, gpsposition)
% simulate behaviour of uav

actionqueue = [];

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

if isempty(actionqueue)
    actionqueue = [10; 0.005; 5];
end


function gpsposition = checkPosition(position)
% simulate measurement of agent position from gps with error margin +/- 3m

% position plus error
gpsposition = position(1:2) + randi([-3 3], 2, 1);


function moved = move(dt, position, actionqueue)
% simulate movement and restrictions
v = actionqueue(1);
u = actionqueue(2);
theta = position(3);

% restricts speed to uav limits
if v<10
    v = 10;
elseif v>20
    v = 20;
end
% restricts turn rate to uav limits
if u < -0.1 * dt
    u = -0.1 * dt;
elseif u > 0.1 * dt
    u = 0.1 * dt;
end

positiondot = [v * sin(theta) * dt; v * cos(theta) * dt; v * u * dt];



moved = position + positiondot;

