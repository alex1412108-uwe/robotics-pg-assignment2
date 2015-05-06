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

target = [500;500];

lastposition = [0;0];

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
    orientation = angletopoint(lastposition,gpsposition);
    
    action = behaviour(t, p, gpsposition, orientation);
    
    %action = navigation(t, lastposition, orientation, position, gpsposition, target);
    
    %%%%%%%%%%%%%%%%%%%%output%%%%%%%%%%%%%%%%%%%%%%%%%%
    % save old gps
    lastposition = gpsposition;
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

%%%%%%%%%%%%%%%%%%%%%%%behaviours%%%%%%%%%%%%%%%%%%%%%%%

function action = behaviour(t, p, gpsposition)
% simulate behaviour of uav

action = [];

if p < 0.1
    action = navigation(t, lastposition, orientation, position, gpsposition, target);
else
    action = trackedgecloud(p, gpsposition);
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


function action = trackedgecloud(p, gpsposition)
% edge follow cloud behaviour

if p > 1
    action = [20;-.01]
else
    action = [20;.01]
end





function action = logspiral(t, gpsposition)
% logarithmic spiral behaviour

action = [10;.05 - log(t)*.01];

% if t>100
%     action = [10;.05 - log(t)*.01 + .001]
% end

%%%%%%%%%%%%%%%%%%%%%%%navigation%%%%%%%%%%%%%%%%%%%%%%

function action = navigation(t, lastposition, orientation, position, gpsposition, target)
% abstract navigation
action = [10;0.1];

angletotarget = angletopoint(gpsposition,target);

orientationtotarget = angletotarget - orientation;

a = target(2)-gpsposition(2);
b = target(1)-gpsposition(1);

distancetotarget = sqrt((a^2)+(b^2));

if orientationtotarget > 0.001
    action(2) = 0.01*orientationtotarget;
elseif orientationtotarget < -0.001
    action(2) = -0.01*orientationtotarget;
else
    action(2) = 0;
end

if distancetotarget > 100
    action(1) = 20;
else
    action(1) = 10;
end

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

function gpsposition = checkPosition(position)
% simulate measurement of agent position from gps with error margin +/- 3m

% position plus error
%gpsposition = position(1:2) + randi([-3 3], 2, 1); % equal distribution
gpsposition = position(1:2) + normrnd(0,3/2.5, 2, 1); % normal distribution

function angle = angletopoint(p1,p2)
% measures angle from a point to another point
opposite = p2(2)-p1(2);
adjacent = p2(1)-p1(1);
%angle = atan(opposite/adjacent); %issues around 0 orientation
angle = atan2(adjacent,opposite);