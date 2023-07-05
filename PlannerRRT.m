%% Planner RRT

close all
clear all
clc

res = 0.25; % Resolution in meters of discrete map
meshres = 0.1; % Resolution in meters of meshes
mapres = 4; % Resoultion in cells per meter of occupancy map
x_size = 20/res;
y_size = 20/res;
z_size = 10/res;

map = zeros(x_size, y_size, z_size) + 1;

for i = 1:x_size
    for j = 1:z_size
        if (i ~= 9 && i ~= 10 && i ~= 11) ||  (j ~= 4 && j ~= 5 && j ~= 6)
            map(i, 10, j) = 0;
            map(i, 9, j) = 0;
            map(i, 11, j) = 0;
        end
    end
end

% Create environment through OccupancyMap

Env3D = occupancyMap3D(mapres);
mapW = x_size*res;  % Environment sizes
mapL = y_size*res;
mapH = z_size*res;
W=2;    % Wall parameters
L=6;
H=2;
l = 0.215; % Radius of the drone
xp=(mapW - W)/2;
yp=0;

% Holed wall obstacle parts as meshes
[xo_1,yo_1,zo_1]=meshgrid(xp:meshres:xp+W, yp:meshres:mapL, 0:meshres:H);
[xo_2,yo_2,zo_2]=meshgrid(xp:meshres:xp+W, yp:meshres:yp+L, H:meshres:(mapH - H));
[xo_3,yo_3,zo_3]=meshgrid(xp:meshres:xp+W, (mapL-L):meshres:mapL, H:meshres:(mapH - H));
[xo_4,yo_4,zo_4]=meshgrid(xp:meshres:xp+W, yp:meshres:mapL, (mapH-H):meshres:mapH);
% Cuboid obstacle
[xo_5,yo_5,zo_5]=meshgrid(2:meshres:7, 1:meshres:3, 0:meshres:5);

% Cylindrical (pipe) obstacle
r = 1;
[xo_6, yo_6, zo_6]=cylinder(r,50);
xo_6(:)=xo_6(:)+5;
yo_6(:)=yo_6(:)+10;
xo_t=xo_6(2,:);
yo_t=yo_6(2,:);
zo_t=zo_6(2,:);
xo_6=xo_6(1,:);
yo_6=yo_6(1,:);
zo_6=zo_6(1,:);
for i= meshres:meshres:mapH
    xo_6 = [xo_6;xo_t];
    yo_6 = [yo_6;yo_t];
    zo_6 = [zo_6;zo_t*i];
end

% creation of the OMap
Obstacle_1=[xo_1(:) yo_1(:) zo_1(:)];
Obstacle_2=[xo_2(:) yo_2(:) zo_2(:)];
Obstacle_3=[xo_3(:) yo_3(:) zo_3(:)];
Obstacle_4=[xo_4(:) yo_4(:) zo_4(:)];
Obstacle_5=[xo_5(:) yo_5(:) zo_5(:)];
Obstacle_6=[xo_6(:) yo_6(:) zo_6(:)];

setOccupancy(Env3D,Obstacle_1,1)
setOccupancy(Env3D,Obstacle_2,1)
setOccupancy(Env3D,Obstacle_3,1)
setOccupancy(Env3D,Obstacle_4,1)
setOccupancy(Env3D,Obstacle_5,1)
setOccupancy(Env3D,Obstacle_6,1)
[xG,yG,zG] = meshgrid(0:meshres:mapW,0:meshres:mapL,-1:meshres:0-meshres);
Ground = [xG(:) yG(:) zG(:)];
inflate(Env3D,l)
setOccupancy(Env3D,Ground,1)
figure(1)
show(Env3D)
axis equal
hold on 

% Matrix creation for the algorithm
map = zeros(x_size, y_size, z_size);
for i = 1:x_size
    for j = 1:y_size
        for k = 1:z_size
            if checkOccupancy(Env3D, [i*res, j*res, k*res]) ~= 1
                map(i, j, k) = 1;
            end
        end
    end
end

q_i = [1/res 2/res 3/res];
q_f = [18/res 18/res 3/res];

%% show map
% 
% D = gpuDevice;
% figure(2)
% plot_map = gpuArray(map);
% for i = 1:x_size
%     for j = 1:y_size
%         for k = 1:z_size
%             if plot_map(i, j, k) == 0
%                 plot3(i, j, k, 'k.-', 'MarkerSize',30, 'LineWidth', 20);
%                 hold on
%             end
%         end
%     end
% end
% reset(D)
% clear plot_map

%% DEBUG
%rng(1)

delta = 5/res;
Iterations = 25; 
increaseIterations = 5;
trial = 0;
maxTries = 5;
endAlgorithm = false;

roadmap = [q_i; q_f];
connections = [];

% belongs_to_subtree knows which nodes are connected to q_i and which ones
% are connected to q_f, to make it easier to try to connect the two
% subtrees after the points have been extracted
belongs_to_subtree = [1 2];

i = 0;

while endAlgorithm == false && trial < maxTries 

    trial = trial + 1;
    
    while i < Iterations-increaseIterations+increaseIterations*trial
        
        % extract q_rand
        q_rand = [randi([1 x_size]), randi([1 y_size]), randi([1, z_size])];
    
        % find q_near
        q_near = q_i;
        index_near = 1;
        collision = false;
        for j = 2:length(roadmap(:, 1))
            if sqrt((q_rand(1)-roadmap(j, 1))^2+(q_rand(2)-roadmap(j, 2))^2+(q_rand(3)-roadmap(j, 3))^2) < sqrt((q_rand(1)-q_near(1))^2+(q_rand(2)-q_near(2))^2+(q_rand(3)-q_near(3))^2) 
                q_near = roadmap(j, :);
                index_near = j;
            end
        end
    
        % build the line between q_near and q_rand, pick q_new, check for
        % collisions on the segment between q_near and q_new. If no collisions 
        % add the point q_new and the segment to the roadmap
        u = [q_rand(1)-q_near(1), q_rand(2)-q_near(2), q_rand(3)-q_near(3)];
        if norm(u) == 0
            collision = true;
        end

        if collision == false
            u = u/norm(u);
            q_new = q_near+delta*u;
            
            % make sure q_new is an integer
            q_new = [ceil(q_new(1)) ceil(q_new(2)) ceil(q_new(3))];
    
            % if q_new is out of bounds we discard it and assume a collision
            if q_new(1) < 1 || q_new(1) > x_size || q_new(2) < 1 || q_new(2) > y_size || q_new(3) < 1 || q_new(3) > z_size
                collision = true;
            end
               
            j = 0;
        end
    
        while collision == false && j < delta - meshres
            j = j+meshres;
            q_current = [q_near(1)+j*u(1), q_near(2)+j*u(2), q_near(3)+j*u(3)];
            if map(ceil(q_current(1)), ceil(q_current(2)), ceil(q_current(3))) == 0 
                collision = true;
                %disp("Collision!")
            end
        end
    
        if collision == false
            roadmap = [roadmap; q_new];
            connections = [connections; [length(roadmap(:, 1)) index_near]];
            %disp("Adding!")
            belongs_to_subtree = [belongs_to_subtree belongs_to_subtree(index_near)];
        end
    
        i = i+1;
    end
    
    % find the two closest nodes in the two subtrees
    minDistance = x_size*y_size*z_size; % impossibly high value to initialize the algorithm
    indexA = 1;
    indexB = 2;
    for i = 1:length(belongs_to_subtree)
        if belongs_to_subtree(i) == 1
            for j = 1:length(belongs_to_subtree)
                if belongs_to_subtree(j) == 2
                    if sqrt((roadmap(i, 1)-roadmap(j, 1))^2+(roadmap(i, 2)-roadmap(j, 2))^2+(roadmap(i, 3)-roadmap(j, 3))^2) < minDistance
                        indexA = i;
                        indexB = j;
                        minDistance = sqrt((roadmap(i, 1)-roadmap(j, 1))^2+(roadmap(i, 2)-roadmap(j, 2))^2+(roadmap(i, 3)-roadmap(j, 3))^2);
                    end
                end
            end
        end
    end
    
    % try to connect the two closest nodes
    collision_connecting_trees = false;
    treesDistance = sqrt((roadmap(indexA, 1)-roadmap(indexB, 1))^2+(roadmap(indexA, 2)-roadmap(indexB, 2))^2+(roadmap(indexA, 3)-roadmap(indexB, 3))^2);
    j = 0;
    u_connect_trees = [roadmap(indexB, 1)-roadmap(indexA, 1), roadmap(indexB, 2)-roadmap(indexA, 2), roadmap(indexB, 3)-roadmap(indexA, 3)];
    u_connect_trees = u_connect_trees/norm(u_connect_trees);
    while collision_connecting_trees == false && j < treesDistance - meshres
        j = j+meshres;
        q_current = [roadmap(indexA, 1)+j*u_connect_trees(1), roadmap(indexA, 2)+j*u_connect_trees(2), roadmap(indexA, 3)+j*u_connect_trees(3)];
        if map(ceil(q_current(1)), ceil(q_current(2)), ceil(q_current(3))) == 0 
            collision_connecting_trees = true;
            %disp("Collision!")
        end
    end
    
    % plot the roadmap 

    figure(1)
    hold on

    roadmap = roadmap*res;
    
    plot3(roadmap(1, 1), roadmap(1, 2), roadmap(1, 3), 'r.', 'MarkerSize', 15);
    hold on
    text(roadmap(1, 1), roadmap(1, 2), roadmap(1, 3), string(1))
    plot3(roadmap(2, 1), roadmap(2, 2), roadmap(2, 3), 'r.', 'MarkerSize', 15);
    hold on
    text(roadmap(2, 1), roadmap(2, 2), roadmap(2, 3), string(2))

    for i = 3:length(roadmap(:, 1))
        plot3(roadmap(i, 1), roadmap(i, 2), roadmap(i, 3), 'g.', 'MarkerSize', 15);
        text(roadmap(i, 1), roadmap(i, 2), roadmap(i, 3), string(i))
        hold on
    end
    
    for i = 1:length(connections(:, 1))
        plot3([roadmap(connections(i, 1), 1) roadmap(connections(i, 2), 1)], [roadmap(connections(i, 1), 2) roadmap(connections(i, 2), 2)], [roadmap(connections(i, 1), 3) roadmap(connections(i, 2), 3)], 'k-');
        hold on
    end

    if collision_connecting_trees == false
        % plot the final connection 
        plot3([roadmap(indexA, 1) roadmap(indexB, 1)], [roadmap(indexA, 2) roadmap(indexB, 2)], [roadmap(indexA, 3) roadmap(indexB, 3)], 'b-');
        hold off
        endAlgorithm = true;
        trial
        
        % generate the textual path from q_i to q_f
        path = [roadmap(indexA, :)];
        while path(1, 1) ~= q_i(1)*res ||  path(1, 2) ~= q_i(2)*res ||  path(1, 3) ~= q_i(3)*res
            [LIA,LOCB] = ismember(roadmap, path(1, :),'rows');
            indexPath = find(LOCB == 1);
            if(indexPath > 2)
                path = [roadmap(connections(indexPath-2, 2), :); path];
            else
                break
            end
        end
        path = [path; roadmap(indexB, :)];
        while path(end, 1) ~= q_f(1)*res ||  path(end, 2) ~= q_f(2)*res ||  path(end, 3) ~= q_f(3)*res
            [LIA,LOCB] = ismember(roadmap, path(end, :),'rows');
            indexPath = find(LOCB == 1);
            if indexPath > 2
                path = [path; roadmap(connections(indexPath-2, 2), :)];
            else
                break
            end
        end
        path
        path(:, 3)=-path(:, 3); % Because of NED configuration
    else
        hold off
    end

    if collision_connecting_trees == true
        disp("Failure while connecting the trees, increasing the iteration number!");
        roadmap = roadmap/res;
    end
end


if trial == maxTries && collision_connecting_trees == true
    disp("Too many tries, total failure!")
end


