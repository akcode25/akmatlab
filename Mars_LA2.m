
% Mars Rover Path Planning using A* Search Algorithm

clc; clear; close all;

%% Parameters and Map Setup

grid_size = 20;         % Grid size (20x20)
start_node = [1, 1];    % Start position (row, col) (top-left)
goal_node = [20, 20];   % Goal position (row, col) (bottom-right)

% Generate obstacle map (1 = obstacle, 0 = free space)

obstacle_map = zeros(grid_size);
rng(1);             % For reproducibility
num_obstacles = 80; % Number of obstacles

% Randomly place obstacles

for k = 1:num_obstacles

    obs_x = randi(grid_size); % random row index bw [1,grid_size]
    obs_y = randi(grid_size); % random col index

    if ~isequal([obs_x, obs_y], start_node) && ~isequal([obs_x, obs_y], goal_node)
        obstacle_map(obs_x, obs_y) = 1; %Ensure obstacles not plcaced at start or goal nodes.
    end

end

% Display initial map

figure;
imagesc(obstacle_map); 
colormap(gray); % 1 → black, 0 → white
hold on;        % prevent overwriting

plot(start_node(2), start_node(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start
plot(goal_node(2), goal_node(1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);   % Goal

title('Mars Rover Grid Terrain with Obstacles');
xlabel('X (Columns)'); 
ylabel('Y (Rows)');

grid on;


%% A* Search Algorithm Implementation

% Define movements (8 possible directions)

% Defines 8 possible movements (up, down, left, right, and diagonals) 
moves = [0 1; 1 0; 0 -1; -1 0; 1 1; -1 -1; -1 1; 1 -1]; 

% Respective costs. Diagonal moves cost sqrt(2) (Euclidean distance).
move_cost = [1 1 1 1 sqrt(2) sqrt(2) sqrt(2) sqrt(2)];

% Initialize open and closed lists

open_list = [];                 % Nodes to explore.
closed_list = zeros(grid_size); % Nodes already visited (0 = unvisited, 1 = visited)

% Cost matrices

g_cost = inf(grid_size);       % Cost from the start node to any node
h_cost = zeros(grid_size);     % Heuristic cost to the goal
f_cost = inf(grid_size);       % Total cost (g_cost + h_cost)

% Start node initialization

g_cost(start_node(1), start_node(2)) = 0;
f_cost(start_node(1), start_node(2)) = heuristic(start_node, goal_node);
% Adds the start node to open_list with its f_cost.
open_list = [start_node, f_cost(start_node(1), start_node(2))];

% Parent nodes for backtracking
parent = zeros(grid_size, grid_size, 2);

%% A* Algorithm Loop

while ~isempty(open_list)

    % Sort open list by f_cost
    % open_list contains rows of data for each node in the format [row, col, f_cost].

    % arranges the rows in ascending order of the third column (f_cost).
    open_list = sortrows(open_list, 3);
    % node with the lowest f_cost (best candidate) is processed next.

    current_node = open_list(1, 1:2);  % [row, col]
    open_list(1, :) = []; % Remove current processed node from open list
    
    % Mark current node as visited
    closed_list(current_node(1), current_node(2)) = 1;
    
    % Check if goal is reached
    if isequal(current_node, goal_node)
        fprintf('Goal Reached!\n');
        break;
    end
    
    % Explore neighbors
    for k = 1:size(moves, 1)

        % calculates the position of a neighboring node based on the current move
        neighbor = current_node + moves(k, :);
        
        % Check if neighbor is valid
        if is_valid(neighbor, grid_size, obstacle_map, closed_list)
            tentative_g_cost = g_cost(current_node(1), current_node(2)) + move_cost(k);
            
            % Update costs if better path found
            if tentative_g_cost < g_cost(neighbor(1), neighbor(2))

                % Updates parent of neighbor to the current_node
                parent(neighbor(1), neighbor(2), :) = current_node;

                % Updates g_cost of neighbor with the new (lower) cost
                g_cost(neighbor(1), neighbor(2)) = tentative_g_cost;

                % calculates, stores heuristic (h_cost) for the neighbor
                h_cost(neighbor(1), neighbor(2)) = heuristic(neighbor, goal_node);

                % Updates the total cost (f_cost) for the neighbor
                f_cost(neighbor(1), neighbor(2)) = g_cost(neighbor(1), neighbor(2)) + h_cost(neighbor(1), neighbor(2));
                
                % Add to open list

                % Checks if the neighbor is not already in the open_list.
                if isempty(open_list) || ~ismember(neighbor, open_list(:, 1:2), 'rows')
                    open_list = [open_list; neighbor, f_cost(neighbor(1), neighbor(2))];
                
                end
            end
        end
    end
end


%% Backtrack to Find Path

path = [];             % 1st col- y cordinate, 2nd col - x cordinate
current = goal_node;

while ~isequal(current, start_node)

    path = [current; path];
    current = squeeze(parent(current(1), current(2), :))';  % converts from 3D to 2D array

    if all(current == 0)  % No path exists  % if a valid parent node does not exist for the current node.
        error('No valid path found!');
    end

end
path = [start_node; path];


%% Plot Final Path

figure;
imagesc(obstacle_map); 
colormap(gray); 
hold on;

plot(start_node(2), start_node(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start
plot(goal_node(2), goal_node(1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);   % Goal

plot(path(:, 2), path(:, 1), 'b-', 'LineWidth', 2); % Path

title('Mars Rover Optimal Path');
xlabel('X (Columns)'); 
ylabel('Y (Rows)');
grid on;


%% Supporting Functions

function h = heuristic(node, goal)

    % Heuristic function: Euclidean distance
    h = sqrt((node(1) - goal(1))^2 + (node(2) - goal(2))^2);
end

function valid = is_valid(node, grid_size, obstacle_map, closed_list)

    % Check if node is within bounds, not an obstacle, and not visited
    valid = node(1) >= 1 && node(1) <= grid_size && ...   % row coordinate within grid bounds
            node(2) >= 1 && node(2) <= grid_size && ...   % col coordinate within grid bounds
            obstacle_map(node(1), node(2)) == 0 && ...    % no obstacle
            closed_list(node(1), node(2)) == 0;           % not already been visited
    % Combines all conditions using logical AND (&&) to determine if node valid for exploration
end
