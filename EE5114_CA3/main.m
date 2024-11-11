%% Clear and Load
close all; clear all; clc;
load 'Map.mat';
start = [1, 1];
goal = [10, 10];
%% Visualize
function [] = visualize(map, path, title_name)
    fig = figure;
    fig.Name = title_name;

    imagesc(map);
    colormap([1 1 1; 0 0 0]);
    colorbar off;
    axis equal tight;

    hold on;
    plot(path(:, 2), path(:, 1), 'r-', 'LineWidth', 2);  
    plot(path(:, 2), path(:, 1), 'ro', 'MarkerFaceColor', 'r');  
end
%% call dijkstra planner
path = dijkstra(Map, start, goal);
if size(path) > 0
    visualize(Map, path, "Dijkstra")
else
    disp("No Path Found for Dijkstra Planner!")
end

%% call a_star planner
path = astar(Map, start, goal);
if size(path) > 0
    visualize(Map, path, "AStar")
else
    disp("No Path Found for AStar Planner!")
end