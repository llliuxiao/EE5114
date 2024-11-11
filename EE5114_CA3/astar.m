%% astar planner
function [path] = astar(map, start, goal)
    x_max = size(map, 1); y_max = size(map, 2);
    nodes(x_max, y_max) = struct();

    % Check firsly
    if (map(start(1), start(2)) == 1) || (map(goal(1), goal(2)) == 1)
        path = [];
        return;
    end

    % Initilize the node matrix
    for i = 1 : x_max
        for j = 1 : y_max
            nodes(i, j).coordinate = [i, j];
            nodes(i, j).distance = inf;
            nodes(i, j).parent_node = [NaN, NaN];
            nodes(i, j).neighbors = 0;
            nodes(i, j).in_openset = false;
            nodes(i, j).in_closeset = (map(i, j) == 1);
            nodes(i, j).priority = inf;
            nodes(i, j).neighbor_coordinates = [];
            if i + 1 <= x_max
                nodes(i, j).neighbors = nodes(i, j).neighbors + 1;
                nodes(i, j).neighbor_coordinates(nodes(i, j).neighbors, :) = [i + 1, j];
            end
            if i - 1 >= 1
                nodes(i, j).neighbors = nodes(i, j).neighbors + 1;
                nodes(i, j).neighbor_coordinates(nodes(i, j).neighbors, :) = [i - 1, j];
            end
            if j + 1 <= y_max
                nodes(i, j).neighbors = nodes(i, j).neighbors + 1;
                nodes(i, j).neighbor_coordinates(nodes(i, j).neighbors, :) = [i, j + 1];
            end
            if j - 1 >= 1
                nodes(i, j).neighbors = nodes(i, j).neighbors + 1;
                nodes(i, j).neighbor_coordinates(nodes(i, j).neighbors, :) = [i, j - 1];
            end
        end
    end

    nodes(start(1), start(2)).priority = 0;
    nodes(start(1), start(2)).distance = 0;
    openset(1) = nodes(start(1), start(2));
    
    % astar search
    while ~isempty(openset)
        [node, openset] = heap_pop(openset, "priority");
        if node.in_closeset
            continue
        end
        nodes(node.coordinate(1), node.coordinate(2)).in_openset = false; 
        nodes(node.coordinate(1), node.coordinate(2)).in_closeset = true;
        if isequal(goal, node.coordinate)
            break;
        end
        for i = 1 : node.neighbors
            coordinate = node.neighbor_coordinates(i, :);
            if nodes(coordinate(1), coordinate(2)).in_closeset
                continue
            end
            if node.distance + 1 < nodes(coordinate(1), coordinate(2)).distance
                nodes(coordinate(1), coordinate(2)).parent_node = node.coordinate;
                h_n = abs(coordinate(1) - goal(1)) + abs(coordinate(2) - goal(2));
                nodes(coordinate(1), coordinate(2)).distance = node.distance + 1;
                nodes(coordinate(1), coordinate(2)).priority = h_n + node.distance + 1;
                nodes(coordinate(1), coordinate(2)).in_openset = true;
                openset = heap_push(openset, nodes(coordinate(1), coordinate(2)), "priority");
            end
        end
    end

    % Not found the path
    if ~nodes(goal(1), goal(2)).in_closeset
        path = [];
        return;
    end

    % Derive the path
    current_node = nodes(goal(1), goal(2));
    path = [goal(1), goal(2)];
    while true
        if isequal(current_node.coordinate, start)
            break;
        end
        current_node = nodes(current_node.parent_node(1), current_node.parent_node(2));
        path = [current_node.coordinate; path];
    end
end