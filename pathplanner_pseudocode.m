
function path_finder(waypoints, init_triplet, environment, max_dist)
    final_path = []    % List of triplets used to get from A to B, along the waypoints in the desired path
    
    % Go through each line segment that the waypoints constitute
    for i in range(1,len(waypoints)):
       % Find the triplets that give the shortest path 
       final_path.extend(path_to_wp(waypoints[i-1], waypoints[i], init_triplet, env)
    end
    return final_path
    
function path_to_wp(previous_wp, wp, init_triplet, environment, max_dist)
    path = deque()   % Deque of triplets used to get the snake from the previous waypoint to the next waypoint
    insert init_triplet into path
    current_triplet = init_triplet
    while(distance_to_wp(snake_head, wp) > max_dist)
        % Use A*-search to iteratively find the next obstacles used in the locomotion,
        % and insert them into the next triplet. A*-search here returns the
        % next triplet
        current_triplet = A*-search(current_triplet, previous_wp, wp, environment) % Project ahead
        path.append(current_triplet)
    return list(path)
        