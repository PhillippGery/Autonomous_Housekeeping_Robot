% =========================================================================
% == MAIN SCRIPT: OPTIMIZE & SIMULATE DYNAMIC PID PATH FOLLOWER          ==
% =========================================================================
% This script uses a multi-start optimization routine to find the best 
% controller gains and lookahead distance for a robot with first-order 
% dynamics and sensor noise.

clear all; clc; close all;

% --- 1. Define the Scenario ---
path_waypoints = [ 0, 0; 
                   2, 2; 
                   2, 4;
                   0, 5];

start_pose = [0, 0, pi/4]; % [x_start, y_start, theta_start]
robot_params.speed_max = 0.31;
robot_params.rotspeed_max = 1.9;
robot_params.goal_tolerance = 0.15;
sim_params.dt = 0.02;
sim_params.total_time = 40; 

% --- 2. Set up Multi-Start Optimization ---
fprintf('--- Setting up Multi-Start Optimization ---\n');

% Number of random starting points to try
num_starts = 20; 

% Parameter bounds [k_rho, kp_ang, ki_ang, kd_ang,  k_beta, lookahead]
lower_bounds = [  0.2,   0.5,     0,     0,    -4.0,   0.1];
upper_bounds = [  4.0,   5.0,    3,   2,   -0.01,  1.0];
num_params = length(lower_bounds);

% Variables to store the best result found
best_cost = inf;
best_params = [];

% Define the cost function as an anonymous function for the optimizer
objective_func = @(params) cost_function(params, path_waypoints, start_pose, robot_params, sim_params);

% Set optimization options (turn off iterative display for the loop)
options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');

% --- 3. Run the Optimization Loop ---
fprintf('--- Running Optimization for %d starting points... ---\n', num_starts);
for i = 1:num_starts
    % Generate a random starting point for the gains within the bounds
    initial_params = lower_bounds + (upper_bounds - lower_bounds) .* rand(1, num_params);
    
    % Run the optimizer from this starting point
    [current_params, current_cost] = fmincon(objective_func, initial_params, [], [], [], [], lower_bounds, upper_bounds, [], options);
    
    fprintf('Run %d/%d: Found cost of %.4f\n', i, num_starts, current_cost);
    
    % If this run found a better solution, save it
    if current_cost < best_cost
        best_cost = current_cost;
        best_params = current_params;
        fprintf('  -> New best cost found!\n');
    end
end

fprintf('\n--- Multi-Start Optimization Complete ---\n');
fprintf('Optimal Parameters:\n');
fprintf('  k_rho:          %.4f\n', best_params(1));
fprintf('  kp_angular:     %.4f\n', best_params(2));
fprintf('  ki_angular:     %.4f\n', best_params(3));
fprintf('  kd_angular:     %.4f\n', best_params(4));
fprintf('  k_beta:         %.4f\n', best_params(5));
fprintf('  lookahead_dist: %.4f\n', best_params(6));
fprintf('Final Best Cost:  %.4f\n\n', best_cost);

% --- 4. Simulate and Plot the Final, Best Result ---
[trajectory, time_taken] = run_simulation(best_params, path_waypoints, start_pose, robot_params, sim_params);

figure;
plot(trajectory(1:end-1,1), trajectory(1:end-1,2), 'b-', 'LineWidth', 2);
hold on; grid on;

plot(path_waypoints(:,1), path_waypoints(:,2), 'r--', 'LineWidth', 1.5);
plot(path_waypoints(:,1), path_waypoints(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
plot(start_pose(1), start_pose(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
quiver(trajectory(1:50:end,1), trajectory(1:50:end,2), ...
       cos(trajectory(1:50:end,3)), sin(trajectory(1:50:end,3)), ...
       0.2, 'k');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Optimal Robot Trajectory with Dynamic Model and Noise');
legend('Robot Trajectory', 'Ideal Path', 'Waypoints', 'Start', 'Heading');
axis equal;


% =========================================================================
% == HELPER FUNCTIONS (UNCHANGED)                                        ==
% =========================================================================
function cost = cost_function(params, path_waypoints, start_pose, robot_params, sim_params)
    W_error = 40.0; 
    W_time = 0.15;
    [trajectory, time_taken] = run_simulation(params, path_waypoints, start_pose, robot_params, sim_params);
    if isempty(trajectory), cost = 1e6; return; end
    total_squared_error = 0;
    path_segments = diff(path_waypoints, 1);
    for i = 1:size(trajectory, 1)
        robot_pos = trajectory(i, 1:2);
        min_dist_sq = inf;
        for j = 1:size(path_segments, 1)
            p1 = path_waypoints(j,:); v = path_segments(j,:);
            t = max(0, min(1, dot(robot_pos - p1, v) / dot(v,v)));
            projection = p1 + t*v;
            dist_sq = sum((robot_pos - projection).^2);
            if dist_sq < min_dist_sq, min_dist_sq = dist_sq; end
        end
        total_squared_error = total_squared_error + min_dist_sq;
    end
    mean_squared_error = total_squared_error / size(trajectory, 1);
    if time_taken >= sim_params.total_time
        time_penalty = 500 + (norm(trajectory(end, 1:2) - path_waypoints(end,:)) * 50);
    else
        time_penalty = time_taken;
    end
    cost = W_error * mean_squared_error + W_time * time_penalty;
end

function [trajectory, time_taken] = run_simulation(params_vec, path_waypoints, start_pose, robot_params, sim_params)
    gains.k_rho=params_vec(1); gains.kp_angular=params_vec(2); gains.ki_angular=params_vec(3);
    gains.kd_angular=params_vec(4); gains.k_beta=params_vec(5); lookahead_dist=params_vec(6);
    dt=sim_params.dt; num_steps=sim_params.total_time/dt;
    tau_v=0.2; tau_omega=0.1; pos_noise_std=0.01; ang_noise_std=0.02;
    true_robot_state=[start_pose,0,0]; trajectory=zeros(num_steps,5);
    integral_error=0; previous_alpha=0; final_step=num_steps;
    for i=1:num_steps
        perceived_state=true_robot_state(1:3)+[normrnd(0,pos_noise_std,[1,2]),normrnd(0,ang_noise_std)];
        min_dist_sq=inf; closest_segment_idx=1; t_closest=0;
        for j=1:size(path_waypoints,1)-1
            p1=path_waypoints(j,:); v=path_waypoints(j+1,:)-p1;
            t=max(0,min(1,dot(perceived_state(1:2)-p1,v)/dot(v,v)));
            projection=p1+t*v; dist_sq=sum((perceived_state(1:2)-projection).^2);
            if dist_sq<min_dist_sq, min_dist_sq=dist_sq; closest_segment_idx=j; t_closest=t; end
        end
        current_target=path_waypoints(end,:);
        dist_traveled_on_segment=t_closest*norm(path_waypoints(closest_segment_idx+1,:)-path_waypoints(closest_segment_idx,:));
        dist_to_find=lookahead_dist;
        for j=closest_segment_idx:size(path_waypoints,1)-1
            segment_start=path_waypoints(j,:); segment_end=path_waypoints(j+1,:);
            segment_vec=segment_end-segment_start; segment_len=norm(segment_vec);
            dist_left_on_segment=segment_len-dist_traveled_on_segment;
            if dist_left_on_segment>=dist_to_find
                current_target=(segment_start+(dist_traveled_on_segment+dist_to_find)/segment_len*segment_vec);
                break;
            else
                dist_to_find=dist_to_find-dist_left_on_segment;
                dist_traveled_on_segment=0;
            end
        end
        if norm(true_robot_state(1:2)-path_waypoints(end,:))<robot_params.goal_tolerance, final_step=i;break; end
        x=perceived_state(1);y=perceived_state(2);theta=perceived_state(3);
        delta_x=current_target(1)-x;delta_y=current_target(2)-y;
        rho=sqrt(delta_x^2+delta_y^2);
        alpha=atan2(sin(-theta+atan2(delta_y,delta_x)),cos(-theta+atan2(delta_y,delta_x)));
        p_term=gains.kp_angular*alpha;
        integral_error=max(min(integral_error+alpha*dt,1.0),-1.0);
        i_term=gains.ki_angular*integral_error;
        derivative_error=(alpha-previous_alpha)/dt;
        d_term=gains.kd_angular*derivative_error; previous_alpha=alpha;
        v_cmd=min(gains.k_rho*rho,robot_params.speed_max);
        if norm(true_robot_state(1:2)-path_waypoints(end,:))<lookahead_dist*1.5
            beta=-theta-alpha; omega_cmd=p_term+i_term+d_term+gains.k_beta*beta;
        else
            omega_cmd=p_term+i_term+d_term;
        end
        omega_cmd=max(min(omega_cmd,robot_params.rotspeed_max),-robot_params.rotspeed_max);
        v_actual=true_robot_state(4); omega_actual=true_robot_state(5);
        v_new=v_actual+(v_cmd-v_actual)*dt/tau_v;
        omega_new=omega_actual+(omega_cmd-omega_actual)*dt/tau_omega;
        true_theta=true_robot_state(3);
        true_robot_state=[true_robot_state(1)+v_new*cos(true_theta)*dt,true_robot_state(2)+v_new*sin(true_theta)*dt,true_robot_state(3)+omega_new*dt,v_new,omega_new];
        trajectory(i,:)=true_robot_state;
    end
    trajectory=trajectory(1:final_step,1:5);
    time_taken=final_step*dt;
end