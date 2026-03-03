% =========================================================================
% PID PATH FOLLOWER — MULTI-START OPTIMIZATION & SIMULATION
% =========================================================================
%
% Uses fmincon (SQP) with multiple random start points to find globally
% robust PID gains and lookahead distance for a differential-drive robot
% model with first-order actuator dynamics and sensor noise.
%
% Optimized parameters:
%   k_rho         — proportional gain for linear speed (distance control)
%   kp_angular    — proportional gain of angular PID
%   ki_angular    — integral gain of angular PID
%   kd_angular    — derivative gain of angular PID
%   k_beta        — final orientation correction gain
%   lookahead_dist— base lookahead distance on the path
%
% Cost function:
%   J = W_error * MSE(cross-track error) + W_time * time_penalty
%
% Robot model:
%   First-order actuator lag: tau_v = 0.2 s, tau_omega = 0.1 s
%   Sensor noise: position sigma = 0.01 m, heading sigma = 0.02 rad
% =========================================================================

clear all; clc; close all;

% -------------------------------------------------------------------------
% 1. SCENARIO DEFINITION
% -------------------------------------------------------------------------
path_waypoints = [0, 0;
                  2, 2;
                  2, 4;
                  0, 5];

start_pose = [0, 0, pi/4];   % [x, y, theta] in metres and radians

robot_params.speed_max      = 0.31;   % m/s
robot_params.rotspeed_max   = 1.9;    % rad/s
robot_params.goal_tolerance = 0.15;   % m

sim_params.dt         = 0.02;   % s
sim_params.total_time = 40;     % s

% -------------------------------------------------------------------------
% 2. MULTI-START OPTIMIZATION SETUP
% -------------------------------------------------------------------------
fprintf('--- Setting up Multi-Start Optimization ---\n');

num_starts = 20;

% Parameter bounds: [k_rho, kp_ang, ki_ang, kd_ang, k_beta, lookahead]
lower_bounds = [0.2,  0.5,  0.0,  0.0, -4.00, 0.1];
upper_bounds = [4.0,  5.0,  3.0,  2.0, -0.01, 1.0];
num_params   = length(lower_bounds);

best_cost   = inf;
best_params = [];

objective_func = @(params) cost_function( ...
    params, path_waypoints, start_pose, robot_params, sim_params);

options = optimoptions('fmincon', ...
    'Display',   'none', ...
    'Algorithm', 'sqp');

% -------------------------------------------------------------------------
% 3. OPTIMIZATION LOOP
% -------------------------------------------------------------------------
fprintf('--- Running Optimization (%d starting points) ---\n', num_starts);

for i = 1:num_starts
    % Random starting point uniformly distributed within bounds
    initial_params = lower_bounds + ...
        (upper_bounds - lower_bounds) .* rand(1, num_params);

    [current_params, current_cost] = fmincon( ...
        objective_func, initial_params, [], [], [], [], ...
        lower_bounds, upper_bounds, [], options);

    fprintf('Run %d/%d: cost = %.4f\n', i, num_starts, current_cost);

    if current_cost < best_cost
        best_cost   = current_cost;
        best_params = current_params;
        fprintf('  -> New best!\n');
    end
end

fprintf('\n--- Optimization Complete ---\n');
fprintf('Optimal Parameters:\n');
fprintf('  k_rho          : %.4f\n', best_params(1));
fprintf('  kp_angular     : %.4f\n', best_params(2));
fprintf('  ki_angular     : %.4f\n', best_params(3));
fprintf('  kd_angular     : %.4f\n', best_params(4));
fprintf('  k_beta         : %.4f\n', best_params(5));
fprintf('  lookahead_dist : %.4f\n', best_params(6));
fprintf('Best cost        : %.4f\n\n', best_cost);

% -------------------------------------------------------------------------
% 4. SIMULATE & PLOT THE BEST RESULT
% -------------------------------------------------------------------------
[trajectory, ~] = run_simulation( ...
    best_params, path_waypoints, start_pose, robot_params, sim_params);

figure;
hold on; grid on; axis equal;

plot(trajectory(1:end-1, 1), trajectory(1:end-1, 2), ...
    'b-', 'LineWidth', 2);
plot(path_waypoints(:, 1), path_waypoints(:, 2), ...
    'r--', 'LineWidth', 1.5);
plot(path_waypoints(:, 1), path_waypoints(:, 2), ...
    'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
plot(start_pose(1), start_pose(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
quiver(trajectory(1:50:end, 1), trajectory(1:50:end, 2), ...
       cos(trajectory(1:50:end, 3)), sin(trajectory(1:50:end, 3)), ...
       0.2, 'k');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Optimal Robot Trajectory with Dynamic Model and Sensor Noise');
legend('Robot Trajectory', 'Ideal Path', 'Waypoints', 'Start', 'Heading');


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function cost = cost_function(params, path_waypoints, start_pose, ...
                               robot_params, sim_params)
% COST_FUNCTION  Weighted sum of mean squared cross-track error and time.
%
%   J = W_error * MSE(cross-track error) + W_time * time_penalty

    W_error = 40.0;
    W_time  = 0.15;

    [trajectory, time_taken] = run_simulation( ...
        params, path_waypoints, start_pose, robot_params, sim_params);

    if isempty(trajectory)
        cost = 1e6;
        return;
    end

    % Compute minimum cross-track distance for every trajectory point
    path_segments       = diff(path_waypoints, 1);
    total_squared_error = 0;

    for i = 1:size(trajectory, 1)
        robot_pos   = trajectory(i, 1:2);
        min_dist_sq = inf;

        for j = 1:size(path_segments, 1)
            p1 = path_waypoints(j, :);
            v  = path_segments(j, :);
            t  = max(0, min(1, dot(robot_pos - p1, v) / dot(v, v)));

            projection  = p1 + t * v;
            dist_sq     = sum((robot_pos - projection).^2);

            if dist_sq < min_dist_sq
                min_dist_sq = dist_sq;
            end
        end

        total_squared_error = total_squared_error + min_dist_sq;
    end

    mean_squared_error = total_squared_error / size(trajectory, 1);

    % Time penalty: large if robot never reached the goal
    if time_taken >= sim_params.total_time
        dist_to_goal = norm(trajectory(end, 1:2) - path_waypoints(end, :));
        time_penalty = 500 + dist_to_goal * 50;
    else
        time_penalty = time_taken;
    end

    cost = W_error * mean_squared_error + W_time * time_penalty;
end


function [trajectory, time_taken] = run_simulation(params_vec, ...
    path_waypoints, start_pose, robot_params, sim_params)
% RUN_SIMULATION  Closed-loop simulation with first-order dynamics & noise.
%
%   Simulates a differential-drive robot tracking path_waypoints using a
%   PID polar-coordinate controller with a dynamic lookahead point.
%
%   Robot model:
%     v(t+1)     = v(t)     + (v_cmd - v(t))    * dt / tau_v
%     omega(t+1) = omega(t) + (omega_cmd-omega(t)) * dt / tau_omega
%
%   Sensor noise applied to perceived state before control computation.

    % --- Unpack parameters ---
    gains.k_rho      = params_vec(1);
    gains.kp_angular = params_vec(2);
    gains.ki_angular = params_vec(3);
    gains.kd_angular = params_vec(4);
    gains.k_beta     = params_vec(5);
    lookahead_dist   = params_vec(6);

    % --- Simulation constants ---
    dt          = sim_params.dt;
    num_steps   = sim_params.total_time / dt;
    tau_v       = 0.2;    % linear actuator time constant (s)
    tau_omega   = 0.1;    % angular actuator time constant (s)
    pos_noise_std = 0.01; % position noise std dev (m)
    ang_noise_std = 0.02; % heading noise std dev (rad)

    % --- State initialisation: [x, y, theta, v, omega] ---
    true_robot_state = [start_pose, 0, 0];
    trajectory       = zeros(num_steps, 5);
    integral_error   = 0;
    previous_alpha   = 0;
    final_step       = num_steps;

    % ----------------------------------------------------------------
    % Main simulation loop
    % ----------------------------------------------------------------
    for i = 1:num_steps

        % --- Apply sensor noise to get perceived state ---
        perceived_state = true_robot_state(1:3) + [ ...
            normrnd(0, pos_noise_std, [1, 2]), ...
            normrnd(0, ang_noise_std)];

        % --- Find the closest point on the path (for lookahead anchor) ---
        min_dist_sq         = inf;
        closest_segment_idx = 1;
        t_closest           = 0;

        for j = 1:size(path_waypoints, 1) - 1
            p1 = path_waypoints(j, :);
            v  = path_waypoints(j+1, :) - p1;
            t  = max(0, min(1, dot(perceived_state(1:2) - p1, v) / dot(v, v)));

            projection = p1 + t * v;
            dist_sq    = sum((perceived_state(1:2) - projection).^2);

            if dist_sq < min_dist_sq
                min_dist_sq         = dist_sq;
                closest_segment_idx = j;
                t_closest           = t;
            end
        end

        % --- Compute lookahead target point ---
        current_target = path_waypoints(end, :);  % default: final goal
        dist_traveled_on_segment = t_closest * norm( ...
            path_waypoints(closest_segment_idx+1, :) - ...
            path_waypoints(closest_segment_idx, :));
        dist_to_find = lookahead_dist;

        for j = closest_segment_idx:size(path_waypoints, 1) - 1
            segment_start = path_waypoints(j, :);
            segment_end   = path_waypoints(j+1, :);
            segment_vec   = segment_end - segment_start;
            segment_len   = norm(segment_vec);
            dist_left     = segment_len - dist_traveled_on_segment;

            if dist_left >= dist_to_find
                current_target = segment_start + ...
                    (dist_traveled_on_segment + dist_to_find) / ...
                    segment_len * segment_vec;
                break;
            else
                dist_to_find             = dist_to_find - dist_left;
                dist_traveled_on_segment = 0;
            end
        end

        % --- Goal reached check ---
        dist_to_goal = norm(true_robot_state(1:2) - path_waypoints(end, :));
        if dist_to_goal < robot_params.goal_tolerance
            final_step = i;
            break;
        end

        % --- PID control in polar coordinates (rho, alpha, beta) ---
        x     = perceived_state(1);
        y     = perceived_state(2);
        theta = perceived_state(3);

        delta_x = current_target(1) - x;
        delta_y = current_target(2) - y;
        rho     = sqrt(delta_x^2 + delta_y^2);
        alpha   = atan2( ...
            sin(-theta + atan2(delta_y, delta_x)), ...
            cos(-theta + atan2(delta_y, delta_x)));

        % Angular PID terms
        p_term = gains.kp_angular * alpha;

        integral_error = max(min(integral_error + alpha * dt, 1.0), -1.0);
        i_term = gains.ki_angular * integral_error;

        derivative_error = (alpha - previous_alpha) / dt;
        d_term           = gains.kd_angular * derivative_error;
        previous_alpha   = alpha;

        % Linear speed command (proportional to distance, clamped)
        v_cmd = min(gains.k_rho * rho, robot_params.speed_max);

        % Angular speed command (add beta correction near goal)
        near_goal = norm(true_robot_state(1:2) - path_waypoints(end, :)) ...
                    < lookahead_dist * 1.5;
        if near_goal
            beta      = -theta - alpha;
            omega_cmd = p_term + i_term + d_term + gains.k_beta * beta;
        else
            omega_cmd = p_term + i_term + d_term;
        end
        omega_cmd = max(min(omega_cmd, robot_params.rotspeed_max), ...
                        -robot_params.rotspeed_max);

        % --- First-order actuator dynamics ---
        v_actual     = true_robot_state(4);
        omega_actual = true_robot_state(5);
        v_new        = v_actual     + (v_cmd     - v_actual)     * dt / tau_v;
        omega_new    = omega_actual + (omega_cmd - omega_actual)  * dt / tau_omega;

        % --- Integrate true robot state (no noise) ---
        true_theta = true_robot_state(3);
        true_robot_state = [ ...
            true_robot_state(1) + v_new * cos(true_theta) * dt, ...
            true_robot_state(2) + v_new * sin(true_theta) * dt, ...
            true_robot_state(3) + omega_new * dt, ...
            v_new, ...
            omega_new];

        trajectory(i, :) = true_robot_state;
    end

    trajectory  = trajectory(1:final_step, 1:5);
    time_taken  = final_step * dt;
end
