%% QUESTION 8:

% 1. Define base attachment points (fixed)
B = [...  % Each row is a point (xi, yi, zi)
    200, 0, 0;
    100, 173.2, 0;
   -100, 173.2, 0;
   -200, 0, 0;
   -100, -173.2, 0;
    100, -173.2, 0
];

% 2. Define platform attachment points (in platform frame)
P = [... % Each row is a point (xi, yi, zi)
     100, 0, 0;
     50, 86.6, 0;
    -50, 86.6, 0;
   -100, 0, 0;
    -50, -86.6, 0;
     50, -86.6, 0
];

% 3. Define leg lengths (example values)
L = [250; 240; 245; 260; 255; 250];  % Length of each leg

% 4. Objective function to minimize
f_obj = @(x) stewart_error(x, B, P, L);

% 5. Initial guess for pose: [x, y, z, alpha, beta, gamma]
x0 = [0; 0; 200; 0; 0; 0];  % Start near z = 200 mm, no rotation

% 6. Use fsolve to solve
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-10);
[x_sol, fval, exitflag] = fsolve(f_obj, x0, options);

disp('Estimated pose:')
disp(['x = ', num2str(x_sol(1)), ', y = ', num2str(x_sol(2)), ...
      ', z = ', num2str(x_sol(3))])
disp(['alpha = ', num2str(x_sol(4)), ', beta = ', num2str(x_sol(5)), ...
      ', gamma = ', num2str(x_sol(6))])

interactive_stewart(B, P);


%% ALL FUNCTIONS SUPPORTING

function F = stewart_error(x, B, P, L)
    % Inputs:
    % x = [x; y; z; alpha; beta; gamma]
    
    pos = x(1:3);         % Translation vector
    alpha = x(4); beta = x(5); gamma = x(6);  % Roll, pitch, yaw

    % Rotation matrix (ZYX order)
    Rz = [cos(gamma) -sin(gamma) 0;
          sin(gamma)  cos(gamma) 0;
          0 0 1];
    Ry = [cos(beta) 0 sin(beta);
          0 1 0;
          -sin(beta) 0 cos(beta)];
    Rx = [1 0 0;
          0 cos(alpha) -sin(alpha);
          0 sin(alpha) cos(alpha)];
      
    R = Rz * Ry * Rx;

    F = zeros(6,1);
    for i = 1:6
        L_vec = pos + R * P(i,:)' - B(i,:)';
        F(i) = norm(L_vec)^2 - L(i)^2;  % Constraint: length match
    end
end

function visualize_stewart(B, P, pose, ax)
    if nargin < 4
        figure;
        ax = gca;
    end

    % Extract pose
    pos = pose(1:3);
    alpha = pose(4);
    beta  = pose(5);
    gamma = pose(6);

    % Rotation matrix
    Rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma),  cos(gamma), 0;
          0,           0,          1];
    Ry = [cos(beta), 0, sin(beta);
          0,         1, 0;
         -sin(beta), 0, cos(beta)];
    Rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha),  cos(alpha)];
    R = Rz * Ry * Rx;

    % Transform
    P_world = (R * P')' + pos';
    B_loop = [B; B(1,:)];
    P_loop = [P_world; P_world(1,:)];

    % --- Plot in the given axes ---
    cla(ax);
    axes(ax); %#ok<LAXES>
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Stewart Platform Visualization');
    view(3);

    % Plot
    plot3(ax, B_loop(:,1), B_loop(:,2), B_loop(:,3), 'bo-', 'LineWidth', 2);
    plot3(ax, P_loop(:,1), P_loop(:,2), P_loop(:,3), 'ro-', 'LineWidth', 2);
    for i = 1:6
        plot3(ax, [B(i,1), P_world(i,1)], ...
                 [B(i,2), P_world(i,2)], ...
                 [B(i,3), P_world(i,3)], 'k--', 'LineWidth', 1.5);
    end

    legend(ax, 'Base', 'Platform', 'Struts')
end


function interactive_stewart(B, P)
    figure('Name', 'Interactive Stewart Platform');

    % Initial pose
    pose = [0; 0; 200; 0; 0; 0];  % [x y z alpha beta gamma]

    % Create sliders for each parameter
    labels = {'X', 'Y', 'Z', 'Roll (α)', 'Pitch (β)', 'Yaw (γ)'};
    mins   = [-100, -100, 150, -pi, -pi, -pi];
    maxs   = [ 100,  100, 300,  pi,  pi,  pi];

    sliders = gobjects(1,6);
    for i = 1:6
        uicontrol('Style', 'text', 'String', labels{i}, ...
            'Position', [20, 400 - 50*i, 80, 20]);
        sliders(i) = uicontrol('Style', 'slider', ...
            'Min', mins(i), 'Max', maxs(i), ...
            'Value', pose(i), ...
            'Position', [100, 400 - 50*i, 300, 20], ...
            'Callback', @(src, ~) update_plot());
    end

    % Axes for visualization
    ax = axes('Position', [0.5 0.2 0.45 0.7]);
    view(3); grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Stewart Platform Pose');

    % Initial plot
    update_plot();

    % Update function
    function update_plot()
        for i = 1:6
            pose(i) = sliders(i).Value;
        end
        cla(ax);
        axes(ax);
        visualize_stewart(B, P, pose, ax);
    end
end