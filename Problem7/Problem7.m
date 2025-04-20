%% QUESTION 7:

theta_vals = -pi:0.01:pi;
p2_range = 0:0.01:13;
pose_counts = zeros(size(p2_range));

for i = 1:length(p2_range)
    p2 = p2_range(i);
    f_vals = f_variable_p2(theta_vals, p2);
    pose_counts(i) = sum(abs(diff(sign(f_vals))) == 2);  % number of real roots
end

% Plot how the number of real roots changes with p2
figure(21);
plot(p2_range, pose_counts, '-o');
xlabel('p_2 value');
ylabel('Number of Real Roots (Poses)');
title('Number of Poses vs. p_2');
grid on;

% Based on the graph, it appears that:
% there are 0 poses when p2 < 3.7 and when p2 > 9.27
% there are 2 poses when 3.7 < p2 < 4.86 and 7.85 < p2 < 9.26
% there are 4 poses when 4.87 < p2 < 6.96 and 7.03 < p2 < 7.84
% there are 6 poses when 6.97 < p2 < 7.02

%% ALL FUNCTIONS SUPPORTING THIS CODE 

% f(theta) function with ability to change p2
function out = f_variable_p2(theta, p2)
    L1 = 3; L2 = 3 * sqrt(2); L3 = 3;
    gamma = pi / 4;
    p1 = 5; p3 = 3;
    x1 = 5; x2 = 0; y2 = 6;

    A2 = L3 * cos(theta) - x1;
    B2 = L3 * sin(theta);
    A3 = L2 * (cos(theta) * cos(gamma) - sin(theta) * sin(gamma)) - x2;
    B3 = L2 * (cos(theta) * sin(gamma) + sin(theta) * cos(gamma)) - y2;

    N1 = B3 .* (p2^2 - p1^2 - A2.^2 - B2.^2) - B2 .* (p3^2 - p1^2 - A3.^2 - B3.^2);
    N2 = -A3 .* (p2^2 - p1^2 - A2.^2 - B2.^2) + A2 .* (p3^2 - p1^2 - A3.^2 - B3.^2);
    D = 2 * (A2 .* B3 - B2 .* A3);

    out = N1.^2 + N2.^2 - p1.^2 * D.^2;
end