function out = f(theta)
    
    % Platform lengths 
    L1 = 2;
    L2 = sqrt(2);
    L3 = sqrt(2);

    % Angle across from L1
    gamma = pi / 2;

    % Strut lengths
    p1 = sqrt(5);
    p2 = sqrt(5);
    p3 = sqrt(5);

    % Strut base positions  
    % Got these from Figure 1.15
    x1 = 4;
    x2 = 0;
    y2 = 4;

    A2 = L3 * cos(theta) - x1;
    B2 = L3 * sin(theta);
    A3 = L2 * (cos(theta) * cos(gamma) - sin(theta) * sin(gamma)) - x2;
    B3 = L2 * (cos(theta) * sin(gamma) + sin(theta) * cos(gamma)) - y2;

    N1 = B3 * (p2^2 - p1^2- A2^2 - B2^2) - B2 * (p3^2 - p1^2 - A3^2 - B3^2);
    N2 = -A3 * (p2^2 - p1^2- A2^2 - B2^2) + A2 * (p3^2 - p1^2 - A3^2 - B3^2);

    D = 2 * (A2 * B3 - B2 * A3);

    out = N1^2 + N2^2 - p1^2 * D^2;

end

% Testing theta = pi/4
f(pi/4)

% Testing theta = -pi/4
f(-pi/4)