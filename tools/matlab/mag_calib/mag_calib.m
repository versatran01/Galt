%% mag_calib.m
% Calibrate IMU magnetometer bias using least-squares.
clear rosbag_wrapper;
clear ros.Bag;
clear;

BAGFILE_PATH = 'imu_calib_steadicam_v5_2015-02-23-16-57-55.bag';
SUBSAMPLE_FACTOR = 5;

bag = ros.Bag.load(BAGFILE_PATH);

% read messages
[msgs, ~] = bag.readAll('/imu/magnetic_field');
N = length(msgs);
fprintf('Got %i IMU messages\n', N);

data = [];
for i=1:N
    data(end+1,:) = msgs{i}.magnetic_field';
end

data = data(1:SUBSAMPLE_FACTOR:end,:);

figure;
hold on;
scatter3(data(:,1),data(:,2),data(:,3));
axis vis3d;
grid on;

set(gcf, 'Renderer', 'OpenGL');

% perform least-squares fit of a sphere
A = zeros(0,4);
b = zeros(0,1);

for i=1:size(data,1)
    x = data(i,1);
    y = data(i,2);
    z = data(i,3);
    
    A(end+1,:) = [2*x 2*y 2*z 1];
    b(end+1,:) = (x*x + y*y + z*z);
end
D = A' * A;
x = D \ (A' * b);
% centre and radius of the circle
center = x(1:3);
rad2 = x(4) + center'*center;
radius = sqrt(rad2);

scatter3(x(1),x(2),x(3),'r');
xlabel('X');
ylabel('Y');
zlabel('Z');

% output center and radius from least-squares fit
fprintf('Least-squares centre:\n');
disp(center)
fprintf('Least-squares radius:\n');
disp(radius)

% initialize non-linear least squares refinement
bias = center;
scale = [1 1 1]';   % NOTE: this scale should be _divided_ by, not multiplied

for iter=1:30
    J = zeros(0,6);
    r = zeros(0,1);
    
    for i=1:size(data,1)
        x = (data(i,1) - bias(1)) / scale(1);
        y = (data(i,2) - bias(2)) / scale(2);
        z = (data(i,3) - bias(3)) / scale(3);
        rad2 = x*x+y*y+z*z;
        
        r(i,1) = rad2;
        J(i,1) = -2*x*x / scale(1);
        J(i,2) = -2*y*y / scale(2);
        J(i,3) = -2*z*z / scale(3);
        J(i,4) = -2*x / scale(1);
        J(i,5) = -2*y / scale(2);
        J(i,6) = -2*z / scale(3);
    end
    r = radius*radius - r;
    
    % cauchy error weighting
    r2 = r.^2;
    mean_r2 = mean(r2);
    W = 1 ./ (1 + r2/mean_r2);
    
    W = diag(W);
    H = J'* W * J;
    H(1:3,1:3) = H(1:3,1:3) * 1.001; % small prior on scale
    
    dx = H \ (J' * W * r);
    scale = scale + dx(1:3);
    bias = bias + dx(4:6);
end

% display output
fprintf('Final bias:\n');
disp(bias)
fprintf('Final scale:\n');
disp(scale)

% plot the final ellipsoid
xr = radius * scale(1);
yr = radius * scale(2);
zr = radius * scale(3);

[x,y,z] = ellipsoid(bias(1),bias(2),bias(3),xr,yr,zr);
surf(x,y,z);
alpha(0.5);
