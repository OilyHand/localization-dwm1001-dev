% Non-Linear Least Squares 기반 위치 추정
% - distances: 1xN 벡터 (Tag와 각 Anchor 사이 거리)
% - anchors: Nx3 행렬 (Anchor의 x,y,z 좌표)
% - location: 1x3 벡터 (계산된 Tag 위치 [x y z])
clear
close all
clc
load("ranging_data.mat")

% 파라미터 설정
MAX_ITER = 10;
TOLERANCE = 1e-6;
ALPHA = 0.05;  % 학습률

% 초기 위치 (Chan 알고리즘으로 바꿀 수도 있음)
x = 1.0;
y = 1.0;
z = 1.0;

N = 4;
distance_sample = customDistance1;
ITER_OUT = length(distance_sample);
location = zeros(ITER_OUT, 3);

figure
grid on
axis equal
hold on

plot3(Anchor(1,1),Anchor(1,2), Anchor(1,3), "o")
plot3(Anchor(2,1),Anchor(2,2), Anchor(2,3), "o")
plot3(Anchor(3,1),Anchor(3,2), Anchor(3,3), "o")
plot3(Anchor(4,1),Anchor(4,2), Anchor(4,3), "o")

for iter_out = 1:ITER_OUT
    distances = distance_sample(iter_out,:);
    for iter = 1:MAX_ITER
        grad = zeros(1, 3);  % [∂E/∂x, ∂E/∂y, ∂E/∂z]
    
        for i = 1:N
            dx = x - Anchor(i,1);
            dy = y - Anchor(i,2);
            dz = z - Anchor(i,3);
    
            est_dist = sqrt(dx^2 + dy^2 + dz^2);
            if est_dist < 1e-6
                continue;
            end
    
            error = est_dist - distances(i);
    
            % Gradient 계산
            grad(1) = grad(1) + (error * dx) / est_dist;  % ∂E/∂x
            grad(2) = grad(2) + (error * dy) / est_dist;  % ∂E/∂y
            grad(3) = grad(3) + (error * dz) / est_dist;  % ∂E/∂z
        end
    
        % 위치 업데이트
        x = x - ALPHA * grad(1);
        y = y - ALPHA * grad(2);
        z = z - ALPHA * grad(3);
    
        % 수렴 조건 확인
        if norm(grad) < TOLERANCE
            break;
        end
    end
    plot3(x,y,z,"ro")
    location(iter_out,:) = [x,y,z];
end

x_data = location(:,1);
y_data = location(:,2);
z_data = location(:,3);

x_mean = mean(x_data);
y_mean = mean(y_data);
z_mean = mean(z_data);

x_dev = std(x_data);
y_dev = std(y_data);
z_dev = std(z_data);

deviations = [x_dev, y_dev, z_dev] .* 100

figure
subplot(1,3,1), histogram(x_data-x_mean), title("x-axis")
subplot(1,3,2), histogram(y_data-y_mean), title("y-axis")
subplot(1,3,3), histogram(z_data-z_mean), title("z-axis")