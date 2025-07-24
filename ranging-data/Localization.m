clc, clear, close all

load("ranging_data.mat")

N = 250;
dists = customDistance1;
pos_est = zeros(N,3);

window = 3;
ma_buf = zeros(window,3);

for k = 1:N
    ds = dists(k,:);
    
    combs = nchoosek(1:4,3);  % 4C3 = 4 조합
    errs = zeros(size(combs,1),1);
    pcs = zeros(size(combs,1),3);
    
    for i = 1:size(combs,1)
        idx = combs(i,:);
        A = Anchor(idx,:);
        r = ds(idx)';
        
        % 선형화된 least-squares 풀이: A*x = b
        % (xi^2+yi^2+zi^2 - ri^2) - (x1^2+y1^2+z1^2 - r1^2)
        P = 2*(A(2:end,:) - A(1,:));
        b = [sum(A(2,:).^2)-r(2)^2 - (sum(A(1,:).^2)-r(1)^2);
             sum(A(3,:).^2)-r(3)^2 - (sum(A(1,:).^2)-r(1)^2)];
        x0 = P\b;
        pcs(i,:) = x0';
        
        % 오차 계산: 예측 거리 vs 측정 거리
        pred_d = sqrt(sum((A - x0').^2,2));
        errs(i) = sum(abs(pred_d - r));
    end
    
    % 최소 error 후보 선택
    [~,mi] = min(errs);
    pos_est(k,:) = pcs(mi,:);
    
    % 이동 평균 (고정 윈도우)
    ma_buf = [ma_buf(2:end,:); pcs(mi,:)];
    pos_est(k,:) = mean(ma_buf,1);
end

% 결과 플롯 (예시)
plot3(pos_est(:,1),pos_est(:,2),pos_est(:,3),'.-');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('추정된 Tag 위치 경로');
grid on;
