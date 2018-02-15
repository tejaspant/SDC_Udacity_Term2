fclose all; clear all; close all; clc;
homeDir = 'C:\Temp\PHD\SDC_Udacity\Tejas\Term2\P2_UKF\CarND-Unscented-Kalman-Filter-Project-master';
cd('NIS_data/');

nis_L = load('NIS_L.txt');
nis_R = load('NIS_R.txt');
%%
%NIS plot
figure(1)
subplot(1,2,1)
plot(1:length(nis_R),nis_R, 'r', 'linewidth', 2);
hold on;
plot([0 length(nis_R)], [7.815, 7.815], 'b', 'linewidth', 3)
xlabel('Time Step', 'Interpreter', 'latex');
ylabel('NIS $\epsilon$', 'Interpreter', 'latex');
title('NIS Value for RADAR');
box on;
xlim([0 250]);
set(gca,'FontSize',30,'linewidth',2);
hold off;

subplot(1,2,2)
plot(1:length(nis_L),nis_L, 'r', 'linewidth', 2);
hold on;
plot([0 length(nis_L)], [5.991, 5.991], 'b', 'linewidth', 3)
xlabel('Time Step', 'Interpreter', 'latex');
ylabel('NIS $\epsilon$', 'Interpreter', 'latex');
title('NIS Value for LIDAR');
box on;
xlim([0 250]);
set(gca,'FontSize',30,'linewidth',2);
hold off;

set(gcf,'Position',[100 100 1600 800]);
set(gcf, 'PaperPositionMode', 'auto');

% firo='C:\Temp\PHD\SDC_Udacity\Tejas\Term2\P2_UKF\CarND-Unscented-Kalman-Filter-Project-master\results_images';
% print(gcf,'-dpng','-r1200',[firo '\nis_lidar_radar.png']);

cd(homeDir);

%%
%Position Plot
cd('C:\Temp\PHD\SDC_Udacity\Tejas\Term2\P1_Extended_Kalman_Filters\CarND-Extended-Kalman-Filter-Project-master\data');
fid = fopen('obj_pose-laser-radar-synthetic-input.txt');
%% State Initialization
use_laser = 1;
use_radar = 1;
tline = fgets(fid); % read first line

% find first laser measurement
while tline(1) ~= 'L' % laser measurement
    tline = fgets(fid); % go to next line
end
    
line_vector = textscan(tline,'%s %f %f %f %f %f %f %f %f %f');
last_time = line_vector{4};
x_est(1) = line_vector{2}; % initialize position p_x
x_est(2) = line_vector{3}; % initialize position p_y

tline = fgets(fid); % go to next line 

% counter 
kL = 1;
kR = 1;
while ischar(tline)  % go through lines of data file
    % find time of measurement
    if tline(1) == 'L' % laser measurement
        if use_laser == false
            tline = fgets(fid); % skip this line and go to next line
            continue;
        else % read laser meas time
            fprintf('kL = %i\n', kL);
            line_vector = textscan(tline,'%s %f %f %f %f %f %f %f %f %f');
            meas_time = line_vector{1,4};
            ground_truthL(kL,1) = line_vector{1,5};
            ground_truthL(kL,2) = line_vector{1,6};
            L_meas(kL,1) = line_vector{1,2};
            L_meas(kL,2) = line_vector{1,3};
            kL = kL + 1;
        end
    elseif  tline(1) == 'R' % radar measurement 
        if use_radar == false
            tline = fgets(fid); % skip this line and go to next line
            continue;
        else % read radar meas time
            fprintf('kR = %i\n', kR);
            line_vector = textscan(tline,'%s %f %f %f %f %f %f %f %f %f %f');
            ground_truthR(kR,1) = line_vector{1,6};
            ground_truthR(kR,2) = line_vector{1,7};
            R_meas(kR,1) = line_vector{1,2} * cos(line_vector{1,3});
            R_meas(kR,2) = line_vector{1,2} * sin(line_vector{1,3});
            kR = kR + 1;
        end
        
    else % neither laser nor radar
        disp('Error: not laser nor radar')
        return;
    end
    
    tline = fgets(fid);
end

cd('C:\Temp\PHD\SDC_Udacity\Tejas\Term2\P2_UKF\CarND-Unscented-Kalman-Filter-Project-master\UKF_pos_estimate');
ukf_pos_est = load('ukf_pos_est.txt');

figure(2)

hold on;
plot(L_meas(:,1), L_meas(:,2), 'bo', 'markersize', 6, 'linewidth', 1);
plot(R_meas(:,1), R_meas(:,2), 'r^', 'markersize', 6, 'linewidth', 1);
plot(ground_truthL(:,1),ground_truthL(:,2), 'm', 'linewidth', 2);
plot(ukf_pos_est(:,1),ukf_pos_est(:,2), 'k-.', 'linewidth', 2);
% plot(ground_truthR(:,1),ground_truthR(:,2), 'g', 'linewidth', 2);
xlabel('x', 'Interpreter', 'latex');
ylabel('y', 'Interpreter', 'latex');
title('UKF Prediction');
box on;
% xlim([0 250]);
legend('Laser Measurement', 'Radar Measurement', 'Ground Truth', 'UKF Prediction', 'location', 'southeast');
set(gca,'FontSize',20,'linewidth',2);
hold off;

set(gcf,'Position',[100 100 1600 800]);
set(gcf, 'PaperPositionMode', 'auto');

firo='C:\Temp\PHD\SDC_Udacity\Tejas\Term2\P2_UKF\CarND-Unscented-Kalman-Filter-Project-master\results_images';
print(gcf,'-dpng','-r1200',[firo '\ukf_prediction.png']);

cd(homeDir);
