clc; clear

%Computes HR and ML acceleration per trial using harmonics method
folderPath = "/MATLAB Drive/PAFO_GaitTrials/No_Device3"; % Folder path w/ gait data.
outFile    = fullfile(folderPath, "GaitMetricsSummary_perTrial_NoDevice.xlsx"); % Create excel for outputting results.

if isfile(outFile) % Delete previous excel to reset.
    delete(outFile);
end

files = [dir(fullfile(folderPath, "*.csv"))]; % Find CSV files from folder path.
if isempty(files)
    error("No .csv files found in: %s", folderPath);
end

%Loops through each CSV file and computes metrics
for k = 1:length(files)
    fileName = fullfile(folderPath, files(k).name);    
    data1 = readtable(fileName); % Read CSV file

    % Read linear acceleration data from sensor
    lin_accX1 = data1{:,2}; % VT
    lin_accY1 = data1{:,3}; % ML
    lin_accZ1 = data1{:,4}; % AP

    t_norm = data1{1,1}; %normalize time
    time = ((data1{:,1}-t_norm) / 1000);

    fs = 100; %Set sampling rate used.

%% ======================== Filter & Detrend each axis ==============================
fc = 20; % cut-off freq: can adjust (20 Hz typical)
[b, a] = butter(2, fc/(fs/2)); % 2nd order Butterworth filter

accX1_filt = filtfilt(b, a, detrend(lin_accX1));
accY1_filt = filtfilt(b, a, detrend(lin_accY1));
accZ1_filt = filtfilt(b, a, detrend(lin_accZ1));

%% ================= Stride Segmentation and Trimming =====================
% Detecting step events using VT acceleration spike during heel contact
[~, locs] = findpeaks(accY1_filt, ...
    'MinPeakDistance',round(0.3*fs),'MinPeakProminence', 0.5);

% Define left foot vs right foot strides (same foot: i -> i+2)
strideStarts = locs(1:end-2);
strideEnds   = locs(3:end);
strideBounds_all = [strideStarts(:) strideEnds(:)];

numStridesAll = size(strideBounds_all,1);
nKeep = 20; %number of strides to keep (keep middle 20 strides)

if numStridesAll < nKeep
    error('Not enough steps!');
end

mid = floor(numStridesAll/2);
half = floor(nKeep/2);

startRow = mid - half + 1;
endRow   = startRow + nKeep - 1;

% safety clamp
startRow = max(1, startRow);
endRow   = min(numStridesAll, endRow);

% if clamp changed length, shift startRow so you still keep nKeep
if (endRow - startRow + 1) < nKeep
    startRow = endRow - nKeep + 1;
end

strideBounds = strideBounds_all(startRow:endRow, :);

% Trim signal to kept desired region
start_idx = strideBounds(1,1);
end_idx   = strideBounds(end,2);
strideBounds = strideBounds - start_idx + 1;

accX1_filt_trimmed = accX1_filt(start_idx:end_idx);
accY1_filt_trimmed = accY1_filt(start_idx:end_idx);
accZ1_filt_trimmed = accZ1_filt(start_idx:end_idx);

time_trimmed = time(start_idx:end_idx);


%% ===================== ML Accleration RMS ===========================
% RMS of ML acceleration gives magnitude of acceleration per trial or per stride (using linear accel) 

aML = accX1_filt_trimmed(:); % ML acceleration
aML = aML - mean(aML); % remove DC bias

ML_RMS_trial = sqrt(mean(aML.^2));  % Whole trimmed trial RMS (single number)

% Calculate Per-stride RMS, then average
nStrides = size(strideBounds,1);
ML_RMS_stride = nan(nStrides,1);

for s = 1:nStrides
    i1 = strideBounds(s,1);
    i2 = strideBounds(s,2);

    a = accX1_filt_trimmed(i1:i2);
    a = a - mean(a);
    ML_RMS_stride(s) = sqrt(mean(a.^2));
end

ML_RMS_avg = mean(ML_RMS_stride, 'omitnan');   % average across strides
ML_RMS_sd  = std(ML_RMS_stride,  'omitnan');   % stride-to-stride variability

%% ======================== ML Acceleration Band Power =============================
% band power shows the total energy concentrated in the gait frequency range (0.5Hz - 5 Hz)
aML = accX1_filt_trimmed;
aML = aML - mean(aML);
ML_power_gait = bandpower(aML, fs, [0.5 5]);

%% ===================== Find Fundamental Frequency (stride freq) from VT ================

locs_trim = locs(locs >= start_idx & locs <= end_idx);
T_step = median(diff(locs_trim))/fs;     % seconds per step
fundamental_freq = 1/(2*T_step);  % stride frequency

%% ===================== Calculate HR per Trial ===========================
% HR calculated but not used

num_harmonics = 40; % Define the number of harmonics (first 20 odd and 20 even)

% --- X Axis (ML) ---
x = accX1_filt_trimmed(:);
x = x - mean(x);
xw = x .* hann(length(x));

Nx = length(xw);
Nx_pad = 2*Nx;

Y = abs(fft(xw, Nx_pad));
Y = Y(1:floor(Nx_pad/2)+1);
f = (0:floor(Nx_pad/2))*(fs/Nx_pad);

df = fs / Nx_pad;      % FFT resolution
winBins = 2;           % ±2 bins
winHz = winBins * df;
Huse = min(num_harmonics, floor((fs/2)/fundamental_freq));
harm_mag = harmonic_mags_localmax(Y, f, fundamental_freq, Huse, winHz);

even_sum_x = sum(harm_mag(2:2:end), 'omitnan');
odd_sum_x  = sum(harm_mag(1:2:end), 'omitnan');
global_HR_x = odd_sum_x / even_sum_x;

% --- Y Axis (VT) ---
y = accY1_filt_trimmed(:);
y = y - mean(y);
yw = y .* hann(length(y));
Ny = length(yw); Ny_pad = 2*Ny;

Y = abs(fft(yw, Ny_pad));
Y = Y(1:floor(Ny_pad/2)+1);
f = (0:floor(Ny_pad/2))*(fs/Ny_pad);

df = fs / Ny_pad;
winHz = winBins * df;

Huse = min(num_harmonics, floor((fs/2)/fundamental_freq));
harm_mag = harmonic_mags_localmax(Y, f, fundamental_freq, Huse, winHz);

even_sum_y = sum(harm_mag(2:2:end), 'omitnan');
odd_sum_y  = sum(harm_mag(1:2:end), 'omitnan');
global_HR_y = even_sum_y / odd_sum_y;

% --- Z Axis (AP) ---
z = accZ1_filt_trimmed(:);
z = z - mean(z);
zw = z .* hann(length(z));
Nz = length(zw); Nz_pad = 2*Nz;

Y = abs(fft(zw, Nz_pad));
Y = Y(1:floor(Nz_pad/2)+1);
f = (0:floor(Nz_pad/2))*(fs/Nz_pad);

df = fs / Nz_pad;
winHz = winBins * df;

Huse = min(num_harmonics, floor((fs/2)/fundamental_freq));
harm_mag = harmonic_mags_localmax(Y, f, fundamental_freq, Huse, winHz);

even_sum_z = sum(harm_mag(2:2:end), 'omitnan');
odd_sum_z  = sum(harm_mag(1:2:end), 'omitnan');
global_HR_z = even_sum_z / odd_sum_z;

%% ==================== Printing Data into Excel =======================

HR_ML = global_HR_x;
HR_VT = global_HR_y;
HR_AP = global_HR_z;

ResultRow = table( ...
    string(files(k).name), fundamental_freq, HR_ML, HR_VT, HR_AP, ...
    ML_RMS_trial, ML_RMS_avg, ML_RMS_sd, ML_power_gait, ...
    'VariableNames', {'File','StrideFreq_Hz','HR_ML','HR_VT','HR_AP', ...
                      'ML_RMS_trial','ML_RMS_avg','ML_RMS_sd','ML_power_gait'} );

if k == 1
    writetable(ResultRow, outFile, 'WriteMode','overwrite');
else
    writetable(ResultRow, outFile, 'WriteMode','append','WriteVariableNames',false);
end

end

%% ===================== Local helper function ============================
function mags = harmonic_mags_localmax(Y, f, f0, K, winHz)
% For each harmonic k*f0, take the maximum magnitude in [fk-winHz, fk+winHz]
mags = nan(1,K);
for k = 1:K
    fk = k*f0;
    idx = (f >= (fk - winHz)) & (f <= (fk + winHz));
    if any(idx)
        mags(k) = max(Y(idx));
    end
end
end