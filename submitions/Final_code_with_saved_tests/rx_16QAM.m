% Receiver 16-QAM
clc; clear all; close all;

% Initialize parameters
MasterClock_Rate = 100000000;           % Sampling rate for digital mixer
Interp_Factor = 64;                    
Decimation_Factor = Interp_Factor;
fs = MasterClock_Rate/Interp_Factor;    % Sampling rate
dt = 1/fs;                              % Sampling time
N = 10000;                              % Number of samples in a frame
frame_time = N/fs;                      % Time for 1 frame
time = (0:dt:dt*(N-1))';
RBW = 1/frame_time;
NFFT = 2^nextpow2(N);                   % Next power of 2 from length of y
fc = 10e6;                              % Carrier frequency
Rb = 50e3;                              % Bitrate

M = 16;                                 % Modulation order
m =log2(M);                             % Bits per symbol

Rs = Rb/m;                              % Symbolrate
Ts_symbol=1/Rs;                         % Symbol time
Ts= 1/fs;                               % Sampling time
fsfd = floor(fs/Rs);                    % Samples per symbol

% Modulation
const = [1+3i 3+3i 3+1i 1+1i 1-1i 3-1i 3-3i 1-3i -3-1i -1-1i -1-3i -3-3i -3+3i -1+3i -1+1i -3+1i]/sqrt(2);         % Constellation for 16-QAM

% Initialize RTRC
rolloff = 0.3;  tau = 1/Rs;  span = 4;
root_raised_cosine_pulse = rtrcpuls(rolloff,tau,fs,span);

%% Create preamble, pilot, guard, message and MF
% Preamble
preamble = [0,0,1,1; 1,0,0,0; 1,0,0,0; 1,1,1,0; 1,0,1,0; 1,0,0,1; 1,0,1,1; 0,0,0,0; 1,1,1,1; 1,0,1,1; 0,1,1,1; ...
            1,1,0,0; 0,0,1,0; 0,0,0,0; 0,1,0,0; 1,1,0,0; 0,0,1,0; 1,1,0,0; 0,0,0,1; 0,0,1,0; 1,1,0,1; 0,0,1,1; 1,0,0,1; ...
            1,0,0,1; 0,1,1,1; 0,0,0,1; 0,1,0,1; 0,1,1,1; 1,0,1,0; 0,1,1,0; 1,1,1,1; 1,1,0,0; 1,0,0,0; 1,1,1,0; 0,1,0,0; ...
            1,0,1,1; 1,0,1,1; 1,1,1,0; 0,1,0,0; 0,0,1,1; 0,0,1,0; 1,1,0,0; 0,1,0,0; 0,0,0,0; 1,1,0,1; 0,0,1,1; 0,1,0,1; ... 
            1,1,1,1; 0,0,1,0; 0,1,1,1];

preamble = const(bi2de(preamble, 'left-msb')'+1);
preamble_length=length(preamble);

% Upsample and MF of preamble
preamble_upsampled = upsample(preamble,fsfd); 
preamble_upsampled_length=length(preamble_upsampled);
preamble_upsampled_RRC = conv(preamble_upsampled, root_raised_cosine_pulse, 'full');

% Pilot and guard
pilot = ones(1,300);
pilot_length=length(pilot);
pilot_upsampled_length=length(pilot)*fsfd;

guard = zeros(1,50); 
guard_length=length(guard);
guard_upsampled_length=length(guard)*fsfd;

% Message
message=['Hello, is it receiveing? Hello, is it receiveing? Hello, is it receiveing?'];
%message=['His connection interested so we a sympathize advantages To said is it shed want do Occasional Middletons everything so too '];

% Message: string to bits
message_binary = logical(reshape(dec2bin(message, 8).',1,[])- '0');

% Zeropadd message 
% Calculate how many zeros to add
even_bit_number=ceil(length(message_binary)/m);
zeros_to_add=zeros(1,(even_bit_number*m)-length(message_binary));
% Add zeros to frame
message_zeropadded=[message_binary zeros_to_add];

% Mapp message
% Reshape to m-vector of bits
message_divided=reshape(message_zeropadded,m, [])';
% Convert frame from binary to decimal
message_dec= bi2de(message_divided,'left-msb')'+1;

% Modulate to symbols
message_symbols= const(message_dec);
message_length=length(message_symbols);
message_upsampled_length=length(message_symbols)*fsfd;

% Create frame
frame=0.2*[guard pilot preamble message_symbols];
frame_length=length(frame);

% Upsample
frame_upsampled = upsample(frame, fsfd);
frame_upsampled_length=length(frame_upsampled);

% Put symbols on pulse
% Conv with RTRC
tx_signal = conv(root_raised_cosine_pulse,frame_upsampled);

% Normalize
tx_signal_norm = tx_signal/max(abs(tx_signal)).';
%scatterplot(tx_signal_norm)

%% Set up the RX
% rx = comm.SDRuReceiver(...
%     'Platform','N200/N210/USRP2',...
%     'IPAddress','192.168.10.10',... 
%     'CenterFrequency',fc,...
%     'EnableBurstMode',1,...
%     'NumFramesInBurst',1,...
%     'DecimationFactor',Decimation_Factor,...
%     'SamplesPerFrame',30*N,...
%     'MasterClockRate',MasterClock_Rate,...
%     'TransportDataType','int16');
% 
% currentTime = 0;
% 
% % Receiving the signal
% for i=1:1
%     [rx_data] = step(rx);
%     rx_data = double(rx_data)/(2^16);
%     
%     [rx_dsb,f] = periodogram(rx_data,hamming(length(rx_data)),NFFT,fs,'centered');
%     rx_dsb = 10*log10(RBW*rx_dsb)+15;% In dBm
%     figure(1), subplot(2,1,1), pwelch(rx_data,[],[],[],fs,'centered','power');title('Power spetrum of rx signal'); % spectrum of received signal after LP
%     figure(1), subplot(2,1,2),plot(real(rx_data)); hold on; plot(real(rx_data)); title('Real part of rx signal'); % received signal after LP
% 
%     rx_data1 = conj(rx_data.');                % conj back since HW has flipped to signal
%     currentTime=currentTime+frame_time             % just to see that it is working
% end
% 
% release(rx);                                    

%% Load saved rx signal
load('test_16qam_planned_values');
% close all

%% Plot received signal
% figure()
% subplot(2,1,1), pwelch(rx_data,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal');
% subplot(2,1,2), plot(real(rx_data)); title('Real part of rx signal');
% 
% figure()
% subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal transposed');
% subplot(2,1,2), plot(real(rx_data1)); title('Real part of rx signal transposed');

%% Coarse frequency sync
% Find frequency peak 
rx_signal_fft=abs(fftshift(fft(rx_data1)));
[~, freq_peak_index]=max(rx_signal_fft);

% Find frequency shift
frequency_offset=(freq_peak_index*2*pi/length(rx_signal_fft)-pi)*fs/(2*pi);

% Create time vector
t=linspace(0,(length(rx_data1)-1)/fs,length(rx_data1));

% Correct for frequency shift off set
rx_siganl_demodulated=rx_data1.*exp(-1i*2*pi*frequency_offset*t);

figure()
subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power');title('Power spetrum of rx signal before coarse frequency correction'); 
subplot(2,1,2), pwelch(rx_siganl_demodulated,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after coarse frequency correction');

%% Matched filtering
% Creat MF
MF = fliplr(conj(root_raised_cosine_pulse));

% Filter with MF
rx_signal_MF = filter(MF, 1, rx_siganl_demodulated);

% Plot
figure()
subplot(2,2,2), pwelch(rx_signal_MF,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after MF'); 
subplot(2,2,1), pwelch(rx_siganl_demodulated,[],[],[],fs,'centered','power');title('Power spetrum of rx signal before MF');
subplot(2,2,4), plot(real(rx_signal_MF)); title('Real part of rx signal after MF');
xlabel('Sample'); ylabel('Amplitude')
subplot(2,2,3), plot(real(rx_siganl_demodulated)); title('Real part of rx signal before MF');
xlabel('Sample'); ylabel('Amplitude')

%% Frame detection
% Calculate correlation
corr = conv(rx_signal_MF,fliplr(conj(preamble_upsampled_RRC)));

% Make sure correlation in not too early or too late on the signal.
corrMax = corr(frame_upsampled_length:(end-(frame_upsampled_length)));

% Find max peak
[corr_peak_val, ~] = max(abs(corrMax)); 

max_peak_val = corr_peak_val == abs(corr);
corr_peak_index = find(max_peak_val,1);  

% Cut frame from rx signal
rx_signal_cut = rx_signal_MF(corr_peak_index-preamble_upsampled_length-guard_upsampled_length-pilot_upsampled_length:corr_peak_index-preamble_upsampled_length-guard_upsampled_length-pilot_upsampled_length+frame_upsampled_length);

% Plot
figure(); 
subplot(1,2,1); hold on
plot(100*real(rx_signal_MF))
plot(abs(corr))
plot(corr_peak_index,0 , '*c')
plot(corr_peak_index-preamble_upsampled_length-guard_upsampled_length-pilot_upsampled_length,0 , '*m')
plot(corr_peak_index-preamble_upsampled_length-guard_upsampled_length-pilot_upsampled_length+frame_upsampled_length,0 , '*m')
title('Correlation and received signal'); xlabel('Sample'); ylabel('Amplitude')
legend('Rx signal', 'Correlation', 'Correlation peak index', 'Start and end of frame')
subplot(1,2,2); plot(real(rx_signal_cut))
title('The detetcted frame'); xlabel('Sample'); ylabel('Amplitude')
legend('Rx frame')

%% Downsampling
% Downsample
rx_signal_downsampled=downsample(rx_signal_cut(1:end), fsfd);

% Plot
figure()
subplot(2,2,1); plot(real(rx_signal_cut)); 
title('Real part of rx signal before downsampling'); xlabel('Sample'); ylabel('Amplitude')
subplot(2,2,2); plot(real(rx_signal_downsampled)); 
title('Real part of rx signal after downsampling'); xlabel('Sample'); ylabel('Amplitude')
subplot(2,2,3); plot(rx_signal_cut,'.'); xlabel('In-Phase'); ylabel('Quadrature')
title('Scatter plot of rx signal before downsampling');
subplot(2,2,4); plot(rx_signal_downsampled,'.'); xlabel('In-Phase'); ylabel('Quadrature')
title('Scatter plot of rx signal after downsampling');

% figure()
% subplot(1,2,2); plot(real(rx_signal_downsampled)); 
% title('Real part of rx signal after downsampling'); xlabel('Sample'); ylabel('Amplitude')
% subplot(1,2,1); plot(rx_signal_downsampled,'.'); 
% title('Scatter plot of rx signal after downsampling'); xlabel('In-Phase'); ylabel('Quadrature')

%% Find preamble for frequency sync
% Correaltion between downsampled rx signal and tx preamble
corr_second = conv(rx_signal_downsampled, fliplr(conj(preamble)));

[~, message_start_index] = max(abs(corr_second)); 

% % Plot correlation 
% figure(); hold on
% plot(abs(corr_second))
% plot(message_start_index,0 , '*c')
% xlabel('Sample'); ylabel('Amplitude')
% title('Correlation of downsampled received signal and transmitted preamble');

% Cut out preamble from downsampled signal
rx_preamble_for_freq_sync = rx_signal_downsampled(message_start_index - preamble_length+1:message_start_index);

% Plot
figure()
subplot(4,1,1), pwelch(rx_preamble_for_freq_sync,[],[],[],fs,'centered','power');title('Power spetrum of rx preamble');
subplot(4,1,2), pwelch(preamble,[],[],[],fs,'centered','power');title('Power spetrum of tx preamble'); 
subplot(4,1,3), plot(real(rx_preamble_for_freq_sync)); title('Real part of rx preamble'); 
xlabel('Sample'); ylabel('Amplitude')
subplot(4,1,4), plot(real(preamble)); title('Real part of tx preamble'); 
xlabel('Sample'); ylabel('Amplitude')

scatterplot(rx_preamble_for_freq_sync); title('Scatter plot rx preamble')

%% Frequency sync

% Calculate phase difference between transmitted preamble and received preamble
diff_angle_preamble=unwrap(angle(preamble)-angle(rx_preamble_for_freq_sync));

% Find coefficients to polyfit line
c = polyfit(1:length(diff_angle_preamble),diff_angle_preamble,1);
% Find coefficients to polyfit line
polyfit_line = polyval(c,1:length(diff_angle_preamble));

% Frequency offset is the slope of the fitted line
freq_grad=(polyfit_line(1)-polyfit_line(end))/length(polyfit_line);

% Calculate frequency shift
freq_shifts=freq_grad*(1:length(rx_signal_downsampled));

% Correct for frequency shift
rx_signal_freq_corr=rx_signal_downsampled.*exp(-1i*freq_shifts);

% Plot
figure()
subplot(1,2,1); plot(rx_signal_downsampled,'.'); 
title('Downsampled frame before frequency syncronization'); xlabel('In-Phase'); ylabel('Quadrature')
subplot(1,2,2); plot(rx_signal_freq_corr,'.');
title('Downsampled frame after frequency syncronization'); xlabel('In-Phase'); ylabel('Quadrature')

%% Cut out preamble and symbols from frequency corrected frame
rx_symbols=rx_signal_freq_corr(message_start_index+1:message_start_index+ message_length);
rx_preamble=rx_signal_freq_corr(message_start_index-preamble_length+1:message_start_index);

%% Phase correction

% Calculate phase difference between received preamble and correct preamble
diff_angle_preamble=unwrap(angle(preamble)-angle(rx_preamble));

% Produce a fitted line from angle of product
c = polyfit(1:length(diff_angle_preamble),diff_angle_preamble,1);
polyfit_line = polyval(c,1:length(diff_angle_preamble));

% Phase offset is the value of first phase
phase_offset = exp(1i*polyfit_line(1));

% Corrects Phase
rx_symbols_phase_corr = rx_symbols.*phase_offset;

figure(), plot(rx_symbols_phase_corr,'.'), title('Message symbols after phase syncronization')

% Plot
figure()
subplot(1,2,1); plot(rx_signal_freq_corr,'.'); 
title('Message symbols before phase syncronization'); xlabel('In-Phase'); ylabel('Quadrature')
subplot(1,2,2); plot(rx_symbols_phase_corr,'.');
title('Message symbols after phase syncronization'); xlabel('In-Phase'); ylabel('Quadrature')


%% Scale symbols

% Calculate scaling
scale=mean(abs(const))/ mean(abs(rx_symbols_phase_corr));

% Scale signal
rx_symbols_scaled=rx_symbols_phase_corr*scale;

figure(), hold on
plot(rx_symbols_scaled,'.')
plot(const,'rx'),
title('Comparision between constallation with received message symbols');
xlabel('In-Phase'); ylabel('Quadrature')

%% Demapp message to bits

% Calculate distance from rx symbols to const
distance=abs(repmat(rx_symbols_scaled.', 1, length(const)) - repmat(const, length(rx_symbols_scaled), 1)).^2;

% Find min distance (closest const symbol)
[~, rx_message_dec] = min(distance, [], 2); 

% Convert from decimal to bin
rx_message_zeropadded = de2bi(rx_message_dec.' - 1,m,'left-msb').';

% Reshape bits
rx_message_zeropadded_reshaped = rx_message_zeropadded(:);

% Remove zeropadding
rx_message_binary=rx_message_zeropadded_reshaped(1:length(message_binary))';

% Convert bin to string
rx_message= char(bin2dec(reshape(char(rx_message_binary + '0'),8, [])'))'

% Calculating BER
bit_error = nnz(rx_message_binary-message_binary); 

% View results and comparision 
disp(['Transmitted message:  ', message])
disp(['Received message:     ', rx_message ]) 
disp(['Bit errors: ', num2str(bit_error)])
