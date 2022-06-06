% Receiver 4-PSK
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
Rb = 100e3;                             % Bitrate

M = 4;                                  % Modulation order
m =log2(M);                             % Bits per symbol

Rs = Rb/m;                              % Symbolrate
Ts_symbol=1/Rs;                         % Symbol time
Ts= 1/fs;                               % Sampling time
fsfd = floor(fs/Rs);                    % Samples per symbol

% Modulation
const = [1+1i 1-1i -1-1i -1+1i]/sqrt(2);         % Constellation for 4-PSK

% Initialize RTRC
rolloff = 0.3;  tau = 1/Rs;  span = 4;
root_raised_cosine_pulse = rtrcpuls(rolloff,tau,fs,span);

%% Create preamble, pilot, guard, message and MF
% Preamble
preamble = [0 1;1 1;1 0;1 1;0 1;0 0;1 1;1 0;1 0;0 0;0 1;0 1;0 1;0 1;1 1;0 0;0 1;0 1;1 1;0 0;0 1;0 1;1 1;0 0;0 1;...
            0 1;0 0;0 0;1 1;1 0;1 1;0 1;0 0;0 0;0 0;0 0;1 1;0 0;0 1;0 0;1 0;0 0;0 0;1 1;1 0;0 0;1 0;0 0;1 1;1 0];

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
load('test_4psk_longer_message_and_higher_bitrate');
%close all

%% Plot received signal
figure()
subplot(2,1,1), pwelch(rx_data,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal');
subplot(2,1,2), plot(real(rx_data)); title('Real part of rx signal');

figure()
subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal transposed');
subplot(2,1,2), plot(real(rx_data1)); title('Real part of rx signal transposed');

%% Coarsa frequency sync
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
subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power');title('Power spetrum of rx signal'); 
subplot(2,1,2), pwelch(rx_siganl_demodulated,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after coara freq sync');

%% Matched filtering
% Creat MF
MF = fliplr(conj(root_raised_cosine_pulse));

% Filter with MF
rx_signal_MF = filter(MF, 1, rx_siganl_demodulated);

% Plot
figure()
subplot(2,1,1), pwelch(rx_signal_MF,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after MF'); 
subplot(2,1,2), plot(real(rx_signal_MF)); title('Real part of rx signal after MF');

%% Frame detection, fixa denna
% Calculate correlation
corr = conv(rx_signal_MF,fliplr(conj(preamble_upsampled_RRC)));

% Make sure correlation in not too early or too late on the signal.
corrMax = corr(frame_upsampled_length:(end-(frame_upsampled_length)));

% Find max peak
[corr_peak_val, ~] = max(abs(corrMax)); 

max_peak_val = corr_peak_val == abs(corr);
corr_peak_index = find(max_peak_val,1);  

% Plot
figure(); hold on
plot(100*real(rx_signal_MF))
plot(abs(corr))
plot(corr_peak_index,0 , '*c')
plot(corr_peak_index-preamble_upsampled_length,0 , '*m')
title('Correlation and received signal');
legend('Rx signal', 'Correlation', 'Correlation peak index (end of preamble)', 'Preamble start according to correlation peak')

%% Cut frame
rx_signal_cut = rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length):corr_peak_index-2*(preamble_upsampled_length)+frame_upsampled_length);

figure(); hold on
subplot(2,1,1)
plot(real(rx_signal_cut))
title('The cuted frame');
subplot(2,1,2) ; hold on
plot(real(rx_signal_MF))
plot(corr_peak_index-2*(preamble_upsampled_length), real(rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length))),'*')
plot(corr_peak_index-2*(preamble_upsampled_length)+frame_upsampled_length, real(rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length)+frame_upsampled_length)),'*')
title('The start and end point of the cuted frame from rx signal');

%% Time sync, option 1
% Create empty matrix for time errors
time_error = zeros(1, length(rx_signal_cut)-1);

steps = 1;
% Step through signal to calculate time error from every "step"-th sample
for i = 1:length(time_error)-steps
    sample1 = rx_signal_cut(:,i);
    sample2 = rx_signal_cut(:,i+steps);
    time_error(i) = abs(sample2-sample1);
end 

% Caluculate mean time error
time_error_mean = floor(length(time_error)/fsfd); 

% Calculate zeros to pad
zeros_to_pad = zeros(1, (fsfd - (length(time_error) - time_error_mean * fsfd)));                     % zeros for padding
% Add zeros
time_error_padded = [time_error, zeros_to_pad];

% Reshape vector
time_error_reshape = reshape(time_error_padded, [fsfd, length(time_error_padded)/fsfd]).';

% Calculate sum for each column
[~, time_diff] = min(sum(time_error_reshape));                                  % get best sample time after removed guard

% Plot eyediagram
eyediagram(rx_signal_cut(time_diff:end), fsfd)

%% Time sync, option 2

%Index for mid signal sample + length of RRC to make sure the signal has started.
mid_index=corr_peak_index+length(root_raised_cosine_pulse);
start_index_time_sync=corr_peak_index;

% Distance between samples
sample_distance=round(fsfd/2);
step=0;

for i=1:50
    early_signal=rx_signal_MF(mid_index-sample_distance);
    mid_signal=rx_signal_MF(mid_index);
    late_signal=rx_signal_MF(mid_index+sample_distance);
   
    time_error_real=real(mid_signal)*(real(early_signal)-real(late_signal));
    time_error_imag=imag(mid_signal)*(imag(early_signal)-imag(late_signal));
    
    tot_time_error=time_error_real+time_error_imag;
    
    if tot_time_error > 1e-06                           %if error is smaller than 0 add one to the position of your mid
        step=step-1;
    elseif tot_time_error < -1e-06                       %if error is smaller than 0 add one to the position of your mid
        step=step+1;
    else
        step=step+0;                            %if there is no error then set delta to 0
    end
    
    mid_index=mid_index+fsfd+step;
    
    start_index_time_sync=start_index_time_sync+step;
    i=i+1;
    
    if  mid_index >= length(rx_signal_MF)-sample_distance-1
        break;
    end
end

diff=abs(corr_peak_index-start_index_time_sync);

eyediagram(rx_signal_cut(diff+1:end), fsfd)

%% Downsampling
% Downsample from time_error index
rx_signal_downsampled=downsample(rx_signal_cut(time_diff:end), fsfd);
% diff or time diff depending on what time sync method

figure()
plot(real(rx_signal_downsampled))
figure(2)
plot(rx_signal_downsampled,'.')

%% Find preamble for frequency sync

% Correaltion between downsampled rx signal and tx preamble
corr_second = conv(rx_signal_downsampled, fliplr(conj(preamble)));

[~, message_start_index] = max(abs(corr_second)); 

figure(); hold on
plot(abs(corr_second))
plot(message_start_index,0 , '*c')
title('Correlation of downsampled received signal and transmitted preamble');

% Cut out preamble from downsampled signal
rx_preamble_for_freq_sync = rx_signal_downsampled(message_start_index - preamble_length+1:message_start_index);

figure()
subplot(4,1,1), pwelch(rx_preamble_for_freq_sync,[],[],[],fs,'centered','power');title('Power spetrum of rx preamble');
subplot(4,1,2), pwelch(preamble,[],[],[],fs,'centered','power');title('Power spetrum of tx preamble'); 
subplot(4,1,3), plot(real(rx_preamble_for_freq_sync)); title('Real part of rx preamble'); 
subplot(4,1,4), plot(real(preamble)); title('Real part of tx preamble'); 
scatterplot(rx_preamble_for_freq_sync)

%% Frequency sync

% Calculate phase difference between transmitted preamble and received preamble
diff_angle_preamble=unwrap(angle(preamble)-angle(rx_preamble_for_freq_sync));

%freq_grad=mean(diff(diff_angle_preamble))/(2*pi)

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
figure(), plot(rx_signal_freq_corr,'.'), title('Downsampled signal after frequency syncronization')


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

figure(), plot(rx_symbols_phase_corr,'.'), title('Downsampled signal after phase syncronization')

%% Scale symbols

% Calculate scaling
scale=mean(abs(const))/ mean(abs(rx_symbols_phase_corr));

% Scale signal
rx_symbols_scaled=rx_symbols_phase_corr*scale;

figure(), hold on
plot(rx_symbols_scaled,'b.')
plot(const,'rx'),
title('Comparision between constallation with received symbols');
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
