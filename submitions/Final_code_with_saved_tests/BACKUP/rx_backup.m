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
Rb = 50e3; %80e3;                              % Bitrate

%M = 16;                                 % Modulation order
M=4;
m =log2(M);                             % Bits per symbol

Rs = Rb/m;                              % Symbolrate
Ts_symbol=1/Rs;                         % Symbol time
Ts= 1/fs;                               % Sampling time
fsfd = floor(fs/Rs);                    % Samples per symbol

% Modulation
%const = [1+3i 3+3i 3+1i 1+1i 1-1i 3-1i 3-3i 1-3i -3-1i -1-1i -1-3i -3-3i -3+3i -1+3i -1+1i -3+1i]/sqrt(2);         % Constellation for 16-QAM
const = [1+1i 1-1i -1-1i -1+1i]/sqrt(2);

% Initialize RTRC
rolloff = 0.3;  tau = 1/Rs;  span = 4;
root_raised_cosine_pulse = rtrcpuls(rolloff,tau,fs,span);

bw = (1+rolloff)/(2*Ts_symbol);             % Bandwidth 

%% Create preamble, pilot, guard, message and MF
% Preamble
% preamble = [0,0,1,1; 1,0,0,0; 1,0,0,0; 1,1,1,0; 1,0,1,0; 1,0,0,1; 1,0,1,1; 0,0,0,0; 1,1,1,1; 1,0,1,1; 0,1,1,1; ...
%             1,1,0,0; 0,0,1,0; 0,0,0,0; 0,1,0,0; 1,1,0,0; 0,0,1,0; 1,1,0,0; 0,0,0,1; 0,0,1,0; 1,1,0,1; 0,0,1,1; 1,0,0,1; ...
%             1,0,0,1; 0,1,1,1; 0,0,0,1; 0,1,0,1; 0,1,1,1; 1,0,1,0; 0,1,1,0; 1,1,1,1; 1,1,0,0; 1,0,0,0; 1,1,1,0; 0,1,0,0; ...
%             1,0,1,1; 1,0,1,1; 1,1,1,0; 0,1,0,0; 0,0,1,1; 0,0,1,0; 1,1,0,0; 0,1,0,0; 0,0,0,0; 1,1,0,1; 0,0,1,1; 0,1,0,1; ... 
%             1,1,1,1; 0,0,1,0; 0,1,1,1];
%       
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

delay = zeros(1,50); 
delay_length=length(delay);
delay_upsampled_length=length(delay)*fsfd;

% Message
%message=['His connection interested so we a sympathize advantages To said is it shed want do Occasional Middletons everything so too Have spot part for his quit may Enable it is square my a regard Often merit stuff first oh up hills as he Servants contempt as although addition Dashwood'];
%message=['His connection interested so we a sympathize advantages To said is it shed want do Occasional Middletons everything so too '];
message=['Hello, is it receiveing? Hello, is it receiveing? Hello, is it receiveing?'];

% Message: string to bits
%message_binary=reshape(dec2bin(message',m)-'0', 1, []);
message_binary = logical(reshape(dec2bin(message, 8).',1,[])- '0');

%%%% Zeropadd message 
% Calculate how many zeros to add
even_bit_number=ceil(length(message_binary)/m);
zeros_to_add=zeros(1,(even_bit_number*m)-length(message_binary));
% Add zeros to frame
zeropadded_message=[message_binary zeros_to_add];

%%%% Mapp message
% Reshape to m-vector of bits
message_divided=reshape(zeropadded_message,m, [])';
% Convert frame from binary to decimal
message_divided_dec= bi2de(message_divided,'left-msb')'+1;

% Modulate to symbols
message_symbols= const(message_divided_dec);
message_length=length(message_symbols);
message_upsampled_length=length(message_symbols)*fsfd;
%scatterplot(symbols)

%%%% Create frame
frame=0.2*[delay pilot preamble message_symbols];
frame_length=length(frame);

%%%% Upsample
frame_upsampled = upsample(frame, fsfd);
frame_upsampled_length=length(frame_upsampled);

%%%% Put symbols on pulse
% Conv with RTRC
tx_signal = conv(frame_upsampled, root_raised_cosine_pulse);

% Normalize
tx_signal_norm = (tx_signal/max(abs(tx_signal)))';

%% Set up the RX
rx = comm.SDRuReceiver(...
    'Platform','N200/N210/USRP2',...
    'IPAddress','192.168.10.10',... 
    'CenterFrequency',fc,...
    'EnableBurstMode',1,...
    'NumFramesInBurst',1,...
    'DecimationFactor',Decimation_Factor,...
    'SamplesPerFrame',30*N,...
    'MasterClockRate',MasterClock_Rate,...
    'TransportDataType','int16');

currentTime = 0;

% Receiving the signal
for i=1:1
    [rx_data] = step(rx);
    rx_data = double(rx_data)/(2^16);
    
    [rx_dsb,f] = periodogram(rx_data,hamming(length(rx_data)),NFFT,fs,'centered');
    rx_dsb = 10*log10(RBW*rx_dsb)+15;% In dBm
    figure(1), subplot(2,1,1), pwelch(rx_data,[],[],[],fs,'centered','power');title('Power spetrum of rx signal'); % spectrum of received signal after LP
    figure(1), subplot(2,1,2),plot(real(rx_data)); hold on; plot(real(rx_data)); title('Real part of rx signal'); % received signal after LP

    rx_data1 = conj(rx_data.');                % conj back since HW has flipped to signal
    currentTime=currentTime+frame_time             % just to see that it is working
end

release(rx);                                    

%% Load saved rx signal
% load('workspace_h_shorter_message');
% rx_data1=signal_rx;
% 
% close all

%% Plot received signal
figure()
subplot(2,1,1), pwelch(rx_data,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal');
subplot(2,1,2), plot(real(rx_data)); title('Real part of rx signal');

figure()
subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power'); title('Power spetrum of rx signal transposed');
subplot(2,1,2), plot(real(rx_data1)); title('Real part of rx signal transposed');

%% Coarsa frequency sync and down conversion to baseband, testa om denna funkar
% Find frequency peak 
rx_data_fft=abs(fftshift(fft(rx_data1)));
[~, index_peak]=max(rx_data_fft);

% Find frequency shift
frequency_offset=(index_peak*2*pi/length(rx_data_fft)-pi)*fs/(2*pi);

% Create time vector
t=linspace(0,(length(rx_data1)-1)/fs,length(rx_data1));
% Istället för t använda T_samp?

% Correct for frequency shift set
rx_data_demodulated=rx_data1.*exp(-1i*2*pi*frequency_offset*t);

figure()
subplot(2,1,1), pwelch(rx_data1,[],[],[],fs,'centered','power');title('Power spetrum of rx signal'); 
subplot(2,1,2), pwelch(rx_data_demodulated,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after caorsa freq sync');

%% Matched filtering
% Creat MF
MF = fliplr(conj(root_raised_cosine_pulse));
MF = MF/max(abs(MF));

% Filter with MF
rx_signal_MF = conv(MF, rx_data_demodulated);
rx_signal_MF = filter(MF, 1, rx_data_demodulated);

rx_signal_MF=rx_signal_MF./max(abs(rx_signal_MF));

% Plot
figure()
subplot(2,1,1), pwelch(rx_signal_MF,[],[],[],fs,'centered','power');title('Power spetrum of rx signal after MF'); 
subplot(2,1,2), plot(real(rx_signal_MF)); title('Real part of rx signal after MF');

%% Frame detection
% Calculate correlation
corr = conv(rx_signal_MF,fliplr(conj(preamble_upsampled_RRC)));

% Find max correlation
[~, corr_peak_index] = max((corr)); 

for div = 0.5:0.5:1.5                   
     [corr_peak_index((div-0.5)/0.5+1), ~] = max(abs(corr(div*1e5:(div+0.5)*1e5)));         
end

max_value = max(corr_peak_index) == abs(corr);                            % match the max correlation with first correlation
corr_indx = find(max_value,1);                                              % findind the location of max corr to start sync

corr_peak_index=corr_indx;

% Plot
figure(); hold on
%plot(100*real(rx_signal_MF))
plot(abs(corr))
plot(corr_peak_index-preamble_upsampled_length,0 , '*c')
plot(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length),0, '*m')
title('Corrolation Overlap with Received Message');

%% Cut frame

MF_output_fram_sync = rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length):corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length)+frame_upsampled_length).'; % start form correlation minus premable etc and go to equal point + length of data 

figure(); hold on
subplot(2,1,1)
plot(real(MF_output_fram_sync))
title('The Cuted Frame from Received Signal');
subplot(2,1,2) ; hold on
plot(real(rx_signal_MF))
plot(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length), real(rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length))),'*')
plot(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length)+110000, real(rx_signal_MF(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length)+110000)),'*')
title('The Start and End Point of Cuted Frame from Received Signal');

%% Time sync
% reshaped the match filter output accoring to sampling size. creates vector with fsfd=125 columns for
% whole data and see where we should start sampling (1-125)
% find difference between them and min should be at most open eye!

MF_output_fram_sync = MF_output_fram_sync.';

error = zeros(1, length(MF_output_fram_sync)-1); % create empty matrix for errors
%[~, error_size] = size(error);
steps = 1; % number of process time, chose 3 to be equal to third of every data point. 
for i = 1:length(error)-steps
    indx1 = MF_output_fram_sync(:,i);
    indx2 = MF_output_fram_sync(:,i+steps);
    error(i) = abs(indx2-indx1);
end 

varian = floor(length(error)/fsfd);                                         % the variance or the error in every simple 
zero_for_span = fsfd - (length(error) - varian * fsfd);                     % zeros for padding
error_span = [error, zeros(1, zero_for_span)];
%error_span = [error, zeros(1, fsfd)];
error_reshape = reshape(error_span, [fsfd, length(error_span)/fsfd]).';

[~, time_point] = min(sum(error_reshape));                                  % get best sample time after removed delay
%eyediagram(MF_output_fram_sync(time_point:end), fsfd)

time_point=121;
eyediagram(MF_output_fram_sync(time_point:end), fsfd)


%%

%% Time sync

% Index for mid signal sample + length of RRC to make sure the signal has started.
% mid_index=corr_peak_index+length(root_raised_cosine_pulse);
% start_index_time_sync=corr_peak_index;
% 
% % Symbls in rx signal
% %symbols_per_rx_signal=round(length(rx_signal_MF)/fsfd);
% 
% % Distance between samples
% sample_distance=round(fsfd/2);
% step=0;
% 
% for i=1:118
%     early_signal=rx_signal_MF(mid_index-sample_distance);
%     mid_signal=rx_signal_MF(mid_index);
%     late_signal=rx_signal_MF(mid_index+sample_distance);
%    
%     time_error_real=real(mid_signal)*(real(early_signal)-real(late_signal));
%     time_error_imag=imag(mid_signal)*(imag(early_signal)-imag(late_signal));
%     
%     tot_time_error=time_error_real+time_error_imag
%     
%     if tot_time_error > 1e-05                            %if error is smaller than 0 add one to the position of your mid
%         step=step-1;
%     elseif tot_time_error < -1e-05                        %if error is smaller than 0 add one to the position of your mid
%         step=step+1;
%     else
%         step=step+0;                            %if there is no error then set delta to 0
%     end
%     
%     mid_index=mid_index+fsfd+step;
%     
%     start_index_time_sync=start_index_time_sync+step;
%     i=i+1;
%     
%     if  mid_index >= length(rx_signal_MF)-sample_distance-1;
%         break;
%     end
% end
% 
% diff=corr_peak_index-start_index_time_sync
% 

%% Downsampling
%for i=1:fsfd

time_point=1;
rx_signal_MF_downsampled=downsample(MF_output_fram_sync(time_point:end), fsfd);

figure()
plot(real(rx_signal_MF_downsampled))
figure(2)
plot(rx_signal_MF_downsampled,'.')

%% Cut message and preamble from frame
corr_second = conv(rx_signal_MF_downsampled, fliplr(conj(preamble)));                  % convolution with preamble pulse to find correlation 

[maxCorr, pre_indx] = max(abs(corr_second)); 


figure(); hold on
plot(abs(corr_second))
plot(pre_indx,0 , '*c')
%plot(corr_peak_index-2*(preamble_upsampled_length+delay_upsampled_length),0, '*m')
title('Corrolation Overlap with Received Message');

% received PREAMBLE to use for frequency sync
rx_preamble = rx_signal_MF_downsampled(pre_indx - length(preamble)+1:pre_indx);


% rx_symbols=rx_signal_MF_downsampled(pre_indx:pre_indx+ message_length);
% 
% figure(1)
% subplot(4,1,1), pwelch(rx_symbols,[],[],[],fs,'centered','power');title('Power spetrum of rx symbols');
% subplot(4,1,2), pwelch(message_symbols,[],[],[],fs,'centered','power');title('Power spetrum of tx symbols'); 
% subplot(4,1,3), plot(real(rx_symbols')); title('Real part of rx symbols'); 
% subplot(4,1,4), plot(real(message_symbols)); title('Real part of tx symbols'); 

figure()
subplot(4,1,1), pwelch(rx_preamble,[],[],[],fs,'centered','power');title('Power spetrum of rx preamble');
subplot(4,1,2), pwelch(preamble,[],[],[],fs,'centered','power');title('Power spetrum of tx preamble'); 
subplot(4,1,3), plot(real(rx_preamble)); title('Real part of rx preamble'); 
subplot(4,1,4), plot(real(preamble)); title('Real part of tx preamble'); 

% scatterplot(rx_symbols)
scatterplot(rx_preamble)

%% Frequency sync option 1, ändra var namn och lite sånt
%we shift the frequency of the rx_preamble, which look like a line, then corrolate the shifted
%virsion with detected preamble (rx_preamble) and find the max power of
%corrolation then put it in matix, after that find the index of max power
%matrix and that will be our frequency offset.
% Bw_point = 200;                                                             % BW size from graph
% for i=-Bw_point:1:Bw_point                                                  % rang of the rx spectrum
%     initial =  i/1000;                                                      % offset increses step by step
%     temp = preamble.*exp(1i*initial*(0:length(rx_preamble)-1));   	        % calculate the signal for then
%     corr_second = conv(rx_preamble, fliplr(conj(temp)));                    % calc correlation, get when most accurate
%     [corr_second_val(i+Bw_point+1), ~] = max(abs(corr_second));             % find correlation 
%     offset(i+Bw_point+1) = initial;                                         % offset value in matrix
% end
% 
% [~,indx_final] = max(corr_second_val);                                      % find maximum correlation power index
% % figure, plot(offset,corr_second_val), title('Find frequency syncronization'),xlabel('Frequency shift [Hz]'),ylabel('Correlation value') % to see so right point, should be a maximum here!
% freq_offset = offset(indx_final)
% 
% rx_symbols_freq_corr = rx_signal_MF_downsampled.*exp(-1i*freq_offset*(0:length(rx_signal_MF_downsampled)-1)); % shift back
% 
% %rx_symbols_freq_corr=rx_signal_MF_downsampled.*exp(-1i*shifts);
% figure(), plot(rx_symbols_freq_corr,'.'), title('Downsampled signal after find freq offset')

%% Frequency sync option 2

% Calculate phase difference between transmitted preamble and received preamble
diff_angle_preamble=unwrap(angle(preamble)-angle(rx_preamble));

%freq_grad=mean(diff(diff_angle_preamble))/(2*pi)

% Find coefficients to polyfit line
c = polyfit(1:length(diff_angle_preamble),diff_angle_preamble,1);
% Find coefficients to polyfit line
polyfit_line = polyval(c,1:length(diff_angle_preamble));

% Frequency offset is the slope of the fitted line
% This one is slightliy off thats why +0.0002
freq_grad=(polyfit_line(1)-polyfit_line(end))/length(polyfit_line);%+0.0003;


% Calculate frequency shift
shifts=freq_grad*(1:length(rx_signal_MF_downsampled));

% Correct for frequency shift
rx_symbols_freq_corr=rx_signal_MF_downsampled.*exp(-1i*shifts);
figure(), plot(rx_symbols_freq_corr,'.'), title('Downsampled signal after find freq offset')
figure(), plot(real(rx_symbols_freq_corr)), title('Downsampled signal after find freq offset')


%% Cut out preamble and symbols from frequency corrected frame
rx_symbols=rx_symbols_freq_corr(pre_indx+1:pre_indx+ message_length);
rx_preamble_shifted=rx_symbols_freq_corr(pre_indx-preamble_length+1:pre_indx);
%rx_frame=rx_symbols_freq_corr(pre_indx-preamble_length+1:pre_indx-preamble_length+frame_length);


%% Phase correction

% Calculate phase difference between received preamble and correct preamble
diff_angle_preamble=unwrap(angle(preamble)-angle(rx_preamble_shifted));

% Produce a fitted line from angle of product
c = polyfit(1:length(diff_angle_preamble),diff_angle_preamble,1);
polyfit_line = polyval(c,1:length(diff_angle_preamble));

% Phase offset is the value of first phase
phaseOffset = exp(1i*polyfit_line(1));

% Corrects Phase
rx_symbols_phase_corr = rx_symbols.*phaseOffset;

figure(), plot(rx_symbols_phase_corr,'.'), title('Downsampled signal after phase syncronization')

%% Scale symbols
scale=mean(abs(const))/ mean(abs(rx_symbols_phase_corr));%+0.15;
%scale=std(const)/ sqrt(std(rx_symbols_phase_corr));
rx_scaled=rx_symbols_phase_corr*scale;

figure(), hold on
plot(rx_scaled,'g.')
plot(const,'o'),
title('comparison between consttellation with received symbol');
%% Demapp message to bits

% Calculate distance from rx symbols to const
distance=abs(repmat(rx_scaled.', 1, length(const)) - repmat(const, length(rx_scaled), 1)).^2;

% Find min distance (closest const symbol)
[~, symbols_rx] = min(distance, [], 2); 

% Convert from decimal to bin
msg_hat = de2bi(symbols_rx.' - 1,m,'left-msb').';

% Reshape bits
bits_reshape = msg_hat(:);

% Remove zeropadding
bits_reshape_removed_zeropadd=bits_reshape(1:length(message_binary))';

% Convert bin to string
rx_message= char(bin2dec(reshape(char(bits_reshape_removed_zeropadd + '0'),8, [])'))'

% Calculating BER
bit_error = nnz(bits_reshape_removed_zeropadd-message_binary); 

% View results and comparision 
disp(['Transmitted message:  ', message])
disp(['Received message:     ', rx_message ]) 
disp(['Bit errors: ', num2str(bit_error)])
%close all

%end