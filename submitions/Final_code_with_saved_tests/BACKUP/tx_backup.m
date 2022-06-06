%%% TRANSMITTER 
%%% GROUP: Walkie Talkies 
clear all; close all; clc;

%% This is the sampling rate for the digital mixer, do not change
MasterClock_Rate= 100000000;
%% Interpolation factor for the Transmitter
Interp_Factor = 64;
%% Decimation factor for the Receiver
Decimation_Factor = Interp_Factor;
%% Sampling rate and time
fs = MasterClock_Rate/Interp_Factor;    % sampling rate
dt = 1/fs;
N = 10000;                              % Number of samples in a frame
frame_time = N/fs;                      % Time for 1 frame
time = (0:dt:dt*(N-1))';
RBW = 1/frame_time;
NFFT = 2^nextpow2(N);                   % Next power of 2 from length of y

fc = 30e6;                              % carrier frequency, iequal to fRF
Rb = 50e3;                              % bit rate (could have been changed)

%% message to send 
%message=['His connection interested so we a sympathize advantages To said is it shed want do Occasional Middletons everything so too Have spot part for his quit may Enable it is square my a regard Often merit stuff first oh up hills as he Servants contempt as although addition Dashwood'];
message=['Hello, is it receiveing? Hello, is it receiveing? Hello, is it receiveing?'];
%message=['His connection interested so we a sympathize advantages To said is it shed want do Occasional Middletons everything so too '];

bits = logical(reshape(dec2bin(message, 8).',1,[])- '0');
flag = 0;
switch flag
    case 0
        const =  [1+1i 1-1i -1-1i -1+1i]/sqrt(2); % Constellation 1 - QPSK/4-QAM 
        preamb = [0 1;1 1;1 0;1 1;0 1;0 0;1 1;1 0;1 0;0 0;0 1;0 1;0 1;0 1;1 1;0 0;0 1;0 1;1 1;0 0;0 1;0 1;1 1;0 0;0 1;...
            0 1;0 0;0 0;1 1;1 0;1 1;0 1;0 0;0 0;0 0;0 0;1 1;0 0;0 1;0 0;1 0;0 0;0 0;1 1;1 0;0 0;1 0;0 0;1 1;1 0];
        preamble = const(bi2de(preamb, 'left-msb')'+1);
    case 1
        const = [1+3i 3+3i 3+1i 1+1i 1-1i 3-1i 3-3i 1-3i -3-1i -1-1i -1-3i -3-3i -3+3i -1+3i -1+1i -3+1i]/sqrt(2); % Constellation 2 - 16QAM
        preamb = [0,0,1,1; 1,0,0,0; 1,0,0,0; 1,1,1,0; 1,0,1,0; 1,0,0,1; 1,0,1,1; 0,0,0,0; 1,1,1,1; 1,0,1,1; 0,1,1,1; ...
            1,1,0,0; 0,0,1,0; 0,0,0,0; 0,1,0,0; 1,1,0,0; 0,0,1,0; 1,1,0,0; 0,0,0,1; 0,0,1,0; 1,1,0,1; 0,0,1,1; 1,0,0,1; ...
            1,0,0,1; 0,1,1,1; 0,0,0,1; 0,1,0,1; 0,1,1,1; 1,0,1,0; 0,1,1,0; 1,1,1,1; 1,1,0,0; 1,0,0,0; 1,1,1,0; 0,1,0,0; ...
            1,0,1,1; 1,0,1,1; 1,1,1,0; 0,1,0,0; 0,0,1,1; 0,0,1,0; 1,1,0,0; 0,1,0,0; 0,0,0,0; 1,1,0,1; 0,0,1,1; 0,1,0,1; ... 
            1,1,1,1; 0,0,1,0; 0,1,1,1];
        preamble = const(bi2de(preamb, 'left-msb')'+1);
end
%preamble = [1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1 -1 1 1 1 1 1 1 -1 -1 1 1 -1 1];
M = length(const);                                                          % Number of symbols in the constellation
bits_length = length(bits);                                                 % bits length
bitperSymb = log2(M);                                                       % Number of bits per symbol
Rs = Rb/bitperSymb;                                                         % Calculate Symbole rate [symbole/Hz]
Tsymb = 1/Rs;                                                               % Symbole Time/Duration [Symbole/sec] 
fsfd = floor(fs/Rs);                                                        % Number of samples per symbol [samples/symb] (it better to have it as interger)
rolloff = 0.3;
span = 4;
Bw = (1+rolloff)/(2*Tsymb);                                                 % Bandwidth
msg = buffer(bits, bitperSymb)';                                            % Bits to Messages, Group bits into bits per symbol
msg_idx = bi2de(msg, 'left-msb')'+1;                                        % Message to symbol index
symbol = const(msg_idx);

 
pilot = ones(1,300);
delay = zeros(1,50);

message_symbols = 0.2*[delay pilot preamble symbol]; 


% upsamplig 
symbols_upsample = upsample(message_symbols, fsfd); % increases the sample rate of 'symbols' by inserting fsfd (1 bit) between samples => enable pulse shap. using conv.

% pulse convolution 
[pulse, ~] = rtrcpuls(rolloff, Tsymb, fs, span); 
tx = conv(pulse, symbols_upsample); 
tx = tx/max(abs(tx));

s_tx = tx.'; % this is the generated signal we send (real)

%% First correlation 
preamble_upsample = upsample(preamble,fsfd); 
preamble_upsample = conv(preamble_upsample, pulse, 'full'); 
[first_corr,lag] = xcorr(s_tx,preamble_upsample);
% first_corr = conv(MF_output,fliplr(conj(preamble_upsample)));
% figure(), plot(abs(first_corr)), title('Correlation for frame syncronization')

figure, pwelch(s_tx,[],[],[],fs,'centered','power'),xlim([-50 50]) % spectrum of transmitted signal

%% Setup the Tx
tx = comm.SDRuTransmitter(... 
'Platform','N200/N210/USRP2',...
'IPAddress','192.168.10.2',... % since that one is at our station
'CenterFrequency',fc,...         % added freq noise here
'EnableBurstMode',1,...
'NumFramesInBurst',1,...
'InterpolationFactor',Interp_Factor,...
'MasterClockRate',MasterClock_Rate,...
'TransportDataType','int16');
%% Sending the signal  
currentTime = 0;
for k = 1:20000 % transmit many times...
    tx(s_tx);
    if k==1
        disp('transmitting')                % just to see that it has started
    end
    currentTime = currentTime+frame_time    % just to see that it is processing
end
release(tx);                                % must be don, otherwise cannot start transmission agian (for functioning of USRP)
