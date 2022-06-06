% Transmitter 4-PSK

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
fc = 30e6;                              % Carrier frequency
Rb = 50e3; %80e3;                       % Bitrate

M = 4;                                 % Modulation order
m =log2(M);                             % Bits per symbol

Rs = Rb/m;                              % Symbolrate
Ts_symbol=1/Rs;                         % Symbol time
fs_symbol=fs/Rs;                        % Samples per symbol
Ts= 1/fs;                               % Sampling time
fsfd = floor(fs/Rs); 

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
message_divided = buffer(message_binary, m)';  

% Convert frame from binary to decimal
message_dec = bi2de(message_divided, 'left-msb')'+1; 

% Modulate to symbols
frame = const(message_dec);
message_length=length(frame);
message_upsampled_length=length(frame)*fsfd;

% Create frame
frame = 0.2*[guard pilot preamble frame]; 
frame_length=length(frame);

% Upsample
frame_upsampled = upsample(frame, fsfd); % increases the sample rate of 'symbols' by inserting fsfd (1 bit) between samples => enable pulse shap. using conv.
frame_upsampled_length=length(frame_upsampled);

% Put symbols on pulse
% Conv with RTRC
tx_signal = conv(root_raised_cosine_pulse, frame_upsampled); 

% Normalize and transpose
tx_signal_norm = tx_signal/max(abs(tx_signal));
tx_signal_norm= tx_signal_norm.';


%% Send to radio tx

% Setup the Tx
tx_signal = comm.SDRuTransmitter(... 
    'Platform','N200/N210/USRP2',...
    'IPAddress','192.168.10.2',... 
    'CenterFrequency',fc,...
    'EnableBurstMode',1,...
    'NumFramesInBurst',1,...
    'InterpolationFactor',Interp_Factor,...
    'MasterClockRate',MasterClock_Rate,...
    'TransportDataType','int16');
currentTime = 0;

% Send to TX USRP
for k = 1:20000 
    tx_signal(tx_signal_norm);
    disp('transmitting') 
    currentTime = currentTime+frame_time  
end
release(tx_signal);   
