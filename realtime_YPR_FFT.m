%% Initialization
close all; clear all; clc;

%% Create serial object for Arduino
baudrate = 115200;
s = serial('COM6','BaudRate',baudrate); % change the COM Port number as needed
s.ReadAsyncMode = 'continuous';
set(s,'InputBufferSize',100000);


%% Connect the serial port to Arduino
try
    fopen(s);
catch err
    fclose(instrfind);
    error('Make sure you select the correct COM Port where the Arduino is connected.');
end


%% Read and plot the data from Arduino on to the graphs
Tmax = 60; Ts = 0.02; ata = 0;
FLAG_CASTING = false;
Flag_Initializing = true;
Angles=zeros(1,4); %initialize 1x2 matrix with zeroes

DATA_T=[];
DATA_R=[];
DATA_P=[];
DATA_Y=[];

while(Flag_Initializing)
    while(strcmp(s.TransferStatus,'read'))
        pause(0.01);
    end    
    readasync(s);
    sms = fscanf(s);
    if ~strcmp(sms(1:3),'ypr')
        fprintf(sms)
    else
        Flag_Initializing = false;
    end
end

i=1;
while i <= 1024   
idx = [];

    while isempty(idx)
        
            sms = fscanf(s);

            idx = find(sms=='r');
            if ~isempty(idx)
                idx = idx(end)+1;
                Angles = sscanf(sms(idx:end),'%f %f %f %f');
            end
    end    

time = Angles(4)/1000000;


    disp([Angles(1),Angles(2)]); 
    DATA_T=[DATA_T,time];
    DATA_R=[DATA_R,Angles(3)];
    DATA_P=[DATA_P,Angles(2)];
    DATA_Y=[DATA_Y,Angles(1)];
    
    if(mod(length(DATA_T),5)==0)
        subplot(3,1,1);
        plot(DATA_T, DATA_Y,'b');
        axis([-6+time, time, -80, 200]);
        hold on;
        
        subplot(3,1,2);
        plot(DATA_T, DATA_P,'k');
        axis([-6+time, time, -80, 200]);
        hold on;
        
        subplot(3,1,3);
        plot(DATA_T, DATA_R,'m');
        axis([-6+time, time, -80, 200]);
        hold on;
    end
    
    i=i+1;  
    drawnow;
end

fclose(s);



















