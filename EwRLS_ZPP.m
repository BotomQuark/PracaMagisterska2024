clear;

TestNone = 0;
Test20min = 1;
TestNonZeroStart = 1;

CurrentTest = 1;


if CurrentTest == TestNone
    Impulses = 40; %40 second test
    ImpulseTime = 2;
    ReferenceAmplitude = 150;
elseif CurrentTest == Test20min
    Impulses = 600; % 20 minute test
    ImpulseTime = 2;
    ReferenceAmplitude = 150;
elseif CurrentTest == TestNonZeroStart
    Impulses = 60;  %2 minute test
    ImpulseTime = 2;
    ReferenceAmplitude = 150;
else
    Impulses = 60; %2 minute test
    ImpulseTime = 2;
    ReferenceAmplitude = 150;
end


UartPort = "COM6";
AlgorithmTime = Impulses*ImpulseTime + 0.001;
SampleTime = 0.016;
NumberOfSamples = floor(AlgorithmTime/SampleTime)+1;
ReadErrorCnt = 0;

% data tables of last AlgorithmTime seconds
TestRpmTable = zeros(1, NumberOfSamples);
TestReferenceSignal = zeros(1, NumberOfSamples);
AlgorithmTimeTable = 0:SampleTime:AlgorithmTime;
ActualSentControl = zeros(1, NumberOfSamples);
ReferenceSignal = zeros(1, NumberOfSamples);


SerialPort = serialport(UartPort, 115200, 'FlowControl', "hardware");

CurrentRPM = 0;
RxData = zeros(1, 6);
TxData = zeros(1, 6);

ReadyToStart = 0;
IsDataValid = 0;

%model
%Tm = 0.082;
Tm = 0.2;
Km = 1;
ModelNumerator = Km;
ModelDenominator = [Tm 1];
ModelCont = tf(ModelNumerator, ModelDenominator);
ModelDiscrete = c2d(ModelCont, SampleTime, 'zoh')

Am0 = ModelDiscrete.Denominator{1}(1);
Am1 = ModelDiscrete.Denominator{1}(2);
Bm = ModelDiscrete.Numerator{1}(2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% CTRL %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Index = 1;
AddNoise = 0;
while Index <= NumberOfSamples

    if Index < 1/Impulses * NumberOfSamples
        ReferenceSignal(Index) = 0;
    elseif Index > (Impulses-1)/Impulses * NumberOfSamples
        ReferenceSignal(Index) = 0;
    else
        Threshold = 2;
        while Threshold < Impulses
            if Index < Threshold/Impulses * NumberOfSamples
                if Threshold == 4
                    % Start RLS after
                    RlsThreshold = Index;
                    RlsThresholdTime = Index * SampleTime;
                end
                if 0 == mod(Threshold,2)
                    ReferenceSignal(Index) = ReferenceAmplitude;
                else
                    ReferenceSignal(Index) = -ReferenceAmplitude;
                end
                Threshold = 100000; % end
            end
        Threshold = Threshold + 1;
        end
    end

    if AddNoise == 1
        if 0 ~= ReferenceSignal(Index)
            ReferenceSignal(Index) = ReferenceSignal(Index)+10*sin(Index*0.001);
        end
    end

    Index = Index + 1;
end

figure(10)
ModelOutputRef =  lsim(ModelDiscrete, ReferenceSignal, AlgorithmTimeTable);
close(10)

flush(SerialPort)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% RLS/ZPP %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RlsRpmTable = zeros(1, NumberOfSamples);
RlsReferenceTable = zeros(1, NumberOfSamples);
RlsControlTable = zeros(1, NumberOfSamples);

TransportDelay = 2;
ModelOrder = 1;
dP = ModelOrder*2;
ForgFact = 0.999;
TotalNumberOfSamples = size(AlgorithmTimeTable,2);

Phi = zeros(1, dP);

ThetaHat = zeros(TotalNumberOfSamples, dP);
P = 1000*eye(dP);
PTrace = zeros(1, NumberOfSamples);


if TestNonZeroStart == CurrentTest
    Index = 1;
    while Index < TotalNumberOfSamples
        ThetaHat(Index, 1) = -0.873679;
        ThetaHat(Index, 2) = 0.64629;
        Index = Index + 1;
    end
    
    P(1,1) = 6.2060e-07;
    P(1,2) = 2.8874e-06;
    P(2,1) = 2.8874e-06;
    P(2,2) = 1.4913e-05;

    RlsThreshold = 0;
    RlsThresholdTime = 0;
end

K = zeros(dP,1);

Yhat = zeros(TotalNumberOfSamples, 1);
E = zeros(TotalNumberOfSamples, 1);
ThetaArray = zeros(TotalNumberOfSamples, dP);

Y2hat = zeros(TotalNumberOfSamples, 1);

disp('- START TEST')
RlsIndex = 10;
while RlsIndex < NumberOfSamples
    RxData = read(SerialPort, 6, "uint8");

    IsDataValid = 0;

    if isempty(RxData)
        disp('- ERROR: no data from serial')
        %break; %< error while reading data
        TestIndex = Index + 1;
    else
        if (RxData(1) == uint8(2) && RxData(6) == uint8(3))
            IsDataValid = 1;
            CurrentRPM = uint32(0);
            CurrentRPM = bitor(CurrentRPM, bitshift(RxData(2), 8*0));
            CurrentRPM = bitor(CurrentRPM, bitshift(RxData(3), 8*1));
            CurrentRPM = bitor(CurrentRPM, bitshift(RxData(4), 8*2));
            CurrentRPM = bitor(CurrentRPM, bitshift(RxData(5), 8*3));
            CurrentRPM = typecast(CurrentRPM, 'int32');
        else
            %Attempt auto shift
            if (RxData(2) == uint8(2) && RxData(1) == uint8(3))
                RxData = read(SerialPort, 5, "uint8");
            elseif (RxData(3) == uint8(2) && RxData(2) == uint8(3))
                RxData = read(SerialPort, 4, "uint8");
            elseif (RxData(4) == uint8(2) && RxData(3) == uint8(3))
                RxData = read(SerialPort, 3, "uint8");
            elseif (RxData(5) == uint8(2) && RxData(4) == uint8(3))
                RxData = read(SerialPort, 1, "uint8");
            elseif (RxData(6) == uint8(2) && RxData(5) == uint8(3))
                RxData = read(SerialPort, 1, "uint8");
            end
        end
    end


    if (1 == IsDataValid)
        CurrentRPM = double(CurrentRPM) / 100;

        if (abs(CurrentRPM) > 470)
            CurrentRPM = RlsRpmTable(RlsIndex-1);
        end
        
        RlsRpmTable(RlsIndex) = CurrentRPM;
        RlsReferenceTable(RlsIndex) = ReferenceSignal(RlsIndex);

        %Plant RLS
        % Save data
        ThetaHatTemp = ThetaHat(RlsIndex-1,:)';

        % Calculate Phi (regresive variables)
        XTmp(1) = RlsControlTable(RlsIndex-TransportDelay);
        YTmp(1) = RlsRpmTable(RlsIndex-1);
        Phi = [-YTmp XTmp]';

        % Calculate Error
        Yhat(RlsIndex) = Phi'*ThetaHatTemp;

        E(RlsIndex) = CurrentRPM - Yhat(RlsIndex);
        % Calculate K
        K = (P*Phi)/(ForgFact+Phi'*P*Phi);
        % Calculate new estimation
        ThetaHatTemp = ThetaHatTemp+K*E(RlsIndex);
        ThetaHat(RlsIndex,:) = ThetaHatTemp';
        P=(P-K*Phi'*P)/ForgFact;
        PTrace(RlsIndex) = P(1,1)+P(2,2);

       if RlsIndex > RlsThreshold

            %ZPP algorithm

            B0_hat = ThetaHat(RlsIndex, 2);
            A1_hat = ThetaHat(RlsIndex, 1);
            
            F1 = Am1 - A1_hat;
            G0 = A1_hat*F1;
            
            Ctrl1 = (G0/B0_hat)*RlsRpmTable(RlsIndex);
            Ctrl2 = Bm/B0_hat*RlsReferenceTable(RlsIndex);
            Ctrl3 = -F1*RlsControlTable(RlsIndex-1);

            RlsControlTable(RlsIndex) = Ctrl1+Ctrl2+Ctrl3;
        else
            RlsControlTable(RlsIndex) = 0.5*ReferenceSignal(RlsIndex);
        end



        if ReferenceSignal(RlsIndex) == 0
            RlsControlTable(RlsIndex) = 0;
        end

        %Data to write
        if (RlsControlTable(RlsIndex) > 100)
            TxCtrl = int32(100*100);
            ActualSentControl(RlsIndex) = 100;
        elseif (RlsControlTable(RlsIndex) < -100)
            TxCtrl = int32(100*-100);
            ActualSentControl(RlsIndex) = -100;
        else
            TxCtrl = int32(100*RlsControlTable(RlsIndex));
            ActualSentControl(RlsIndex) = RlsControlTable(RlsIndex);
        end
    else
        RlsReferenceTable(RlsIndex) = 0;
        TxCtrl = int32(0);
    end

    TxData(1) = uint8(2);
    TxData(2) = uint8(bitand(int32(0xFF), TxCtrl));
    TxData(3) = uint8(bitshift(bitand(int32(0xFF00), TxCtrl), -8*1));
    TxData(4) = uint8(bitshift(bitand(int32(0xFF0000), TxCtrl), -8*2));
    TxData(5) = uint8(bitshift(bitand(int32(0xFF000000), TxCtrl), -8*3));
    TxData(6) = uint8(3);

    % ensure the first bit is set correctly
    if RlsReferenceTable(RlsIndex) < 0
        TxData(5) = bitor(uint8(0x80), TxData(5));
    end

    % Write 
    write(SerialPort, TxData, "uint8"); 
    RlsIndex = RlsIndex + 1;
end


write(SerialPort, int32(0), "int32"); 
write(SerialPort, int32(0), "int32"); 
delete(SerialPort);
disp('- FINISHED RLS TEST')


disp('-  PLOTTING NOW')

%plant
PlantDiscrete = tf(ThetaHatTemp(2), [1 ThetaHatTemp(1)], SampleTime)

b_hat =  0.8863;
a_hat = -0.8227;

Fig = figure(1);
Fig.Position = [100 900 1200 400];
plot(AlgorithmTimeTable, ThetaHat(:,1), "b-", "LineWidth", 2);
hold on;
plot(AlgorithmTimeTable, ThetaHat(:,2), "r-", "LineWidth", 2);
legend("a_1", "b_0");
hold off;
grid on;
xlabel('Czas[s]')
ylim([-1.1 1.1]);
xlim([RlsThresholdTime AlgorithmTime(end)])

Fig2 = figure(2);
Fig2.Position = [100 300 1200 400];
stairs(AlgorithmTimeTable, E, "LineWidth", 2);
xlabel('Czas[s]')
ylabel("[rpm]")
ylim([-100 100]);
xlim([RlsThresholdTime AlgorithmTime(end)])
grid on

Fig3 = figure(3);
Fig3.Position = [1000 600 1200 400];
stairs(AlgorithmTimeTable, ReferenceSignal, "r-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, RlsControlTable, "b:", "LineWidth", 2);
stairs(AlgorithmTimeTable, ActualSentControl, "b-", "LineWidth", 2);
hold off
legend("Sygnał referencyjny[rpm]","Obliczony sygnał sterowania[%]", "Ograniczony sygnał sterowania[%]");
grid on
ylim([-200 200])
xlim([RlsThresholdTime AlgorithmTime(end)])

Fig4 = figure(4);
Fig4.Position = [1000 200 1200 400];
stairs(AlgorithmTimeTable, ReferenceSignal, "k-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, ActualSentControl, "c-", "LineWidth", 2);
stairs(AlgorithmTimeTable+(TransportDelay*SampleTime), ModelOutputRef, "m-", "LineWidth", 2);
stairs(AlgorithmTimeTable, Yhat, "r-", "LineWidth", 2);
stairs(AlgorithmTimeTable, RlsRpmTable, "b-", "LineWidth", 2);
hold off
legend("Wartosc referencyjna[rpm]", "Wartosc sterujaca[%]", "Odpowiedz modelu[rpm]", "Estymacja jednokrokowa[rpm]", "Wyjscie obiektu[rpm]")
xlabel('Czas[s]')
ylim([-200 200])
xlim([RlsThresholdTime AlgorithmTime(end)])

Fig5 = figure(5);
Fig5.Position = [1000 900 1200 400];
plot(AlgorithmTimeTable, PTrace, "b-", "LineWidth", 2);
grid on;
xlabel('Czas[s]')
xlim([RlsThresholdTime AlgorithmTime(end)])

InputSim = [AlgorithmTimeTable; RlsControlTable]';
OutputSim = [AlgorithmTimeTable; RlsRpmTable]';


