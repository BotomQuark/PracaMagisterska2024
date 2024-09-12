clear;


TestNone = 0;
Test20min = 1;
TestNonZeroStart = 2;

CurrentTest = 0;


if CurrentTest == TestNone
    Periods = 15;  %2 minute test
    PeriodTime = 4;
    Amplitude = 150;
elseif CurrentTest == Test20min
    Periods = 300; % 20 minute test
    PeriodTime = 2;
    Amplitude = 150;
elseif CurrentTest == TestNonZeroStart
    Periods = 30;  %4 minute test
    PeriodTime = 4;
    Amplitude = 150;
else
    Periods = 15;  %2 minute test
    PeriodTime = 4;
    Amplitude = 150;
end

UartPort = "COM6";
AlgorithmTime = PeriodTime+PeriodTime + Periods*PeriodTime;
SampleTime = 0.002;
DegreesPerImpulse = 360/280;
AlgorithmTimeTable = SampleTime:SampleTime:AlgorithmTime;

ReadErrorCnt = 0;

SerialPort = serialport(UartPort, 115200, 'FlowControl', "hardware");

CurrentRPM = 0;
RxData = zeros(1, 6);
TxData = zeros(1, 6);

ReadyToStart = 0;
IsDataValid = 0;

a1 = -0.9452;
b0 = 0.1391;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% CTRL %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ReferenceSignal = zeros(size(AlgorithmTimeTable));
Index = 1;
while Index < AlgorithmTime/SampleTime

    if Index < PeriodTime/SampleTime
        ReferenceSignal(Index) = 0;
    elseif Index > (size(AlgorithmTimeTable,2) - PeriodTime/SampleTime)
        ReferenceSignal(Index) = 0;
    else
        SineTime = SampleTime*(Index - PeriodTime/SampleTime);
        ReferenceSignal(Index) = Amplitude*sin(SineTime*pi);
    end
    Index = Index + 1;
end
plot(AlgorithmTimeTable, ReferenceSignal)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% MIT %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tp = 0.082;
Kp = 5;
%Tm = 0.082;
Tm = 0.2;
%Km = 5;
Km = 1;
alpha1 = 0.01;
alpha2 = 0.01;

MitRpmTable = zeros(1, AlgorithmTime/SampleTime);
MitReferenceTable = zeros(1, AlgorithmTime/SampleTime);
AlgorithmTimeTable = SampleTime:SampleTime:AlgorithmTime;

TmpChangeNumerator = 1/Tm;
TmpChangeDenominator = [1 TmpChangeNumerator];
TmpChangeCont = tf(TmpChangeNumerator, TmpChangeDenominator);
TmpChangeDiscrete = c2d(TmpChangeCont, SampleTime);
Tmp1ChangePhi = [0, 0];
Tmp2ChangePhi = [0, 0];
TmpChangeCoeff = [TmpChangeDiscrete.Numerator{1}(2) TmpChangeDiscrete.Denominator{1}(2)];

ModelNumerator = Km;
ModelDenominator = [Tm 1];
ModelCont = tf(ModelNumerator, ModelDenominator);
ModelDiscrete = c2d(ModelCont, SampleTime);
ModelNumerator = ModelDiscrete.Numerator;
ModelDenominator = ModelDiscrete.Denominator;
ModelCoeff = [ModelDiscrete.Numerator{1}(2) ModelDiscrete.Denominator{1}(2)];
ModelPhi = [0, 0];
bm0 = ModelCoeff(1);
am1 = ModelCoeff(2);

ModelY = zeros(1, size(AlgorithmTimeTable, 2));
ModelError = zeros(1, size(AlgorithmTimeTable, 2));
Theta1 = zeros(1, size(AlgorithmTimeTable, 2));
Theta2 = zeros(1, size(AlgorithmTimeTable, 2));


if TestNonZeroStart == CurrentTest
    Theta1 = 0.127187*ones(1, size(AlgorithmTimeTable, 2));
    Theta2 = -0.0714282*ones(1, size(AlgorithmTimeTable, 2));
end

PlantControlTable = zeros(1, size(AlgorithmTimeTable, 2));
ActualControlTable = zeros(1, size(AlgorithmTimeTable, 2));

Beta1 = zeros(1, size(AlgorithmTimeTable, 2));
Beta2 = zeros(1, size(AlgorithmTimeTable, 2));

TransportDelay = 13;
PlantControl = 0;
gamma1 = 0.001; % To set as needed
gamma2 = 0.001; % To set as needed
LastTmp1Change = 0;
LastTmp2Change = 0;
MitIndex = TransportDelay+2;
while MitIndex < AlgorithmTime/SampleTime
    RxData = read(SerialPort, 6, "uint8");

    IsDataValid = 0;

    if isempty(RxData)
        disp('- ERROR: no data from serial')
        %break; %< error while reading data
        TestIndex = TestIndex + 1;
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
        % Actual algorithm here
        CurrentRPM = double(CurrentRPM) / 100;
        if (abs(CurrentRPM) > 420)
            CurrentRPM = MitRpmTable(MitIndex-1);
            ReadErrorCnt = ReadErrorCnt+1;
        end
        MitRpmTable(MitIndex) = CurrentRPM;

        %Get control value
        MitReferenceTable(MitIndex) = ReferenceSignal(MitIndex);
       
        ModelPhi(1) = MitReferenceTable(MitIndex-TransportDelay);
        ModelPhi(2) = -ModelY(MitIndex);
        ModelY(MitIndex+1) = ModelPhi * ModelCoeff';
        ModelError(MitIndex+1) = MitRpmTable(MitIndex) - ModelY(MitIndex+1);

        
        %Calculate new Theta
        Tmp1ChangePhi(1) = MitReferenceTable(MitIndex-1);
        Tmp1ChangePhi(2) = LastTmp1Change;
        Tmp2ChangePhi(1) = MitRpmTable(MitIndex-1);
        Tmp2ChangePhi(2) = LastTmp2Change;
    

        LastTmp1Change = Tmp1ChangePhi*TmpChangeCoeff';
        LastTmp2Change = Tmp2ChangePhi*TmpChangeCoeff';

        Beta1(MitIndex+1) = LastTmp1Change;
        Beta2(MitIndex+1) = LastTmp2Change;

        Theta1Change = (LastTmp1Change*ModelError(MitIndex+1))/(LastTmp1Change^2 + alpha1);
        Theta2Change = (LastTmp2Change*ModelError(MitIndex+1))/(LastTmp2Change^2 + alpha2);

        Theta1(MitIndex+1) = Theta1(MitIndex) - SampleTime*gamma2*Theta1Change;
        Theta2(MitIndex+1) = Theta2(MitIndex) + SampleTime*gamma1*Theta2Change;

        PlantControl = MitReferenceTable(MitIndex)*Theta1(MitIndex+1);
        PlantControl = PlantControl - Theta2(MitIndex+1)*MitRpmTable(MitIndex);
        PlantControlTable(MitIndex) = PlantControl;

        % Write control value back
        if (PlantControl > 100)
            PlantControl = 100;
        elseif (PlantControl < -100)
            PlantControl = -100;
        end

        ActualControlTable(MitIndex) = PlantControl; 
        TxCtrl = int32(100*PlantControl);

        TxData(1) = uint8(2);
        TxData(2) = uint8(bitand(int32(0xFF), TxCtrl));
        TxData(3) = uint8(bitshift(bitand(int32(0xFF00), TxCtrl), -8*1));
        TxData(4) = uint8(bitshift(bitand(int32(0xFF0000), TxCtrl), -8*2));
        TxData(5) = uint8(bitshift(bitand(int32(0xFF000000), TxCtrl), -8*3));
        TxData(6) = uint8(3);
            
        % ensure the first bit is set correctly
        if PlantControl < 0
            TxData(5) = bitor(uint8(0x80), TxData(5));
        end
        write(SerialPort, TxData, "uint8"); 
        MitIndex = MitIndex+1;
    end
   
end






write(SerialPort, int32(0), "int32"); 
write(SerialPort, int32(0), "int32"); 
delete(SerialPort);
disp('- FINISHED RLS TEST')

disp('-  PLOTTING NOW')
% make plot
Fig = figure(1);
Fig.Position = [100 0 1200 400];
clf
stairs(AlgorithmTimeTable, MitReferenceTable, "r-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, PlantControlTable, "b:", "LineWidth", 2);
stairs(AlgorithmTimeTable, ActualControlTable, "b-", "LineWidth", 2);
legend("Sygnał referencyjny[rpm]","Obliczony sygnał sterowania[%]", "Ograniczony sygnał sterowania[%]");
%ylim([-200 200])
xlim([AlgorithmTimeTable(1) AlgorithmTimeTable(end)])
xlabel("Czas[s]")
hold off
ylim([-110 110])
grid on


Fig2 = figure(2);
Fig2.Position = [100 100 1200 400];
clf
stairs(AlgorithmTimeTable, MitReferenceTable, "k-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, ModelY, "m-", "LineWidth", 2);
stairs(AlgorithmTimeTable, MitRpmTable, "r-", "LineWidth", 2);
hold off
grid on
legend("Sygnał referencyjny", "Model", "MIT",'Location', 'southwest')
ylim([-160 160])
xlim([AlgorithmTimeTable(1) AlgorithmTimeTable(end)])
ylabel('Prędkość obrotowa [rpm]')
xlabel("Czas[s]")

Fig3 = figure(3);
Fig3.Position = [100 200 1200 400];
clf
yyaxis left
stairs(AlgorithmTimeTable, ModelError, "LineWidth", 2);
ylim([-150 150])
grid on
yyaxis right
stairs(AlgorithmTimeTable, Theta1, "m-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, Theta2, "r-", "LineWidth", 2);
hold off
ylim([-1 1])
xlim([AlgorithmTimeTable(1) AlgorithmTimeTable(end)])
legend('Błąd wyjścia modelu[rpm]', 'Theta1[%/rpm]', 'Theta2[%/rpm]')
xlabel("Czas[s]")
grid on

Fig4 = figure(4);
Fig4.Position = [100 300 400 400];
clf
plot(Theta1, Theta2, "k-", "LineWidth", 2);
xlabel('Theta1[%/rpm]')
ylabel('Theta2[%/rpm]')
grid on
ReadErrorCnt



Fig5 = figure(5);
Fig5.Position = [1000 200 400 400]; 
plot(AlgorithmTimeTable, Beta1, "r-", "LineWidth", 2);
hold on
plot(AlgorithmTimeTable, Beta1, "b-", "LineWidth", 2);
hold off
grid on
legend("Beta1", "Beta2");

InputSim = [AlgorithmTimeTable; ActualControlTable]';
OutputSim = [AlgorithmTimeTable; MitRpmTable]';
