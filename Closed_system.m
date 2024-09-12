clear;

UartPort = "COM6";
Impulses = 8;
ImpulseTime = 1;
%AlgorithmTime = Impulses*ImpulseTime;
AlgorithmTime = Impulses*ImpulseTime + 0.001;
SampleTime = 0.002;
DegreesPerImpulse = 360/280;
NumberOfSamples = floor(AlgorithmTime/SampleTime)+1;
ReadErrorCnt = 0;

% data tables of last AlgorithmTime seconds
TestRpmTable = zeros(1, NumberOfSamples);
TestControlTable = zeros(1, NumberOfSamples);
AlgorithmTimeTable = 0:SampleTime:AlgorithmTime;
TestIndex = 1;
ControlSignal = zeros(1, NumberOfSamples);
ClosedRpmTable = zeros(1, NumberOfSamples);
ClosedControlTable = zeros(1, NumberOfSamples);
ControlError = zeros(1, NumberOfSamples);

SerialPort = serialport(UartPort, 115200, 'FlowControl', "hardware");

CurrentRPM = 0;
RxData = zeros(1, 6);
TxData = zeros(1, 6);

ReadyToStart = 0;
IsDataValid = 0;

%model
Tm = 0.035;
Km = 2.3575;
ModelNumerator = Km;
ModelDenominator = [Tm 1];
ModelCont = tf(ModelNumerator, ModelDenominator);
ModelDiscrete = c2d(ModelCont, SampleTime)
ModelNumerator = ModelDiscrete.Numerator;
ModelDenominator = ModelDiscrete.Denominator;
ModelPhi = [0, 0, 0];
ModelCoeff = [ModelDiscrete.Numerator{1} ModelDiscrete.Denominator{1}(2)]


Ku = 0.5;
Tu = 0.09;

Kp = 0.42*Ku;
Ki = 0.54*Ku/Tu;
Kni = Ki*SampleTime;

Kp = 0.18
Kni = 0.002

%Kp = 0.25
%Kni = 0.006

ErrorSum = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% CTRL %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Index = 2;
while Index < NumberOfSamples

    if Index < 1/Impulses * NumberOfSamples
        ControlSignal(Index) = 0;
    elseif Index > (Impulses-1)/Impulses * NumberOfSamples
        ControlSignal(Index) = 0;
    else
        Threshold = 2;
        while Threshold < Impulses
            if Index < Threshold/Impulses * NumberOfSamples
                if 0 == mod(Threshold,2)
                    ControlSignal(Index) = 100.00;
                else
                    ControlSignal(Index) = 100.00;
                end
                Threshold = 100000; % end
            end
        Threshold = Threshold + 1;
        end
    end
    Index = Index + 1;
end



ClosedIndex = 10;
while ClosedIndex < NumberOfSamples
    RxData = read(SerialPort, 6, "uint8");

    IsDataValid = 0;

    if isempty(RxData)
        disp('- ERROR: no data from serial')
        %break; %< error while reading data
        ClosedIndex = ClosedIndex + 1;
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

        if (abs(CurrentRPM) > 500)
            CurrentRPM = ClosedRpmTable(ClosedIndex-1);
            ReadErrorCnt = ReadErrorCnt+1;
        end
        
        ClosedRpmTable(ClosedIndex) = CurrentRPM;

        % Closed start calculating after first 5 seconds
        if ClosedIndex < (1/12)*NumberOfSamples
            TxCtrl = int32(0);
        else
            
            ControlError(ClosedIndex) = ControlSignal(ClosedIndex) - ClosedRpmTable(ClosedIndex);

            ErrorSum = ErrorSum + ControlError(ClosedIndex);
            ClosedControlTable(ClosedIndex) = ControlError(ClosedIndex)*Kp + ErrorSum*Kni;

            if ClosedControlTable(ClosedIndex) > 100
                TxCtrl = int32(100*100);
            elseif ClosedControlTable(ClosedIndex) < -100
                TxCtrl = int32(-100*100);
            else
                TxCtrl = int32(100*ClosedControlTable(ClosedIndex));
            end
        end

    else
        ClosedControlTable(ClosedIndex) = 0;
        TxCtrl = int32(0);
    end

    TxData(1) = uint8(2);
    TxData(2) = uint8(bitand(int32(0xFF), TxCtrl));
    TxData(3) = uint8(bitshift(bitand(int32(0xFF00), TxCtrl), -8*1));
    TxData(4) = uint8(bitshift(bitand(int32(0xFF0000), TxCtrl), -8*2));
    TxData(5) = uint8(bitshift(bitand(int32(0xFF000000), TxCtrl), -8*3));
    TxData(6) = uint8(3);

    % ensure the first bit is set correctly
    if ClosedControlTable(ClosedIndex) < 0
        TxData(5) = bitor(uint8(0x80), TxData(5));
    end

    % Write 
    write(SerialPort, TxData, "uint8"); 
    ClosedIndex = ClosedIndex + 1;
end


write(SerialPort, int32(0), "int32"); 
write(SerialPort, int32(0), "int32"); 
delete(SerialPort);
disp('- FINISHED Closed TEST')


disp('-  PLOTTING NOW')
Fig = figure(1);
Fig.Position = [100 150 1200 400];
%yyaxis left
stairs(AlgorithmTimeTable, ClosedRpmTable, "k-", "LineWidth", 2);
hold on
grid on
stairs(AlgorithmTimeTable, ControlSignal, "b-", "LineWidth", 2);
xlabel('Czas[s]')
title("Odpowiedź układu otwartego")
grid on

K = 5;
T = 0.02;
ref = tf([K], [T 1]);
refdisc = c2d(ref, SampleTime)
%AlgorithmTimeTable = SampleTime:SampleTime:AlgorithmTime;
refout = lsim(refdisc, ControlSignal, AlgorithmTimeTable);
%stairs(AlgorithmTimeTable, refout, "m-", "LineWidth", 2);

Lgd = legend("Prędkość obrotowa [RPM]", "Wartość referencyjna [RPM]", "Odpowiedź manualnie dobranej transmitancji [RPM]");
Lgd.FontSize = 14;
hold off


Fig2 = figure(2);
Fig2.Position = [100 100 1200 400];
stairs(AlgorithmTimeTable, ControlSignal, "k-", "LineWidth", 2);
hold on
%stairs(AlgorithmTimeTable, ControlError, "g-", "LineWidth", 2);
stairs(AlgorithmTimeTable, ClosedControlTable, "b-", "LineWidth", 2);
stairs(AlgorithmTimeTable, ClosedRpmTable, "r-", "LineWidth",2)
hold off
%Lgd = legend("Wartość sygnału referencyjnego [rpm]", "Wartość uchybu [rpm]", "Wartość sygnału sterowania [%]", "Wartość sygnału wyjściowego[rpm]");
Lgd = legend("Wartość sygnału referencyjnego [rpm]", "Wartość sygnału sterowania [%]", "Wartość sygnału wyjściowego[rpm]");
Lgd.FontSize = 14;
xlabel('Czas[s]')
grid on
