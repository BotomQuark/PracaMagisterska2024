clear;


UartPort = "COM6";
AlgorithmTime = Impulses*ImpulseTime + 0.001;
SampleTime = 0.016;
NumberOfSamples = floor(AlgorithmTime/SampleTime)+1;
ReadErrorCnt = 0;

% data tables of last AlgorithmTime seconds
TestRpmTable = zeros(1, NumberOfSamples);
TestControlTable = zeros(1, NumberOfSamples);
AlgorithmTimeTable = 0:SampleTime:AlgorithmTime;
TestIndex = 1;
ControlSignal = zeros(1, NumberOfSamples);

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
                    ControlSignal(Index) = 30.00;
                else
                    ControlSignal(Index) = -30.00;
                end
                Threshold = 100000; % end
            end
        Threshold = Threshold + 1;
        end
    end
    Index = Index + 1;
end

flush(SerialPort)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% TEST %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while TestIndex < NumberOfSamples
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
        % First check if data is correct 
        if (0 == ReadyToStart)
            ReadyToStart = 1;
        else
            % Actual algorithm here
            CurrentRPM = double(CurrentRPM) / 100;

            if (abs(CurrentRPM) > 300)
                ReadErrorCnt = ReadErrorCnt+1;
            end

            TestControlTable(TestIndex) = ControlSignal(TestIndex);
            TestRpmTable(TestIndex) = CurrentRPM;

            TxCtrl = int32(100*TestControlTable(TestIndex));

            TxData(1) = uint8(2);
            TxData(2) = uint8(bitand(int32(0xFF), TxCtrl));
            TxData(3) = uint8(bitshift(bitand(int32(0xFF00), TxCtrl), -8*1));
            TxData(4) = uint8(bitshift(bitand(int32(0xFF0000), TxCtrl), -8*2));
            TxData(5) = uint8(bitshift(bitand(int32(0xFF000000), TxCtrl), -8*3));
            TxData(6) = uint8(3);
            
            % ensure the first bit is set correctly
            if TestControlTable(TestIndex) < 0
                TxData(5) = bitor(uint8(0x80), TxData(5));
            end

            % Write 
            write(SerialPort, TxData, "uint8"); 
            TestIndex = TestIndex+1;
        end
    end
end
disp('- FINISHED TEST')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% RLS %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RlsRpmTable = zeros(1, NumberOfSamples);
RlsControlTable = zeros(1, NumberOfSamples);

ModelOrder = 1;
dP = ModelOrder*2;
ForgFact = 0.99;
TotalNumberOfSamples = size(AlgorithmTimeTable,2);

Phi = zeros(1, dP);
ThetaHat = zeros(TotalNumberOfSamples, dP);
P = 10000*eye(dP);
K = zeros(dP,1);

Yhat = zeros(TotalNumberOfSamples, 1);
E = zeros(TotalNumberOfSamples, 1);
ThetaArray = zeros(TotalNumberOfSamples, dP);

TransportDelay = 2;

RlsIndex = 10;
while RlsIndex < NumberOfSamples
    RxData = read(SerialPort, 6, "uint8");

    IsDataValid = 0;

    if isempty(RxData)
        disp('- ERROR: no data from serial')
        %break; %< error while reading data
        RlsIndex = RlsIndex + 1;
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
            ReadErrorCnt = ReadErrorCnt+1;
        end
        
        RlsRpmTable(RlsIndex) = CurrentRPM;
        RlsControlTable(RlsIndex) = ControlSignal(RlsIndex);

        % RLS start calculating after first 5 seconds
        if RlsIndex < (1/12)*NumberOfSamples
            TxCtrl = int32(0);
        else
            % Save data
            ThetaHatTemp = ThetaHat(RlsIndex-1,:)';
            % Calculate Phi (regresive variables)
            XTmp = RlsControlTable(RlsIndex-1-TransportDelay);
            YTmp = RlsRpmTable(RlsIndex-1);
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

            TxCtrl = int32(100*RlsControlTable(RlsIndex));
        end

    else
        RlsControlTable(RlsIndex) = 0;
        TxCtrl = int32(0);
    end

    TxData(1) = uint8(2);
    TxData(2) = uint8(bitand(int32(0xFF), TxCtrl));
    TxData(3) = uint8(bitshift(bitand(int32(0xFF00), TxCtrl), -8*1));
    TxData(4) = uint8(bitshift(bitand(int32(0xFF0000), TxCtrl), -8*2));
    TxData(5) = uint8(bitshift(bitand(int32(0xFF000000), TxCtrl), -8*3));
    TxData(6) = uint8(3);

    % ensure the first bit is set correctly
    if RlsControlTable(RlsIndex) < 0
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


Fig = figure(1);
Fig.Position = [100 900 1200 400];
stairs(AlgorithmTimeTable, RlsRpmTable, "k-", "LineWidth", 2);
hold on
grid on
stairs(AlgorithmTimeTable, RlsControlTable, "b-", "LineWidth", 2);
xlabel('Czas[s]')
grid on

K = 5;
T = 0.082;
ref = tf([K], [T 1]);
refdisc = c2d(ref, SampleTime)
%AlgorithmTimeTable = SampleTime:SampleTime:AlgorithmTime;
refout = lsim(refdisc, ControlSignal, AlgorithmTimeTable);
stairs(AlgorithmTimeTable+(2*SampleTime), refout, "m-", "LineWidth", 2);

Lgd = legend("Rzeczywista prędkość obrotowa [RPM]", "Sygnał sterowania [%]", "Odpowiedź manualnie dobranej transmitancji [RPM]");
Lgd.FontSize = 14;
hold off

b_hat = refdisc.Numerator{1}(2);
a_hat = refdisc.Denominator{1}(2);


Fig2 = figure(2);
Fig2.Position = [100 300 1200 400];
plot(AlgorithmTimeTable, ThetaHat(:,1), "b-", "LineWidth", 2);
hold on;
plot(AlgorithmTimeTable, ThetaHat(:,2), "r-", "LineWidth", 2);
plot([AlgorithmTimeTable(1) AlgorithmTimeTable(end)], [b_hat b_hat], "r:", "LineWidth", 1);
plot([AlgorithmTimeTable(1) AlgorithmTimeTable(end)], [a_hat a_hat], "b:", "LineWidth", 1);
legend("A1", "B1");
hold off;
grid on;
ylabel('Wartość współczynników [RPM/0.01%]')
xlabel('Czas[s]')
ylim([-2 2]);

Fig3 = figure(3);
Fig3.Position = [1000 600 1200 400];
stairs(AlgorithmTimeTable, E, "LineWidth", 2);
xlabel('Czas[s]')
ylabel('[-]')
title("Błąd predykcji jednokrokowej")
ylim([-30 30]);
grid on


Fig4 = figure(4);
Fig4.Position = [1000 200 1200 400];
stairs(AlgorithmTimeTable, RlsRpmTable, "k-", "LineWidth", 2);
hold on
stairs(AlgorithmTimeTable, RlsControlTable, "b-", "LineWidth", 2);
stairs(AlgorithmTimeTable, Yhat, "r-", "LineWidth", 2);
hold off
xlabel('Czas[s]')
Lgd = legend("Rzeczywista prędkość obrotowa [RPM]", "Sygnał sterowania [%]", "Jednokrokowa predykcja wyjścia[RPM]");
Lgd.FontSize = 14;
grid on
ylim([-200 200])

