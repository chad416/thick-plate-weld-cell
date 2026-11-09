FUNCTION_BLOCK FB_WeldCell
(* Robotic multi-pass weld cell: pass sequence, interlocks, safety stop, fault handling.
   IEC 61131-3 Structured Text. Tested in CI through tools/st (ST -> Python) and exported
   to Siemens SCL by weldcell.scl. Safety-oriented demonstration only: a real cell uses a
   certified safety PLC and safety devices. *)
VAR_INPUT
    Start : BOOL;               (* start a part (rising edge) *)
    Reset : BOOL;               (* fault / safety-stop reset (rising edge) *)
    SafetyOK : BOOL;            (* light curtain clear and door closed *)
    Clamped : BOOL;
    RobotHome : BOOL;
    RobotBusy : BOOL;
    RobotDone : BOOL;
    ArcFault : BOOL;
    PosInPosition : BOOL;
    FumeOK : BOOL;
    GasOK : BOOL;
    WireOK : BOOL;
    WeldCurrent : REAL;         (* A, from the power source *)
    PassCount : INT;
    PreheatMin : REAL;          (* degC *)
    InterpassMax : REAL;        (* degC *)
END_VAR
VAR_IN_OUT
    aSeam : ARRAY[1..200] OF INT;       (* seam index of each pass *)
    aJob : ARRAY[1..200] OF INT;        (* robot job number of each pass *)
    aTilt : ARRAY[1..200] OF REAL;      (* positioner set points, deg *)
    aRot : ARRAY[1..200] OF REAL;
    aIlo : ARRAY[1..200] OF REAL;       (* WPS current band, A *)
    aIhi : ARRAY[1..200] OF REAL;
    aSeamTemp : ARRAY[1..16] OF REAL;   (* seam temperatures from the pyrometer scanner *)
END_VAR
VAR_OUTPUT
    State : INT;
    FaultCode : INT;            (* 0 none, 1 clamp, 2 positioner, 3 gas, 4 fume, 5 arc, 6 temperature sensor/heater, 7 robot, 8 wire *)
    PassIdx : INT;
    ClampCmd : BOOL;
    PosStart : BOOL;
    PosTiltSet : REAL;
    PosRotSet : REAL;
    RobotJob : INT;
    RobotStart : BOOL;
    RobotHold : BOOL;
    ArcEnable : BOOL;
    FumeCmd : BOOL;
    GasCmd : BOOL;
    HeaterOn : BOOL;
    PassDone : BOOL;            (* TRUE for one scan: record the pass *)
    PassRetries : INT;
    PassInterrupted : BOOL;
    PassNok : BOOL;
    QualityAlarm : BOOL;
    Waiting : BOOL;             (* waiting for the seam to cool to interpass temperature *)
    Done : BOOL;
END_VAR
VAR
    LastState : INT;
    ResumeState : INT;
    tStep : TON;                (* time in the current step *)
    tOut : TON;                 (* current outside the WPS band *)
    rStart : R_TRIG;
    rReset : R_TRIG;
    SafetyStop : BOOL;
    Seam : INT;
    Retry : INT;
    SinceClean : INT;
    TempAtStart : REAL;
    CurTilt : REAL;
    CurRot : REAL;
    T : REAL;
END_VAR
VAR CONSTANT
    S_IDLE : INT := 0;
    S_CLAMP : INT := 10;
    S_NEXT : INT := 20;
    S_POSITION : INT := 30;
    S_POS_WAIT : INT := 31;
    S_TEMP : INT := 40;
    S_PREARC : INT := 50;
    S_WELD_START : INT := 60;
    S_WELD : INT := 61;
    S_POST : INT := 70;
    S_ADVANCE : INT := 75;
    S_CLEAN_START : INT := 80;
    S_CLEAN : INT := 81;
    S_FINISH : INT := 90;
    S_UNCLAMP : INT := 95;
    S_FAULT : INT := 100;
    T_CLAMP : TIME := T#10S;
    T_POS : TIME := T#90S;
    T_PREARC : TIME := T#8S;
    T_BUSY : TIME := T#5S;
    T_ROBOT : TIME := T#2H;
    T_TEMP_MAX : TIME := T#1H;  (* longest plausible interpass wait or preheat *)
    CLEAN_EVERY : INT := 6;
    JOB_CLEAN : INT := 900;
    MIN_RISE : REAL := 5.0;
    HEATER_HYST : REAL := 10.0;
    TEMP_MARGIN : REAL := 6.0;  (* pyrometer uncertainty (3 sigma + 1 K) kept off both limits *)
END_VAR

(* ---- edges, step timer, safety stop ---------------------------------------- *)
rStart(CLK := Start);
rReset(CLK := Reset);
tStep(IN := (State = LastState) AND NOT SafetyStop, PT := T_ROBOT);
LastState := State;

IF NOT SafetyOK THEN
    IF NOT SafetyStop AND (State = S_WELD OR State = S_WELD_START) THEN
        PassInterrupted := TRUE;
    END_IF;
    SafetyStop := TRUE;
END_IF;
IF SafetyStop AND SafetyOK AND rReset.Q THEN
    SafetyStop := FALSE;
END_IF;
RobotHold := SafetyStop;

(* ---- sequence ---------------------------------------------------------------- *)
CASE State OF
    S_IDLE:
        IF rStart.Q AND PassCount > 0 AND NOT SafetyStop AND FaultCode = 0 THEN
            PassIdx := 1;
            SinceClean := 0;
            Done := FALSE;
            QualityAlarm := FALSE;
            State := S_CLAMP;
        END_IF;

    S_CLAMP:
        ClampCmd := TRUE;
        IF Clamped THEN
            State := S_NEXT;
        ELSIF tStep.ET >= T_CLAMP THEN
            FaultCode := 1;
            ResumeState := S_CLAMP;
            State := S_FAULT;
        END_IF;

    S_NEXT:
        PassDone := FALSE;
        IF PassIdx > PassCount THEN
            State := S_FINISH;
        ELSE
            Seam := aSeam[PassIdx];
            Retry := 0;
            PassRetries := 0;
            PassInterrupted := FALSE;
            PassNok := FALSE;
            State := S_POSITION;
        END_IF;

    S_POSITION:
        PosTiltSet := aTilt[PassIdx];
        PosRotSet := aRot[PassIdx];
        IF ABS(PosTiltSet - CurTilt) < 0.5 AND ABS(PosRotSet - CurRot) < 0.5 THEN
            State := S_TEMP;
        ELSIF RobotHome AND NOT SafetyStop THEN
            PosStart := TRUE;
            State := S_POS_WAIT;
        ELSIF tStep.ET >= T_POS THEN
            FaultCode := 2;
            ResumeState := S_POSITION;
            State := S_FAULT;
        END_IF;

    S_POS_WAIT:
        IF NOT RobotHome OR SafetyStop THEN
            PosStart := FALSE;                     (* interlock: never move with the robot out *)
            State := S_POSITION;
        ELSIF PosInPosition AND tStep.ET >= T#1S THEN
            PosStart := FALSE;
            CurTilt := PosTiltSet;
            CurRot := PosRotSet;
            State := S_TEMP;
        ELSIF tStep.ET >= T_POS THEN
            PosStart := FALSE;
            FaultCode := 2;
            ResumeState := S_POSITION;
            State := S_FAULT;
        END_IF;

    S_TEMP:
        T := aSeamTemp[Seam];
        IF tStep.ET >= T_TEMP_MAX THEN
            HeaterOn := FALSE;                     (* no cooling / heating within the expected time: sensor or heater *)
            Waiting := FALSE;
            FaultCode := 6;
            ResumeState := S_TEMP;
            State := S_FAULT;
        ELSIF T > InterpassMax - TEMP_MARGIN THEN
            Waiting := TRUE;
            HeaterOn := FALSE;
        ELSIF HeaterOn THEN
            IF T >= PreheatMin + HEATER_HYST THEN
                HeaterOn := FALSE;
            END_IF;
        ELSIF T < PreheatMin + TEMP_MARGIN THEN
            HeaterOn := TRUE;
            Waiting := FALSE;
        ELSE
            Waiting := FALSE;
            TempAtStart := T;
            State := S_PREARC;
        END_IF;

    S_PREARC:
        FumeCmd := TRUE;
        GasCmd := TRUE;
        IF FumeOK AND GasOK AND WireOK AND NOT SafetyStop THEN
            State := S_WELD_START;
        ELSIF tStep.ET >= T_PREARC THEN
            IF NOT FumeOK THEN
                FaultCode := 4;
            ELSIF NOT GasOK THEN
                FaultCode := 3;
            ELSE
                FaultCode := 8;
            END_IF;
            ResumeState := S_PREARC;
            State := S_FAULT;
        END_IF;

    S_WELD_START:
        RobotJob := aJob[PassIdx];
        IF PosInPosition AND NOT SafetyStop AND NOT RobotBusy THEN
            RobotStart := TRUE;
        END_IF;
        IF RobotBusy THEN
            RobotStart := FALSE;
            State := S_WELD;
        ELSIF tStep.ET >= T_BUSY THEN
            RobotStart := FALSE;
            FaultCode := 7;
            ResumeState := S_WELD_START;
            State := S_FAULT;
        END_IF;

    S_WELD:
        tOut(IN := WeldCurrent > 20.0 AND (WeldCurrent < aIlo[PassIdx] OR WeldCurrent > aIhi[PassIdx]), PT := T#2S);
        IF tOut.Q THEN
            PassNok := TRUE;
            QualityAlarm := TRUE;
        END_IF;
        IF ArcFault THEN
            Retry := Retry + 1;
            PassRetries := Retry;
            IF Retry <= 1 THEN
                State := S_PREARC;                 (* one automatic retry *)
            ELSE
                FaultCode := 5;
                ResumeState := S_PREARC;
                State := S_FAULT;
            END_IF;
        ELSIF RobotDone AND NOT RobotBusy THEN
            State := S_POST;
        ELSIF tStep.ET >= T_ROBOT THEN
            FaultCode := 7;
            ResumeState := S_WELD_START;
            State := S_FAULT;
        END_IF;

    S_POST:
        PassDone := TRUE;
        tOut(IN := FALSE, PT := T#2S);
        IF aSeamTemp[Seam] - TempAtStart < MIN_RISE THEN
            FaultCode := 6;                        (* no temperature rise after a pass: sensor implausible *)
            ResumeState := S_ADVANCE;
            State := S_FAULT;
        ELSE
            State := S_ADVANCE;
        END_IF;

    S_ADVANCE:
        PassDone := FALSE;
        GasCmd := FALSE;
        PassIdx := PassIdx + 1;
        SinceClean := SinceClean + 1;
        IF SinceClean >= CLEAN_EVERY AND PassIdx <= PassCount THEN
            State := S_CLEAN_START;
        ELSE
            State := S_NEXT;
        END_IF;

    S_CLEAN_START:
        RobotJob := JOB_CLEAN;
        IF NOT SafetyStop AND NOT RobotBusy THEN
            RobotStart := TRUE;
        END_IF;
        IF RobotBusy THEN
            RobotStart := FALSE;
            State := S_CLEAN;
        ELSIF tStep.ET >= T_BUSY THEN
            RobotStart := FALSE;
            FaultCode := 7;
            ResumeState := S_CLEAN_START;
            State := S_FAULT;
        END_IF;

    S_CLEAN:
        IF RobotDone AND NOT RobotBusy THEN
            SinceClean := 0;
            State := S_NEXT;
        END_IF;

    S_FINISH:
        FumeCmd := FALSE;
        GasCmd := FALSE;
        HeaterOn := FALSE;
        PosTiltSet := 0.0;
        PosRotSet := 0.0;
        IF ABS(CurTilt) < 0.5 AND ABS(CurRot) < 0.5 THEN
            State := S_UNCLAMP;
        ELSIF RobotHome AND NOT SafetyStop THEN
            PosStart := TRUE;
            IF PosInPosition AND tStep.ET >= T#1S THEN
                PosStart := FALSE;
                CurTilt := 0.0;
                CurRot := 0.0;
                State := S_UNCLAMP;
            END_IF;
        ELSE
            PosStart := FALSE;
        END_IF;

    S_UNCLAMP:
        ClampCmd := FALSE;
        IF NOT Clamped THEN
            Done := TRUE;
            State := S_IDLE;
        END_IF;

    S_FAULT:
        RobotStart := FALSE;
        PosStart := FALSE;
        GasCmd := FALSE;
        HeaterOn := FALSE;
        IF rReset.Q AND SafetyOK THEN
            FaultCode := 0;
            Retry := 0;
            State := ResumeState;
        END_IF;
END_CASE;

(* ---- permissives, evaluated every scan whatever the step --------------------- *)
ArcEnable := (State = S_WELD_START OR State = S_WELD) AND NOT SafetyStop AND FumeOK AND GasOK AND WireOK
             AND FaultCode = 0;
IF NOT RobotHome THEN
    PosStart := FALSE;
END_IF;
END_FUNCTION_BLOCK
