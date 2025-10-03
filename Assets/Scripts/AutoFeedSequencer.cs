using UnityEngine;
using System;

#region Tag wrappers (unchanged, but names below carry units)
[Serializable]
public class BoolIn
{
    public realvirtual.PLCInputBool tag;
    public bool v, prev;
    public bool Rising => v && !prev;
    public bool Falling => !v && prev;
    public void Sample() { prev = v; v = tag && tag.Value; }
}
[Serializable]
public class BoolOut
{
    public realvirtual.PLCOutputBool tag;
    public void Set(bool x) { if (tag) tag.Value = x; }
    public bool Get() { return tag ? tag.Value : false; }
}
[Serializable]
public class FloatIn
{
    public realvirtual.PLCInputFloat tag;
    public float v, prev;
    public void Sample() { prev = v; v = tag ? tag.Value : 0f; }
}
[Serializable]
public class FloatOut
{
    public realvirtual.PLCOutputFloat tag;
    public void Set(float x) { if (tag) tag.Value = x; }
    public float Get() { return tag ? tag.Value : 0f; }
}
#endregion

/// <summary>
/// Auto feed + cut sequencer (UNITS: position = millimeters, speed = mm/s)
/// Requires:
///  - Conv_Infeed_Position_mm (FloatIn): conveyor position in millimeters (same as Drive position units)
///  - Conv_Infeed_TargetSpeed_mmps (FloatOut): conveyor target speed in mm/s
///  - Conv_Infeed_Fwd (BoolOut): direction/enable forward
///  - PE_Cutter_Entry (BoolIn): entry eye
///  - LC_Cutter_Guard_OK (BoolIn), LS_Blade_Up (BoolIn), LS_Blade_Down (BoolIn)
///  - Blade_JogDown (BoolOut), Blade_JogUp (BoolOut)
/// Lifecycle: Cmd_Start/Stop/Reset, EStop_OK
/// </summary>
public class AutoFeedSequencer : MonoBehaviour
{
    // ===== Lifecycle (Inputs) =====
    [Header("Lifecycle (Inputs)")]
    public BoolIn Cmd_Start, Cmd_Stop, Cmd_Reset;
    public BoolIn EStop_OK;

    // ===== Conveyor & sensors (Inputs) =====
    [Header("Conveyor & Sensors (Inputs)  [mm / mm/s]")]
    public FloatIn Conv_Infeed_Position_mm;   // encoder / drive position in MILLIMETERS
    public BoolIn Conv_Infeed_IsDriving;     // optional
    public BoolIn PE_Cutter_Entry;

    // ===== Blade safety & limits (Inputs) =====
    [Header("Blade Safety & Limits (Inputs)")]
    public BoolIn LC_Cutter_Guard_OK;
    public BoolIn LS_Blade_Up;
    public BoolIn LS_Blade_Down;

    // ===== Conveyor (Outputs) =====
    [Header("Conveyor (Outputs)  [mm / mm/s]")]
    public FloatOut Conv_Infeed_TargetSpeed_mmps; // target speed in MILLIMETERS PER SECOND
    public BoolOut Conv_Infeed_Fwd;

    // ===== Blade jog (Outputs) =====
    [Header("Blade Jog (Outputs)")]
    public BoolOut Blade_JogDown;             // jog forward (down)
    public BoolOut Blade_JogUp;               // jog backward (up)

    // ===== Speed & length configuration (MM / MM/S) =====
    [Header("Setpoints (Units: mm & mm/s)")]
    [Tooltip("Belt command speed in mm/s (this is a setpoint to your Drive)")]
    public float InfeedSpeed_mmps = 800.0f;

    [Tooltip("Desired length to push into the cutter AFTER the entry eye trips (mm)")]
    public float CutLength_mm = 100.0f;       // e.g., 100 mm

    [Header("Timings & Watchdogs (seconds)")]
    [Tooltip("Settle time after stopping at the entry sensor before starting metering")]
    public float SettleTime_s = 0.20f;

    [Tooltip("Max time allowed to reach the entry sensor in the approach phase")]
    public float WD_ToEntry_s = 5.0f;

    [Tooltip("Safety factor for feed watchdog. WD = max(0.2s, WD_Feed_Scale * (CutLength_mm / measuredSpeed_mmps))")]
    public float WD_Feed_Scale = 1.5f;

    [Tooltip("Max time allowed for blade to go down to LS_Blade_Down")]
    public float WD_CutDown_s = 5.0f;

    [Tooltip("Max time allowed for blade to return up to LS_Blade_Up")]
    public float WD_CutUp_s = 5.0f;

    [Header("Policy")]
    [Tooltip("If true, allow blade to retract UP even if the guard opens mid-stroke; down is always blocked when guard opens.")]
    public bool AllowRetractionOnGuardOpen = true;

    [Header("Debugging")]
    public bool VerboseLogs = true;
    public float LogInterval = 0.10f; // 10 Hz
    float _nextLogAt = 0f;
    string lastFault = "";

    // ===== FSM =====
    public enum STATE { S_RESET, S0_APPROACH, S1_METER_FEED, S2_CUT_DOWN, S3_CUT_UP, S_HOLD, S_FAULT }
    [SerializeField] public STATE State = STATE.S_RESET;
    STATE _prevState;

    // ===== Internals (mm / mm/s) =====
    float stateTimer, settleTimer;
    bool runEnable;
    bool feedInterlockLatched;      // latched at S1 start (safe at entry)
    bool cutPermissionLatched;      // latched at S2 start (safe to initiate stroke)

    float posAtEntry_mm;             // snapshot in MILLIMETERS
    float measuredSpeed_mmps;        // derived from encoder (mm/s)

    float feedWD_s;

    // ===== Unity hooks =====
    void Start() { Enter(STATE.S_RESET); }

    void FixedUpdate()
    {
        // 1) Sample inputs and derive measured speed
        SampleInputs();
        DeriveMeasuredSpeed(); // fills measuredSpeed_mmps

        // 2) Derived gating
        runEnable = EStop_OK.v;

        // 3) Global stop/reset
        if (Cmd_Reset.Rising || !runEnable) { Enter(STATE.S_RESET); }
        if (Cmd_Stop.Rising) StopAllMotion();

        // 4) Edge logs
        SensorEdgeLogs();

        // 5) FSM
        switch (State)
        {
            case STATE.S_RESET: Tick_RESET(); break;
            case STATE.S0_APPROACH: Tick_S0_APPROACH(); break;
            case STATE.S1_METER_FEED: Tick_S1_METER_FEED(); break;
            case STATE.S2_CUT_DOWN: Tick_S2_CUT_DOWN(); break;
            case STATE.S3_CUT_UP: Tick_S3_CUT_UP(); break;
            case STATE.S_HOLD: Tick_S_HOLD(); break;
            case STATE.S_FAULT: Tick_S_FAULT(); break;
        }

        // 6) Verbose logs
        if (VerboseLogs && Time.time >= _nextLogAt) { _nextLogAt = Time.time + Mathf.Max(0.01f, LogInterval); LogTick(); }
    }

    // ===== State ticks =====
    void Tick_RESET()
    {
        StopAllMotion();
        if (runEnable && LS_Blade_Up.v && Cmd_Start.Rising)
            Enter(STATE.S0_APPROACH);
    }

    void Tick_S0_APPROACH()
    {
        // Approach → run until the entry eye
        Conv_Infeed_TargetSpeed_mmps.Set(InfeedSpeed_mmps);
        Conv_Infeed_Fwd.Set(runEnable);

        if (PE_Cutter_Entry.v)
        {
            Conv_Infeed_Fwd.Set(false); // hard stop
            settleTimer += Time.fixedDeltaTime;

            if (settleTimer >= SettleTime_s)
            {
                posAtEntry_mm = Conv_Infeed_Position_mm.v;

                // WD sized from MEASURED speed (mm/s)
                float v = Mathf.Max(1f, measuredSpeed_mmps); // avoid divide-by-zero; treat 1 mm/s as minimum for WD calc
                float t_expected = CutLength_mm / v;
                feedWD_s = Mathf.Max(0.2f, t_expected * WD_Feed_Scale);

                Enter(STATE.S1_METER_FEED);
            }
        }
        else
        {
            Watchdog(WD_ToEntry_s, "Approach watchdog");
        }
    }

    void Tick_S1_METER_FEED()
    {
        // Must stay safe during metering (blade up & guard ok). Eye can be anything now.
        if (!feedInterlockLatched) { Fault("Feed latch not set"); return; }
        if (!runEnable) { Fault("EStop lost during metering"); return; }
        if (!LC_Cutter_Guard_OK.v) { Fault("Guard opened during metering"); return; }
        if (!LS_Blade_Up.v) { Fault("Blade not up during metering"); return; }

        Conv_Infeed_TargetSpeed_mmps.Set(InfeedSpeed_mmps);
        Conv_Infeed_Fwd.Set(true);

        float delta_mm = Conv_Infeed_Position_mm.v - posAtEntry_mm;

        // Sanity guards against bogus jumps (negative / huge)
        if (delta_mm < -0.5f) { Fault($"Negative Δ {delta_mm:F1} mm"); return; }
        if (delta_mm > CutLength_mm * 10f) { Fault($"Δ spike {delta_mm:F1} mm"); return; }

        if (delta_mm >= CutLength_mm)
        {
            Conv_Infeed_Fwd.Set(false);
            // Latch stroke permission RIGHT NOW (preconditions must be true NOW)
            cutPermissionLatched = runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;
            Debug.Log($"[SEQ] Stroke permission latched: {cutPermissionLatched} (Guard={LC_Cutter_Guard_OK.v}, BladeUp={LS_Blade_Up.v})");
            Enter(STATE.S2_CUT_DOWN);
            return;
        }

        Watchdog(feedWD_s, "Metering watchdog");
    }

    void Tick_S2_CUT_DOWN()
    {
        // During DOWN: Blade_Up will go false. Gate on E-Stop + Guard + latched permission.
        if (!runEnable) { Fault("EStop lost on down"); return; }
        if (!LC_Cutter_Guard_OK.v) { Fault("Guard opened on down"); return; }
        if (!cutPermissionLatched) { Fault("Stroke not permitted"); return; }

        Blade_JogUp.Set(false);
        Blade_JogDown.Set(true);

        if (LS_Blade_Down.v)
        {
            Blade_JogDown.Set(false);
            Enter(STATE.S3_CUT_UP);
            return;
        }

        Watchdog(WD_CutDown_s, "Cut-down watchdog");
    }

    void Tick_S3_CUT_UP()
    {
        if (!runEnable) { Fault("EStop lost on up"); return; }
        if (!AllowRetractionOnGuardOpen && !LC_Cutter_Guard_OK.v) { Fault("Guard opened on up (blocked)"); return; }

        Blade_JogDown.Set(false);
        Blade_JogUp.Set(true);

        if (LS_Blade_Up.v && !LS_Blade_Down.v)
        {
            Blade_JogUp.Set(false);
            Enter(STATE.S_HOLD);
            return;
        }

        Watchdog(WD_CutUp_s, "Cut-up watchdog");
    }

    void Tick_S_HOLD()
    {
        StopAllMotion();
        // Await next subsystem or Reset
    }

    void Tick_S_FAULT()
    {
        StopAllMotion();
    }

    // ===== Helpers =====
    void Enter(STATE s)
    {
        _prevState = State;
        State = s;
        stateTimer = 0f;
        settleTimer = 0f;

        if (s == STATE.S1_METER_FEED)
        {
            // Set feed latch ONLY if safe to begin metering
            feedInterlockLatched = runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;
        }
        if (s == STATE.S_RESET || s == STATE.S_FAULT)
        {
            feedInterlockLatched = false;
            cutPermissionLatched = false;
        }

        Debug.Log($"[SEQ] STATE: {_prevState} -> {State} @ {Time.time:0.000}s  (FeedLatched={feedInterlockLatched}, StrokeLatched={cutPermissionLatched})");
        if (s == STATE.S_RESET && !string.IsNullOrEmpty(lastFault))
            Debug.Log($"[SEQ] Reset after FAULT: {lastFault}");

        if (s == STATE.S1_METER_FEED)
            Debug.Log($"[SEQ] Entry snapshot pos_mm={Conv_Infeed_Position_mm.v:F1}; Cut={CutLength_mm:F1} mm; v_meas={measuredSpeed_mmps:F1} mm/s");
    }

    void StopAllMotion()
    {
        Conv_Infeed_Fwd.Set(false);
        Blade_JogDown.Set(false);
        Blade_JogUp.Set(false);
    }

    void Watchdog(float maxSec, string label)
    {
        stateTimer += Time.fixedDeltaTime;
        if (stateTimer > maxSec)
            Fault($"{label} exceeded {maxSec:0.00}s in {State}");
    }

    void Fault(string why)
    {
        lastFault = why;
        Debug.LogWarning($"[SEQ] FAULT: {why} @ {Time.time:0.000}s (State={State})");
        Enter(STATE.S_FAULT);
    }

    void SampleInputs()
    {
        Cmd_Start.Sample(); Cmd_Stop.Sample(); Cmd_Reset.Sample();
        EStop_OK.Sample();
        PE_Cutter_Entry.Sample();
        Conv_Infeed_Position_mm.Sample();
        Conv_Infeed_IsDriving.Sample();
        LC_Cutter_Guard_OK.Sample();
        LS_Blade_Up.Sample();
        LS_Blade_Down.Sample();
    }

    void SensorEdgeLogs()
    {
        if (PE_Cutter_Entry.Rising) Debug.Log($"[EDGE] PE_Cutter_Entry ↑ @ {Time.time:0.000}s");
        if (PE_Cutter_Entry.Falling) Debug.Log($"[EDGE] PE_Cutter_Entry ↓ @ {Time.time:0.000}s");
        if (LS_Blade_Up.Rising) Debug.Log($"[EDGE] LS_Blade_Up ↑ @ {Time.time:0.000}s");
        if (LS_Blade_Up.Falling) Debug.Log($"[EDGE] LS_Blade_Up ↓ @ {Time.time:0.000}s");
        if (LS_Blade_Down.Rising) Debug.Log($"[EDGE] LS_Blade_Down ↑ @ {Time.time:0.000}s");
        if (LS_Blade_Down.Falling) Debug.Log($"[EDGE] LS_Blade_Down ↓ @ {Time.time:0.000}s");
        if (LC_Cutter_Guard_OK.Rising) Debug.Log($"[EDGE] Guard OK ↑ @ {Time.time:0.000}s");
        if (LC_Cutter_Guard_OK.Falling) Debug.Log($"[EDGE] Guard OK ↓ @ {Time.time:0.000}s");
    }

    // Derive measured speed from encoder (mm/s)
    void DeriveMeasuredSpeed()
    {
        float du_mm = Conv_Infeed_Position_mm.v - Conv_Infeed_Position_mm.prev;
        // If your drive wraps, handle it HERE by adding/subtracting pitch; for standard linear conveyors, no wrap:
        float dt = Mathf.Max(1e-4f, Time.fixedDeltaTime);
        measuredSpeed_mmps = du_mm / dt;
        // absolute speed magnitude (optional): measuredSpeed_mmps = Mathf.Abs(measuredSpeed_mmps);
    }

    void LogTick()
    {
        float delta_mm = Conv_Infeed_Position_mm.v - posAtEntry_mm;
        string line =
            $"[TICK {Time.time:0.000}] State={State} | " +
            $"Run={runEnable} FeedLatched={feedInterlockLatched} StrokeLatched={cutPermissionLatched} | " +
            $"PE={PE_Cutter_Entry.v} Up={LS_Blade_Up.v} Down={LS_Blade_Down.v} Guard={LC_Cutter_Guard_OK.v} | " +
            $"pos={Conv_Infeed_Position_mm.v:F1} mm Δ={delta_mm:F1} mm (L={CutLength_mm:F1} mm) | " +
            $"v_meas={measuredSpeed_mmps:F1} mm/s | " +
            $"Cmd: Fwd={Conv_Infeed_Fwd.Get()} Vset={Conv_Infeed_TargetSpeed_mmps.Get():F1} mm/s | " +
            $"tState={stateTimer:0.000}s";
        Debug.Log(line);
    }
}
