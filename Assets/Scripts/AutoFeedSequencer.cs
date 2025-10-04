using UnityEngine;
using System;
using realvirtual; // MU, Sensor

#region Tag wrappers (units baked in names)
[Serializable] public class BoolIn { public PLCInputBool tag; public bool v, prev; public bool Rising => v && !prev; public bool Falling => !v && prev; public void Sample() { prev = v; v = tag && tag.Value; } }
[Serializable] public class BoolOut { public PLCOutputBool tag; public void Set(bool x) { if (tag) tag.Value = x; } public bool Get() { return tag ? tag.Value : false; } }
[Serializable] public class FloatIn { public PLCInputFloat tag; public float v, prev; public void Sample() { prev = v; v = tag ? tag.Value : 0f; } }
[Serializable] public class FloatOut { public PLCOutputFloat tag; public void Set(float x) { if (tag) tag.Value = x; } public float Get() { return tag ? tag.Value : 0f; } }
#endregion

/// Auto feed + cut sequencer (UNITS: mm, mm/s) with runtime MU capture, stage switching only on BladeDown↑.
public class AutoFeedSequencer : MonoBehaviour
{
    // ===== Lifecycle (Inputs)
    [Header("Lifecycle (Inputs)")]
    public BoolIn Cmd_Start, Cmd_Stop, Cmd_Reset;
    public BoolIn EStop_OK;

    // ===== Conveyor & eyes (Inputs)
    [Header("Conveyor & Eyes (Inputs)  [mm / mm/s]")]
    public FloatIn Conv_Infeed_Position_mm;
    public BoolIn Conv_Infeed_IsDriving;   // optional
    public BoolIn PE_Cutter_Entry;         // PLC mirror of entry sensor (occupied)
    public BoolIn PE_Cutter_Exit;          // PLC exit eye

    // ===== Entry Sensor (runtime MU capture)
    [Header("Entry Sensor (runtime MU capture)")]
    [Tooltip("Same physical sensor as PE_Cutter_Entry, but the Sensor component. Used to get the spawned MU instance.")]
    public Sensor EntrySensor;

    // ===== Blade safety & limits (Inputs)
    [Header("Blade Safety & Limits (Inputs)")]
    public BoolIn LC_Cutter_Guard_OK;
    public BoolIn LS_Blade_Up;
    public BoolIn LS_Blade_Down;

    // ===== Conveyor (Outputs)
    [Header("Conveyor (Outputs)  [mm / mm/s]")]
    public FloatOut Conv_Infeed_TargetSpeed_mmps;
    public BoolOut Conv_Infeed_Fwd;

    // ===== Blade jog (Outputs)
    [Header("Blade Jog (Outputs)")]
    public BoolOut Blade_JogDown; // down
    public BoolOut Blade_JogUp;   // up

    // ===== Spawn output (PLC)
    [Header("Cut Piece Spawning (PLC Output)")]
    [Tooltip("Boolean to the Source that generates the cut piece. TRUE on BladeDown↑, FALSE when PE_Cutter_Exit↓.")]
    public BoolOut Source_CutPiece_Generate;

    [Tooltip("Gate spawn/swap on a permitted stroke only.")]
    public bool RequireCutPermission = true;

    // ===== Setpoints & watchdogs
    [Header("Setpoints (Units: mm & mm/s)")]
    public float InfeedSpeed_mmps = 800f;
    public float CutLength_mm = 100f;

    [Header("Timings & Watchdogs (seconds)")]
    public float SettleTime_s = 0.20f;
    public float WD_ToEntry_s = 5.0f;
    public float WD_Feed_Scale = 1.5f;
    public float WD_CutDown_s = 5.0f;
    public float WD_CutUp_s = 5.0f;

    [Header("Policy")]
    public bool AllowRetractionOnGuardOpen = true;

    [Header("Debugging")]
    public bool VerboseLogs = true;
    public float LogInterval = 0.10f;
    float _nextLogAt = 0f;
    string lastFault = "";

    // ===== FSM
    public enum STATE { S_RESET, S0_APPROACH, S1_METER_FEED, S2_CUT_DOWN, S3_CUT_UP, S_HOLD, S_FAULT }
    [SerializeField] public STATE State = STATE.S_RESET;
    STATE _prevState;

    // ===== Internals
    float stateTimer, settleTimer;
    bool runEnable;
    bool feedInterlockLatched;
    bool cutPermissionLatched;
    float posAtEntry_mm, measuredSpeed_mmps, feedWD_s;

    // Runtime MU captured from the sensor; shown read-only in Inspector
    [SerializeField] MU _currentMU;
    int _appearanceIdx = 0; // 0 = before cut. After 1st cut -> 1, etc. Never wraps during same sheet.

    void Start()
    {
        Enter(STATE.S_RESET);

        if (EntrySensor != null)
            EntrySensor.EventMUSensor.AddListener(OnEntrySensorEvent);
        else
            Debug.LogWarning("[SEQ] EntrySensor not assigned; cannot capture runtime MU instance.");
    }

    void FixedUpdate()
    {
        // Inputs & speed
        SampleInputs();
        DeriveMeasuredSpeed();
        runEnable = EStop_OK.v;

        // Global stop/reset
        if (Cmd_Reset.Rising || !runEnable) Enter(STATE.S_RESET);
        if (Cmd_Stop.Rising) StopAllMotion();

        // Edge logs
        SensorEdgeLogs();

        // FSM
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

        // Level-based spawn + appearance switching
        HandleSpawnAndAppearance();

        // Logs
        if (VerboseLogs && Time.time >= _nextLogAt) { _nextLogAt = Time.time + Mathf.Max(0.01f, LogInterval); LogTick(); }
    }

    // ===== Runtime MU capture from entry sensor (no visual switching here) =====
    void OnEntrySensorEvent(MU mu, bool occupied)
    {
        if (!occupied) return;
        _currentMU = mu;          // capture the spawned instance
        _appearanceIdx = 0;       // reset internal stage counter
        // IMPORTANT: do NOT ApplyAppearance here — prevents “switching on sensor”
        if (VerboseLogs) Debug.Log($"[ENTRY] Captured MU '{(_currentMU ? _currentMU.name : "null")}', stage=0 (no visual change on sensor).");
    }

    // ===== States
    void Tick_RESET()
    {
        StopAllMotion();
        Source_CutPiece_Generate.Set(false);
        _appearanceIdx = 0;
        _currentMU = null;
        if (runEnable && LS_Blade_Up.v && Cmd_Start.Rising) Enter(STATE.S0_APPROACH);
    }

    void Tick_S0_APPROACH()
    {
        Conv_Infeed_TargetSpeed_mmps.Set(InfeedSpeed_mmps);
        Conv_Infeed_Fwd.Set(runEnable);

        if (PE_Cutter_Entry.v)
        {
            Conv_Infeed_Fwd.Set(false);
            settleTimer += Time.fixedDeltaTime;

            if (settleTimer >= SettleTime_s)
            {
                posAtEntry_mm = Conv_Infeed_Position_mm.v;
                float v = Mathf.Max(1f, measuredSpeed_mmps);
                feedWD_s = Mathf.Max(0.2f, (CutLength_mm / v) * WD_Feed_Scale);
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
        if (!feedInterlockLatched) { Fault("Feed latch not set"); return; }
        if (!runEnable) { Fault("EStop lost during metering"); return; }
        if (!LC_Cutter_Guard_OK.v) { Fault("Guard opened during metering"); return; }
        if (!LS_Blade_Up.v) { Fault("Blade not up during metering"); return; }

        Conv_Infeed_TargetSpeed_mmps.Set(InfeedSpeed_mmps);
        Conv_Infeed_Fwd.Set(true);

        float delta_mm = Conv_Infeed_Position_mm.v - posAtEntry_mm;
        if (delta_mm < -0.5f) { Fault($"Negative Δ {delta_mm:F1} mm"); return; }
        if (delta_mm > CutLength_mm * 10f) { Fault($"Δ spike {delta_mm:F1} mm"); return; }

        if (delta_mm >= CutLength_mm)
        {
            Conv_Infeed_Fwd.Set(false);
            cutPermissionLatched = runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;
            Debug.Log($"[SEQ] Stroke permission latched: {cutPermissionLatched}");
            Enter(STATE.S2_CUT_DOWN);
            return;
        }

        Watchdog(feedWD_s, "Metering watchdog");
    }

    void Tick_S2_CUT_DOWN()
    {
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

    void Tick_S_HOLD() => StopAllMotion();
    void Tick_S_FAULT() { StopAllMotion(); Source_CutPiece_Generate.Set(false); }

    // ===== Helpers
    void Enter(STATE s)
    {
        _prevState = State;
        State = s;
        stateTimer = 0f; settleTimer = 0f;

        if (s == STATE.S1_METER_FEED)
            feedInterlockLatched = runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;

        if (s == STATE.S_RESET || s == STATE.S_FAULT)
        {
            feedInterlockLatched = false;
            cutPermissionLatched = false;
        }

        Debug.Log($"[SEQ] STATE: {_prevState} -> {State} @ {Time.time:0.000}s (FeedLatched={feedInterlockLatched}, StrokeLatched={cutPermissionLatched})");
        if (s == STATE.S1_METER_FEED)
            Debug.Log($"[SEQ] Entry snapshot pos={Conv_Infeed_Position_mm.v:F1}mm; Cut={CutLength_mm:F1}mm; v={measuredSpeed_mmps:F1}mm/s");
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
        if (stateTimer > maxSec) Fault($"{label} exceeded {maxSec:0.00}s in {State}");
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
        PE_Cutter_Exit.Sample();
        Conv_Infeed_Position_mm.Sample();
        Conv_Infeed_IsDriving.Sample();
        LC_Cutter_Guard_OK.Sample();
        LS_Blade_Up.Sample();
        LS_Blade_Down.Sample();
        // NOTE: no visual switching on PE_Cutter_Entry here.
    }

    void SensorEdgeLogs()
    {
        if (PE_Cutter_Entry.Rising) Debug.Log($"[EDGE] PE_Cutter_Entry ↑ @ {Time.time:0.000}s");
        if (PE_Cutter_Entry.Falling) Debug.Log($"[EDGE] PE_Cutter_Entry ↓ @ {Time.time:0.000}s");
        if (PE_Cutter_Exit.Rising) Debug.Log($"[EDGE] PE_Cutter_Exit ↑ @ {Time.time:0.000}s");
        if (PE_Cutter_Exit.Falling) Debug.Log($"[EDGE] PE_Cutter_Exit ↓ @ {Time.time:0.000}s");
        if (LS_Blade_Up.Rising) Debug.Log($"[EDGE] LS_Blade_Up ↑ @ {Time.time:0.000}s");
        if (LS_Blade_Up.Falling) Debug.Log($"[EDGE] LS_Blade_Up ↓ @ {Time.time:0.000}s");
        if (LS_Blade_Down.Rising) Debug.Log($"[EDGE] LS_Blade_Down ↑ @ {Time.time:0.000}s");
        if (LS_Blade_Down.Falling) Debug.Log($"[EDGE] LS_Blade_Down ↓ @ {Time.time:0.000}s");
        if (LC_Cutter_Guard_OK.Rising) Debug.Log($"[EDGE] Guard OK ↑ @ {Time.time:0.000}s");
        if (LC_Cutter_Guard_OK.Falling) Debug.Log($"[EDGE] Guard OK ↓ @ {Time.time:0.000}s");
    }

    // ===== Spawn + MUAppearences switching =====
    void HandleSpawnAndAppearance()
    {
        bool inCutPhase = (State == STATE.S2_CUT_DOWN) || (State == STATE.S3_CUT_UP);
        bool gateOK = !RequireCutPermission || cutPermissionLatched;

        // Only switch on BladeDown rising
        if (LS_Blade_Down.Rising && inCutPhase && gateOK)
        {
            // Spawn signal level = true
            if (!Source_CutPiece_Generate.Get())
            {
                Source_CutPiece_Generate.Set(true);
                if (VerboseLogs) Debug.Log("[CUT] Generate=TRUE (BladeDown↑)");
            }

            // Advance stage to NEXT appearance (first cut -> index 1), clamp to last
            if (_currentMU != null && _currentMU.MUAppearences != null && _currentMU.MUAppearences.Count > 0)
            {
                int last = _currentMU.MUAppearences.Count - 1;
                _appearanceIdx = Mathf.Min(_appearanceIdx + 1, last); // no wrap during same sheet
                ApplyAppearanceByIndex(_appearanceIdx);
                if (VerboseLogs) Debug.Log($"[VIS] MU '{_currentMU.name}' -> MUAppearences idx={_appearanceIdx}/{last}");
            }
            else if (VerboseLogs)
            {
                Debug.LogWarning("[VIS] No runtime MU or MUAppearences not configured; skipping visual swap.");
            }
        }

        // When the cut piece leaves the exit eye -> drop generate
        if (PE_Cutter_Exit.Falling && Source_CutPiece_Generate.Get())
        {
            Source_CutPiece_Generate.Set(false);
            if (VerboseLogs) Debug.Log("[CUT] Generate=FALSE (Exit eye ↓)");
        }
    }

    // Enable selected appearance, DISABLE ALL others (MUSwitcher-style)
    void ApplyAppearanceByIndex(int idx)
    {
        if (_currentMU == null || _currentMU.MUAppearences == null || _currentMU.MUAppearences.Count == 0) return;
        idx = Mathf.Clamp(idx, 0, _currentMU.MUAppearences.Count - 1);

        for (int i = 0; i < _currentMU.MUAppearences.Count; i++)
        {
            var go = _currentMU.MUAppearences[i];
            if (!go) continue;
            go.SetActive(i == idx);
        }
    }

    // Measured speed from encoder (mm/s)
    void DeriveMeasuredSpeed()
    {
        float du_mm = Conv_Infeed_Position_mm.v - Conv_Infeed_Position_mm.prev;
        float dt = Mathf.Max(1e-4f, Time.fixedDeltaTime);
        measuredSpeed_mmps = du_mm / dt;
    }

    void LogTick()
    {
        float delta_mm = Conv_Infeed_Position_mm.v - posAtEntry_mm;
        string line =
            $"[TICK {Time.time:0.000}] State={State} | " +
            $"Run={runEnable} FeedLatched={feedInterlockLatched} StrokeLatched={cutPermissionLatched} | " +
            $"PEin={PE_Cutter_Entry.v} PEx={PE_Cutter_Exit.v} Up={LS_Blade_Up.v} Down={LS_Blade_Down.v} Guard={LC_Cutter_Guard_OK.v} | " +
            $"pos={Conv_Infeed_Position_mm.v:F1}mm Δ={delta_mm:F1}mm (L={CutLength_mm:F1}mm) v={measuredSpeed_mmps:F1}mm/s | " +
            $"Gen={Source_CutPiece_Generate.Get()} MU={(_currentMU ? _currentMU.name : "null")} stageIdx={_appearanceIdx}/{(_currentMU && _currentMU.MUAppearences != null ? _currentMU.MUAppearences.Count - 1 : -1)}";
        Debug.Log(line);
    }
}
