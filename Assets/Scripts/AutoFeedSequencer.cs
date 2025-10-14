using UnityEngine;
using System;
using realvirtual; // MU, Sensor

#region Tag wrappers (units baked in names)
[Serializable] public class BoolIn { public PLCInputBool tag; public bool v, prev; public bool Rising => v && !prev; public bool Falling => !v && prev; public void Sample() { prev = v; v = tag && tag.Value; } }
[Serializable] public class BoolOut { public PLCOutputBool tag; public void Set(bool x) { if (tag) tag.Value = x; } public bool Get() { return tag ? tag.Value : false; } }
[Serializable] public class FloatIn { public PLCInputFloat tag; public float v, prev; public void Sample() { prev = v; v = tag ? tag.Value : 0f; } }
[Serializable] public class FloatOut { public PLCOutputFloat tag; public void Set(float x) { if (tag) tag.Value = x; } public float Get() { return tag ? tag.Value : 0f; } }
#endregion

/// Auto feed + cut sequencer (UNITS: mm, mm/s) with:
/// - Boundary-gated Stop (no mid-step aborts)
/// - Latched E-Stop (hard kill) requiring Reset
/// - Safe re-feed after exit clear
/// - Outfeed to pick with watchdogs
/// - Spawn level: LS_Blade_Down↑ .. PE_Cutter_Exit↓
/// - Handshake to PickPlaceSequencer (PnP_Start/Busy/Done)
public class AutoFeedSequencer : MonoBehaviour
{
    // ===== Lifecycle (Inputs)
    [Header("Lifecycle (Inputs)")]
    public BoolIn Cmd_Start, Cmd_Stop, Cmd_Reset;
    public BoolIn EStop_OK;

    // ===== Conveyor & eyes (Inputs)
    [Header("Conveyor & Eyes (Inputs)  [mm / mm/s]")]
    public FloatIn Conv_Infeed_Position_mm;
    public BoolIn Conv_Infeed_IsDriving;       // optional
    public BoolIn PE_Cutter_Entry;             // long sheet at entry
    public BoolIn PE_Cutter_Exit;              // cut piece in throat
    public BoolIn PE_Pick_Pos;                 // piece arrived at pick

    // ===== Entry Sensor (runtime MU capture)
    [Header("Entry Sensor (runtime MU capture)")]
    public Sensor EntrySensor;

    // ===== Blade safety & limits (Inputs)
    [Header("Blade Safety & Limits (Inputs)")]
    public BoolIn LC_Cutter_Guard_OK;
    public BoolIn LS_Blade_Up;
    public BoolIn LS_Blade_Down;

    // ===== Infeed (Outputs)
    [Header("Infeed (Outputs)  [mm / mm/s]")]
    public FloatOut Conv_Infeed_TargetSpeed_mmps;
    public BoolOut Conv_Infeed_Fwd;

    // ===== Outfeed (Outputs)
    [Header("Outfeed (Outputs)  [mm / mm/s]")]
    public FloatOut Conv_Outfeed_TargetSpeed_mmps;
    public BoolOut Conv_Outfeed_Fwd;

    // ===== Blade jog (Outputs)
    [Header("Blade Jog (Outputs)")]
    public BoolOut Blade_JogDown; // down
    public BoolOut Blade_JogUp;   // up

    // ===== Spawn output (PLC)
    [Header("Cut Piece Spawning (PLC Output)")]
    [Tooltip("Boolean to the Source that generates the cut piece. TRUE on BladeDown↑, FALSE when PE_Cutter_Exit↓.")]
    public BoolOut Source_CutPiece_Generate;

    [Tooltip("Gate spawn/visuals on a permitted stroke only.")]
    public bool RequireCutPermission = true;

    // ===== Handshake with Pick&Place
    [Header("PnP Handshake (PLC)")]
    public BoolOut PnP_Start;   // pulse when piece reached pick
    public BoolIn PnP_Busy;    // true while PnP runs
    public BoolIn PnP_Done;    // rising-edge at end of a PnP cycle
    bool _pnpPulseArmed = false; // drop PnP_Start next frame

    // ===== Setpoints & watchdogs
    [Header("Setpoints (Units: mm & mm/s)")]
    public float InfeedSpeed_mmps = 800f;
    public float OutfeedSpeed_mmps = 800f;
    [Tooltip("Length to feed after entry eye trips (mm)")]
    public float CutLength_mm = 100f;

    [Header("Timings & Watchdogs (seconds)")]
    public float SettleTime_s = 0.20f;
    public float WD_ToEntry_s = 5.0f;
    public float WD_Feed_Scale = 1.5f;   // feed WD ~ (CutLength/Speed) * scale
    public float WD_CutDown_s = 5.0f;
    public float WD_CutUp_s = 5.0f;
    public float WD_ExitClear_s = 3.0f;   // time to clear cutter throat
    public float WD_ToPick_s = 5.0f;   // time from exit clear to reach pick
    public float WD_Refeed_s = 5.0f;   // time from NextFeedArmed to next BladeDown (paused while Stop=1)

    [Header("Policy")]
    public bool AllowRetractionOnGuardOpen = true;

    [Header("Debugging")]
    public bool VerboseLogs = true;
    public float LogInterval = 0.10f;
    float _nextLogAt = 0f;
    string lastFault = "";

    // ===== FSM
    public enum STATE { S_RESET, S0_APPROACH, S1_METER_FEED, S2_CUT_DOWN, S3_CUT_UP, S4_RELEASE_TO_PICK, S_HOLD, S_FAULT }
    [SerializeField] public STATE State = STATE.S_RESET;
    STATE _prevState;

    // ===== Internals
    float stateTimer, settleTimer;
    bool runEnable;
    bool EStopLatched;                 // latched hard kill
    bool feedInterlockLatched;
    bool cutPermissionLatched;
    bool NextFeedArmed;                // set at Exit↓ if Entry==1, cleared when entering next S1
    float posAtEntry_mm, measuredSpeed_mmps, feedWD_s;

    // Refeed and pick watchdog helpers
    float refeedTimer_s = 0f;
    bool pickTimerActive = false;
    float pickTimer_s = 0f;
    bool exitClearedThisCycle = false;

    // Runtime MU for visuals
    [SerializeField] MU _currentMU;
    int _appearanceIdx = 0;

    void Start()
    {
        Enter(STATE.S_RESET);
        if (EntrySensor != null)
            EntrySensor.EventMUSensor.AddListener(OnEntrySensorEvent);
        else
            Debug.LogWarning("[SEQ] EntrySensor not assigned; cannot capture runtime MU instance.");
    }
    void OnDestroy()
    {
        if (EntrySensor != null)
            EntrySensor.EventMUSensor.RemoveListener(OnEntrySensorEvent);
    }

    void FixedUpdate()
    {
        // Inputs & speed
        SampleInputs();
        DeriveMeasuredSpeed();

        // Drop PnP start pulse one frame later
        if (_pnpPulseArmed) { _pnpPulseArmed = false; PnP_Start.Set(false); }

        // E-Stop latch: any drop kills immediately and latches until Reset
        if (!EStop_OK.v)
        {
            if (!EStopLatched) Debug.LogWarning("[SEQ] E-STOP triggered -> latched");
            EStopLatched = true;
            Enter(STATE.S_RESET); // forces outputs safe immediately
        }
        runEnable = !EStopLatched && EStop_OK.v;

        // Reset clears E-Stop latch and faults
        if (Cmd_Reset.Rising)
        {
            EStopLatched = false;
            Enter(STATE.S_RESET);
        }

        // Edge logs
        SensorEdgeLogs();

        // FSM (Stop is boundary-gated inside ticks; NOT an immediate motion kill)
        switch (State)
        {
            case STATE.S_RESET: Tick_RESET(); break;
            case STATE.S0_APPROACH: Tick_S0_APPROACH(); break;
            case STATE.S1_METER_FEED: Tick_S1_METER_FEED(); break;
            case STATE.S2_CUT_DOWN: Tick_S2_CUT_DOWN(); break;
            case STATE.S3_CUT_UP: Tick_S3_CUT_UP(); break;
            case STATE.S4_RELEASE_TO_PICK: Tick_S4_RELEASE_TO_PICK(); break;
            case STATE.S_HOLD: Tick_S_HOLD(); break;
            case STATE.S_FAULT: Tick_S_FAULT(); break;
        }

        // Spawn + visuals
        HandleSpawnAndAppearance();

        // Cross-state watchdog timers
        if (NextFeedArmed && !Cmd_Stop.v)
        {
            refeedTimer_s += Time.fixedDeltaTime;
            if (refeedTimer_s > WD_Refeed_s) Fault("Refeed timeout");
        }
        if (pickTimerActive)
        {
            pickTimer_s += Time.fixedDeltaTime;
            if (pickTimer_s > WD_ToPick_s) Fault("ToPick timeout");
        }

        // Logs
        if (VerboseLogs && Time.time >= _nextLogAt)
        {
            _nextLogAt = Time.time + Mathf.Max(0.01f, LogInterval);
            LogTick();
        }
    }

    // ===== Runtime MU capture (no visual switch here)
    void OnEntrySensorEvent(MU mu, bool occupied)
    {
        if (!occupied) return;
        _currentMU = mu;
        _appearanceIdx = 0;
        if (VerboseLogs) Debug.Log($"[ENTRY] Captured MU '{(_currentMU ? _currentMU.name : "null")}', stage=0 (no visual change on sensor).");
    }

    // ===== States
    void Tick_RESET()
    {
        StopAllMotion();
        Source_CutPiece_Generate.Set(false);
        PnP_Start.Set(false);
        _pnpPulseArmed = false;

        _appearanceIdx = 0;
        _currentMU = null;

        feedInterlockLatched = false;
        cutPermissionLatched = false;
        NextFeedArmed = false;
        refeedTimer_s = 0f;
        pickTimerActive = false;
        pickTimer_s = 0f;
        exitClearedThisCycle = false;

        // Only allow Start when NOT latched and blade is up
        if (runEnable && LS_Blade_Up.v && Cmd_Start.Rising)
            Enter(STATE.S0_APPROACH);
    }

    void Tick_S0_APPROACH()
    {
        // Approach until entry eye
        Conv_Infeed_TargetSpeed_mmps.Set(InfeedSpeed_mmps);
        Conv_Infeed_Fwd.Set(runEnable);

        if (PE_Cutter_Entry.v)
        {
            Conv_Infeed_Fwd.Set(false); // stop, settle at the edge
            settleTimer += Time.fixedDeltaTime;

            if (settleTimer >= SettleTime_s)
            {
                // Boundary-gated Stop: hold here without starting S1
                if (Cmd_Stop.v) return;

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
        if (!runEnable) { Fault("EStop/Run lost during metering"); return; }
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

            // Boundary-gated Stop: hold here with infeed off; don't start S2
            if (Cmd_Stop.v) return;

            cutPermissionLatched = runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;
            Debug.Log($"[SEQ] Stroke permission latched: {cutPermissionLatched}");
            Enter(STATE.S2_CUT_DOWN);
            return;
        }

        Watchdog(feedWD_s, "Metering watchdog");
    }

    void Tick_S2_CUT_DOWN()
    {
        if (!runEnable) { Fault("EStop/Run lost on down"); return; }
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
        if (!runEnable) { Fault("EStop/Run lost on up"); return; }
        if (!AllowRetractionOnGuardOpen && !LC_Cutter_Guard_OK.v) { Fault("Guard opened on up (blocked)"); return; }

        Blade_JogDown.Set(false);
        Blade_JogUp.Set(true);

        if (LS_Blade_Up.v && !LS_Blade_Down.v)
        {
            Blade_JogUp.Set(false);

            // Boundary-gated Stop before entering S4
            if (Cmd_Stop.v) return;

            Enter(STATE.S4_RELEASE_TO_PICK);
            return;
        }

        Watchdog(WD_CutUp_s, "Cut-up watchdog");
    }

    void Tick_S4_RELEASE_TO_PICK()
    {
        // Outfeed runs until pick; independent from infeed/refeed
        Conv_Outfeed_TargetSpeed_mmps.Set(OutfeedSpeed_mmps);
        Conv_Outfeed_Fwd.Set(true);

        // Exit clear ends spawn and arms next feed (if entry has material)
        if (!exitClearedThisCycle && PE_Cutter_Exit.Falling)
        {
            exitClearedThisCycle = true;
            Source_CutPiece_Generate.Set(false);

            if (PE_Cutter_Entry.v)
            {
                NextFeedArmed = true;
                refeedTimer_s = 0f; // start refeed watchdog (paused by Stop)
                if (VerboseLogs) Debug.Log("[SEQ] Exit clear -> NextFeedArmed=TRUE");
            }
            else
            {
                NextFeedArmed = false; // END infeed cycle when entry is empty
                if (VerboseLogs) Debug.Log("[SEQ] Exit clear -> Entry empty, ending infeed cycle");
            }

            // Start pick timer after throat is clear
            pickTimerActive = true;
            pickTimer_s = 0f;
        }

        // If exit didn’t clear in time -> fault
        if (!exitClearedThisCycle) Watchdog(WD_ExitClear_s, "Exit clear watchdog");

        // Stop outfeed exactly once at pick position + fire PnP start pulse
        if (PE_Pick_Pos.Rising)
        {
            Conv_Outfeed_Fwd.Set(false);
            pickTimerActive = false; // arrival reached; stop pick watchdog

            // Fire PnP_Start pulse if PnP not already busy
            if (!PnP_Busy.v)
            {
                PnP_Start.Set(true);
                _pnpPulseArmed = true; // auto-drop next frame
                if (VerboseLogs) Debug.Log("[SEQ] PnP_Start pulse sent (piece at pick)");
            }
        }

        // If PnP is running, we hold refeed until PnP_Done
        if (PnP_Busy.v) return;

        // If PnP just finished, allow refeed (regular logic below will advance)
        if (PnP_Done.Rising && VerboseLogs) Debug.Log("[SEQ] PnP_Done received");

        // Refeed path: only if armed, safe, STOP=0
        if (NextFeedArmed && runEnable && LC_Cutter_Guard_OK.v && LS_Blade_Up.v && !Cmd_Stop.v)
        {
            if (!PE_Cutter_Entry.v)
            {
                // Sheet ran out after arming; end infeed cycle.
                NextFeedArmed = false;
            }
            else
            {
                posAtEntry_mm = Conv_Infeed_Position_mm.v;
                float v = Mathf.Max(1f, measuredSpeed_mmps);
                feedWD_s = Mathf.Max(0.2f, (CutLength_mm / v) * WD_Feed_Scale);

                // Consume the arm ONLY when we actually start the next meter
                NextFeedArmed = false;
                refeedTimer_s = 0f; // clean slate for next cycle

                Enter(STATE.S1_METER_FEED);
                return;
            }
        }
        // If STOP=1, we simply hold in this state; outfeed continues until pick.
    }

    void Tick_S_HOLD() { StopAllMotion(); }
    void Tick_S_FAULT() { StopAllMotion(); Source_CutPiece_Generate.Set(false); }

    // ===== Helpers
    void Enter(STATE s)
    {
        _prevState = State;
        State = s;
        stateTimer = 0f; settleTimer = 0f;

        if (s == STATE.S1_METER_FEED)
            feedInterlockLatched = (!EStopLatched) && EStop_OK.v && LC_Cutter_Guard_OK.v && LS_Blade_Up.v;

        if (s == STATE.S_RESET || s == STATE.S_FAULT)
        {
            feedInterlockLatched = false;
            cutPermissionLatched = false;
            NextFeedArmed = false;
            refeedTimer_s = 0f;
            pickTimerActive = false;
            pickTimer_s = 0f;
            exitClearedThisCycle = false;
        }

        if (s == STATE.S4_RELEASE_TO_PICK)
        {
            // starting release phase
            exitClearedThisCycle = false;
            pickTimerActive = false; // starts only when exit clears
            pickTimer_s = 0f;
        }

        Debug.Log($"[SEQ] STATE: {_prevState} -> {State} @ {Time.time:0.000}s (Stop={Cmd_Stop.v}, EStopLatched={EStopLatched}, FeedLatched={feedInterlockLatched}, StrokeLatched={cutPermissionLatched}, NextFeedArmed={NextFeedArmed})");
        if (s == STATE.S1_METER_FEED)
            Debug.Log($"[SEQ] Entry snapshot pos={Conv_Infeed_Position_mm.v:F1}mm; Cut={CutLength_mm:F1}mm; v={measuredSpeed_mmps:F1}mm/s");
    }

    void StopAllMotion()
    {
        Conv_Infeed_Fwd.Set(false);
        Conv_Outfeed_Fwd.Set(false);
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
        PE_Pick_Pos.Sample();
        Conv_Infeed_Position_mm.Sample();
        Conv_Infeed_IsDriving.Sample();
        LC_Cutter_Guard_OK.Sample();
        LS_Blade_Up.Sample();
        LS_Blade_Down.Sample();

        // PnP handshake
        PnP_Busy.Sample();
        PnP_Done.Sample();
    }

    void SensorEdgeLogs()
    {
        if (PE_Cutter_Entry.Rising) Debug.Log($"[EDGE] PE_Cutter_Entry ↑ @ {Time.time:0.000}s");
        if (PE_Cutter_Entry.Falling) Debug.Log($"[EDGE] PE_Cutter_Entry ↓ @ {Time.time:0.000}s");
        if (PE_Cutter_Exit.Rising) Debug.Log($"[EDGE] PE_Cutter_Exit ↑ @ {Time.time:0.000}s");
        if (PE_Cutter_Exit.Falling) Debug.Log($"[EDGE] PE_Cutter_Exit ↓ @ {Time.time:0.000}s");
        if (PE_Pick_Pos.Rising) Debug.Log($"[EDGE] PE_Pick_Pos ↑ @ {Time.time:0.000}s");
        if (PE_Pick_Pos.Falling) Debug.Log($"[EDGE] PE_Pick_Pos ↓ @ {Time.time:0.000}s");
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

        // On BladeDown↑: generate & advance visual
        if (LS_Blade_Down.Rising && inCutPhase && gateOK)
        {
            if (!Source_CutPiece_Generate.Get())
            {
                Source_CutPiece_Generate.Set(true);
                if (VerboseLogs) Debug.Log("[CUT] Generate=TRUE (BladeDown↑)");
            }

            if (_currentMU != null && _currentMU.MUAppearences != null && _currentMU.MUAppearences.Count > 0)
            {
                int last = _currentMU.MUAppearences.Count - 1;
                _appearanceIdx = Mathf.Min(_appearanceIdx + 1, last);
                ApplyAppearanceByIndex(_appearanceIdx);
                if (VerboseLogs) Debug.Log($"[VIS] MU '{_currentMU.name}' -> MUAppearences idx={_appearanceIdx}/{last}");
            }
            else if (VerboseLogs)
            {
                Debug.LogWarning("[VIS] No runtime MU or MUAppearences not configured; skipping visual swap.");
            }
        }
        // On Exit↓: Source level resets in S4 (hand-off point).
    }

    // Enable selected appearance, disable all others
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
            $"Stop={Cmd_Stop.v} EStopLatched={EStopLatched} FeedLatched={feedInterlockLatched} StrokeLatched={cutPermissionLatched} NextFeedArmed={NextFeedArmed} | " +
            $"PEin={PE_Cutter_Entry.v} PEx={PE_Cutter_Exit.v} Pick={PE_Pick_Pos.v} Up={LS_Blade_Up.v} Down={LS_Blade_Down.v} Guard={LC_Cutter_Guard_OK.v} | " +
            $"pos={Conv_Infeed_Position_mm.v:F1}mm Δ={delta_mm:F1}mm (L={CutLength_mm:F1}mm) v={measuredSpeed_mmps:F1}mm/s | " +
            $"Gen={Source_CutPiece_Generate.Get()} PnP(Busy={PnP_Busy.v}) | " +
            $"refeedT={refeedTimer_s:0.00}s pickT={(pickTimerActive ? pickTimer_s : 0f):0.00}s";
        Debug.Log(line);
    }
}
