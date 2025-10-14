using UnityEngine;
using System;
using realvirtual; // PLC tags

/// Deterministic 4-axis pick & place, waypoint-index driven (UNITS: mm & mm/s for speeds; destinations are index IDs).
/// Stop = boundary gate (finish current step then hold). E-Stop = latched hard kill requiring Reset.
/// Handshake with cutter: PnP_Start (as Cmd_Start input), PnP_Busy (output), PnP_Done (one-frame pulse output).
public class PickPlaceSequencer : MonoBehaviour
{
    // ===== Lifecycle / handshake =====
    [Header("Lifecycle (Inputs)")]
    public BoolIn Cmd_Start;        // one-shot from AutoFeedSequencer (PnP_Start pulse)
    public BoolIn Cmd_Stop;         // boundary-gated stop (level)
    public BoolIn Cmd_Reset;        // one-shot
    public BoolIn EStop_OK;         // level; latched internally

    [Header("Handshake to Cutter (Outputs)")]
    public BoolOut PnP_Busy;        // 1 while a PnP cycle is running
    public BoolOut PnP_Done;        // one-frame pulse at end of successful cycle
    bool _donePulseArmed = false;   // drop PnP_Done next frame

    // ===== Interlocks / cell inputs =====
    [Header("Cell Interlocks (Inputs)")]
    public BoolIn PE_Pick_Pos;          // part staged at pick (conveyor stopped)
    public BoolIn Press_Area_Clear;     // safe to enter press area
    public BoolIn Press_ReadyForPlace;  // press nest ready (can be tied high)

    // ===== Gripper IO =====
    [Header("Gripper IO")]
    public BoolOut Vacuum_On;           // output
    public BoolIn Vacuum_OK;           // input
    public bool HoldVacuumOnEStop = true;
    public int VacuumRetries = 1;   // attempts to re-peck if no Vacuum_OK
    public float PlaceDwell_s = 0.10f;
    public float WD_Vacuum_OK_s = 0.5f;
    public float WD_Vacuum_Release_s = 0.4f;

    // ===== Axes (Drive_DestinationMotor) =====
    [Serializable]
    public class Axis
    {
        [Header("Outputs")]
        public FloatOut DestinationIndex;   // integer indices (use whole numbers)
        public FloatOut TargetSpeed_mmps;   // mm/s (for rotary, store deg/s here and name accordingly)
        public BoolOut StartDrive;         // one-shot pulse

        [Header("Inputs")]
        public BoolIn IsAtPosition;         // 1 when at commanded destination
        public BoolIn IsAtSpeed;            // optional
        public BoolIn IsDriving;            // optional

        public void PulseStart() { if (StartDrive != null) StartDrive.Set(true); }
        public void ClearStart() { if (StartDrive != null) StartDrive.Set(false); }
    }

    [Header("Axes (X,Y,Z,R)")]
    public Axis AxisX;
    public Axis AxisY;
    public Axis AxisZ;
    public Axis AxisR;

    // ===== Waypoint indices & speeds (per axis) =====
    [Serializable]
    public struct Pose
    {
        public int Xidx, Yidx, Zidx, Ridx;    // destination indices in your motors
        public float Vxy_mmps;                // speed for X & Y
        public float Vz_mmps;                 // speed for Z
        public float Vr_degps;                // rotary speed (deg/s) or mm/s if linear R
    }

    [Header("Waypoints (indices & speeds)")]
    public Pose HOME;
    public Pose PICK_CLEAR;
    public Pose PICK_DOWN;
    public Pose CARRY_CLEAR;
    public Pose PLACE_CLEAR;
    public Pose PLACE_DOWN;
    public Pose RET_CLEAR; // optional between-cycles park

    // ===== Watchdogs (seconds) =====
    [Header("Watchdogs (seconds)")]
    public float WD_Move_PickClr = 2.0f;
    public float WD_Move_PickDown = 1.5f;
    public float WD_Move_Lift = 1.5f;
    public float WD_Move_PlaceClr = 2.0f;
    public float WD_Move_PlaceDown = 1.5f;
    public float WD_Move_RetClr = 2.0f;

    // ===== Debug =====
    [Header("Debug")]
    public bool VerboseLogs = true;
    public float LogInterval = 0.15f;
    float _nextLogAt = 0f;

    // ===== FSM =====
    public enum STATE { P_RESET, P0_IDLE, P1_APPROACH_PICK, P2_PICK_DOWN, P3_LIFT_CLEAR, P4_TRANSIT_TO_PLACE, P5_PLACE_DOWN, P6_RELEASE, P7_RETRACT_CLEAR, P_DONE, P_FAULT }
    [SerializeField] public STATE State = STATE.P_RESET;
    STATE _prevState;

    // ===== Internals =====
    bool EStopLatched;
    float stateTimer;
    int retryCount;

    void Start() { Enter(STATE.P_RESET); }

    void FixedUpdate()
    {
        // Drop PnP_Done pulse after one frame
        if (_donePulseArmed) { _donePulseArmed = false; PnP_Done.Set(false); }

        // Sample inputs
        Cmd_Start.Sample(); Cmd_Stop.Sample(); Cmd_Reset.Sample(); EStop_OK.Sample();
        PE_Pick_Pos.Sample(); Press_Area_Clear.Sample(); Press_ReadyForPlace.Sample();
        Vacuum_OK.Sample();

        AxisX.IsAtPosition.Sample(); AxisX.IsAtSpeed.Sample(); AxisX.IsDriving.Sample();
        AxisY.IsAtPosition.Sample(); AxisY.IsAtSpeed.Sample(); AxisY.IsDriving.Sample();
        AxisZ.IsAtPosition.Sample(); AxisZ.IsAtSpeed.Sample(); AxisZ.IsDriving.Sample();
        AxisR.IsAtPosition.Sample(); AxisR.IsAtSpeed.Sample(); AxisR.IsDriving.Sample();

        // E-Stop latch
        if (!EStop_OK.v)
        {
            if (!EStopLatched && VerboseLogs) Debug.LogWarning("[PnP] E-STOP -> latched");
            EStopLatched = true;
            SafeOutputsOnAbort();
            Enter(STATE.P_RESET);
        }

        // Reset
        if (Cmd_Reset.Rising)
        {
            EStopLatched = false;
            Enter(STATE.P_RESET);
        }

        // FSM
        switch (State)
        {
            case STATE.P_RESET: Tick_RESET(); break;
            case STATE.P0_IDLE: Tick_P0_IDLE(); break;
            case STATE.P1_APPROACH_PICK: Tick_P1_APPROACH_PICK(); break;
            case STATE.P2_PICK_DOWN: Tick_P2_PICK_DOWN(); break;
            case STATE.P3_LIFT_CLEAR: Tick_P3_LIFT_CLEAR(); break;
            case STATE.P4_TRANSIT_TO_PLACE: Tick_P4_TRANSIT_TO_PLACE(); break;
            case STATE.P5_PLACE_DOWN: Tick_P5_PLACE_DOWN(); break;
            case STATE.P6_RELEASE: Tick_P6_RELEASE(); break;
            case STATE.P7_RETRACT_CLEAR: Tick_P7_RETRACT_CLEAR(); break;
            case STATE.P_DONE: Tick_P_DONE(); break;
            case STATE.P_FAULT: Tick_P_FAULT(); break;
        }

        // Logs
        if (VerboseLogs && Time.time >= _nextLogAt)
        {
            _nextLogAt = Time.time + Mathf.Max(0.05f, LogInterval);
            Debug.Log($"[PnP TICK {Time.time:0.000}] State={State} Stop={Cmd_Stop.v} EstopLatched={EStopLatched} Pick={PE_Pick_Pos.v} Clear={Press_Area_Clear.v} Ready={Press_ReadyForPlace.v} VacOK={Vacuum_OK.v}");
        }
    }

    // ===== STATE Ticks =====
    void Tick_RESET()
    {
        SafeOutputsOnAbort();
        PnP_Busy.Set(false);
        PnP_Done.Set(false);
        _donePulseArmed = false;
        retryCount = 0;

        // On reset, command HOME once when we get Start (but only if EStop not latched)
        if (!EStopLatched && Cmd_Start.Rising)
        {
            CommandPose(HOME);
            Enter(STATE.P0_IDLE);
        }
    }

    void Tick_P0_IDLE()
    {
        PnP_Busy.Set(false);
        // ensure Done is low in idle
        PnP_Done.Set(false);

        // Start cycle only if: Start pulse, part present, press area clear
        if (!EStopLatched && Cmd_Start.Rising && PE_Pick_Pos.v && Press_Area_Clear.v)
        {
            PnP_Busy.Set(true);
            if (Cmd_Stop.v) return; // boundary hold

            CommandPose(PICK_CLEAR);
            Enter(STATE.P1_APPROACH_PICK);
        }
    }

    void Tick_P1_APPROACH_PICK()
    {
        if (AllAtPose(PICK_CLEAR))
        {
            if (Cmd_Stop.v) return; // boundary hold
            CommandPose(PICK_DOWN, onlyZ: true); // Z-only approach
            Enter(STATE.P2_PICK_DOWN);
            return;
        }
        Watchdog(WD_Move_PickClr, "Move to PICK_CLEAR");
    }

    void Tick_P2_PICK_DOWN()
    {
        if (AxisZ.IsAtPosition.v)
        {
            Vacuum_On.Set(true);

            if (Vacuum_OK.v)
            {
                if (Cmd_Stop.v) return; // boundary hold after grip
                CommandPose(CARRY_CLEAR, onlyZ: true);
                Enter(STATE.P3_LIFT_CLEAR);
                return;
            }
            Watchdog(WD_Vacuum_OK_s, "Vacuum_OK");
            return;
        }
        Watchdog(WD_Move_PickDown, "Move to PICK_DOWN");
    }

    void Tick_P3_LIFT_CLEAR()
    {
        if (AxisZ.IsAtPosition.v)
        {
            if (Cmd_Stop.v) return; // boundary hold
            if (Press_Area_Clear.v && Press_ReadyForPlace.v)
            {
                CommandPose(PLACE_CLEAR, xy: true, r: true); // XYR to place, Z stays high
                Enter(STATE.P4_TRANSIT_TO_PLACE);
                return;
            }
            // optional: add interlock watchdog if desired
        }
        else
        {
            Watchdog(WD_Move_Lift, "Lift to CARRY_CLEAR");
        }
    }

    void Tick_P4_TRANSIT_TO_PLACE()
    {
        if (!Press_Area_Clear.v) { /* hold until safe */ return; }

        if (AtPoseXYR(PLACE_CLEAR))
        {
            if (Cmd_Stop.v) return; // boundary hold
            CommandPose(PLACE_DOWN, onlyZ: true); // descend Z only
            Enter(STATE.P5_PLACE_DOWN);
            return;
        }
        Watchdog(WD_Move_PlaceClr, "Transit to PLACE_CLEAR");
    }

    void Tick_P5_PLACE_DOWN()
    {
        if (AxisZ.IsAtPosition.v)
        {
            if (Cmd_Stop.v) return; // boundary hold before release
            Enter(STATE.P6_RELEASE);
            return;
        }
        Watchdog(WD_Move_PlaceDown, "Move to PLACE_DOWN");
    }

    void Tick_P6_RELEASE()
    {
        Vacuum_On.Set(false);
        if (Vacuum_OK.Falling || stateTimer >= PlaceDwell_s)
        {
            if (Cmd_Stop.v) return; // boundary hold
            CommandPose(RET_CLEAR);
            Enter(STATE.P7_RETRACT_CLEAR);
            return;
        }
        Watchdog(WD_Vacuum_Release_s, "Vacuum release");
    }

    void Tick_P7_RETRACT_CLEAR()
    {
        if (AllAtPose(RET_CLEAR))
        {
            Enter(STATE.P_DONE);
            return;
        }
        Watchdog(WD_Move_RetClr, "Retract to RET_CLEAR");
    }

    void Tick_P_DONE()
    {
        // Emit one-frame Done pulse
        if (!PnP_Done.Get())
        {
            PnP_Done.Set(true);
            _donePulseArmed = true; // will clear next FixedUpdate
        }

        PnP_Busy.Set(false);

        if (Cmd_Stop.v) return; // boundary hold (no-op)
        // Return to idle ready for next Start
        Enter(STATE.P0_IDLE);
    }

    void Tick_P_FAULT()
    {
        SafeOutputsOnAbort();
        PnP_Busy.Set(false);
        // Wait for Reset
    }

    // ===== Helpers =====
    void Enter(STATE s)
    {
        _prevState = State; State = s; stateTimer = 0f;
        if (VerboseLogs) Debug.Log($"[PnP] STATE: {_prevState} -> {State} @ {Time.time:0.000}s");

        if (s == STATE.P2_PICK_DOWN) retryCount = 0;
        if (s == STATE.P_RESET) { PnP_Done.Set(false); _donePulseArmed = false; }
    }

    void Watchdog(float maxSec, string label)
    {
        stateTimer += Time.fixedDeltaTime;
        if (stateTimer > maxSec)
        {
            if (State == STATE.P2_PICK_DOWN && retryCount < VacuumRetries)
            {
                retryCount++;
                if (VerboseLogs) Debug.LogWarning($"[PnP] {label} timeout -> retry #{retryCount}");
                Vacuum_On.Set(false);
                CommandPose(PICK_CLEAR, onlyZ: true); // Z up
                Enter(STATE.P1_APPROACH_PICK);
                return;
            }
            Debug.LogError($"[PnP] FAULT: {label} exceeded {maxSec:0.00}s in {State}");
            Enter(STATE.P_FAULT);
        }
    }

    void SafeOutputsOnAbort()
    {
        AxisX.ClearStart(); AxisY.ClearStart(); AxisZ.ClearStart(); AxisR.ClearStart();
        if (!HoldVacuumOnEStop) Vacuum_On.Set(false);
        // Keep PnP_Done low unless finishing a cycle
        PnP_Done.Set(false); _donePulseArmed = false;
    }

    // Pose commands
    void CommandPose(Pose p, bool onlyZ = false, bool xy = false, bool r = false)
    {
        // Decide axes: default = move all; onlyZ moves Z only; (xy / r) choose subsets
        bool moveX = (!onlyZ) && (xy || (!xy && !r));
        bool moveY = (!onlyZ) && (xy || (!xy && !r));
        bool moveR = (!onlyZ) && (r || (!xy && !r));
        bool moveZ = (onlyZ || (!xy && !r)) || onlyZ;

        if (moveX) { AxisX.DestinationIndex.Set(p.Xidx); AxisX.TargetSpeed_mmps.Set(p.Vxy_mmps); AxisX.PulseStart(); AxisX.ClearStart(); }
        if (moveY) { AxisY.DestinationIndex.Set(p.Yidx); AxisY.TargetSpeed_mmps.Set(p.Vxy_mmps); AxisY.PulseStart(); AxisY.ClearStart(); }
        if (moveR) { AxisR.DestinationIndex.Set(p.Ridx); AxisR.TargetSpeed_mmps.Set(p.Vr_degps); AxisR.PulseStart(); AxisR.ClearStart(); }
        if (moveZ) { AxisZ.DestinationIndex.Set(p.Zidx); AxisZ.TargetSpeed_mmps.Set(p.Vz_mmps); AxisZ.PulseStart(); AxisZ.ClearStart(); }

        if (VerboseLogs)
            Debug.Log($"[PnP CMD] Pose -> X:{(moveX ? p.Xidx : -1)} Y:{(moveY ? p.Yidx : -1)} Z:{(moveZ ? p.Zidx : -1)} R:{(moveR ? p.Ridx : -1)} | Vxy={p.Vxy_mmps} Vz={p.Vz_mmps} Vr={p.Vr_degps}");
    }

    bool AllAtPose(Pose p)
    {
        return AxisX.IsAtPosition.v && AxisY.IsAtPosition.v && AxisZ.IsAtPosition.v && AxisR.IsAtPosition.v;
    }
    bool AtPoseXYR(Pose p)
    {
        return AxisX.IsAtPosition.v && AxisY.IsAtPosition.v && AxisR.IsAtPosition.v;
    }
}
