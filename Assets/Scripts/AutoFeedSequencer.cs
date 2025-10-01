using UnityEngine;
using System;

[Serializable]
public class BoolIn { public realvirtual.PLCInputBool tag; public bool v, prev; public bool Rising => v && !prev; public void Sample() { prev = v; v = tag && tag.Value; } }
[Serializable]
public class BoolOut { public realvirtual.PLCOutputBool tag; public void Set(bool x) { if (tag) tag.Value = x; } }
[Serializable]
public class FloatIn { public realvirtual.PLCInputFloat tag; public float v; public void Sample() { v = tag ? tag.Value : 0f; } }
[Serializable]
public class FloatOut { public realvirtual.PLCOutputFloat tag; public void Set(float x) { if (tag) tag.Value = x; } }

public class AutoFeedSequencer : MonoBehaviour
{
    // ===== Inputs =====
    [Header("Lifecycle (Inputs)")]
    public BoolIn Cmd_Start, Cmd_Stop, Cmd_Reset;
    public BoolIn EStop_OK;

    [Header("Conveyor & Sensor (Inputs)")]
    public BoolIn Conv_Infeed_IsDriving;
    public FloatIn Conv_Infeed_Position;   // optional
    public BoolIn PE_Cutter_Entry;

    // ===== Outputs =====
    [Header("Conveyor (Outputs)")]
    public FloatOut Conv_Infeed_TargetSpeed; // e.g., TransportGuided target speed
    public BoolOut Conv_Infeed_Fwd;

    // ===== Config =====
    [Header("Config")]
    [Tooltip("Speed while indexing to cutter (m/s or unit in your driver)")]
    public float InfeedSpeed = 0.8f;
    [Tooltip("Time to wait after stopping at PE_Cutter_Entry (sec)")]
    public float SettleTime = 0.20f;
    [Tooltip("Max time allowed to reach PE_Cutter_Entry (sec)")]
    public float WD_ToEntry = 5.0f;

    // ===== State =====
    public enum FEED_STATE { S_RESET, S0_IDLE, S1_INDEX_CUT, S_FAULT }
    [SerializeField] FEED_STATE State = FEED_STATE.S_RESET;

    float stateEnterTime, stateTimer, settleTimer;
    bool runEnable, alarmTimeout;

    void Start() { Enter(FEED_STATE.S_RESET); }

    void FixedUpdate()
    {
        // 1) Sample inputs
        Cmd_Start.Sample(); Cmd_Stop.Sample(); Cmd_Reset.Sample();
        EStop_OK.Sample();
        Conv_Infeed_IsDriving.Sample();
        Conv_Infeed_Position.Sample(); // optional, not used yet
        PE_Cutter_Entry.Sample();

        // 2) Safety gating
        runEnable = EStop_OK.v;

        // 3) Reset at any time or if EStop lost
        if (Cmd_Reset.Rising || !runEnable) { Enter(FEED_STATE.S_RESET); }

        // 4) Tick FSM
        switch (State)
        {
            case FEED_STATE.S_RESET: Tick_RESET(); break;
            case FEED_STATE.S0_IDLE: Tick_S0(); break;
            case FEED_STATE.S1_INDEX_CUT: Tick_S1(); break;
            case FEED_STATE.S_FAULT: Tick_FAULT(); break;
        }

        // 5) Stop command is immediate (levels)
        if (Cmd_Stop.Rising) StopConveyor();
    }

    // ===== STATES =====
    void Tick_RESET()
    {
        StopConveyor();
        alarmTimeout = false;

        // Clean start only if safe + explicit Start
        if (runEnable && Cmd_Start.Rising)
            Enter(FEED_STATE.S0_IDLE);
    }

    void Tick_S0()
    {
        // Run towards cutter until sensor goes high
        Conv_Infeed_TargetSpeed.Set(InfeedSpeed);
        Conv_Infeed_Fwd.Set(runEnable && !Cmd_Stop.v);

        // Watchdog: must reach entry within WD_ToEntry
        if (PE_Cutter_Entry.v)
        {
            StopConveyor();
            settleTimer += Time.fixedDeltaTime;
            if (settleTimer >= SettleTime)
                Enter(FEED_STATE.S1_INDEX_CUT); // Done with feed stage, parked at cutter
        }
        else
        {
            Watchdog(WD_ToEntry);
        }
    }

    void Tick_S1()
    {
        // Parked & stable at cutter entry. Hold conveyor off.
        StopConveyor();
        // This state is the clean handoff to the next sequencer stage (CUT).
        // We remain here until an external controller advances, or until Reset.
    }

    void Tick_FAULT()
    {
        StopConveyor();
        // Wait for Reset to recover
    }

    // ===== Helpers =====
    void Enter(FEED_STATE s)
    {
        State = s;
        stateEnterTime = Time.fixedTime;
        stateTimer = 0f; settleTimer = 0f;
        // Debug.Log($"[FEED] -> {s} @ {Time.time:F3}");
    }

    void StopConveyor()
    {
        Conv_Infeed_Fwd.Set(false);
        // keep target speed as configured; only level output off
    }

    void Watchdog(float maxSec)
    {
        stateTimer += Time.fixedDeltaTime;
        if (stateTimer > maxSec)
        {
            alarmTimeout = true;
            Enter(FEED_STATE.S_FAULT);
            // Debug.LogWarning("[FEED] Watchdog: failed to reach PE_Cutter_Entry in time");
        }
    }

    // ===== Public inspectors (optional) =====
    [ContextMenu("Force Start")]
    void _DevStart() { Cmd_Start.prev = false; Cmd_Start.v = true; }
    [ContextMenu("Force Stop")]
    void _DevStop() { Cmd_Stop.prev = false; Cmd_Stop.v = true; }
    [ContextMenu("Force Reset")]
    void _DevReset() { Cmd_Reset.prev = false; Cmd_Reset.v = true; }
}
