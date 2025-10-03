// realvirtual.io (formerly game4automation) (R) a Framework for Automation Concept Design, Virtual Commissioning and 3D-HMI
// Copyright(c) 2019 realvirtual GmbH - Usage of this source code only allowed based on License conditions see https://realvirtual.io/unternehmen/lizenz

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using NaughtyAttributes;
using UnityEngine.InputSystem; // <— New Input System
#if REALVIRTUAL_AGX
using AGXUnity;
#endif
using Mesh = UnityEngine.Mesh;
using Random = System.Random;

namespace realvirtual
{
    //! Unity event triggered when an MU is created, passing the created MU as parameter
    [System.Serializable]
    public class realvirtualEventMUCreated : UnityEvent<MU> { }

    [SelectionBase]
    #region doc
    //! Source component that generates MUs (Movable Units) during simulation runtime, simulating production systems and material supply points.
    //! (Documentation block unchanged for brevity)
    #endregion
    [HelpURL("https://doc.realvirtual.io/components-and-scripts/source")]
    public class Source : BaseSource, ISignalInterface, IXRPlaceable
    {
        // -------- New Input System (replacing legacy key polls) --------
        [Header("Input (New Input System)")]
        [Tooltip("Action that triggers DeleteAll() (e.g., bound to <Keyboard>/delete)")]
        public InputActionReference DeleteAllAction;
        [Tooltip("Action that triggers Generate() once (e.g., bound to <Keyboard>/g or a button)")]
        public InputActionReference CreateOnSourceAction;

        // Public / UI Variables
#if REALVIRTUAL_AGX
        public bool UseAGXPhysics;
#else
        [HideInInspector] public bool UseAGXPhysics = false;
#endif
        [Header("General Settings")]
        [Tooltip("GameObject to use as template for creating MUs (leave empty to use this GameObject)")]
        public GameObject ThisObjectAsMU; //!< The referenced GameObject which should be used as a prototype for the MU. If it is null it will be this GameObject.
        [Tooltip("Optional destination where MUs will be created (leave empty to create at source position)")]
        public GameObject Destination; //!< The destination GameObject where the generated MU should be placed
        [Tooltip("Enable or disable MU generation")]
        public bool Enabled = true; //!< If set to true the Source is enabled
        [Tooltip("Freezes the source template position during simulation")]
        public bool FreezeSourcePosition = true; //!< If set to true the Source itself (the MU template) is fixed to its position
        [Tooltip("Hide the source template during simulation")]
        public bool DontVisualize = true; //!< True if the Source should not be visible during Simulation time
        [Tooltip("Hide source when simulation is stopped or paused")]
        public bool HideOnStop = true; //!< Hide this source when simulation is stopped / paused;
        [Tooltip("Mass of generated MUs in kilograms")]
        public float Mass = 1; //!< Mass of the generated MU in kilograms.
        [Tooltip("Manually set the center of mass for generated MUs")]
        public bool SetCenterOfMass = false;
        [Tooltip("Center of mass position in local coordinates")]
        public Vector3 CenterOfMass = new Vector3(0, 0, 0); //!< Center of mass position for the generated MU in local coordinates.
        [Tooltip("Layer name for generated MUs (leave empty to keep default layer)")]
        public string GenerateOnLayer = ""; //!< Layer where the MUs should be generated to - if kept empty no layers are changed
        [HideInInspector] public bool ChangeDefaultLayer = true;  //!< If set to true Layers are automatically changed if default Layer is detected
        [ReorderableList]
        [Tooltip("Component names to remove from generated MUs (e.g., Source scripts)")]
        public List<string> OnCreateDestroyComponents = new List<string>(); //!< Destroy these components on MU when MU is created as a copy of the source - is used to delete additional source scripts

        [Header("Create in Intverval (0 if not)")]
        [Tooltip("Delay in seconds before starting interval generation (0 = no delay)")]
        public float StartInterval = 0; //!< Start MU creation with the given seconds after simulation start
        [Tooltip("Time interval in seconds between MU generation (0 = disabled)")]
        public float Interval = 0; //!< Interval in seconds between the generation of MUs. Needs to be set to 0 if no interval generation is wished.

        [Header("Automatic Generation on Distance")]
        [Tooltip("Automatically generate new MU when last MU reaches specified distance")]
        public bool AutomaticGeneration = true; //!< Automatic generation of MUs if last MU is above the GenerateIfDistance distance from MU
        [ShowIf("AutomaticGeneration")]
        [Tooltip("Distance in mm from source to trigger new MU generation")]
        public float GenerateIfDistance = 300; //!< Distance in millimeters from Source when new MUs should be generated.
        [ShowIf("AutomaticGeneration")]
        [Tooltip("Add random variation to generation distance")]
        public bool RandomDistance = false; //!< If turned on Distance is Random Number with plus / minus Range Distance
        [ShowIf("RandomDistance")]
        [Tooltip("Random distance variation range in mm (+/- from base distance)")]
        public float RangeDistance = 300;  //!< Range of the distance in millimeters (plus / minus) if RandomDistance is turned on

        [Header("Number of MUs")]
        [Tooltip("Limit the maximum number of MUs that can be generated")]
        public bool LimitNumber = false;
        [ShowIf("LimitNumber")]
        [Tooltip("Maximum number of MUs to generate")]
        public int MaxNumberMUs = 1;
        [ShowIf("AutomaticGeneration")][ReadOnly] public int Created = 0;

        [Header("Source IO's")]
        [Tooltip("Toggle to generate a new MU (set to true to generate)")]
        public bool GenerateMU = true; //!< When changing from false to true a new MU is generated.
        [Tooltip("Toggle to delete all MUs generated by this source (set to true to delete)")]
        public bool DeleteAllMU; //!< When changing from false to true all MUs generated by this Source are deleted.

        [Header("Source Signals")]
        [Tooltip("PLC signal to trigger MU generation")]
        public PLCOutputBool SourceGenerate; //!< When changing from false to true a new MU is generated.
        [Tooltip("PLC signal to enable distance-based MU generation")]
        public PLCOutputBool SourceGenerateOnDistance; //!< When true MUs are generated on Distance

        [Header("Events")]
        public realvirtualEventMUCreated EventMUCreated; //!< Event triggered when a new MU is created

        [HideInInspector] public bool PositionOverwrite = false;

        // Private Variables
        private bool _generatebefore = false;
        private bool _deleteallmusbefore = false;
        private bool _tmpoccupied;
        private GameObject _lastgenerated;
        private int ID = 0;
        private bool _generatenotnull = false;
        private readonly List<GameObject> _generated = new List<GameObject>();
        private float nextdistance;
        private bool lastgenerateddeleted = false;
        private float xrscale = 1;
        private RigidbodyConstraints rbconstraints;
        private bool initautomaticgeneration;
        private bool signaldistancenotnull;
        private bool signalautomaticgenarationnotnull;

        //! Deletes all MU generated by this Source
        public void DeleteAll()
        {
            foreach (GameObject obj in _generated)
                Destroy(obj);
            _generated.Clear();
        }

        //! Is called when MU is deleted which has been created by this source
        public void OnMUDelete(MU mu)
        {
            if (mu.gameObject == _lastgenerated)
                lastgenerateddeleted = true;
        }

        //! Deletes all MU generated by this Source
        public void DeleteAllImmediate()
        {
            foreach (GameObject obj in _generated)
                DestroyImmediate(obj);
            _generated.Clear();
        }

        //! Event called on Init in XR Space.
        //! IMPLEMENTS IXRPlaceable::OnXRInit
        public void OnXRInit(GameObject placedobj)
        {
            xrscale = placedobj.transform.localScale.x;
        }

        //! Event when XR is Starting placing.
        //! IMPLEMENTS IXRPlaceable::OnXRStartPlace
        public void OnXRStartPlace(GameObject placedobj)
        {
            PositionOverwrite = true;
        }

        //! Event when XR is Ending placing.
        //! IMPLEMENTS IXRPlaceable::OnXREndPlace
        public void OnXREndPlace(GameObject placedobj)
        {
            PositionOverwrite = false;
            xrscale = placedobj.transform.localScale.x;
        }

        protected void Reset()
        {
            if (ThisObjectAsMU == null)
                ThisObjectAsMU = gameObject;
        }

        void GenerateInterval()
        {
            if (!PositionOverwrite)
                Generate();
        }

        new void Awake()
        {
            base.Awake();
            var rb = GetComponentInChildren<Rigidbody>();
            if (rb != null && rb.constraints != RigidbodyConstraints.FreezeAll)
                rbconstraints = rb.constraints;
            else
                rbconstraints = RigidbodyConstraints.None;
        }

        void OnEnable()
        {
            // Hook New Input actions
            if (DeleteAllAction != null)
            {
                DeleteAllAction.action.performed += OnDeleteAllPerformed;
                DeleteAllAction.action.Enable();
            }
            if (CreateOnSourceAction != null)
            {
                CreateOnSourceAction.action.performed += OnCreateOnSourcePerformed;
                CreateOnSourceAction.action.Enable();
            }
        }

        void OnDisable()
        {
            // Unhook New Input actions
            if (DeleteAllAction != null)
            {
                DeleteAllAction.action.performed -= OnDeleteAllPerformed;
                DeleteAllAction.action.Disable();
            }
            if (CreateOnSourceAction != null)
            {
                CreateOnSourceAction.action.performed -= OnCreateOnSourcePerformed;
                CreateOnSourceAction.action.Disable();
            }
        }

        protected void Start()
        {
            if (SourceGenerate != null)
            {
                _generatenotnull = true;
                AutomaticGeneration = false;
            }

            if (SourceGenerateOnDistance != null)
                signaldistancenotnull = true;

            if (SourceGenerateOnDistance != null)
                signalautomaticgenarationnotnull = true;

            if (ThisObjectAsMU == null)
                ErrorMessage("Object to be created needs to be defined in [This Object As MU]");

            if (ThisObjectAsMU != null && ThisObjectAsMU.GetComponent<MU>() == null)
                ThisObjectAsMU.AddComponent<MU>();

            if (Interval > 0)
                InvokeRepeating(nameof(GenerateInterval), StartInterval, Interval);

            // Don't show source and don't collide - source is just a blueprint for generating the MUs
            SetVisibility(!DontVisualize);
            SetCollider(false);
            SetFreezePosition(FreezeSourcePosition);
#if REALVIRTUAL_AGX
            if (UseAGXPhysics)
            {
                var rbodies = GetComponentsInChildren<RigidBody>();
                foreach (var rbody in rbodies)
                    rbody.enabled = false;
            }
#endif

            var col = GetComponent<Collider>();
            if (col != null) col.enabled = false;

            // Deactivate all fixers if included in Source
            var fixers = GetComponentsInChildren<IFix>();
            foreach (var fix in fixers) fix.DeActivate(true);

            nextdistance = GenerateIfDistance;
        }

        //! For Layout Editor mode Start is called when the simulation is started
        protected override void OnStartSim()
        {
            if (HideOnStop) SetVisibility(true);
            if (DontVisualize) SetVisibility(false);
        }

        //! For Layout Editor mode Stop is called when the simulation is stopped
        protected override void OnStopSim()
        {
            if (!HideOnStop) Invoke(nameof(DelayOnStop), 0.1f);
        }

        private void DelayOnStop() => SetVisibility(true);

        private void FixedUpdate()
        {
            if (signaldistancenotnull)
                if (AutomaticGeneration == false && SourceGenerateOnDistance.Value == true)
                    initautomaticgeneration = true;

            if (signalautomaticgenarationnotnull)
                AutomaticGeneration = SourceGenerateOnDistance.Value;

            // NOTE: Legacy key polls removed. Deletion/creation hotkeys now come via Input Actions callbacks.

            if (PositionOverwrite)
                return;

            if (_generatenotnull)
                GenerateMU = SourceGenerate.Value;

            // Generate on Signal Generate MU
            if (_generatebefore != GenerateMU)
            {
                if (GenerateMU)
                {
                    _generatebefore = GenerateMU;
                    Generate();
                }
            }

            // Handle automatic generation based on distance
            if (AutomaticGeneration)
            {
                if (_lastgenerated != null || lastgenerateddeleted || initautomaticgeneration)
                {
                    float distance = _lastgenerated != null
                        ? Vector3.Distance(_lastgenerated.transform.position, transform.position) * realvirtualController.Scale / xrscale
                        : 0;

                    bool create = distance > nextdistance || lastgenerateddeleted || (initautomaticgeneration && _lastgenerated == null);

                    if (create)
                    {
                        lastgenerateddeleted = false;
                        Generate();
                        nextdistance = RandomDistance
                            ? GenerateIfDistance + UnityEngine.Random.Range(-RangeDistance, RangeDistance)
                            : GenerateIfDistance;
                        initautomaticgeneration = false;
                    }
                }
            }

            if (!GenerateMU)
                _generatebefore = false;

            if (DeleteAllMU != _deleteallmusbefore && DeleteAllMU)
                DeleteAll();

            _deleteallmusbefore = DeleteAllMU;
        }

        // ===== New Input System callbacks =====
        private void OnDeleteAllPerformed(InputAction.CallbackContext ctx)
        {
            // Optional: gate behind UI toggle if you have one
            DeleteAll();
        }

        private void OnCreateOnSourcePerformed(InputAction.CallbackContext ctx)
        {
            if (!PositionOverwrite)
                Generate();
        }

        //! Generates an MU.
        public MU Generate()
        {
#if !REALVIRTUAL_AGX
            UseAGXPhysics = false;
#endif
            if (LimitNumber && (Created >= MaxNumberMUs))
                return null;

            if (!Enabled)
                return null;

            GameObject newmu = GameObject.Instantiate(ThisObjectAsMU, transform.position, transform.rotation);
            if (GenerateOnLayer != "")
                if (LayerMask.NameToLayer(GenerateOnLayer) != -1)
                    newmu.layer = LayerMask.NameToLayer(GenerateOnLayer);

            if (ChangeDefaultLayer)
            {
                var box = newmu.GetComponentInChildren<BoxCollider>();
                if (box != null)
                {
                    if (box.gameObject.layer == LayerMask.NameToLayer("Default"))
                        box.gameObject.layer = LayerMask.NameToLayer("rvMU");
                }

                var mesh = newmu.GetComponentInChildren<MeshCollider>();
                if (mesh != null)
                {
                    if (mesh.gameObject.layer == LayerMask.NameToLayer("Default"))
                        mesh.gameObject.layer = LayerMask.NameToLayer("rvMUTransport");
                }
            }

            Source source = newmu.GetComponent<Source>();

            Created++;
            if (!UseAGXPhysics)
            {
                Rigidbody newrigid = newmu.GetComponentInChildren<Rigidbody>();
                if (newrigid == null)
                    newrigid = newmu.AddComponent<Rigidbody>();

                newrigid.mass = Mass;

                Collider collider = newmu.GetComponentInChildren<Collider>();
                if (collider == null)
                {
                    var newboxcollider = newmu.AddComponent<BoxCollider>();
                    MeshFilter mumsmeshfilter = newmu.GetComponentInChildren<MeshFilter>();
                    Mesh mumesh = mumsmeshfilter != null ? mumsmeshfilter.mesh : null;
                    GameObject obj = mumsmeshfilter != null ? mumsmeshfilter.gameObject : null;
                    if (mumesh != null && obj != null)
                    {
                        Vector3 globalcenter = obj.transform.TransformPoint(mumesh.bounds.center);
                        Vector3 globalsize = obj.transform.TransformVector(mumesh.bounds.size);
                        newboxcollider.center = newmu.transform.InverseTransformPoint(globalcenter);
                        Vector3 size = newmu.transform.InverseTransformVector(globalsize);
                        if (size.x < 0) size.x = -size.x;
                        if (size.y < 0) size.y = -size.y;
                        if (size.z < 0) size.z = -size.z;
                        newboxcollider.size = size;
                    }
                }

                newrigid.mass = Mass;
                if (SetCenterOfMass)
                    newrigid.centerOfMass = CenterOfMass;
            }
            else
            {
#if REALVIRTUAL_AGX
                // Enable AGX Rigidbodies when newmu is created
                var rbodies = newmu.GetComponentsInChildren<RigidBody>();
                foreach (var rbody in rbodies)
                    rbody.enabled = true;
#endif
            }

            if (source != null)
            {
                source.SetVisibility(true);
                source.SetCollider(true);
                if (rbconstraints == RigidbodyConstraints.FreezeAll)
                    source.SetFreezePosition(false);
                else
                    source.SetRbConstraints(rbconstraints);

                source.Enabled = false;
                source.enabled = false;
            }

            ID++;
            MU mu = newmu.GetComponent<MU>();
            if (Destination != null)
                newmu.transform.parent = Destination.transform;

            newmu.transform.localScale = this.transform.localScale;

            if (mu == null)
            {
                ErrorMessage("Object generated by source need to have MU script attached!");
            }
            else
            {
                mu.InitMu(name, ID, realvirtualController.GetMUID(newmu));
            }

            mu.CreatedBy = this;

            DestroyImmediate(source);
            // Destroy Additional Components
            foreach (var componentname in OnCreateDestroyComponents)
            {
                Component[] components = newmu.GetComponents(typeof(Component));
                foreach (Component component in components)
                {
                    var ty = component.GetType();
                    if (ty.ToString() == componentname)
                        Destroy(component);
                }
            }

            // Activate all Fixers if included
            var fixers = mu.GetComponentsInChildren<IFix>();
            foreach (var fix in fixers) fix.DeActivate(false);

            _lastgenerated = newmu;
            _generated.Add(newmu);
            EventMUCreated.Invoke(mu);
            var isources = newmu.GetComponents<ISourceCreated>();
            foreach (var isource in isources) isource.OnSourceCreated();
            return mu;
        }
    }
}
