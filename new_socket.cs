using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using SocketIO;

public class SocketIOBridge : MonoBehaviour
{
    [Header("Connection UI (optional)")]
    public Button ConnectionButton;
    public Text   ConnectionLabel;

    [Header("Telemetry Settings")]
    [Tooltip("How many times per second to send telemetry to the server.")]
    public float telemetryHz = 30f;

    [Header("Simulation Settings")]
    public bool TimeOfDayAPI = false;
    public TimeOfDay[] TimeOfDay;
    public bool WeatherAPI = false;
    public WeatherManager[] Weather;

    [Header("Vehicle Interfaces")]
    public CoSimManager[] CoSimManagers;
    public ResetManager[] ResetManagers;
    public Rigidbody[] VehicleRigidBodies;
    public VehicleController[] VehicleControllers;
    public AutomobileController[] AutomobileControllers;
    public TwistController[] TwistControllers;
    public VehicleLighting[] VehicleLightings;
    public CarLighting[] CarLightings;
    public ROVLighting[] ROVLightings;
    public WheelEncoder[] LeftWheelEncoders;
    public WheelEncoder[] RightWheelEncoders;
    public GPS[] PositioningSystems;
    public IMU[] InertialMeasurementUnits;
    public LIDAR[] LIDARUnits;
    public bool sendIntensity = true;
    public LIDAR3D[] LIDAR3DUnits;
    public Camera[] FrontCameras;
    public Camera[] RearCameras;
    public bool SideCameras = false;
    public LapTimer[] LapTimers;
    public TLController[] TrafficLightControllers;

    private SocketIOComponent socket;

    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>();
        if (socket == null)
        {
            Debug.LogError("SocketIOComponent not found on 'SocketIO' GameObject!");
            enabled = false;
            return;
        }

        socket.On("connect",    OnConnected);
        socket.On("disconnect", OnDisconnected);
        socket.On("control",    OnControl);
    }

    void OnConnected(SocketIOEvent e)
    {
        Debug.Log("Socket.IO connected");
        if (ConnectionButton) ConnectionButton.interactable = false;
        if (ConnectionLabel)  ConnectionLabel.text = "Connected";
        StartCoroutine(TelemetryRoutine());
    }

    void OnDisconnected(SocketIOEvent e)
    {
        Debug.Log("Socket.IO disconnected");
        if (ConnectionButton) ConnectionButton.interactable = true;
        if (ConnectionLabel)  ConnectionLabel.text = "Disconnected";
        StopAllCoroutines();
    }

    IEnumerator TelemetryRoutine()
    {
        var wait = new WaitForSeconds(1f / telemetryHz);
        while (socket != null && socket.IsConnected)
        {
            SendTelemetry();
            yield return wait;
        }
    }

    void SendTelemetry()
    {
        var data = new Dictionary<string, string>();
        int total = Mathf.Max(
            VehicleControllers?.Length ?? 0,
            AutomobileControllers?.Length ?? 0
        );

        for (int i = 0; i < total; i++)
        {
            string prefix = $"V{i+1} ";
            // VehicleController or AutomobileController
            if (VehicleControllers != null && i < VehicleControllers.Length)
            {
                var vc = VehicleControllers[i];
                data[prefix + "Throttle"] = vc.CurrentThrottle.ToString("F4");
                data[prefix + "Steering"] = vc.CurrentSteeringAngle.ToString("F4");
            }
            else if (AutomobileControllers != null && i < AutomobileControllers.Length)
            {
                var ac = AutomobileControllers[i];
                data[prefix + "Throttle"] = ac.CurrentThrottle.ToString("F4");
                data[prefix + "Steering"] = ac.CurrentSteeringAngle.ToString("F4");
                data[prefix + "Brake"]    = ac.CurrentBrake.ToString("F4");
                data[prefix + "Handbrake"]= ac.CurrentHandbrake.ToString("F4");
                data[prefix + "Collisions"] = ac.collisionCount.ToString();
            }
            // Rigidbody speed
            if (VehicleRigidBodies != null && i < VehicleRigidBodies.Length)
            {
                float spd = VehicleRigidBodies[i]
                    .transform.InverseTransformDirection(
                        VehicleRigidBodies[i].velocity
                    ).z;
                data[prefix + "Speed"] = Mathf.Abs(spd).ToString("F4");
            }
            // Encoders
            if (LeftWheelEncoders != null && i < LeftWheelEncoders.Length &&
                RightWheelEncoders != null && i < RightWheelEncoders.Length)
            {
                var le = LeftWheelEncoders[i];
                var re = RightWheelEncoders[i];
                data[prefix + "EncoderTicks"] = $"{le.Ticks} {re.Ticks}";
                data[prefix + "EncoderAngles"] = $"{le.Angle:F4} {re.Angle:F4}";
            }
            // GPS
            if (PositioningSystems != null && i < PositioningSystems.Length)
            {
                var pos = PositioningSystems[i].CurrentPosition;
                data[prefix + "Position"] =
                    $"{pos[0]:F4} {pos[1]:F4} {pos[2]:F4}";
            }
            // IMU
            if (InertialMeasurementUnits != null && i < InertialMeasurementUnits.Length)
            {
                var imu = InertialMeasurementUnits[i];
                var q = imu.CurrentOrientationQuaternion;
                var eul = imu.CurrentOrientationEulerAngles;
                var w = imu.CurrentAngularVelocity;
                var a = imu.CurrentLinearAcceleration;
                data[prefix + "OrientationQuat"] =
                    $"{q[0]:F4} {q[1]:F4} {q[2]:F4} {q[3]:F4}";
                data[prefix + "OrientationEuler"] =
                    $"{eul[0]:F4} {eul[1]:F4} {eul[2]:F4}";
                data[prefix + "AngVel"] =
                    $"{w[0]:F4} {w[1]:F4} {w[2]:F4}";
                data[prefix + "LinAccel"] =
                    $"{a[0]:F4} {a[1]:F4} {a[2]:F4}";
            }
            // LIDAR
            if (LIDARUnits != null && i < LIDARUnits.Length)
            {
                var lidar = LIDARUnits[i];
                data[prefix + "LidarRate"] = lidar.CurrentScanRate.ToString("F4");
                if (lidar.CurrentRangeArray != null && lidar.CurrentRangeArray.Length > 0)
                {
                    data[prefix + "LidarRanges"] =
                        DataCompressor.CompressArray(lidar.CurrentRangeArray);
                    if (sendIntensity)
                        data[prefix + "LidarIntensities"] =
                            DataCompressor.CompressArray(
                                lidar.CurrentIntensityArray
                            );
                }
            }
        }

        socket.Emit("telemetry", new JSONObject(data));
    }

    void OnControl(SocketIOEvent e)
    {
        var jsonObject = e.data;
        if (jsonObject == null || jsonObject.Count == 0)
        {
            Debug.LogWarning("[Control] Empty payload");
            return;
        }

        // TimeOfDay
        if (TimeOfDayAPI && TimeOfDay != null && TimeOfDay.Length > 0)
        {
            try {
                var t = TimeOfDay[0];
                t.automaticUpdate = (jsonObject.GetField("Auto Time")?.str == "True");
                t.timeScale = float.Parse(jsonObject.GetField("Time Scale")?.str ?? "1.0");
                t.timeOfDay  = float.Parse(jsonObject.GetField("Time")?.str ?? "12.0");
            } catch (Exception ex) {
                Debug.LogError($"TimeOfDay parse error: {ex.Message}");
            }
        }
        // Weather
        if (WeatherAPI && Weather != null && Weather.Length > 0)
        {
            try {
                if (jsonObject.HasField("Weather"))
                {
                    int w = int.Parse(jsonObject.GetField("Weather").str);
                    var mgr = Weather[0];
                    mgr.weatherPreset = (WeatherManager.WeatherPreset)w;
                }
                var wm = Weather[0];
                wm.CloudIntensity = float.Parse(jsonObject.GetField("Clouds")?.str ?? "0.0");
                wm.FogIntensity   = float.Parse(jsonObject.GetField("Fog")?.str ?? "0.0");
                wm.RainIntensity  = float.Parse(jsonObject.GetField("Rain")?.str ?? "0.0");
                wm.SnowIntensity  = float.Parse(jsonObject.GetField("Snow")?.str ?? "0.0");
            } catch (Exception ex) {
                Debug.LogError($"Weather parse error: {ex.Message}");
            }
        }

        // VehicleControllers
        if (VehicleControllers != null && VehicleControllers.Length > 0)
        {
            for (int i = 0; i < VehicleControllers.Length; i++)
            {
                var vc = VehicleControllers[i];
                if (vc == null) continue;

                // Reset
                if (ResetManagers != null && i < ResetManagers.Length && ResetManagers[i] != null &&
                    jsonObject.HasField("Reset"))
                {
                    ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str);
                }
                // CoSim vs live
                bool useCoSim = jsonObject.HasField($"V{i+1} CoSim") &&
                                int.Parse(jsonObject.GetField($"V{i+1} CoSim").str) == 1;

                if (CoSimManagers != null && i < CoSimManagers.Length && CoSimManagers[i] != null &&
                    VehicleRigidBodies != null && i < VehicleRigidBodies.Length && VehicleRigidBodies[i] != null)
                {
                    if (useCoSim)
                    {
                        VehicleRigidBodies[i].isKinematic = true;
                        Vector3 pos = new Vector3(
                            -float.Parse(jsonObject.GetField($"V{i+1} PosY").str),
                             float.Parse(jsonObject.GetField($"V{i+1} PosZ").str),
                             float.Parse(jsonObject.GetField($"V{i+1} PosX").str)
                        );
                        Quaternion rot = new Quaternion(
                            float.Parse(jsonObject.GetField($"V{i+1} RotY").str),
                           -float.Parse(jsonObject.GetField($"V{i+1} RotZ").str),
                           -float.Parse(jsonObject.GetField($"V{i+1} RotX").str),
                            float.Parse(jsonObject.GetField($"V{i+1} RotW").str)
                        );
                        var cm = CoSimManagers[i];
                        cm.CoSimPosition = pos;
                        cm.CoSimRotation = rot;
                        cm.CoSimTimer    = 0f;
                        cm.enabled       = true;
                        continue;
                    }
                    else
                    {
                        CoSimManagers[i].enabled = false;
                        VehicleRigidBodies[i].isKinematic = false;
                        ApplyDriveCommands(jsonObject, i);
                    }
                }
                else
                {
                    if (VehicleRigidBodies != null && i < VehicleRigidBodies.Length)
                        VehicleRigidBodies[i].isKinematic = false;
                    ApplyDriveCommands(jsonObject, i);
                }

                // Vehicle lights
                if (VehicleLightings != null && i < VehicleLightings.Length && VehicleLightings[i] != null)
                {
                    if (jsonObject.HasField($"V{i+1} Headlights"))
                        VehicleLightings[i].Headlights =
                            int.Parse(jsonObject.GetField($"V{i+1} Headlights").str);
                    if (jsonObject.HasField($"V{i+1} Indicators"))
                        VehicleLightings[i].Indicators =
                            int.Parse(jsonObject.GetField($"V{i+1} Indicators").str);
                }
            }
        }

        // AutomobileControllers
        if (AutomobileControllers != null && AutomobileControllers.Length > 0)
        {
            for (int i = 0; i < AutomobileControllers.Length; i++)
            {
                var ac = AutomobileControllers[i];
                if (ac == null) continue;

                if (ResetManagers != null && i < ResetManagers.Length && ResetManagers[i] != null && jsonObject.HasField("Reset"))
                {
                    ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str);
                }

                bool useCoSim = jsonObject.HasField($"V{i+1} CoSim") &&
                                int.Parse(jsonObject.GetField($"V{i+1} CoSim").str) == 1;
                if (CoSimManagers != null && i < CoSimManagers.Length && CoSimManagers[i] != null &&
                    VehicleRigidBodies != null && i < VehicleRigidBodies.Length && VehicleRigidBodies[i] != null)
                {
                    if (useCoSim)
                    {
                        VehicleRigidBodies[i].isKinematic = true;
                        var pos = new Vector3(
                            -float.Parse(jsonObject.GetField($"V{i+1} PosY").str),
                             float.Parse(jsonObject.GetField($"V{i+1} PosZ").str),
                             float.Float.Parse(jsonObject.GetField($"V{i+1} PosX").str)
                        );
                        var rot = new Quaternion(
                            float.Parse(jsonObject.GetField($"V{i+1} RotY").str),
                           -float.Parse(jsonObject.GetField($"V{i+1} RotZ").str),
                           -float.Parse(jsonObject.GetField($"V{i+1} RotX").str),
                            float.Parse(jsonObject.GetField($"V{i+1} RotW").str)
                        );
                        var cm = CoSimManagers[i];
                        cm.CoSimPosition = pos;
                        cm.CoSimRotation = rot;
                        cm.CoSimTimer    = 0f;
                        cm.enabled       = true;
                        continue;
                    }
                    else
                    {
                        CoSimManagers[i].enabled = false;
                        VehicleRigidBodies[i].isKinematic = false;
                        ApplyAutomobileCommands(jsonObject, i);
                    }
                }
                else
                {
                    if (VehicleRigidBodies != null && i < VehicleRigidBodies.Length)
                        VehicleRigidBodies[i].isKinematic = false;
                    ApplyAutomobileCommands(jsonObject, i);
                }

                // Car lighting
                if (CarLightings != null && i < CarLightings.Length && CarLightings[i] != null)
                {
                    if (jsonObject.HasField($"V{i+1} Headlights"))
                        CarLightings[i].Headlights =
                            int.Parse(jsonObject.GetField($"V{i+1} Headlights").str);
                    if (jsonObject.HasField($"V{i+1} Indicators"))
                        CarLightings[i].Indicators =
                            int.Parse(jsonObject.GetField($"V{i+1} Indicators").str);
                }
                // ROV lighting
                if (ROVLightings != null && i < ROVLightings.Length && ROVLightings[i] != null)
                {
                    if (jsonObject.HasField($"V{i+1} Headlights"))
                        ROVLightings[i].Headlights =
                            int.Parse(jsonObject.GetField($"V{i+1} Headlights").str);
                }
            }
        }

        // Traffic lights
        if (TrafficLightControllers != null && TrafficLightControllers.Length > 0)
        {
            for (int i = 0; i < TrafficLightControllers.Length; i++)
            {
                if (jsonObject.HasField($"TL{i+1} State"))
                    TrafficLightControllers[i].CurrentState =
                        int.Parse(jsonObject.GetField($"TL{i+1} State").str);
            }
        }
    }

    private void ApplyDriveCommands(JSONObject json, int i)
    {
        if (TwistControllers != null && i < TwistControllers.Length && TwistControllers[i] != null)
        {
            var tc = TwistControllers[i];
            if (json.HasField($"V{i+1} Linear Velocity"))
                tc.vSetpoint = float.Parse(json.GetField($"V{i+1} Linear Velocity").str);
            if (json.HasField($"V{i+1} Angular Velocity"))
                tc.wSetpoint = float.Parse(json.GetField($"V{i+1} Angular Velocity").str);
        }
        else if (VehicleControllers != null && i < VehicleControllers.Length && VehicleControllers[i] != null)
        {
            var vc = VehicleControllers[i];
            if (json.HasField($"V{i+1} Throttle"))
                vc.CurrentThrottle = float.Parse(json.GetField($"V{i+1} Throttle").str);
            if (json.HasField($"V{i+1} Steering"))
                vc.CurrentSteeringAngle = float.Parse(json.GetField($"V{i+1} Steering").str);
        }
    }

    private void ApplyAutomobileCommands(JSONObject json, int i)
    {
        if (TwistControllers != null && i < TwistControllers.Length && TwistControllers[i] != null)
        {
            var tc = TwistControllers[i];
            if (json.HasField($"V{i+1} Linear Velocity"))
                tc.vSetpoint = float.Parse(json.GetField($"V{i+1} Linear Velocity").str);
            if (json.HasField($"V{i+1} Angular Velocity"))
                tc.wSetpoint = float.Parse(json.GetField($"V{i+1} Angular Velocity").str);
        }
        else if (AutomobileControllers != null && i < AutomobileControllers.Length && AutomobileControllers[i] != null)
        {
            var ac = AutomobileControllers[i];
            if (json.HasField($"V{i+1} Throttle"))
                ac.CurrentThrottle = float.Parse(json.GetField($"V{i+1} Throttle").str);
            if (json.HasField($"V{i+1} Steering"))
                ac.CurrentSteeringAngle = float.Parse(json.GetField($"V{i+1} Steering").str);
            if (json.HasField($"V{i+1} Brake"))
                ac.CurrentBrake = float.Parse(json.GetField($"V{i+1} Brake").str);
            if (json.HasField($"V{i+1} Handbrake"))
                ac.CurrentHandbrake = float.Parse(json.GetField($"V{i+1} Handbrake").str);
        }
    }
}
