using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using SocketIO;

public class Socket : MonoBehaviour
{
    /*
    This script establishes a bi-directional connection between the simulator
    application and an external interface, now capable of receiving
    throttle/steering commands for ROS2 AutoDRIVE.
    */

    private SocketIOComponent socket; // Socket.IO instance
    public Button ConnectionButton; // GUI button
    public Text ConnectionLabel; // GUI button label

    public bool WeatherAPI = false;
    public WeatherManager[] Weather; // `WeatherManager` reference
    private int weather = 1;
    public bool TimeOfDayAPI = false;
    public TimeOfDay[] TimeOfDay; // `TimeOfDay` reference

    public CoSimManager[] CoSimManagers; // `CoSimManager` references
    public ResetManager[] ResetManagers; // `ResetManager` references
    public Rigidbody[] VehicleRigidBodies; // Vehicle rigid bodies
    public VehicleController[] VehicleControllers; // `VehicleController` references
    public AutomobileController[] AutomobileControllers; // `AutomobileController` references
    public TwistController[] TwistControllers; // `TwistController` references
    public VehicleLighting[] VehicleLightings; // `VehicleLighting` references
    public CarLighting[] CarLightings; // `CarLighting` references
    public ROVLighting[] ROVLightings; // `ROVLighting` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public GPS[] PositioningSystems; // `IPS` references
    public IMU[] InertialMeasurementUnits; // `IMU` references
    public LIDAR[] LIDARUnits; // `LIDAR` references
    public bool IntensityArray = true; // Choose to publish 2D LIDAR Intensity Array
    public LIDAR3D[] LIDAR3DUnits; // `LIDAR3D` references
    public Camera[] FrontCameras; // Vehicle front camera references
    public Camera[] RearCameras; // Vehicle rear camera references
    public bool SideCameras = false; // Rename front/rear camera frames as left/right
    public LapTimer[] LapTimers; // Lap timer references

    public TLController[] TrafficLightControllers; // Traffic light controller references

    private Vector3 CoSimPosition;
    private Quaternion CoSimRotation;

    // Add a variable to control emission frequency
    public float emitFrequencyHz = 30.0f; // Emit 30 times per second
    private float nextEmitTime = 0f;

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `Bridge` and corresponding event handler `OnBridge`
        // NEW: Event for receiving vehicle commands
        socket.On("VehicleControl", OnVehicleControl); // Listen for "VehicleControl" events
        socket.On("disconnect", OnDisconnect); // Declare disconnection event `disconnect` and corresponding event handler `OnDisconnect`

        // Temporary render textures for all vehicles except the first one (first vehicle will render to GUI)
        if (FrontCameras.Length != 0)
        {
            for (int i = 1; i < FrontCameras.Length; i++)
            {
                FrontCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        if (RearCameras.Length != 0)
        {
            for (int i = 1; i < RearCameras.Length; i++)
            {
                RearCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
    }

    // --- NEW: Add an Update method for continuous emission ---
    void Update()
    {
        // Debug.Log("Unity: Update loop running and checking for emit."); // Consider uncommenting for debugging
        if (socket.IsConnected && Time.time >= nextEmitTime)
        {
            // Debug.Log("Unity: Time to emit telemetry. Socket is connected."); // Consider uncommenting for debugging
            EmitTelemetry(); // Call without an event object
            nextEmitTime = Time.time + (1.0f / emitFrequencyHz);
        }
        else if (!socket.IsConnected)
        {
            // Debug.LogWarning("Unity: Socket not connected. Cannot emit telemetry."); // Consider uncommenting for debugging
        }
    }
    // --------------------------------------------------------

    void OnConnect(SocketIOEvent obj)
    {
        Debug.Log("Connected to Socket.IO server!");
        ConnectionButton.interactable = false;
        ConnectionLabel.text = "Connected";
    }

    void OnDisconnect(SocketIOEvent obj)
    {
        Debug.Log("Disconnected from Socket.IO server!");
        ConnectionButton.interactable = true;
        ConnectionLabel.text = "Disconnected";
    }

    void OnBridge(SocketIOEvent obj)
    {
        // Debug.Log("Bridge event received from Python."); // Consider uncommenting for debugging
        JSONObject jsonObject = obj.data; // Read incoming data and store it in a `JSONObject`

        // Set time of day
        if (TimeOfDayAPI && (TimeOfDay.Length != 0))
        {
            TimeOfDay[0].automaticUpdate = (jsonObject.GetField("Auto Time").str == "True"); // Set automatic update
            TimeOfDay[0].timeScale = float.Parse(jsonObject.GetField("Time Scale").str); // Set time scale
            TimeOfDay[0].timeOfDay = float.Parse(jsonObject.GetField("Time").str); // Set time of day
        }

        // Set weather
        if (WeatherAPI && (Weather.Length != 0))
        {
            weather = int.Parse(jsonObject.GetField("Weather").str); // Set weather
            if (weather == 0) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Custom;
            else if (weather == 1) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Sunny;
            else if (weather == 2) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Cloudy;
            else if (weather == 3) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightFog;
            else if (weather == 4) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyFog;
            else if (weather == 5) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightRain;
            else if (weather == 6) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyRain;
            else if (weather == 7) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightSnow;
            else if (weather == 8) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavySnow;
            Weather[0].CloudIntensity = float.Parse(jsonObject.GetField("Clouds").str); // Set cloud intensity
            Weather[0].FogIntensity = float.Parse(jsonObject.GetField("Fog").str); // Set fog intensity
            Weather[0].RainIntensity = float.Parse(jsonObject.GetField("Rain").str); // Set rain intensity
            Weather[0].SnowIntensity = float.Parse(jsonObject.GetField("Snow").str); // Set snow intensity
        }

        // Write data to traffic lights
        if (TrafficLightControllers.Length != 0)
        {
            for (int i = 0; i < TrafficLightControllers.Length; i++)
            {
                if (jsonObject.HasField("TL" + (i + 1).ToString() + " State"))
                {
                    TrafficLightControllers[i].CurrentState = int.Parse(jsonObject.GetField("TL" + (i + 1).ToString() + " State").str); // Set traffic light
                }
            }
        }

        // The logic for vehicle control (throttle/steering) is moved to OnVehicleControl
        // to separate concerns and make it clearer what's a command vs. other bridge data.
        // The original "Bridge" event can still update vehicle settings like reset/cosim/lights.
        // This ensures compatibility with the existing bridge messages.

        // Handle Reset Managers
        if (ResetManagers.Length != 0)
        {
            for (int i = 0; i < ResetManagers.Length; i++)
            {
                if (jsonObject.HasField("Reset"))
                {
                    ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str);
                }
            }
        }

        // Handle CoSim Managers
        if (CoSimManagers.Length != 0)
        {
            for (int i = 0; i < CoSimManagers.Length; i++)
            {
                string coSimField = "V" + (i + 1).ToString() + " CoSim";
                if (jsonObject.HasField(coSimField))
                {
                    if (int.Parse(jsonObject.GetField(coSimField).str) == 1)
                    {
                        VehicleRigidBodies[i].isKinematic = true;
                        CoSimPosition.x = -float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " PosY").str);
                        CoSimPosition.y = float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " PosZ").str);
                        CoSimPosition.z = float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " PosX").str);
                        CoSimRotation.x = float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " RotY").str);
                        CoSimRotation.y = -float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " RotZ").str);
                        CoSimRotation.z = -float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " RotX").str);
                        CoSimRotation.w = float.Parse(jsonObject.GetField("V" + (i + 1).ToString() + " RotW").str);
                        CoSimManagers[i].CoSimTimer = 0.0f;
                        CoSimManagers[i].CoSimPosition = CoSimPosition;
                        CoSimManagers[i].CoSimRotation = CoSimRotation;
                        CoSimManagers[i].enabled = true;
                    }
                    else
                    {
                        CoSimManagers[i].enabled = false;
                        VehicleRigidBodies[i].isKinematic = false;
                    }
                }
            }
        }

        // Handle Vehicle Lightings
        if (VehicleLightings.Length != 0)
        {
            for (int i = 0; i < VehicleLightings.Length; i++)
            {
                string headlightsField = "V" + (i + 1).ToString() + " Headlights";
                string indicatorsField = "V" + (i + 1).ToString() + " Indicators";
                if (jsonObject.HasField(headlightsField))
                {
                    VehicleLightings[i].Headlights = int.Parse(jsonObject.GetField(headlightsField).str);
                }
                if (jsonObject.HasField(indicatorsField))
                {
                    VehicleLightings[i].Indicators = int.Parse(jsonObject.GetField(indicatorsField).str);
                }
            }
        }

        // Handle Car Lightings
        if (CarLightings.Length != 0)
        {
            for (int i = 0; i < CarLightings.Length; i++)
            {
                string headlightsField = "V" + (i + 1).ToString() + " Headlights";
                string indicatorsField = "V" + (i + 1).ToString() + " Indicators";
                if (jsonObject.HasField(headlightsField))
                {
                    CarLightings[i].Headlights = int.Parse(jsonObject.GetField(headlightsField).str);
                }
                if (jsonObject.HasField(indicatorsField))
                {
                    CarLightings[i].Indicators = int.Parse(jsonObject.GetField(indicatorsField).str);
                }
            }
        }

        // Handle ROV Lightings
        if (ROVLightings.Length != 0)
        {
            for (int i = 0; i < ROVLightings.Length; i++)
            {
                string headlightsField = "V" + (i + 1).ToString() + " Headlights";
                if (jsonObject.HasField(headlightsField))
                {
                    ROVLightings[i].Headlights = int.Parse(jsonObject.GetField(headlightsField).str);
                }
            }
        }
    }

    // --- NEW: Event handler for receiving vehicle control commands ---
    void OnVehicleControl(SocketIOEvent obj)
    {
        // Debug.Log("VehicleControl event received from Python. Applying commands."); // Consider uncommenting for debugging
        JSONObject jsonObject = obj.data; // Read incoming data

        // Iterate through all possible vehicle controllers and apply commands
        if (VehicleControllers.Length != 0)
        {
            for (int i = 0; i < VehicleControllers.Length; i++)
            {
                // Only apply commands if the vehicle is in a controllable driving mode
                if (VehicleControllers[i].CurrentDrivingMode == 1) // Assuming 1 means manual/external control
                {
                    // Check for TwistController first (linear/angular velocity)
                    if (TwistControllers.Length > i && TwistControllers[i] != null)
                    {
                        string linearVelField = "V" + (i + 1).ToString() + " Linear Velocity";
                        string angularVelField = "V" + (i + 1).ToString() + " Angular Velocity";

                        if (jsonObject.HasField(linearVelField))
                        {
                            TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField(linearVelField).str);
                        }
                        if (jsonObject.HasField(angularVelField))
                        {
                            TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField(angularVelField).str);
                        }
                    }
                    // If no TwistController or not in twist mode, apply throttle/steering directly
                    else
                    {
                        string throttleField = "V" + (i + 1).ToString() + " Throttle";
                        string steeringField = "V" + (i + 1).ToString() + " Steering";

                        if (jsonObject.HasField(throttleField))
                        {
                            VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField(throttleField).str);
                        }
                        if (jsonObject.HasField(steeringField))
                        {
                            VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField(steeringField).str);
                        }
                    }
                }
            }
        }

        if (AutomobileControllers.Length != 0)
        {
            for (int i = 0; i < AutomobileControllers.Length; i++)
            {
                // Only apply commands if the vehicle is in a controllable driving mode
                if (AutomobileControllers[i].CurrentDrivingMode == 1) // Assuming 1 means manual/external control
                {
                    // Check for TwistController first (linear/angular velocity)
                    if (TwistControllers.Length > i && TwistControllers[i] != null)
                    {
                        string linearVelField = "V" + (i + 1).ToString() + " Linear Velocity";
                        string angularVelField = "V" + (i + 1).ToString() + " Angular Velocity";

                        if (jsonObject.HasField(linearVelField))
                        {
                            TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField(linearVelField).str);
                        }
                        if (jsonObject.HasField(angularVelField))
                        {
                            TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField(angularVelField).str);
                        }
                    }
                    // If no TwistController or not in twist mode, apply throttle/steering/brake directly
                    else
                    {
                        string throttleField = "V" + (i + 1).ToString() + " Throttle";
                        string steeringField = "V" + (i + 1).ToString() + " Steering";
                        string brakeField = "V" + (i + 1).ToString() + " Brake";
                        string handbrakeField = "V" + (i + 1).ToString() + " Handbrake";

                        if (jsonObject.HasField(throttleField))
                        {
                            AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField(throttleField).str);
                        }
                        if (jsonObject.HasField(steeringField))
                        {
                            AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField(steeringField).str);
                        }
                        if (jsonObject.HasField(brakeField))
                        {
                            AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField(brakeField).str);
                        }
                        if (jsonObject.HasField(handbrakeField))
                        {
                            AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField(handbrakeField).str);
                        }
                    }
                }
            }
        }
    }
    // -------------------------------------------------------------------

    void EmitTelemetry() // Modified signature: no obj parameter
    {
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            // Debug.Log("Attempting to write data..."); // Consider uncommenting for debugging
            Dictionary<string, string> data = new Dictionary<string, string>(); // Create new `data` dictionary

            // Read data from traffic lights
            if (TrafficLightControllers.Length != 0)
            {
                for (int i = 0; i < TrafficLightControllers.Length; i++)
                {
                    data["TL" + (i + 1).ToString() + " State"] = TrafficLightControllers[i].CurrentState.ToString(); // Get status
                }
            }
            // Read data from vehicles
            if (VehicleControllers.Length != 0 || AutomobileControllers.Length != 0)
            {
                for (int i = 0; i < VehicleControllers.Length; i++)
                {
                    data["V" + (i + 1).ToString() + " Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V" + (i + 1).ToString() + " Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V" + (i + 1).ToString() + " Speed"] = System.Math.Abs(System.Math.Round(VehicleControllers[i].Vehicle.transform.InverseTransformDirection(VehicleControllers[i].Vehicle.GetComponent<Rigidbody>().velocity).z, 4)).ToString("F4"); // Get speed
                }
                for (int i = 0; i < AutomobileControllers.Length; i++)
                {
                    data["V" + (i + 1).ToString() + " Throttle"] = AutomobileControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V" + (i + 1).ToString() + " Steering"] = AutomobileControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V" + (i + 1).ToString() + " Brake"] = AutomobileControllers[i].CurrentBrake.ToString("F4"); // Get brake
                    data["V" + (i + 1).ToString() + " Handbrake"] = AutomobileControllers[i].CurrentHandbrake.ToString("F4"); // Get handbrake
                    data["V" + (i + 1).ToString() + " Collisions"] = AutomobileControllers[i].collisionCount.ToString(); // Get collision count
                    data["V" + (i + 1).ToString() + " Speed"] = System.Math.Abs(System.Math.Round(AutomobileControllers[i].currSpeed / AutomobileControllers[i].speedMultiplier, 4)).ToString("F4"); // Get speed
                }
                for (int i = 0; i < VehicleControllers.Length + AutomobileControllers.Length; i++) // Assumed that VehicleControllers.Length+AutomobileControllers.Length >= others
                {
                    if (LeftWheelEncoders.Length > i && RightWheelEncoders.Length > i)
                    {
                        data["V" + (i + 1).ToString() + " Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                        data["V" + (i + 1).ToString() + " Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F4") + " " + RightWheelEncoders[i].Angle.ToString("F4"); // Get encoder angles
                    }
                    if (PositioningSystems.Length > i)
                    {
                        data["V" + (i + 1).ToString() + " Position"] = PositioningSystems[i].CurrentPosition[0].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[1].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[2].ToString("F4"); // Get vehicle position
                    }
                    if (InertialMeasurementUnits.Length > i)
                    {
                        data["V" + (i + 1).ToString() + " Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F4"); // Get vehicle orientation (Quaternion)
                        data["V" + (i + 1).ToString() + " Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F4"); // Get vehicle orientation (Euler Angles)
                        data["V" + (i + 1).ToString() + " Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F4"); // Get angular velocity of the vehicle
                        data["V" + (i + 1).ToString() + " Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F4"); // Get linear acceleration of the vehicle
                    }
                    if (LIDARUnits.Length > i)
                    {
                        data["V" + (i + 1).ToString() + " LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F4"); // Get LIDAR scan rate
                        if (LIDARUnits[i].CurrentRangeArray != null && LIDARUnits[i].CurrentRangeArray.Length > 0)
                        {
                            // Debug.Log($"[EmitTelemetry] Sending {LIDARUnits[i].CurrentRangeArray.Length} LiDAR points for V{i + 1}"); // Consider uncommenting for debugging
                            data["V" + (i + 1).ToString() + " LIDAR Range Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentRangeArray); // Get LIDAR range array
                            if (IntensityArray) data["V" + (i + 1).ToString() + " LIDAR Intensity Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentIntensityArray); // Get LIDAR intensity array
                        }
                    }
                    if (LIDAR3DUnits.Length > i) data["V" + (i + 1).ToString() + " LIDAR Pointcloud"] = Convert.ToBase64String(LIDAR3DUnits[i].CurrentPointcloud); // Get LIDAR pointcloud
                    if (FrontCameras.Length > i)
                    {
                        if (SideCameras) data["V" + (i + 1).ToString() + " Left Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get left camera image
                        else data["V" + (i + 1).ToString() + " Front Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get front camera image
                    }
                    if (RearCameras.Length > i)
                    {
                        if (SideCameras) data["V" + (i + 1).ToString() + " Right Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get right camera image
                        else data["V" + (i + 1).ToString() + " Rear Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get rear camera image
                    }
                }
            }
            if (LapTimers.Length != 0)
            {
                for (int i = 0; i < LapTimers.Length; i++)
                {
                    data["V" + (i + 1).ToString() + " Lap Count"] = LapTimers[i].LapCount.ToString(); // Get lap count
                    data["V" + (i + 1).ToString() + " Lap Time"] = LapTimers[i].LapTime.ToString("F4"); // Get lap time
                    data["V" + (i + 1).ToString() + " Last Lap Time"] = LapTimers[i].LastLapTime.ToString("F4"); // Get last lap count
                    data["V" + (i + 1).ToString() + " Best Lap Time"] = LapTimers[i].BestLapTime.ToString("F4"); // Get best lap time
                    data["V" + (i + 1).ToString() + " Collisions"] = LapTimers[i].CollisionCount.ToString(); // Get collision count
                }
            }
            if (socket.IsConnected) // Only emit if the socket is connected
            {
                // Debug.Log("Unity: Actually emitting 'Bridge' event."); // Consider uncommenting for debugging
                socket.Emit("Bridge", new JSONObject(data)); // Emit telemetry data
            }
            else
            {
                Debug.LogWarning("Socket not connected, cannot emit telemetry."); // Log if not connected
            }
        });
    }
}
