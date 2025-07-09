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
    application and an external interface.
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
    public float emitFrequencyHz = 5.0f; // Emit 30 times per second
    private float nextEmitTime = 0f;

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `Bridge` and corresponding event handler `OnBridge`
        socket.On("VehicleControl", OnVehicleControl); // NEW: Declare event `VehicleControl` for incoming commands
        socket.On("disconnect", OnDisconnect); // Declare disconnection event `disconnect` and corresponding event handler `OnDisconnect`
        // Temporary render textures for all vehicles except the first one (first vehicle will render to GUI)
        if(FrontCameras.Length != 0)
        {
            for(int i=1;i<FrontCameras.Length;i++)
            {
                FrontCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        if(RearCameras.Length != 0)
        {
            for(int i=1;i<RearCameras.Length;i++)
            {
                RearCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
    }

    // --- NEW: Add an Update method for continuous emission ---
    void Update()
    {
        // Debug.Log("Unity: Update loop running and checking for emit."); // Suppressed for cleaner output
        if (socket.IsConnected && Time.time >= nextEmitTime)
        {
            // Debug.Log("Unity: Time to emit telemetry. Socket is connected."); // Suppressed for cleaner output
            EmitTelemetry(null);
            nextEmitTime = Time.time + (1.0f / emitFrequencyHz);
        }
        else if (!socket.IsConnected)
        {
            // Debug.LogWarning("Unity: Socket not connected. Cannot emit telemetry."); // Suppressed for cleaner output
        }
    }
    // --------------------------------------------------------

    void OnConnect(SocketIOEvent obj)
    {
        //Debug.Log("Connected!");
        ConnectionButton.interactable = false;
        ConnectionLabel.text = "Connected";
        //EmitTelemetry(obj); // Keep this if you want an immediate send on connect
    }

    void OnDisconnect(SocketIOEvent obj)
    {
        //Debug.Log("Disconnected!");
        ConnectionButton.interactable = true;
        ConnectionLabel.text = "Disconnected";
        //EmitTelemetry(obj); // Keep this if you want an immediate send on disconnect
    }

    // This method handles incoming *sensor/telemetry* data requests from Python (currently unused by Python)
    // The Python bridge primarily acts as a sender for sensor data and receiver for control commands.
    void OnBridge(SocketIOEvent obj)
    {
        //Debug.Log("Bridge event received from Python. This event is typically used for initial setup or generic data flow, not control commands.");
        JSONObject jsonObject = obj.data; // Read incoming data and store it in a `JSONObject`

        // This section remains for any generic 'Bridge' events that might carry
        // environment data or other non-control parameters in the future.
        // It's currently not used by your Python 'autodrive_incoming_bridge_2.py'
        // for control commands, but keeping it ensures backward compatibility if
        // the 'Bridge' event starts carrying other types of data.

        // Set time of day
        if(TimeOfDayAPI && (TimeOfDay.Length !=0))
        {
            TimeOfDay[0].automaticUpdate = (jsonObject.GetField("Auto Time").str == "True"); // Set automatic update
            TimeOfDay[0].timeScale = float.Parse(jsonObject.GetField("Time Scale").str); // Set time scale
            TimeOfDay[0].timeOfDay = float.Parse(jsonObject.GetField("Time").str); // Set time of day
        }

        // Set weather
        if(WeatherAPI && (Weather.Length !=0))
        {
            weather = int.Parse(jsonObject.GetField("Weather").str); // Set weather
            if(weather == 0) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Custom;
            else if(weather == 1) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Sunny;
            else if(weather == 2) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Cloudy;
            else if(weather == 3) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightFog;
            else if(weather == 4) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyFog;
            else if(weather == 5) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightRain;
            else if(weather == 6) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyRain;
            else if(weather == 7) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightSnow;
            else if(weather == 8) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavySnow;
            Weather[0].CloudIntensity = float.Parse(jsonObject.GetField("Clouds").str); // Set cloud intensity
            Weather[0].FogIntensity = float.Parse(jsonObject.GetField("Fog").str); // Set fog intensity
            Weather[0].RainIntensity = float.Parse(jsonObject.GetField("Rain").str); // Set rain intensity
            Weather[0].SnowIntensity = float.Parse(jsonObject.GetField("Snow").str); // Set snow intensity
        }

        // Write data to traffic lights (if any are sent via 'Bridge' event)
        if(TrafficLightControllers.Length != 0)
        {
            for(int i=0;i<TrafficLightControllers.Length;i++)
            {
                if (jsonObject.HasField("TL"+(i+1).ToString()+" State")) // Check if field exists before parsing
                {
                    TrafficLightControllers[i].CurrentState = int.Parse(jsonObject.GetField("TL"+(i+1).ToString()+" State").str); // Set traffic light
                }
            }
        }

        // NO VEHICLE CONTROL LOGIC HERE ANYMORE. It's moved to OnVehicleControl.
    }


    // --- NEW: Separate event handler for VehicleControl commands ---
    void OnVehicleControl(SocketIOEvent obj)
    {
        //Debug.Log("VehicleControl event received from Python. Processing incoming commands.");
        JSONObject jsonObject = obj.data; // Read incoming data and store it in a `JSONObject`
        // Debug.Log("Incoming VehicleControl data: " + obj.data.ToString()); // For detailed debugging

        // Write control data to vehicles (Throttle, Steering)
        // This logic was previously in OnBridge, now it's correctly placed here.
        if(VehicleControllers.Length != 0)
        {
            for(int i=0;i<VehicleControllers.Length;i++)
            {
                // Ensure the JSON object has the expected fields before parsing
                if (jsonObject.HasField("V"+(i+1).ToString()+" Throttle"))
                {
                    VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                    // Debug.Log($"V{i+1} Throttle set to: {VehicleControllers[i].CurrentThrottle}"); // Debug
                }
                if (jsonObject.HasField("V"+(i+1).ToString()+" Steering"))
                {
                    VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                    // Debug.Log($"V{i+1} Steering set to: {VehicleControllers[i].CurrentSteeringAngle}"); // Debug
                }

                // Handle Reset and CoSim
                if (ResetManagers.Length > i) // Check array bounds
                {
                    if (jsonObject.HasField("Reset"))
                    {
                        ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                }
                if (CoSimManagers.Length > i) // Check array bounds
                {
                    if(jsonObject.HasField("V"+(i+1).ToString()+" CoSim") && int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                    {
                        if (VehicleRigidBodies.Length > i) VehicleRigidBodies[i].isKinematic = true;
                        CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY").str); // Set position X-component
                        CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ").str); // Set position Y-component
                        CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX").str); // Set position Z-component
                        CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY").str); // Set rotation X-component
                        CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ").str); // Set rotation Y-component
                        CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX").str); // Set rotation Z-component
                        CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW").str); // Set rotation W-component
                        CoSimManagers[i].CoSimTimer = 0.0f;
                        CoSimManagers[i].CoSimPosition = CoSimPosition;
                        CoSimManagers[i].CoSimRotation = CoSimRotation;
                        CoSimManagers[i].enabled = true;
                    }
                    else
                    {
                        CoSimManagers[i].enabled = false;
                        if (VehicleRigidBodies.Length > i) VehicleRigidBodies[i].isKinematic = false;

                        // Only apply TwistController if it's enabled and fields exist
                        if (TwistControllers.Length > i && TwistControllers[i] != null)
                        {
                            if (jsonObject.HasField("V"+(i+1).ToString()+" Linear Velocity"))
                            {
                                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                            }
                            if (jsonObject.HasField("V"+(i+1).ToString()+" Angular Velocity"))
                            {
                                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                            }
                        }
                    }
                }

                // Handle VehicleLighting (if configured and fields exist)
                if(VehicleLightings.Length > i)
                {
                    if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                    {
                        VehicleLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                    }
                    if (jsonObject.HasField("V"+(i+1).ToString()+" Indicators"))
                    {
                        VehicleLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                }
            }
        }

        // Similar logic for AutomobileControllers if you use them for F1TENTH
        if(AutomobileControllers.Length != 0)
        {
            for(int i=0;i<AutomobileControllers.Length;i++)
            {
                // Ensure the JSON object has the expected fields before parsing
                if (jsonObject.HasField("V"+(i+1).ToString()+" Throttle"))
                {
                    AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                }
                if (jsonObject.HasField("V"+(i+1).ToString()+" Steering"))
                {
                    AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                }
                if (jsonObject.HasField("V"+(i+1).ToString()+" Brake"))
                {
                    AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str); // Set brake
                }
                if (jsonObject.HasField("V"+(i+1).ToString()+" Handbrake"))
                {
                    AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str); // Set handbrake
                }

                // Handle Reset and CoSim
                if (ResetManagers.Length > i) // Check array bounds
                {
                    if (jsonObject.HasField("Reset"))
                    {
                        ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                }
                if (CoSimManagers.Length > i) // Check array bounds
                {
                    if(jsonObject.HasField("V"+(i+1).ToString()+" CoSim") && int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                    {
                        if (VehicleRigidBodies.Length > i) VehicleRigidBodies[i].isKinematic = true;
                        CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY").str); // Set position X-component
                        CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ").str); // Set position Y-component
                        CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX").str); // Set position Z-component
                        CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY").str); // Set rotation X-component
                        CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ").str); // Set rotation Y-component
                        CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX").str); // Set rotation Z-component
                        CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW").str); // Set rotation W-component
                        CoSimManagers[i].CoSimTimer = 0.0f;
                        CoSimManagers[i].CoSimPosition = CoSimPosition;
                        CoSimManagers[i].CoSimRotation = CoSimRotation;
                        CoSimManagers[i].enabled = true;
                    }
                    else
                    {
                        CoSimManagers[i].enabled = false;
                        if (VehicleRigidBodies.Length > i) VehicleRigidBodies[i].isKinematic = false;

                        // Only apply TwistController if it's enabled and fields exist
                        if (TwistControllers.Length > i && TwistControllers[i] != null)
                        {
                            if (jsonObject.HasField("V"+(i+1).ToString()+" Linear Velocity"))
                            {
                                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                            }
                            if (jsonObject.HasField("V"+(i+1).ToString()+" Angular Velocity"))
                            {
                                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                            }
                        }
                    }
                }

                // Handle CarLighting and ROVLighting (if configured and fields exist)
                if(CarLightings.Length > i)
                {
                    if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                    {
                        CarLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                    }
                    if (jsonObject.HasField("V"+(i+1).ToString()+" Indicators"))
                    {
                        CarLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                }
                if(ROVLightings.Length > i)
                {
                    if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                    {
                        ROVLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                    }
                }
            }
        }
    }
    // -----------------------------------------------------------------

    void EmitTelemetry(SocketIOEvent obj) // obj is not really used for data, consider removing it from signature
    {
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            //Debug.Log("Attempting to write data...");
            Dictionary<string, string> data = new Dictionary<string, string>(); // Create new `data` dictionary
            // Read data from traffic lights
            if(TrafficLightControllers.Length != 0)
            {
                for(int i=0;i<TrafficLightControllers.Length;i++)
                {
                    data["TL"+(i+1).ToString()+" State"] = TrafficLightControllers[i].CurrentState.ToString(); // Get status
                }
            }
            // Read data from vehicles
            if(VehicleControllers.Length != 0 || AutomobileControllers.Length != 0)
            {
                for (int i=0;i<VehicleControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(VehicleControllers[i].Vehicle.transform.InverseTransformDirection(VehicleControllers[i].Vehicle.GetComponent<Rigidbody>().velocity).z, 4)).ToString("F4"); // Get speed
                }
                for(int i=0;i<AutomobileControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = AutomobileControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = AutomobileControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V"+(i+1).ToString()+" Brake"] = AutomobileControllers[i].CurrentBrake.ToString("F4"); // Get brake
                    data["V"+(i+1).ToString()+" Handbrake"] = AutomobileControllers[i].CurrentHandbrake.ToString("F4"); // Get handbrake
                    data["V"+(i+1).ToString()+" Collisions"] = AutomobileControllers[i].collisionCount.ToString(); // Get collision count
                    data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(AutomobileControllers[i].currSpeed / AutomobileControllers[i].speedMultiplier, 4)).ToString("F4"); // Get speed
                }
                for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length;i++) // Assumed that VehicleControllers.Length+AutomobileControllers.Length >= others
                {
                    data["V"+(i+1).ToString()+" Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                    data["V"+(i+1).ToString()+" Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F4") + " " + RightWheelEncoders[i].Angle.ToString("F4"); // Get encoder angles
                    data["V"+(i+1).ToString()+" Position"] = PositioningSystems[i].CurrentPosition[0].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[1].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[2].ToString("F4"); // Get vehicle position
                    data["V"+(i+1).ToString()+" Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F4"); // Get vehicle orientation (Quaternion)
                    data["V"+(i+1).ToString()+" Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F4"); // Get vehicle orientation (Euler Angles)
                    data["V"+(i+1).ToString()+" Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F4"); // Get angular velocity of the vehicle
                    data["V"+(i+1).ToString()+" Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F4"); // Get linear acceleration of the vehicle
                    if(LIDARUnits.Length != 0)
                    {
                        data["V"+(i+1).ToString()+" LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F4"); // Get LIDAR scan rate
                        //if(LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1] != null)
                        if(LIDARUnits[i].CurrentRangeArray != null && LIDARUnits[i].CurrentRangeArray.Length > 0)
                        {
                            // Debug.Log($"[EmitTelemetry] Sending {LIDARUnits[i].CurrentRangeArray.Length} LiDAR points for V{i+1}"); // Suppressed for cleaner output
                            data["V" + (i + 1).ToString() + " LIDAR Range Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentRangeArray); // Get LIDAR range array
                            if (IntensityArray) data["V" + (i + 1).ToString() + " LIDAR Intensity Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentIntensityArray); // Get LIDAR intensity array
                        }
                    }
                    if(LIDAR3DUnits.Length != 0) data["V"+(i+1).ToString()+" LIDAR Pointcloud"] = Convert.ToBase64String(LIDAR3DUnits[i].CurrentPointcloud); // Get LIDAR pointcloud
                    if(FrontCameras.Length != 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Left Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get left camera image
                        else data["V"+(i+1).ToString()+" Front Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get front camera image
                    }
                    if(RearCameras.Length != 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Right Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get right camera image
                        else data["V"+(i+1).ToString()+" Rear Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get rear camera image
                    }
                }
            }
            if (LapTimers.Length != 0)
            {
                for (int i = 0; i < LapTimers.Length; i++)
                {
                    data["V"+(i+1).ToString()+" Lap Count"] = LapTimers[i].LapCount.ToString(); // Get lap count
                    data["V"+(i+1).ToString()+" Lap Time"] = LapTimers[i].LapTime.ToString("F4"); // Get lap time
                    data["V"+(i+1).ToString()+" Last Lap Time"] = LapTimers[i].LastLapTime.ToString("F4"); // Get last lap count
                    data["V"+(i+1).ToString()+" Best Lap Time"] = LapTimers[i].BestLapTime.ToString("F4"); // Get best lap time
                    data["V"+(i+1).ToString()+" Collisions"] = LapTimers[i].CollisionCount.ToString(); // Get collision count
                }
            }
            if (socket.IsConnected) // Only emit if the socket is connected
            {
                // Debug.Log("Unity: Actually emitting 'Bridge' event."); // Suppressed for cleaner output
                socket.Emit("Bridge", new JSONObject(data)); // Write data to server
            }
            else
            {
                // Debug.LogWarning("Socket not connected, cannot emit telemetry."); // Suppressed for cleaner output
            }
        });
    }
}
