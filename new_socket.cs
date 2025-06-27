using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using SocketIO; // Ensure this is correctly referenced

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
    public float emitFrequencyHz = 30.0f; // Emit 30 times per second
    private float nextEmitTime = 0f;

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `Bridge` and corresponding event handler `OnBridge`
        socket.On("disconnect", OnDisconnect); // Declare disconnection event `disconnect` and corresponding event handler `OnDisconnect`
        
        // Temporary render textures for all vehicles except the first one (first vehicle will render to GUI)
        if(FrontCameras != null && FrontCameras.Length > 1) // Check for null array and length
        {
            for(int i=1;i<FrontCameras.Length;i++)
            {
                if (FrontCameras[i] != null) // Ensure camera itself is not null
                    FrontCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        if(RearCameras != null && RearCameras.Length > 1) // Check for null array and length
        {
            for(int i=1;i<RearCameras.Length;i++)
            {
                if (RearCameras[i] != null) // Ensure camera itself is not null
                    RearCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
    }

    void Update()
    {
        // Debug.Log("Unity: Update loop running and checking for emit."); // Consider commenting this out if too verbose in console
        if (socket.IsConnected && Time.time >= nextEmitTime)
        {
            Debug.Log("Unity: Time to emit telemetry. Socket is connected.");
            EmitTelemetry(null); // No event object needed for timed emission
            nextEmitTime = Time.time + (1.0f / emitFrequencyHz);
        }
        else if (!socket.IsConnected)
        {
            Debug.LogWarning("Unity: Socket not connected. Telemetry not emitting.");
        }
    }

    void OnConnect(SocketIOEvent obj)
    {
        Debug.Log("Unity: Connected to Socket.IO server!");
        ConnectionButton.interactable = false;
        ConnectionLabel.text = "Connected";
        // No need to call EmitTelemetry here anymore, Update() handles continuous emission.
    }

    void OnDisconnect(SocketIOEvent obj)
    {
        Debug.Log("Unity: Disconnected from Socket.IO server!");
        ConnectionButton.interactable = true;
        ConnectionLabel.text = "Disconnected";
        // No need to call EmitTelemetry here.
    }

    void OnBridge(SocketIOEvent obj)
    {
        Debug.Log("Unity: Received 'Bridge' event from server. Processing incoming commands.");
        
        if (obj.data == null)
        {
            Debug.LogWarning("Unity: Received 'Bridge' event with no data.");
            return;
        }

        JSONObject jsonObject = obj.data; // Read incoming data and store it in a `JSONObject`
        // Debug.Log("Incoming JSON: " + jsonObject.ToString()); // Log the full incoming JSON for debugging

        // Set time of day
        if(TimeOfDayAPI && (TimeOfDay != null && TimeOfDay.Length !=0))
        {
            try {
                TimeOfDay[0].automaticUpdate = (jsonObject.GetField("Auto Time")?.str == "True");
                TimeOfDay[0].timeScale = float.Parse(jsonObject.GetField("Time Scale")?.str ?? "1.0");
                TimeOfDay[0].timeOfDay = float.Parse(jsonObject.GetField("Time")?.str ?? "12.0");
            } catch (Exception e) { Debug.LogError($"Error parsing TimeOfDay data: {e.Message}"); }
        }

        // Set weather
        if(WeatherAPI && (Weather != null && Weather.Length !=0))
        {
            try {
                if (jsonObject.HasField("Weather"))
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
                }
                Weather[0].CloudIntensity = float.Parse(jsonObject.GetField("Clouds")?.str ?? "0.0"); // Set cloud intensity
                Weather[0].FogIntensity = float.Parse(jsonObject.GetField("Fog")?.str ?? "0.0"); // Set fog intensity
                Weather[0].RainIntensity = float.Parse(jsonObject.GetField("Rain")?.str ?? "0.0"); // Set rain intensity
                Weather[0].SnowIntensity = float.Parse(jsonObject.GetField("Snow")?.str ?? "0.0"); // Set snow intensity
            } catch (Exception e) { Debug.LogError($"Error parsing Weather data: {e.Message}"); }
        }

        // Write data to vehicles (VehicleControllers)
        if(VehicleControllers != null && VehicleControllers.Length != 0)
        {
            for(int i=0;i<VehicleControllers.Length;i++)
            {
                if(VehicleControllers[i] == null) continue; // Skip if VehicleController is null

                if(VehicleControllers[i].CurrentDrivingMode == 1)
                {
                    if (ResetManagers != null && ResetManagers.Length > i && ResetManagers[i] != null)
                    {
                        if (jsonObject.HasField("Reset"))
                            ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                    
                    if (CoSimManagers != null && CoSimManagers.Length > i && CoSimManagers[i] != null && VehicleRigidBodies != null && VehicleRigidBodies.Length > i && VehicleRigidBodies[i] != null)
                    {
                        if(jsonObject.HasField("V"+(i+1).ToString()+" CoSim") && int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                        {
                            VehicleRigidBodies[i].isKinematic = true;
                            CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY")?.str ?? "0"); // Set position X-component
                            CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ")?.str ?? "0"); // Set position Y-component
                            CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX")?.str ?? "0"); // Set position Z-component
                            CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY")?.str ?? "0"); // Set rotation X-component
                            CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ")?.str ?? "0"); // Set rotation Y-component
                            CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX")?.str ?? "0"); // Set rotation Z-component
                            CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW")?.str ?? "1"); // Set rotation W-component
                            CoSimManagers[i].CoSimTimer = 0.0f;
                            CoSimManagers[i].CoSimPosition = CoSimPosition;
                            CoSimManagers[i].CoSimRotation = CoSimRotation;
                            CoSimManagers[i].enabled = true;
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
                        if (VehicleRigidBodies != null && VehicleRigidBodies.Length > i && VehicleRigidBodies[i] != null)
                            VehicleRigidBodies[i].isKinematic = false;
                        ApplyDriveCommands(jsonObject, i);
                    }
                    
                    if(VehicleLightings != null && VehicleLightings.Length > i && VehicleLightings[i] != null)
                    {
                        if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                            VehicleLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                        if (jsonObject.HasField("V"+(i+1).ToString()+" Indicators"))
                            VehicleLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                }
            }
        }
        
        // Write data to vehicles (AutomobileControllers)
        if(AutomobileControllers != null && AutomobileControllers.Length != 0)
        {
            for(int i=0;i<AutomobileControllers.Length;i++)
            {
                if(AutomobileControllers[i] == null) continue; // Skip if AutomobileController is null

                if(AutomobileControllers[i].CurrentDrivingMode == 1)
                {
                    if (ResetManagers != null && ResetManagers.Length > i && ResetManagers[i] != null)
                    {
                        if (jsonObject.HasField("Reset"))
                            ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                    
                    if (CoSimManagers != null && CoSimManagers.Length > i && CoSimManagers[i] != null && VehicleRigidBodies != null && VehicleRigidBodies.Length > i && VehicleRigidBodies[i] != null)
                    {
                        if(jsonObject.HasField("V"+(i+1).ToString()+" CoSim") && int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                        {
                            VehicleRigidBodies[i].isKinematic = true;
                            CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY")?.str ?? "0"); // Set position X-component
                            CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ")?.str ?? "0"); // Set position Y-component
                            CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX")?.str ?? "0"); // Set position Z-component
                            CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY")?.str ?? "0"); // Set rotation X-component
                            CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ")?.str ?? "0"); // Set rotation Y-component
                            CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX")?.str ?? "0"); // Set rotation Z-component
                            CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW")?.str ?? "1"); // Set rotation W-component
                            CoSimManagers[i].CoSimTimer = 0.0f;
                            CoSimManagers[i].CoSimPosition = CoSimPosition;
                            CoSimManagers[i].CoSimRotation = CoSimRotation;
                            CoSimManagers[i].enabled = true;
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
                        if (VehicleRigidBodies != null && VehicleRigidBodies.Length > i && VehicleRigidBodies[i] != null)
                            VehicleRigidBodies[i].isKinematic = false;
                        ApplyAutomobileCommands(jsonObject, i);
                    }
                    
                    if(CarLightings != null && CarLightings.Length > i && CarLightings[i] != null)
                    {
                        if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                            CarLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                        if (jsonObject.HasField("V"+(i+1).ToString()+" Indicators"))
                            CarLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                    if(ROVLightings != null && ROVLightings.Length > i && ROVLightings[i] != null)
                    {
                        if (jsonObject.HasField("V"+(i+1).ToString()+" Headlights"))
                            ROVLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                    }
                }
            }
        }

        // Write data to traffic lights
        if(TrafficLightControllers != null && TrafficLightControllers.Length != 0)
        {
            for(int i=0;i<TrafficLightControllers.Length;i++)
            {
                if(TrafficLightControllers[i] == null) continue; // Skip if TLController is null
                if (jsonObject.HasField("TL"+(i+1).ToString()+" State"))
                    TrafficLightControllers[i].CurrentState = int.Parse(jsonObject.GetField("TL"+(i+1).ToString()+" State").str); // Set traffic light
            }
        }
    }

    // Helper method to apply vehicle drive commands
    private void ApplyDriveCommands(JSONObject jsonObject, int i)
    {
        if (TwistControllers != null && TwistControllers.Length > i && TwistControllers[i] != null)
        {
            if (jsonObject.HasField("V"+(i+1).ToString()+" Linear Velocity"))
                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity")?.str ?? "0.0"); // Set linear velocity
            if (jsonObject.HasField("V"+(i+1).ToString()+" Angular Velocity"))
                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity")?.str ?? "0.0"); // Set angular velocity
        }
        else if (VehicleControllers != null && VehicleControllers.Length > i && VehicleControllers[i] != null)
        {
            if (jsonObject.HasField("V"+(i+1).ToString()+" Throttle"))
                VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle")?.str ?? "0.0"); // Set throttle
            if (jsonObject.HasField("V"+(i+1).ToString()+" Steering"))
                VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering")?.str ?? "0.0"); // Set steering angle
        }
    }

    // Helper method to apply automobile drive commands
    private void ApplyAutomobileCommands(JSONObject jsonObject, int i)
    {
        if (TwistControllers != null && TwistControllers.Length > i && TwistControllers[i] != null)
        {
            if (jsonObject.HasField("V"+(i+1).ToString()+" Linear Velocity"))
                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity")?.str ?? "0.0"); // Set linear velocity
            if (jsonObject.HasField("V"+(i+1).ToString()+" Angular Velocity"))
                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity")?.str ?? "0.0"); // Set angular velocity
        }
        else if (AutomobileControllers != null && AutomobileControllers.Length > i && AutomobileControllers[i] != null)
        {
            if (jsonObject.HasField("V"+(i+1).ToString()+" Throttle"))
                AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle")?.str ?? "0.0"); // Set throttle
            if (jsonObject.HasField("V"+(i+1).ToString()+" Steering"))
                AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering")?.str ?? "0.0"); // Set steering angle
            if (jsonObject.HasField("V"+(i+1).ToString()+" Brake"))
                AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake")?.str ?? "0.0"); // Set brake
            if (jsonObject.HasField("V"+(i+1).ToString()+" Handbrake"))
                AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake")?.str ?? "0.0"); // Set handbrake
        }
    }


    void EmitTelemetry(SocketIOEvent obj) // obj is not really used for data, consider removing it from signature
    {
        // Enqueue the action to the main thread, as Unity API calls should be on the main thread
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            //Debug.Log("Attempting to write data...");
            Dictionary<string, string> data = new Dictionary<string, string>(); // Create new `data` dictionary
            
            // Read data from traffic lights
            if(TrafficLightControllers != null && TrafficLightControllers.Length != 0)
            {
                for(int i=0;i<TrafficLightControllers.Length;i++)
                {
                    if(TrafficLightControllers[i] != null)
                        data["TL"+(i+1).ToString()+" State"] = TrafficLightControllers[i].CurrentState.ToString(); // Get status
                }
            }
            
            // Read data from vehicles (VehicleControllers and AutomobileControllers)
            // Combine these loops for consistency if possible, or handle separately carefully.
            // Assuming `i` maps directly to a vehicle index for all these arrays.
            int totalVehicles = Mathf.Max(VehicleControllers != null ? VehicleControllers.Length : 0, 
                                          AutomobileControllers != null ? AutomobileControllers.Length : 0);

            for(int i=0;i<totalVehicles;i++) 
            {
                // Handle VehicleController data
                if(VehicleControllers != null && i < VehicleControllers.Length && VehicleControllers[i] != null)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    if(VehicleControllers[i].Vehicle != null && VehicleControllers[i].Vehicle.GetComponent<Rigidbody>() != null)
                    {
                        data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(VehicleControllers[i].Vehicle.transform.InverseTransformDirection(VehicleControllers[i].Vehicle.GetComponent<Rigidbody>().velocity).z, 4)).ToString("F4"); // Get speed
                    }
                }
                
                // Handle AutomobileController data
                if(AutomobileControllers != null && i < AutomobileControllers.Length && AutomobileControllers[i] != null)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = AutomobileControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = AutomobileControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V"+(i+1).ToString()+" Brake"] = AutomobileControllers[i].CurrentBrake.ToString("F4"); // Get brake
                    data["V"+(i+1).ToString()+" Handbrake"] = AutomobileControllers[i].CurrentHandbrake.ToString("F4"); // Get handbrake
                    data["V"+(i+1).ToString()+" Collisions"] = AutomobileControllers[i].collisionCount.ToString(); // Get collision count
                    data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(AutomobileControllers[i].currSpeed / AutomobileControllers[i].speedMultiplier, 4)).ToString("F4"); // Get speed
                }
                
                // Common sensor data (assuming these arrays align by vehicle index)
                if(LeftWheelEncoders != null && LeftWheelEncoders.Length > i && LeftWheelEncoders[i] != null &&
                   RightWheelEncoders != null && RightWheelEncoders.Length > i && RightWheelEncoders[i] != null)
                {
                    data["V"+(i+1).ToString()+" Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                    data["V"+(i+1).ToString()+" Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F4") + " " + RightWheelEncoders[i].Angle.ToString("F4"); // Get encoder angles
                }
                if(PositioningSystems != null && PositioningSystems.Length > i && PositioningSystems[i] != null)
                {
                    data["V"+(i+1).ToString()+" Position"] = PositioningSystems[i].CurrentPosition[0].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[1].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[2].ToString("F4"); // Get vehicle position
                }
                if(InertialMeasurementUnits != null && InertialMeasurementUnits.Length > i && InertialMeasurementUnits[i] != null)
                {
                    data["V"+(i+1).ToString()+" Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F4"); // Get vehicle orientation (Quaternion)
                    data["V"+(i+1).ToString()+" Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F4"); // Get vehicle orientation (Euler Angles)
                    data["V"+(i+1).ToString()+" Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F4"); // Get angular velocity of the vehicle
                    data["V"+(i+1).ToString()+" Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F4"); // Get linear acceleration of the vehicle
                }
                
                if(LIDARUnits != null && LIDARUnits.Length > i && LIDARUnits[i] != null)
                {
                    data["V"+(i+1).ToString()+" LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F4"); // Get LIDAR scan rate
                    if(LIDARUnits[i].CurrentRangeArray != null && LIDARUnits[i].CurrentRangeArray.Length > 0)
                    {
                        Debug.Log($"[EmitTelemetry] Sending {LIDARUnits[i].CurrentRangeArray.Length} LiDAR points for V{i+1}");
                        data["V" + (i + 1).ToString() + " LIDAR Range Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentRangeArray); // Get LIDAR range array
                        if (IntensityArray) data["V" + (i + 1).ToString() + " LIDAR Intensity Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentIntensityArray); // Get LIDAR intensity array
                    }
                    else
                    {
                        Debug.LogWarning($"[EmitTelemetry] LIDARUnits[{i}].CurrentRangeArray is null or empty. Not sending LiDAR data for V{i+1}");
                    }
                }
                if(LIDAR3DUnits != null && LIDAR3DUnits.Length > i && LIDAR3DUnits[i] != null)
                {
                    if (LIDAR3DUnits[i].CurrentPointcloud != null && LIDAR3DUnits[i].CurrentPointcloud.Length > 0)
                    {
                        data["V"+(i+1).ToString()+" LIDAR Pointcloud"] = Convert.ToBase64String(LIDAR3DUnits[i].CurrentPointcloud); // Get LIDAR pointcloud
                    }
                    else
                    {
                        Debug.LogWarning($"[EmitTelemetry] LIDAR3DUnits[{i}].CurrentPointcloud is null or empty. Not sending 3D LiDAR data for V{i+1}");
                    }
                }
                
                if(FrontCameras != null && FrontCameras.Length > i && FrontCameras[i] != null)
                {
                    byte[] frameData = FrameGrabber.CaptureFrame(FrontCameras[i]);
                    if (frameData != null && frameData.Length > 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Left Camera Image"] = Convert.ToBase64String(frameData); // Get left camera image
                        else data["V"+(i+1).ToString()+" Front Camera Image"] = Convert.ToBase64String(frameData); // Get front camera image
                    }
                    else
                    {
                        Debug.LogWarning($"[EmitTelemetry] Front Camera [{i}] frame data is null or empty. Not sending camera data for V{i+1}");
                    }
                }
                if(RearCameras != null && RearCameras.Length > i && RearCameras[i] != null)
                {
                    byte[] frameData = FrameGrabber.CaptureFrame(RearCameras[i]);
                    if (frameData != null && frameData.Length > 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Right Camera Image"] = Convert.ToBase64String(frameData); // Get right camera image
                        else data["V"+(i+1).ToString()+" Rear Camera Image"] = Convert.ToBase64String(frameData); // Get rear camera image
                    }
                    else
                    {
                        Debug.LogWarning($"[EmitTelemetry] Rear Camera [{i}] frame data is null or empty. Not sending camera data for V{i+1}");
                    }
                }
            }
            
            if (LapTimers != null && LapTimers.Length != 0)
            {
                for (int i = 0; i < LapTimers.Length; i++)
                {
                    if (LapTimers[i] != null)
                    {
                        data["V"+(i+1).ToString()+" Lap Count"] = LapTimers[i].LapCount.ToString(); // Get lap count
                        data["V"+(i+1).ToString()+" Lap Time"] = LapTimers[i].LapTime.ToString("F4"); // Get lap time
                        data["V"+(i+1).ToString()+" Last Lap Time"] = LapTimers[i].LastLapTime.ToString("F4"); // Get last lap count
                        data["V"+(i+1).ToString()+" Best Lap Time"] = LapTimers[i].BestLapTime.ToString("F4"); // Get best lap time
                        data["V"+(i+1).ToString()+" Collisions"] = LapTimers[i].CollisionCount.ToString(); // Get collision count
                    }
                }
            }

            if (socket.IsConnected) // Only emit if the socket is connected
            {
                // Debug.Log("Unity: Actually emitting 'Bridge' event with " + data.Count + " fields."); // Verbose log
                socket.Emit("Bridge", new JSONObject(data)); // Write data to server
            }
            else
            {
                // This warning is also in Update, so might be redundant if Update's warning is sufficient.
                // Debug.LogWarning("Socket not connected, cannot emit telemetry.");
            }
        });
    }
}
