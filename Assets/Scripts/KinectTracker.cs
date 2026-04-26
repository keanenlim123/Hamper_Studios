using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Azure.Kinect.BodyTracking;
using System.Threading;
using System.Threading.Tasks;

public class KinectTracker : MonoBehaviour
{
    private Device device;
    private Tracker tracker;
    public Transform[] jointMarkers; // Assign your sphere here

    private Vector3 pendingPosition;
    private readonly object lockObj = new object();
    private bool hasNewData = false;
    private bool isRunning = false;
    private CancellationTokenSource cts;

    void Start()
    {
        try
        {
            device = Device.Open(0);
            Debug.Log("Device opened");

            device.StartCameras(new DeviceConfiguration
            {
                DepthMode = DepthMode.NFOV_2x2Binned,
                ColorResolution = ColorResolution.Off,
                CameraFPS = FPS.FPS30,
                SynchronizedImagesOnly = false
            });
            Debug.Log("Cameras started");

            var calibration = device.GetCalibration();

            string modelPath = System.IO.Path.Combine(Application.dataPath, "Plugins", "dnn_model_2_0_lite_op11.onnx");
            string persistentPath = System.IO.Path.Combine(Application.persistentDataPath, "dnn_model_2_0_lite_op11.onnx");

            if (!System.IO.File.Exists(persistentPath))
                System.IO.File.Copy(modelPath, persistentPath);

            tracker = Tracker.Create(calibration, new TrackerConfiguration
            {
                ProcessingMode = TrackerProcessingMode.Cpu,
                SensorOrientation = SensorOrientation.Default,
                ModelPath = persistentPath
            });
            Debug.Log("Tracker created — stand in front of the Kinect!");

            isRunning = true;
            cts = new CancellationTokenSource();
            Task.Run(() => TrackingLoop(cts.Token));
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed: " + e.Message);
        }
    }

    private void TrackingLoop(CancellationToken token)
    {
        while (isRunning && !token.IsCancellationRequested)
        {
            try
            {
                using var capture = device.GetCapture();
                tracker.EnqueueCapture(capture);

                using var frame = tracker.PopResult(System.TimeSpan.FromMilliseconds(1000));

                if (frame != null && frame.NumberOfBodies > 0)
                {
                    var skeleton = frame.GetBodySkeleton(0);
                    var pelvis = skeleton.GetJoint(JointId.Pelvis);

                    // Convert mm to meters and flip Z so forward = positive
                    Vector3 newPos = new Vector3(
                        pelvis.Position.X / 1000f,
                        pelvis.Position.Y / 1000f,
                        pelvis.Position.Z / 1000f
                    );

                    lock (lockObj)
                    {
                        pendingPosition = newPos;
                        hasNewData = true;
                    }

                    Debug.Log($"Body at: {newPos}");
                }
                else
                {
                    Debug.Log("No body detected");
                }
            }
            catch (System.Exception e)
            {
                if (isRunning)
                    Debug.LogError("Tracking error: " + e.Message);
            }
        }
    }

    void Update()
    {
        lock (lockObj)
        {
            if (hasNewData)
            {
                if (jointMarkers != null && jointMarkers.Length > 0 && jointMarkers[0] != null)
                    jointMarkers[0].position = pendingPosition;

                hasNewData = false;
            }
        }
    }

    void OnDestroy()
    {
        isRunning = false;
        cts?.Cancel();
        tracker?.Dispose();
        device?.StopCameras();
        device?.Dispose();
    }
}