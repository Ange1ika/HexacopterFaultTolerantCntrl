using UnityEngine;

/// <summary>
/// Создаёт сцену программно при запуске.
/// Прикрепи к пустому GameObject "SceneSetup" и запусти в Editor.
/// 
/// ЛИБО создай сцену вручную по инструкции README.md
/// </summary>
public class SceneSetup : MonoBehaviour
{
    void Awake()
    {
        SetupCamera();
        SetupLighting();
        SetupGround();
        SetupDrone();
        Debug.Log("[SceneSetup] Сцена создана!");
    }

    void SetupCamera()
    {
        var cam = Camera.main;
        if (cam == null) return;
        cam.transform.position = new Vector3(0, 20, -25);
        cam.transform.LookAt(Vector3.up * 10);
        cam.backgroundColor = new Color(0.05f, 0.05f, 0.1f);
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.farClipPlane = 1000f;
        cam.gameObject.AddComponent<CameraFollow>();
    }

    void SetupLighting()
    {
        RenderSettings.ambientLight = new Color(0.3f, 0.3f, 0.4f);
        var sun = new GameObject("Sun");
        var light = sun.AddComponent<Light>();
        light.type      = LightType.Directional;
        light.intensity = 1.2f;
        light.color     = new Color(1f, 0.95f, 0.85f);
        sun.transform.rotation = Quaternion.Euler(45, 30, 0);
    }

    void SetupGround()
    {
        // Сетка-земля
        var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.name = "Ground";
        ground.transform.position   = Vector3.zero;
        ground.transform.localScale = new Vector3(10, 1, 10);
        var mat = new Material(Shader.Find("Standard"));
        mat.color = new Color(0.15f, 0.15f, 0.2f);
        ground.GetComponent<Renderer>().material = mat;

        // Координатные оси
        DrawAxis(Vector3.right   * 5, Color.red,   "X");
        DrawAxis(Vector3.up      * 5, Color.green,  "Y");
        DrawAxis(Vector3.forward * 5, Color.blue,   "Z");
    }

    void DrawAxis(Vector3 end, Color color, string label)
    {
        var go = new GameObject($"Axis_{label}");
        var lr = go.AddComponent<LineRenderer>();
        lr.SetPositions(new[] { Vector3.zero, end });
        lr.startWidth = lr.endWidth = 0.05f;
        lr.material   = new Material(Shader.Find("Sprites/Default"));
        lr.startColor = lr.endColor = color;
    }

    void SetupDrone()
    {
        // Основной корпус
        var drone = new GameObject("Hexacopter");

        var body = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        body.name = "Body";
        body.transform.SetParent(drone.transform);
        body.transform.localScale    = new Vector3(0.4f, 0.06f, 0.4f);
        body.transform.localPosition = Vector3.zero;
        var bodyMat = new Material(Shader.Find("Standard"));
        bodyMat.color = new Color(0.2f, 0.2f, 0.25f);
        body.GetComponent<Renderer>().material = bodyMat;

        // 6 моторов + лопасти
        var rotorTransforms = new Transform[6];
        float b = 0.215f;
        for (int i = 0; i < 6; i++)
        {
            float angle = i * 60f * Mathf.Deg2Rad;
            float rx    = b * Mathf.Cos(angle);
            float rz    = b * Mathf.Sin(angle);

            // Луч
            var arm = GameObject.CreatePrimitive(PrimitiveType.Cube);
            arm.name = $"Arm_{i+1}";
            arm.transform.SetParent(drone.transform);
            arm.transform.localPosition = new Vector3(rx * 0.5f, 0, rz * 0.5f);
            arm.transform.localScale    = new Vector3(0.03f, 0.02f, b);
            arm.transform.LookAt(drone.transform.position +
                                  new Vector3(rx, 0, rz));

            // Мотор
            var motor = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            motor.name = $"Motor_{i+1}";
            motor.transform.SetParent(drone.transform);
            motor.transform.localPosition = new Vector3(rx, 0.03f, rz);
            motor.transform.localScale    = new Vector3(0.06f, 0.04f, 0.06f);
            var mMat = new Material(Shader.Find("Standard"));
            mMat.color = new Color(0.8f, 0.3f, 0.1f);
            motor.GetComponent<Renderer>().material = mMat;

            // Лопасть
            var rotor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rotor.name = $"Rotor_{i+1}";
            rotor.transform.SetParent(motor.transform);
            rotor.transform.localPosition = Vector3.up * 0.6f;
            rotor.transform.localScale    = new Vector3(3f, 0.05f, 0.4f);
            var rMat = new Material(Shader.Find("Standard"));
            rMat.color = new Color(0.9f, 0.9f, 0.9f, 0.7f);
            rotor.GetComponent<Renderer>().material = rMat;

            rotorTransforms[i] = rotor.transform;
        }

        // Индикатор отказа (красная сфера)
        var faultInd = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        faultInd.name = "FaultIndicator";
        faultInd.transform.SetParent(drone.transform);
        faultInd.transform.localPosition = new Vector3(0, 0.3f, 0);
        faultInd.transform.localScale    = Vector3.one * 0.15f;
        var fMat = new Material(Shader.Find("Standard"));
        fMat.color = Color.red;
        fMat.EnableKeyword("_EMISSION");
        fMat.SetColor("_EmissionColor", Color.red * 2f);
        faultInd.GetComponent<Renderer>().material = fMat;
        faultInd.SetActive(false);

        // Trail
        var trail = drone.AddComponent<TrailRenderer>();
        trail.time        = 30f;
        trail.startWidth  = 0.08f;
        trail.endWidth    = 0.01f;
        trail.startColor  = Color.cyan;
        trail.endColor    = new Color(0, 1, 1, 0);
        trail.material    = new Material(Shader.Find("Sprites/Default"));

        // SimulatorClient
        var simGO   = new GameObject("SimManager");
        var client  = simGO.AddComponent<SimulatorClient>();

        // DroneController
        var ctrl = drone.AddComponent<DroneController>();
        ctrl.simClient     = client;
        ctrl.rotors        = rotorTransforms;
        ctrl.faultIndicator = faultInd;
        ctrl.trailRenderer  = trail;

        // HUD
        var hudGO = new GameObject("HUD");
        var hud   = hudGO.AddComponent<DroneHUD>();
        hud.simClient = client;

        // Trajectory
        var trajGO = new GameObject("TrajectoryVis");
        var tvis   = trajGO.AddComponent<TrajectoryVisualizer>();
        tvis.simClient       = client;
        tvis.droneTransform  = drone.transform;
    }
}
