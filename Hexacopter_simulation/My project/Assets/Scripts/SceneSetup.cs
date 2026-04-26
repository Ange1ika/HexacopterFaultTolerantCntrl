using UnityEngine;

/// <summary>
/// Создаёт всю сцену программно при старте.
/// Прикрепи к пустому GameObject в сцене — больше ничего настраивать не нужно.
/// </summary>
public class SceneSetup : MonoBehaviour
{
    [Header("Arena")]
    public float arenaSize     = 40f;
    public float wallHeight    = 5f;
    public float wallThickness = 0.3f;

    void Awake()
    {
        SetupLighting();
        SetupGround();
        SetupArenaWalls();

        // Дрон создаётся первым — камеры привязываются к нему
        var droneTransform = SetupDrone();
        SetupCamera(droneTransform);

        Debug.Log("[SceneSetup] Сцена готова.");
    }

    // ── Дрон ─────────────────────────────────────────────────────────────────
    Transform SetupDrone()
    {
        var drone = new GameObject("Hexacopter");

        // ── Корпус ───────────────────────────────────────────────────────────
        var body = MakePrimitive(PrimitiveType.Cylinder, "Body", drone.transform,
            localPos:   new Vector3(0f, 0.02f, 0f),
            localScale: new Vector3(0.4f, 0.06f, 0.4f),
            color:      new Color(0.2f, 0.2f, 0.25f));

        // ── 6 лучей + моторов + лопастей ─────────────────────────────────────
        var rotorTransforms = new Transform[6];
        float armLen = 0.215f;

        for (int i = 0; i < 6; i++)
        {
            float angle = i * 60f * Mathf.Deg2Rad;
            float rx    = armLen * Mathf.Cos(angle);
            float rz    = armLen * Mathf.Sin(angle);

            // Луч
            var arm = MakePrimitive(PrimitiveType.Cube, $"Arm_{i+1}", drone.transform,
                localPos:   new Vector3(rx * 0.5f, 0f, rz * 0.5f),
                localScale: new Vector3(0.03f, 0.02f, armLen),
                color:      new Color(0.25f, 0.25f, 0.3f));
            arm.transform.LookAt(drone.transform.position + new Vector3(rx, 0f, rz));

            // Мотор
            var motor = MakePrimitive(PrimitiveType.Cylinder, $"Motor_{i+1}", drone.transform,
                localPos:   new Vector3(rx, 0.03f, rz),
                localScale: new Vector3(0.06f, 0.04f, 0.06f),
                color:      new Color(0.8f, 0.3f, 0.1f));

            // Лопасть (дочерний объект мотора — вращается вместе с ним)
            var rotor = MakePrimitive(PrimitiveType.Cube, $"Rotor_{i+1}", motor.transform,
                localPos:   new Vector3(0f, 0.6f, 0f),
                localScale: new Vector3(3f, 0.05f, 0.4f),
                color:      new Color(0.9f, 0.9f, 0.9f));

            rotorTransforms[i] = rotor.transform;
        }

        // ── Индикатор отказа ─────────────────────────────────────────────────
        var faultInd = MakePrimitive(PrimitiveType.Sphere, "FaultIndicator", drone.transform,
            localPos:   new Vector3(0f, 0.3f, 0f),
            localScale: Vector3.one * 0.15f,
            color:      Color.red,
            emissive:   true);
        faultInd.SetActive(false);

        // ── Trail ─────────────────────────────────────────────────────────────
        var trail        = drone.AddComponent<TrailRenderer>();
        trail.time       = 30f;
        trail.startWidth = 0.08f;
        trail.endWidth   = 0.01f;
        trail.startColor = Color.cyan;
        trail.endColor   = new Color(0f, 1f, 1f, 0f);
        trail.material   = new Material(Shader.Find("Sprites/Default"));

        // ── SimulatorClient ───────────────────────────────────────────────────
        var simGO  = new GameObject("SimManager");
        var client = simGO.AddComponent<SimulatorClient>();

        // ── DroneController ───────────────────────────────────────────────────
        var ctrl            = drone.AddComponent<DroneController>();
        ctrl.simClient      = client;
        ctrl.rotors         = rotorTransforms;
        ctrl.faultIndicator = faultInd;
        ctrl.trailRenderer  = trail;

        // ── HUD ───────────────────────────────────────────────────────────────
        var hud = new GameObject("HUD").AddComponent<DroneHUD>();
        hud.simClient = client;

        // ── Визуализатор траектории ───────────────────────────────────────────
        var tvis             = new GameObject("TrajectoryVisualizer").AddComponent<TrajectoryVisualizer>();
        tvis.simClient      = client;
        tvis.droneTransform = drone.transform;

        return drone.transform;
    }

    // ── Камеры ────────────────────────────────────────────────────────────────
    void SetupCamera(Transform droneTarget)
    {
        // Основная камера — близко, следует за дроном
        var cam = Camera.main;
        if (cam != null)
        {
            cam.backgroundColor = new Color(0.05f, 0.05f, 0.1f);
            cam.clearFlags      = CameraClearFlags.SolidColor;
            cam.farClipPlane    = 1000f;

            var follow        = cam.gameObject.AddComponent<CameraFollow>();
            follow.target     = droneTarget;
            follow.offset     = new Vector3(0f, 2f, -5f);
            follow.smoothSpeed = 5f;
        }

        // Дальняя камера — картинка в картинке (верхний правый угол)
        var farCamGO           = new GameObject("FarCamera");
        var farCam             = farCamGO.AddComponent<Camera>();
        farCam.clearFlags      = CameraClearFlags.SolidColor;
        farCam.backgroundColor = new Color(0.03f, 0.03f, 0.06f);
        farCam.farClipPlane    = 1500f;
        farCam.fieldOfView     = 95f;
        farCam.depth           = 1;                                    // поверх основной
        farCam.rect            = new Rect(0.67f, 0.67f, 0.31f, 0.31f);

        var farFollow        = farCamGO.AddComponent<CameraFollow>();
        farFollow.target     = droneTarget;
        farFollow.offset     = new Vector3(0f, 15f, -20f);
        farFollow.smoothSpeed = 3.5f;
    }

    // ── Окружение ─────────────────────────────────────────────────────────────
    void SetupLighting()
    {
        RenderSettings.ambientLight = new Color(0.3f, 0.3f, 0.4f);
        var sun   = new GameObject("Sun");
        var light = sun.AddComponent<Light>();
        light.type      = LightType.Directional;
        light.intensity = 1.2f;
        light.color     = new Color(1f, 0.95f, 0.85f);
        sun.transform.rotation = Quaternion.Euler(45f, 30f, 0f);
    }

    void SetupGround()
    {
        var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.name = "Ground";
        ground.transform.localScale = new Vector3(arenaSize / 10f, 1f, arenaSize / 10f);
        var mat = new Material(Shader.Find("Standard"));
        mat.color = new Color(0.15f, 0.15f, 0.2f);
        ground.GetComponent<Renderer>().material = mat;

        DrawAxis(Vector3.right   * 5f, Color.red,   "X");
        DrawAxis(Vector3.up      * 5f, Color.green,  "Y");
        DrawAxis(Vector3.forward * 5f, Color.blue,   "Z");
    }

    void SetupArenaWalls()
    {
        var root = new GameObject("ArenaWalls");
        float h = arenaSize * 0.5f;
        float y = wallHeight * 0.5f;

        CreateWall(root.transform, "Wall_North", new Vector3(0f,  y,  h), new Vector3(arenaSize, wallHeight, wallThickness));
        CreateWall(root.transform, "Wall_South", new Vector3(0f,  y, -h), new Vector3(arenaSize, wallHeight, wallThickness));
        CreateWall(root.transform, "Wall_East",  new Vector3( h,  y, 0f), new Vector3(wallThickness, wallHeight, arenaSize));
        CreateWall(root.transform, "Wall_West",  new Vector3(-h,  y, 0f), new Vector3(wallThickness, wallHeight, arenaSize));
    }

    void CreateWall(Transform parent, string wallName, Vector3 pos, Vector3 scale)
    {
        var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = wallName;
        wall.transform.SetParent(parent);
        wall.transform.position   = pos;
        wall.transform.localScale = scale;
        var mat = new Material(Shader.Find("Standard"));
        mat.color = new Color(0.23f, 0.23f, 0.28f);
        wall.GetComponent<Renderer>().material = mat;
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

    // ── Вспомогательный метод создания примитива ──────────────────────────────
    static GameObject MakePrimitive(
        PrimitiveType type,
        string        objName,
        Transform     parent,
        Vector3       localPos,
        Vector3       localScale,
        Color         color,
        bool          emissive = false)
    {
        var go = GameObject.CreatePrimitive(type);
        go.name = objName;
        go.transform.SetParent(parent);
        go.transform.localPosition = localPos;
        go.transform.localScale    = localScale;

        // Убираем коллайдер — физика нам не нужна
        var col = go.GetComponent<Collider>();
        if (col) Object.Destroy(col);

        var mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        if (emissive)
        {
            mat.EnableKeyword("_EMISSION");
            mat.SetColor("_EmissionColor", color * 2f);
        }
        go.GetComponent<Renderer>().material = mat;
        return go;
    }
}