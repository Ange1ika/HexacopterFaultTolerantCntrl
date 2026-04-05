using UnityEngine;
using System.Collections.Generic;

public class TrajectoryVisualizer : MonoBehaviour
{
    [Header("References")]
    public SimulatorClient simClient;
    public Transform       droneTransform;

    [Header("Spiral preview")]
    public float radius          = 4f;
    public float omega_t         = 0.1f;
    public float w_t             = 1f;
    public float previewDuration = 100f;
    public int   spiralPoints    = 500;
    public Color spiralColor     = new Color(0.3f, 0.6f, 1f, 0.6f);

    [Header("Actual path")]
    public int   maxPathPoints = 2000;
    public Color normalColor   = Color.green;
    public Color faultColor    = Color.red;

    private LineRenderer       _spiralLine;
    private LineRenderer       _pathLine;
    private Queue<Vector3>     _pathPoints = new();
    private Queue<bool>        _pathFaults = new();  // храним fault вместо Color

    void Start()
    {
        // Линия спирали
        var spiralGO = new GameObject("SpiralPreview");
        spiralGO.transform.SetParent(transform);
        _spiralLine = spiralGO.AddComponent<LineRenderer>();
        _spiralLine.positionCount = spiralPoints;
        _spiralLine.startWidth = _spiralLine.endWidth = 0.04f;
        _spiralLine.material   = new Material(Shader.Find("Sprites/Default"));
        _spiralLine.startColor = _spiralLine.endColor = spiralColor;
        DrawSpiral();

        // Линия реального пути
        var pathGO = new GameObject("ActualPath");
        pathGO.transform.SetParent(transform);
        _pathLine = pathGO.AddComponent<LineRenderer>();
        _pathLine.startWidth    = _pathLine.endWidth = 0.06f;
        _pathLine.material      = new Material(Shader.Find("Sprites/Default"));
        _pathLine.useWorldSpace = true;
    }

    void DrawSpiral()
    {
        float dt = previewDuration / spiralPoints;
        for (int i = 0; i < spiralPoints; i++)
        {
            float t = i * dt;
            float x = radius * Mathf.Cos(omega_t * t);
            float y = radius * Mathf.Sin(omega_t * t);
            float z = w_t * t;
            _spiralLine.SetPosition(i, new Vector3(x, z, y)); // ROS→Unity
        }
    }

    void Update()
    {
        if (droneTransform == null || simClient == null) return;

        bool fault = simClient.faultActive;

        _pathPoints.Enqueue(droneTransform.position);
        _pathFaults.Enqueue(fault);

        while (_pathPoints.Count > maxPathPoints)
        {
            _pathPoints.Dequeue();
            _pathFaults.Dequeue();
        }

        // Обновляем позиции
        var pts    = new Vector3[_pathPoints.Count];
        var faults = new bool[_pathFaults.Count];
        _pathPoints.CopyTo(pts, 0);
        _pathFaults.CopyTo(faults, 0);

        _pathLine.positionCount = pts.Length;
        _pathLine.SetPositions(pts);

        // ── Gradient: максимум 8 ключей ──────────────────────────────
        // Берём 8 равномерно распределённых точек из пути
        const int MAX_KEYS = 8;
        int n = pts.Length;

        if (n < 2)
        {
            _pathLine.startColor = _pathLine.endColor = normalColor;
            _spiralLine.enabled  = !fault;
            return;
        }

        int keyCount = Mathf.Min(MAX_KEYS, n);
        var colorKeys = new GradientColorKey[keyCount];
        var alphaKeys = new GradientAlphaKey[]
        {
            new GradientAlphaKey(1f, 0f),
            new GradientAlphaKey(1f, 1f)
        };

        for (int k = 0; k < keyCount; k++)
        {
            // Индекс в массиве точек (равномерно)
            int idx  = (k == keyCount - 1) ? n - 1 : k * (n - 1) / (keyCount - 1);
            float t  = (float)k / (keyCount - 1);
            colorKeys[k] = new GradientColorKey(
                faults[idx] ? faultColor : normalColor, t);
        }

        var grad = new Gradient();
        grad.SetKeys(colorKeys, alphaKeys);
        _pathLine.colorGradient = grad;

        _spiralLine.enabled = !fault;
    }
}