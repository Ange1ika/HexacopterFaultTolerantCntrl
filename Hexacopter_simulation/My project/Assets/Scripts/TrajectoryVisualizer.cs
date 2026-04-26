using UnityEngine;
using System.Collections.Generic;

public class TrajectoryVisualizer : MonoBehaviour
{
    [Header("References")]
    public SimulatorClient simClient;
    public Transform       droneTransform;


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

        // Линия реального пути
        var pathGO = new GameObject("ActualPath");
        pathGO.transform.SetParent(transform);
        _pathLine = pathGO.AddComponent<LineRenderer>();
        _pathLine.startWidth    = _pathLine.endWidth = 0.06f;
        _pathLine.material      = new Material(Shader.Find("Sprites/Default"));
        _pathLine.useWorldSpace = true;
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