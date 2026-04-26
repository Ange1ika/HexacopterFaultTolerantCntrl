using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using UnityEngine;

/// <summary>
/// Подключается к Python симулятору по TCP и получает состояние дрона.
/// Прикрепи к любому GameObject в сцене (например, SimManager).
/// </summary>
public class SimulatorClient : MonoBehaviour
{
    [Header("Connection")]
    public string host = "127.0.0.1";
    public int    port = 9999;

    [Header("State (read-only)")]
    public float simTime;
    public bool  faultActive;
    public bool  landed;
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 desiredPosition;
    public Vector3 startPosition;
    public Vector3 goalPosition;
    public float[] rotorSpeeds = new float[6];

    private bool _markersCreated;
    private bool _obstaclesCreated;
    private bool _hasState;

    // Внутреннее состояние
    private TcpClient       _client;
    private Thread          _recvThread;
    private ConcurrentQueue<DronePacket> _queue = new();
    private bool            _running;

    // ── Lifecycle ────────────────────────────────────────────────────────
    void Start()
    {
        _hasState = false;
        _running = true;
        _recvThread = new Thread(ReceiveLoop) { IsBackground = true };
        _recvThread.Start();
        Debug.Log($"[SimClient] Подключение к {host}:{port}...");
    }

    void Update()
    {
        // Применяем последний пакет из очереди (в основном потоке Unity)
        DronePacket pkt = null;
        while (_queue.TryDequeue(out var tmp)) pkt = tmp;   // берём самый свежий

        if (pkt != null) ApplyPacket(pkt);
    }

    void OnDestroy()
    {
        _running = false;
        _client?.Close();
        _recvThread?.Join(500);
    }

    // ── Сетевой поток ────────────────────────────────────────────────────
    void ReceiveLoop()
    {
        while (_running)
        {
            try
            {
                _client = new TcpClient();
                _client.Connect(host, port);
                Debug.Log("[SimClient] Подключено!");

                var stream = _client.GetStream();
                var lenBuf = new byte[4];

                while (_running)
                {
                    // Читаем префикс длины (4 байта big-endian)
                    ReadExact(stream, lenBuf, 4);
                    int len = (lenBuf[0] << 24) | (lenBuf[1] << 16)
                            | (lenBuf[2] <<  8) |  lenBuf[3];

                    if (len <= 0 || len > 65536) continue;

                    var msgBuf = new byte[len];
                    ReadExact(stream, msgBuf, len);

                    string json = Encoding.UTF8.GetString(msgBuf).TrimEnd('\n');
                    var pkt = JsonUtility.FromJson<DronePacket>(json);
                    if (pkt != null) _queue.Enqueue(pkt);
                }
            }
            catch (Exception e)
            {
                if (_running)
                {
                    Debug.LogWarning($"[SimClient] Переподключение... ({e.Message})");
                    Thread.Sleep(1000);
                }
            }
        }
    }

    static void ReadExact(NetworkStream s, byte[] buf, int count)
    {
        int offset = 0;
        while (offset < count)
            offset += s.Read(buf, offset, count - offset);
    }

    // ── Применение пакета ────────────────────────────────────────────────
    void ApplyPacket(DronePacket p)
    {
        _hasState    = true;
        simTime     = p.t;
        faultActive = p.fault != 0;
        landed      = p.landed != 0;

        // ROS (X-forward, Y-left, Z-up)  →  Unity (X-right, Y-up, Z-forward)
        position = new Vector3(p.px, p.pz, p.py);
        desiredPosition = new Vector3(p.pdx, p.pdz, p.pdy);
        startPosition = new Vector3(p.psx, p.psz, p.psy);
        goalPosition  = new Vector3(p.pgx, p.pgz, p.pgy);

        rotation = new Quaternion(-p.qx, p.qz, p.qy, p.qw);

        rotorSpeeds = p.omega_r;

        if (!_markersCreated)
        {
            CreateMarker("StartMarker", startPosition, Color.green);
            CreateMarker("GoalMarker", goalPosition, Color.red);
            _markersCreated = true;
        }

        if (!_obstaclesCreated)
        {
            CreatePlannedObstacles(p.obstacle_types, p.obstacle_params, p.obstacle_count);
            _obstaclesCreated = true;
        }
    }

    void CreateMarker(string name, Vector3 position, Color color)
    {
        var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = name;
        marker.transform.position = position;
        marker.transform.localScale = Vector3.one * 0.45f;

        var mr = marker.GetComponent<Renderer>();
        if (mr != null)
        {
            var mat = new Material(Shader.Find("Standard"));
            mat.color = color;
            mat.EnableKeyword("_EMISSION");
            mat.SetColor("_EmissionColor", color * 1.3f);
            mr.material = mat;
        }
    }

    void CreatePlannedObstacles(int[] types, float[] parameters, int count)
    {
        if (types == null || parameters == null) return;

        int n = Mathf.Min(count, Mathf.Min(types.Length, parameters.Length / 7));
        var root = new GameObject("PlannedObstacles");

        for (int i = 0; i < n; i++)
        {
            int k = i * 7;
            int type = types[i]; // 0=box(cube/rect), 1=cylinder
            float cx = parameters[k + 0];
            float cy = parameters[k + 1];
            float cz = parameters[k + 2];
            float a  = parameters[k + 3];
            float b  = parameters[k + 4];
            float c  = parameters[k + 5];

            // ROS: x,y,z -> Unity: x,z,y
            Vector3 centerUnity = new Vector3(cx, cz, cy);

            GameObject go;
            Vector3 sizeUnity;
            if (type == 1)
            {
                // cylinder: a=radius, b=height
                go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                sizeUnity = new Vector3(2f * a, 0.5f * b, 2f * a);
            }
            else
            {
                // box: a=sx, b=sy, c=sz (ROS)
                go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                sizeUnity = new Vector3(a, c, b);
            }

            go.name = $"PlanObstacle_{i + 1}";
            go.transform.SetParent(root.transform);
            go.transform.position = centerUnity;
            go.transform.localScale = sizeUnity;

            var mr = go.GetComponent<Renderer>();
            if (mr != null)
            {
                var mat = new Material(Shader.Find("Standard"));
                mat.color = new Color(0.62f, 0.62f, 0.72f, 1f);
                mr.material = mat;
            }
        }
    }

    // ── Публичные геттеры для других скриптов ────────────────────────────
    public bool IsConnected => _client != null && _client.Connected;
    public bool HasState => _hasState;
}

/// <summary>Структура JSON-пакета от Python (должна совпадать с build_packet())</summary>
[Serializable]
public class DronePacket
{
    public float   t;
    public int     fault;
    public float   px, py, pz;
    public float   qw, qx, qy, qz;
    public float   psi, theta, phi;
    public float   pdx, pdy, pdz;
    public float   psx, psy, psz;
    public float   pgx, pgy, pgz;
    public int     landed;
    public int     obstacle_count;
    public int[]   obstacle_types;
    public float[] obstacle_params;
    public float[] omega_r;
    public float[] omega_cmd;
}
