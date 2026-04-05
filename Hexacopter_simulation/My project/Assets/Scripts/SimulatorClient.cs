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
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 desiredPosition;
    public float[] rotorSpeeds = new float[6];

    // Внутреннее состояние
    private TcpClient       _client;
    private Thread          _recvThread;
    private ConcurrentQueue<DronePacket> _queue = new();
    private bool            _running;

    // ── Lifecycle ────────────────────────────────────────────────────────
    void Start()
    {
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
        simTime     = p.t;
        faultActive = p.fault != 0;

        // ROS (X-forward, Y-left, Z-up)  →  Unity (X-right, Y-up, Z-forward)
        position = new Vector3(p.px, p.pz, p.py);
        desiredPosition = new Vector3(p.pdx, p.pdz, p.pdy);

        // Кватернион: ROS [w,x,y,z] → Unity с учётом смены рук СК
        rotation = new Quaternion(-p.qx, p.qz, p.qy, p.qw);

        rotorSpeeds = p.omega_r;
    }

    // ── Публичные геттеры для других скриптов ────────────────────────────
    public bool IsConnected => _client != null && _client.Connected;
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
    public float[] omega_r;
    public float[] omega_cmd;
}
