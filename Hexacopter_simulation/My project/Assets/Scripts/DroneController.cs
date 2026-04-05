using UnityEngine;

/// <summary>
/// Двигает 3D модель дрона по данным от SimulatorClient.
/// Прикрепи к GameObject дрона. Назначь SimClient в инспекторе.
/// </summary>
public class DroneController : MonoBehaviour
{
    [Header("References")]
    public SimulatorClient simClient;

    [Header("Rotor Objects (6 штук)")]
    public Transform[] rotors = new Transform[6];

    [Header("Visual")]
    public GameObject faultIndicator;      // красный объект при отказе
    public TrailRenderer trailRenderer;    // след траектории

    [Header("Settings")]
    public float rotorVisualSpeed = 0.5f;  // множитель скорости вращения лопастей
    public bool  smoothing        = true;
    public float smoothFactor     = 15f;

    private Vector3    _targetPos;
    private Quaternion _targetRot;
    private bool       _wasInFault;

    void Start()
    {
        _targetPos = transform.position;
        _targetRot = transform.rotation;

        if (faultIndicator) faultIndicator.SetActive(false);
    }

    void Update()
    {
        if (simClient == null) return;

        // ── Позиция и ориентация ─────────────────────────────────────────
        _targetPos = simClient.position;
        _targetRot = simClient.rotation;

        if (smoothing)
        {
            transform.position = Vector3.Lerp(
                transform.position, _targetPos, Time.deltaTime * smoothFactor);
            transform.rotation = Quaternion.Slerp(
                transform.rotation, _targetRot, Time.deltaTime * smoothFactor);
        }
        else
        {
            transform.position = _targetPos;
            transform.rotation = _targetRot;
        }

        // ── Вращение лопастей ────────────────────────────────────────────
        for (int i = 0; i < rotors.Length && i < simClient.rotorSpeeds.Length; i++)
        {
            if (rotors[i] == null) continue;
            float rpm = simClient.rotorSpeeds[i] * rotorVisualSpeed * Mathf.Rad2Deg;
            // Чётные — по часовой, нечётные — против
            float dir = (i % 2 == 0) ? 1f : -1f;
            rotors[i].Rotate(Vector3.up, dir * rpm * Time.deltaTime);
        }

        // ── Индикатор отказа ─────────────────────────────────────────────
        bool fault = simClient.faultActive;
        if (faultIndicator) faultIndicator.SetActive(fault);

        if (fault && !_wasInFault)
        {
            Debug.Log($"[Drone] ОТКАЗ МОТОРА в t={simClient.simTime:F1}s @ {transform.position}");
            if (trailRenderer) trailRenderer.startColor = Color.red;
        }
        _wasInFault = fault;
    }
}
