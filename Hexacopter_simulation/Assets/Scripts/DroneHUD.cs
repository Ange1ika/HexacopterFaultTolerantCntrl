using UnityEngine;

/// <summary>
/// HUD с телеметрией.
/// Прикрепи к любому GameObject. Назначь SimClient.
/// </summary>
public class DroneHUD : MonoBehaviour
{
    public SimulatorClient simClient;

    // Цвета
    private readonly Color _normal = new Color(0.2f, 1f, 0.2f, 1f);   // зелёный
    private readonly Color _fault  = new Color(1f, 0.2f, 0.2f, 1f);   // красный
    private GUIStyle _style;

    void OnGUI()
    {
        if (simClient == null) return;
        if (_style == null)
        {
            _style = new GUIStyle(GUI.skin.label)
            {
                fontSize  = 16,
                fontStyle = FontStyle.Bold
            };
        }

        bool fault = simClient.faultActive;
        _style.normal.textColor = fault ? _fault : _normal;

        float x = 20, y = 20, w = 340, lh = 22;

        // Фон
        GUI.Box(new Rect(x - 5, y - 5, w + 10, lh * 12 + 10), "");

        GUI.Label(new Rect(x, y, w, lh),
            $"Время симуляции: {simClient.simTime:F1} s", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            fault ? "⚠ FAULT — HOVER MODE" : "✓ Нормальный полёт", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            $"Позиция:  x={simClient.position.x:F2}  y={simClient.position.y:F2}  z={simClient.position.z:F2}", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            $"Цель:     x={simClient.desiredPosition.x:F2}  y={simClient.desiredPosition.y:F2}  z={simClient.desiredPosition.z:F2}", _style); y += lh;

        Vector3 euler = simClient.rotation.eulerAngles;
        GUI.Label(new Rect(x, y, w, lh),
            $"Крен: {euler.z:F1}°  Тангаж: {euler.x:F1}°  Рыск: {euler.y:F1}°", _style); y += lh;

        // Скорости роторов
        GUI.Label(new Rect(x, y, w, lh), "Роторы (рад/с):", _style); y += lh;
        if (simClient.rotorSpeeds != null)
        {
            for (int i = 0; i < simClient.rotorSpeeds.Length; i++)
            {
                float spd = simClient.rotorSpeeds[i];
                bool  dead = spd < 1f;
                _style.normal.textColor = dead ? _fault : _normal;
                GUI.Label(new Rect(x + (i % 3) * 110, y + (i / 3) * lh,
                    110, lh), $"M{i+1}: {spd:F0}", _style);
            }
            _style.normal.textColor = fault ? _fault : _normal;
            y += lh * 2 + 4;
        }

        // Статус подключения
        _style.normal.textColor = simClient.IsConnected
            ? _normal : new Color(1f, 0.6f, 0f);
        GUI.Label(new Rect(x, y, w, lh),
            simClient.IsConnected ? "● Python: подключён" : "○ Python: ожидание...", _style);
    }
}
