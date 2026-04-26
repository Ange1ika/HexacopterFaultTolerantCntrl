using UnityEngine;

public class DroneHUD : MonoBehaviour
{
    public SimulatorClient simClient;

    private readonly Color _normal = new Color(0.2f, 1f, 0.2f, 1f);
    private readonly Color _fault  = new Color(1f, 0.2f, 0.2f, 1f);
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

        bool  fault = simClient.faultActive;
        float x = 20, y = 20, w = 340, lh = 22;

        // Считаем высоту заранее чтобы фон не выходил за контент:
        //   5 строк текста + заголовок роторов + 2 ряда роторов + строка подключения
        float boxH = lh * 5          // simTime, fault, pos, desired, euler
                   + lh              // "Роторы:"
                   + lh * 2 + 4     // два ряда роторов (M1-M3, M4-M6)
                   + lh             // статус подключения
                   + 14;            // отступы (padding top+bottom)

        GUI.Box(new Rect(x - 5, y - 5, w + 10, boxH), "");

        _style.normal.textColor = fault ? _fault : _normal;

        GUI.Label(new Rect(x, y, w, lh),
            $"Время: {simClient.simTime:F1} s", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            fault ? "⚠ FAULT — HOVER MODE" : "✓ Нормальный полёт", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            $"Позиция:  {simClient.position.x:F2}  {simClient.position.y:F2}  {simClient.position.z:F2}", _style); y += lh;

        GUI.Label(new Rect(x, y, w, lh),
            $"Цель:     {simClient.desiredPosition.x:F2}  {simClient.desiredPosition.y:F2}  {simClient.desiredPosition.z:F2}", _style); y += lh;

        Vector3 euler = simClient.rotation.eulerAngles;
        GUI.Label(new Rect(x, y, w, lh),
            $"Крен: {euler.z:F1}°  Тангаж: {euler.x:F1}°  Рыск: {euler.y:F1}°", _style); y += lh;

        // Роторы
        GUI.Label(new Rect(x, y, w, lh), "Роторы (рад/с):", _style); y += lh;
        if (simClient.rotorSpeeds != null)
        {
            for (int i = 0; i < Mathf.Min(simClient.rotorSpeeds.Length, 6); i++)
            {
                float spd  = simClient.rotorSpeeds[i];
                _style.normal.textColor = spd < 1f ? _fault : _normal;
                GUI.Label(
                    new Rect(x + (i % 3) * 110, y + (i / 3) * lh, 110, lh),
                    $"M{i + 1}: {spd:F0}", _style);
            }
            y += lh * 2 + 4;
        }

        // Статус подключения
        _style.normal.textColor = simClient.IsConnected
            ? _normal : new Color(1f, 0.6f, 0f);
        GUI.Label(new Rect(x, y, w, lh),
            simClient.IsConnected ? "● Python: подключён" : "○ Python: ожидание...", _style);
    }
}

