# Hexacopter Fault-Tolerant Simulation
## Ubuntu 22.04 + Unity Hub

---

## 1. Python симулятор

### Установка
```bash
sudo apt install python3-pip
pip3 install numpy scipy
```

### Запуск
```bash
cd python_sim/
python3 simulator.py
```

Ты увидишь:
```
[SIM] Запуск симуляции. dt=0.01s, отправка=0.02s
[SIM] Отказ мотора 3 в t=10.0s
[NET] TCP сервер слушает 127.0.0.1:9999
```

---

## 2. Unity проект

### Версия Unity
- **Unity 2022.3 LTS** (через Unity Hub → Install → 2022.3.x LTS)
- Модуль: **Linux Build Support** (опционально)

### Создание проекта
1. Unity Hub → **New Project**
2. Шаблон: **3D (Core)**
3. Название: `HexacopterSim`
4. Нажми **Create project**

### Добавление скриптов
Скопируй все `.cs` файлы из `unity_project/Assets/Scripts/`
в папку `Assets/Scripts/` твоего проекта Unity.

### Настройка сцены

**Вариант А — автоматически:**
1. В Hierarchy создай пустой GameObject → назови `SceneSetup`
2. Добавь компонент `SceneSetup`
3. Нажми **Play** — сцена создастся сама

**Вариант Б — вручную:**

1. **Создай SimManager:**
   - Hierarchy → Create Empty → `SimManager`
   - Add Component → `SimulatorClient`

2. **Создай модель дрона:**
   - Hierarchy → Create Empty → `Hexacopter`
   - Add Component → `DroneController`
   - Назначь `Sim Client` → перетащи `SimManager`

3. **Создай HUD:**
   - Hierarchy → Create Empty → `HUD`
   - Add Component → `DroneHUD`
   - Назначь `Sim Client`

4. **Создай траекторию:**
   - Hierarchy → Create Empty → `TrajectoryVis`
   - Add Component → `TrajectoryVisualizer`
   - Назначь `Sim Client` и `Drone Transform`

5. **Камера:**
   - Выбери `Main Camera`
   - Add Component → `CameraFollow`

---

## 3. Запуск

```
Терминал 1:
    cd python_sim/
    python3 simulator.py

Затем в Unity:
    Нажми ▶ Play
```

Дрон начнёт лететь по спирали.  
В t=10s мотор 3 отказывает → дрон зависает на месте.

---

## 4. Параметры симуляции

Изменяй в `control_allocation.py`:
```python
T_FAIL = {3: 10.0}        # отказ мотора 3 в t=10s
# T_FAIL = {3: 10.0, 6: 15.0}  # два отказа
```

Изменяй в `simulator.py`:
```python
SIM_DT         = 0.01    # шаг физики
SIM_DURATION   = 100.0   # длительность (0 = бесконечно)
```

Изменяй траекторию в `SimulatorClient`:
```python
traj = TrajectoryPlanner(radius=4.0, omega_t=0.1, w_t=1.0)
```

---

## 5. Структура проекта

```
hexacopter_unity/
├── python_sim/
│   ├── simulator.py          ← главный файл запуска
│   ├── uav_params.py         ← параметры из MATLAB
│   ├── dynamics.py           ← физика + кватернионы (RK4)
│   ├── controllers.py        ← LQR + SO(3) attitude
│   ├── control_allocation.py ← fault-tolerant allocation
│   └── trajectory.py         ← спираль / hover
└── unity_project/Assets/Scripts/
    ├── SimulatorClient.cs     ← TCP клиент
    ├── DroneController.cs     ← движение модели
    ├── DroneHUD.cs            ← телеметрия на экране
    ├── TrajectoryVisualizer.cs← визуализация пути
    ├── CameraFollow.cs        ← камера за дроном
    └── SceneSetup.cs          ← авто-создание сцены
```
