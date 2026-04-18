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

## 3. Запуск

```
Терминал 1:
    cd python_sim/
    python3 simulator.py

Затем в Unity:
    Нажми ▶ Play
```


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
