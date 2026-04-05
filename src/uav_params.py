import numpy as np

class UAVParams:
    """Параметры гексакоптера — прямой перевод из MATLAB main.m"""

    # Основные
    Nr   = 6
    m    = 1.2        # кг
    g    = 9.81
    b    = 0.215      # м, длина плеча

    # Инерция
    J    = np.diag([0.01, 0.01, 0.018])  # кг·м²
    Jinv = np.linalg.inv(J)

    # Аэродинамическое демпфирование
    D_fv       = np.diag([0.055, 0.055, 0.022])
    D_tauomega = np.diag([0.48,  0.48,  2.37])
    D          = np.block([[D_fv,           np.zeros((3,3))],
                           [np.zeros((3,3)), D_tauomega]])

    # Моторы
    Omega_max = 10300 * 2*np.pi / 60   # рад/с
    Omega_min =  1260 * 2*np.pi / 60
    k_m       = 1 / 0.05               # 1/с
    k_f       = 3.65e-6                # Н/(рад/с)²
    sigma     = 0.09                   # м, torque-to-thrust

    # Посылка
    m_box = 0.4
    r_c   = np.array([0.01, -0.01, -0.075])

    # Генерация матрицы входов F (6×6)
    @staticmethod
    def build_input_map():
        """
        Воспроизводит MATLAB:
          F = [e e e e e e; F_low]  → shape 4×6
          строка 0: тяга   [1 1 1 1 1 1]
          строки 1-3: моменты tau_x, tau_y, tau_z
        """
        b     = UAVParams.b
        sigma = UAVParams.sigma
        chi   = np.array([-1, 1, -1, 1, -1, 1])
        gamma = np.linspace(0, 5*np.pi/3, 6)
        e     = np.array([0.0, 0.0, 1.0])

        pos = b * np.array([np.cos(gamma), np.sin(gamma), np.zeros(6)])  # 3×6

        F_low = np.zeros((3, 6))
        for i in range(6):
            px = pos[:, i]
            cross = np.array([[ 0,    -px[2],  px[1]],
                               [ px[2], 0,     -px[0]],
                               [-px[1], px[0],  0    ]])
            F_low[:, i] = (cross - chi[i]*sigma*np.eye(3)) @ e

        # MATLAB: F=[e e e e e e; F_low] → 4×6
        thrust_row = np.ones((1, 6))          # e[2]=1 для всех роторов
        F = np.vstack([thrust_row, F_low])    # 4×6
        return F

    @staticmethod
    def total_mass_matrix():
        """M_tot с учётом посылки"""
        m, r = UAVParams.m, UAVParams.r_c
        J    = UAVParams.J
        S_bg = m * np.array([[ 0,    -0,      0  ],
                              [ 0,     0,     -0  ],
                              [ 0,     0,      0  ]])   # r_bg = 0
        M_uav = np.block([[m*np.eye(3), S_bg.T],
                          [S_bg,        J     ]])

        mb   = UAVParams.m_box
        xc,yc,zc = r
        S    = mb * np.array([[ 0,   -zc,  yc],
                               [ zc,   0,  -xc],
                               [-yc,  xc,   0 ]])
        Jb   = (1/6)*mb*0.2**2*np.eye(3)
        Jb  += mb * np.array([[yc**2+zc**2, -xc*yc,     -xc*zc    ],
                               [-yc*xc,      xc**2+zc**2,-yc*zc    ],
                               [-zc*xc,     -zc*yc,      xc**2+yc**2]])
        M_box = np.block([[mb*np.eye(3), S.T],
                          [S,            Jb ]])
        return M_uav + M_box
