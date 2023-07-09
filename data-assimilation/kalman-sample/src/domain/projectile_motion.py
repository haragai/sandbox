import numpy as np

class ProjectileMotion():
    def __init__(self):
        self.m = 1              # 物体質量 [kg]
        self.v0 = 30            # 初速度 [m/s]
        self.theta = np.pi / 4  # 投射角度 [rad]
        self.r = 0.1            # 空気抵抗 [kg/s]
        self.g = 9.80665        # 重力加速度 [m/s/s]
        self.ts = 0             # 開始時刻 [s]
        self.tf = 5             # 終了時刻 [s]
        self.dt = 0.01          # 時間刻み幅 [s]
        self.t = np.arange(self.ts, self.tf, self.dt)   # 時刻[s]
        self.seed = 334         # 乱数シード
        self.s_obse = 2         # 観測値作成のための分散

    # 簡易物理モデルに基づく状態空間モデルの作成
    def state_space_system_model(self):
        v0, theta, g, dt = self.v0, self.theta, self.g, self.dt

        # 状態ベクトルの初期値
        xx_0 = np.array([0, v0*np.cos(theta), 0, 0, v0*np.sin(theta), -g])

        # システム行列
        Ft = np.array([[1, dt, 0, 0,  0,       0],
                       [0,  1, 0, 0,  0,       0],
                       [0,  0, 1, 0,  0,       0],
                       [0,  0, 0, 1, dt, dt**2/2],
                       [0,  0, 0, 0,  1,      dt],
                       [0,  0, 0, 0,  0,       1]])

        # システムノイズ行列
        Gt = np.ones_like(Ft)

        # システムノイズの分散共分散行列
        Qt = np.array([[.01,   0,   0,   0,   0,   0],
                       [  0, .01,   0,   0,   0,   0],
                       [  0,   0, .01,   0,   0,   0],
                       [  0,   0,   0, .01,   0,   0],
                       [  0,   0,   0,   0, .01,   0],
                       [  0,   0,   0,   0,   0, .01]])

        # 観測モデル行列
        Ht = np.array([[1, 0, 0, 0, 0, 0],
                       [0, 0, 0, 1, 0, 0]])

        # 観測ノイズ
        Rt = np.array([[3, 0],
                       [0, 3]])

        return xx_0, Ft, Gt, Qt, Ht, Rt

    # 簡易物理モデルに基づく位置作成（プロット用途にのみ使用）
    def rouph_position_generator(self):
        v0, theta, g, t = self.v0, self.theta, self.g, self.t
        x_rouph = v0*np.cos(theta) * t
        y_rouph = v0*np.sin(theta) * t - 1/2 * g * t**2

        return x_rouph, y_rouph

    # 詳細物理モデルに基づく位置作成（プロット用途にのみ使用）
    def elaborate_position_generator(self):
        m, v0, theta, r, g, t = self.m, self.v0, self.theta, self.r, self.g, self.t
        x_elab = m*v0 /r * (1 - np.exp(-r/m * t)) * np.cos(theta)
        y_elab = m/r * ((v0*np.sin(theta) + m/r *g) * (1 - np.exp(-r/m *t)) - g*t)

        return  x_elab, y_elab

    # 詳細物理モデルにガウスノイズを加えて観測位置作成
    def observed_position_generator(self):
        x_elab, y_elab = self.elaborate_position_generator()
        np.random.seed(self.seed)  # 乱数シード設定
        x_obse = x_elab + np.random.normal(0, self.s_obse, len(x_elab))
        y_obse = y_elab + np.random.normal(0, self.s_obse, len(y_elab))

        return x_obse, y_obse