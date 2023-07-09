import numpy as np

class KalmanFilter():
    def __init__(self, xx_0, Ft, Gt, Qt, y, Ht, Rt):
        self.x = xx_0.copy()  # 状態ベクトルの初期値
        self.V = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])                            
                            # 状態ベクトルの初期値の分散共分散行列

        self.Ft = Ft        # 時刻tのシステム行列
        self.Gt = Gt        # 時刻tのシステムノイズ行列
        self.Qt = Qt        # 時刻tのシステムモデルの分散共分散行列
        self.y = y          # 観測ベクトル（を時系列に行列化）
        self.Ht = Ht        # 時刻tの観測モデル行列
        self.Rt = Rt        # 時刻tの観測モデルの分散共分散行列

        self.ll2 = 0        # 対数尤度関数 第2項
        self.ll3 = 0        # 対数尤度関数 第3項

    def state_and_likelihood_estimation(self, num):
        x, V, Ft, Gt, Qt, y, Ht, Rt, ll2, ll3 = self.x, self.V, self.Ft, self.Gt, self.Qt, self.y, self.Ht, self.Rt, self.ll2, self.ll3

        # 各時刻の状態変数用意
        xx = np.empty((num, len(x)))
        xx[0] = x.copy()

        for i in range(num-1):
            # 予測
            x = Ft @ x
            V = Ft @ V @ Ft.T + Gt @ Qt @ Gt.T

            # 途中尤度計算
            e = y[i+1] - Ht @ x
            d = Ht @ V @ Ht.T + Rt
            ll2 += np.log((np.linalg.det(d)))
            ll3 += e.T @ np.linalg.inv(d) @ e

            # 更新則
            Kt = V @ Ht.T @ np.linalg.inv(Ht @ V @ Ht.T + Rt)
            x = x + Kt @ (y[i+1] - Ht @ x)
            V = V - Kt @ Ht @ V

            xx[i+1] = x.copy()

        # 尤度
        ll = -1/2 * (y.shape[1]*num*np.log(2*np.pi) + ll2 + ll3)

        return xx, ll