import matplotlib.pyplot as plt

class PlotPosition():
    def __init__(self):
        # figとaxes用意
        self.fig_kf = plt.figure(figsize=(12.8, 4.8), tight_layout=True)
        self.gs = self.fig_kf.add_gridspec(2, 2)
        self.ax_xy = self.fig_kf.add_subplot(self.gs[:, 0])
        self.ax_y = self.fig_kf.add_subplot(self.gs[0, 1])
        self.ax_x = self.fig_kf.add_subplot(self.gs[1, 1])

    def plot_result(self, x_rouph, y_rouph, x_elab, y_elab, x_obse, y_obse, xx, ll):
        # xy座標
        self.ax_xy.plot(x_obse, y_obse, marker='^', lw=0, label='observed data')
        self.ax_xy.plot(x_elab, y_elab, label='elaborate model')
        self.ax_xy.plot(x_rouph, y_rouph, label='simple model')
        self.ax_xy.plot(xx[:, 0], xx[:, 3], marker='.', lw=0, label='kalman filter')
        self.ax_xy.set_title('Likelihood: ' + str(ll))
        self.ax_xy.set_xlabel('x[m]')
        self.ax_xy.set_ylabel('y[m]')
        self.ax_xy.legend()
        self.ax_xy.grid()

        # x軸y軸それぞれ
        self.ax_x.plot(x_obse, label='x_observed', color='tab:blue')
        self.ax_x.plot(xx[:, 0], label='x_kalmanfiler', color='tab:red')
        #self.ax_x.set_title('Likelihood: ' + str(ll))
        self.ax_x.set_ylabel('x[m]')
        self.ax_x.legend()
        self.ax_x.grid()
        self.ax_y.plot(y_obse, label='y_observed', color='tab:blue')
        self.ax_y.plot(xx[:, 3], label='y_kalmanfiler', color='tab:red')
        self.ax_y.set_ylabel('y[m]')
        self.ax_y.legend()
        self.ax_y.grid()

        # プロット
        plt.show()

        return 0