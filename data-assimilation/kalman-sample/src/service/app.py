import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import numpy as np

import src.service.plot_position as PP
import src.domain.kalman_filter as KF
import src.domain.projectile_motion as PM

def app():
    sample_projectile_motion = PM.ProjectileMotion()
    x_rouph, y_rouph = sample_projectile_motion.rouph_position_generator()
    x_elab, y_elab = sample_projectile_motion.elaborate_position_generator()
    x_obse, y_obse = sample_projectile_motion.observed_position_generator()
    # 観測値を2次元配列に変更
    yy = np.array([ [x_obse[idx], y_obse[idx]] for idx in range(len(x_obse)) ])
    
    xx0, Ft, Gt, Qt, Ht, Rt = sample_projectile_motion.state_space_system_model()
    
    num = len(sample_projectile_motion.t)
    sample_kalman_filter = KF.KalmanFilter(xx0, Ft, Gt, Qt, yy, Ht, Rt)
    xx, ll = sample_kalman_filter.state_and_likelihood_estimation(num)

    sample_position_plotter = PP.PlotPosition()
    sample_position_plotter.plot_result(x_rouph, y_rouph, x_elab, y_elab, x_obse, y_obse, xx, ll)

    return 0

if __name__ == '__main__':
    app()
