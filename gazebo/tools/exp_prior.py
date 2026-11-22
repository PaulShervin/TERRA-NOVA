import sys, math, numpy as np
sys.path.insert(0,'../ros2_ws/src/mining_sim_gz')
from mining_sim_gz.cloud_utils import level_points, scan_points_2d_vertical, quat_to_matrix
from mining_sim_gz.se2 import between, wrap
from mining_sim_gz.slam_core import SlamEstimator, odometry_noise_model
d=np.load('../out/rec_explore.npz',allow_pickle=True); t,gt,od,imu,xyz=d['t'],d['gt'],d['odom'],d['imu'],d['xyz']
imuyaw=np.array([math.atan2(*quat_to_matrix(*q)[[1,0],0]) for q in imu])
pts=[scan_points_2d_vertical(level_points(xyz[i].astype(float),tuple(imu[i]))) for i in range(len(t))]
def run(scale=0.02,gyro_bias=0.0004,**kw):
    rng=np.random.default_rng(1); slam=SlamEstimator(**kw); slam.init(gt[0,[0,1,3]]); prev=od[0,[0,1,3]]
    for i in range(len(t)):
        cur=od[i,[0,1,3]]; dt=0.2
        dyaw=(wrap(imuyaw[i]-imuyaw[i-1]) if i else 0.0)+gyro_bias*dt+rng.normal()*0.0009*math.sqrt(0.01*dt)
        delta=odometry_noise_model(between(prev,cur),rng,scale_error=scale,yaw_scale=0,yaw_bias_per_m=0,yaw_sigma=0); delta[2]=dyaw; prev=cur
        slam.update(delta,pts[i],t[i]-t[0],truth=gt[i,[0,1,3]])
    return slam.metrics()
for label,kw in (('good robot (2% scale, bias 0.0004)',dict()),('worse robot (6% scale, bias 0.004)',dict(scale=0.06,gyro_bias=0.004))):
    print('---',label)
    for r in (0.01,0.005,0.003,0.002):
        for along0,perm in ((0.02,0.05),(0.01,0.03)):
            m=run(prior_sigma_along0=along0,prior_sigma_along_per_m=perm,prior_sigma_r=r,**kw)
            print(f'rot sigma {r:5.3f}  along {along0}+{perm}/m : ATE slam {m["ate"]:.2f} | odom-only {m["ate_odom"]:.2f} | final {m["final_error"]:.2f} vs {m["final_error_odom"]:.2f}')
