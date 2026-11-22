import sys, math, numpy as np
sys.path.insert(0,'../ros2_ws/src/mining_sim_gz')
from mining_sim_gz.cloud_utils import level_points, scan_points_2d_vertical, scan_points_2d, relative_heights, quat_to_matrix, vertical_structure_mask
from mining_sim_gz.se2 import between, wrap
from mining_sim_gz.slam_core import SlamEstimator, odometry_noise_model
d=np.load('../out/rec_explore.npz',allow_pickle=True); t,gt,od,imu,xyz=d['t'],d['gt'],d['odom'],d['imu'],d['xyz']
imuyaw=np.array([math.atan2(*quat_to_matrix(*q)[[1,0],0]) for q in imu]); N=200
lv=[level_points(xyz[i].astype(float),tuple(imu[i])) for i in range(N)]
def sel_old(l): return scan_points_2d(l,relative_heights(l))
def sel_vert(l): return scan_points_2d_vertical(l)
def sel_band(l):  # plain height band in the levelled sensor frame: 0.25..1.7 m above the floor assuming a flat floor 0.972 m below
    ok=(l[:,2]>-0.72)&(l[:,2]<0.73)&(np.hypot(l[:,0],l[:,1])<20); p=l[ok,:2]; return p[::max(1,len(p)//700)]
def sel_vert_strict(l): return scan_points_2d_vertical(l,min_extent=0.4,z_min=-0.6,z_max=0.7)
def run(sel, **kw):
    rng=np.random.default_rng(1); slam=SlamEstimator(**kw); slam.init(gt[0,[0,1,3]]); prev=od[0,[0,1,3]]
    for i in range(N):
        cur=od[i,[0,1,3]]; dyaw=wrap(imuyaw[i]-imuyaw[i-1]) if i else 0.0
        delta=odometry_noise_model(between(prev,cur),rng,yaw_scale=0,yaw_bias_per_m=0,yaw_sigma=0); delta[2]=dyaw; prev=cur
        slam.update(delta,sel(lv[i]),t[i]-t[0],truth=gt[i,[0,1,3]])
    m=slam.metrics(); e=slam.pose
    return f'ATE {m["ate"]:.2f} (odom {m["ate_odom"]:.2f}) final yaw err {math.degrees(wrap(e[2]-gt[N-1,3])):.1f} deg  fit {np.mean([s.fit for s in slam.samples]):.2f}'
for name,sel in (('old rel-height band',sel_old),('vertical-extent (new)',sel_vert),('flat height band',sel_band),('vertical strict',sel_vert_strict)):
    print(f'{name:24s}', run(sel))
