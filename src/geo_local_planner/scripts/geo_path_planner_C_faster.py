#!/usr/bin/env python3
from casadi import *
from casadi import DM, cos, sin, fabs, if_else, logic_and
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.transforms import Affine2D
import math
from matplotlib.cm import get_cmap
from matplotlib.patches import Ellipse
import matplotlib.ticker as ticker
import time

x  = MX.sym('x', 1)  # state
u  = MX.sym('u', 4)  # control

#-------fine tuning parameters-------
X_val = 4 #sensor detection range
ns = 40  #number of point
x_stepl = 0.2 #step length
trav_dist = 70 #travel distance
#n_obs = 7 # number of obstacles
#n_obs = MX.sym('n_obs',1)
#----------------------------------

#障碍物例子 （问题定义部分的例图）
# obs_val = np.array([4.2, 0.3,  0.7,  0.5 , np.pi/6,
#                     7,  0.4,  0.7,  0.3, np.pi/4,
#                     11.58, -0.81,  0.7,  0.4, np.pi/3,
#                     10, 10, 0.01, 0.01,0,
#                     10, 10, 0.01, 0.01,0])

#障碍物case1，多个障碍物,交叉集中在两侧
# obs_val = np.array([2.3, 0.4, 0.5, 0.5,0,
#                    5.2, -0.4, 0.7,  0.5 , np.pi/6,
#                    8.5, 0.4, 0.6, 0.6, np.pi/2,
#                    12, -0.4, 0.7, 0.3, np.pi/4,])
                   #11, -0.4, 1, 0.5, 0,
                   #13, -0.4, 0.7, 0.4, np.pi/3,])

# 障碍物case2，多个障碍物,集中在单侧
# obs_val = np.array([2.3, 0.4, 0.5, 0.5,0,
#                    4.2, 0.4, 0.7,  0.5 , np.pi/6,
#                    6.5, 0.4, 0.6, 0.6, np.pi/2,
#                    8, 0.4, 0.7, 0.3, np.pi/4,
#                    10.5, 0.4, 1, 0.5, 0,
#                    12, 0.4, 0.7, 0.4, np.pi/3,])


# 障碍物case3，自然分布
# obs_val = np.array([3.2, 2, 0.5, 0.5,0,
#                     4, -2.5, 3, 0.6, 0,
#                     4.2, 0.3, 0.7,  0.5 , np.pi/6,
#                     5.5, 1.5, 0.6, 0.6, np.pi/2,
#                     7, 2.4, 0.7, 0.3, np.pi/4,
#                     9, -1, 1, 0.5, 0,
#                     11.58, -0.81, 0.7, 0.4, np.pi/3,])

# 障碍物case4，自然分布
# obs_val = np.array([
#                     3.2, 1, 0.8, 0.5,np.pi/7,
#                     4.2, 0.5, 0.7,  0.5 , np.pi/6,
#                     5,2.3,1,0.5,0,
#                     7,-1.5,0.6,0.6, np.pi/2,
#                     8, 0, 1, 0.6, 0,
#                     #5.5, 1.5, 0.6, 0.6, np.pi/2,
#                     #7, 2.4, 0.7, 0.3, np.pi/4,
#                     #9, -1, 1, 0.5, 0,
#                     11.58, -0.81, 0.7, 0.4, np.pi/3,])

# 障碍物case5，自然分布

# obs_val = np.array([
#                     3.2, 1, 0.6, 0.2,np.pi/2,
#                     #4.2, 0.5, 0.7,  0.5 , np.pi/6,
#                     7, 0, 1, 0.6, 0,
#                     5.5, -1.5, 0.6, 0.6, np.pi/2,
#                     #7, 2.4, 0.7, 0.3, np.pi/4,
#                     9, -1.2, 1, 0.3, np.pi/2,                    
#                     11.58, -0.81, 0.9, 0.4, np.pi/3,
#                     11,2,0.5,0.7,0,
#                     ])

# 障碍物case6，自然分布,行道树
obs_val = np.array([                       
                    1, 2, 0.5, 0.5,0,
                    1,-2,0.5,0.5,0,
                    4, 2, 0.5, 0.5,0,
                    4,-2,0.5,0.5,0,
                    4,0.8,1,0.5,0,
                    7, 2, 0.5, 0.5,0,
                    7,-2,0.5,0.5,0,
                    8.5,1.5,1,1.3,0,
                    10, 2, 0.5, 0.5,0,
                    10,-2,0.5,0.5,0,
                    12,-0.8,0.8,0.3,np.pi/6,
                    13, 2, 0.5, 0.5,0,
                    13,-2,0.5,0.5,0,
                    ])

#障碍物例子，比较N




#障碍物例子，多个障碍物,分别集中在两侧
# obs_val = np.array([2.3, 0.4, 0.5, 0.5,0,
#                    4.2, 0.4, 0.7,  0.5 , np.pi/6,
#                    6.5, 0.4, 0.6, 0.6, np.pi/2,
#                    10, -0.4, 0.7, 0.3, np.pi/4,
#                    11, -0.4, 1, 0.5, 0,
#                    13, -0.4, 0.7, 0.4, np.pi/3,])

#障碍物例子，多个障碍物,置于中线(nicht so gut)
# obs_val = np.array([ 3, 0, 0.5, 0.5,0,
#                    4, 0, 0.7,  0.5 , np.pi/6,
#                    8.5, 0, 0.6, 0.6, np.pi/2,
#                    10, 0, 0.7, 0.3, np.pi/4,
#                    11, -0.4, 1, 0.5, 0,
#                    13, -0.4, 0.7, 0.4, np.pi/3,])

obs_dm = DM(obs_val.reshape((-1, 1)))  # 形状 [5*n_obs, 1]
n_obs = obs_dm.numel() // 5

                 

X  = MX.sym('X',1) # Follow the length of the path
dx = X/ns # length between two point
y_star = MX.sym('y_star',1)
k_star = MX.sym('k_star',1)
x_star = MX.sym('x_star',1)
safe_dis = MX.sym('safe_dis',1)
car_width = MX.sym('car_width',1)
global_coeff = MX.sym('global_coeff',4) # global_path_coeff
rt_coeff = MX.sym('rt_coeff',4) # rt_side
lt_coeff = MX.sym('lt_coeff',4) # lt_side
#obs = MX.sym('obs',5)
obs = MX.sym('obs',5*n_obs)

y_local = u[0] * x ** 3 + u[1] * x ** 2 + u[2] * x + u[3]
k = 3 * u[0] * x ** 2 + 2 * u[1] * x + u[2]
k2 = 6 * u[0] * x + 2 * u[1]
K = fabs(k2) / ((1 + k ** 2) ** (3 / 2)) #curvature
S = sqrt(1 + k ** 2)

f_up_y = y_local + (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_down_y = y_local - (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_up_x = x - (((car_width / 2) * k) / sqrt(k ** 2 + 1))
f_down_x = x + (((car_width / 2) * k) / sqrt(k ** 2 + 1))


#------Create CasADi functions------
y_fn = Function('y_fn', [x, u], [y_local])
k_fn = Function('k_fn', [x, u], [k])
k2_fn = Function('k2_fn', [x, u], [k2])
K_fn = Function('K_fn', [x, u], [K])
S_fn = Function('S_fn', [x, u], [S])
f_up_y_fn = Function('f_up_y_fn', [x, u,car_width], [f_up_y])
f_down_y_fn = Function('f_down_y_fn', [x, u,car_width], [f_down_y])
f_up_x_fn = Function('f_up_x_fn', [x, u,car_width], [f_up_x])
f_down_x_fn = Function('f_down_x_fn', [x, u,car_width], [f_down_x])

# ziel_fun J
coeff_diff = [u[0] - global_coeff[0], 
              u[1] - global_coeff[1], 
              u[2] - global_coeff[2], 
              u[3] - global_coeff[3]]
inter_diff_sq = (pow(coeff_diff[0], 2) / 7) * pow(x, 7) + \
                ((2 * coeff_diff[0] * coeff_diff[1] / 6) * pow(x, 6)) + \
                (((2 * coeff_diff[0] * coeff_diff[2] + pow(coeff_diff[1], 2)) / 5) * pow(x, 5)) + \
                (((2 * coeff_diff[0] * coeff_diff[3] + 2 * coeff_diff[1] * coeff_diff[2]) / 4) * pow(x, 4)) + \
                (((2 * coeff_diff[1] * coeff_diff[3] + pow(coeff_diff[2], 2)) / 3) * pow(x, 3)) + \
                (((2 * coeff_diff[2] * coeff_diff[3]) / 2) * pow(x, 2)) + \
                (pow(coeff_diff[3], 2) * x)
intergral = Function('intergral', [x, u, global_coeff], [inter_diff_sq])
J =(intergral(x_star+X, u, global_coeff) - intergral(x_star, u ,global_coeff))

# constrains
y_lt = lt_coeff[0]*pow(x,3) + lt_coeff[1]*pow(x,2) + lt_coeff[2]*pow(x,1) + lt_coeff[3]*pow(x,0)
y_rt = rt_coeff[0]*pow(x,3) + rt_coeff[1]*pow(x,2) + rt_coeff[2]*pow(x,1) + rt_coeff[3]*pow(x,0)
y_lt_fn = Function('y_lt_fn', [x, lt_coeff], [y_lt])
y_rt_fn = Function('y_rt_fn', [x, rt_coeff], [y_rt])

g_1 = []
g_2 = []
g_3 = []

# 预计算每个障碍物的常量参数（包含外扩半轴与在全局x轴上的半投影）
obs_parsed = []
for j in range(n_obs):
    idx = 5*j
    xc = obs_dm[idx + 0]
    yc = obs_dm[idx + 1]
    a  = obs_dm[idx + 2]
    b  = obs_dm[idx + 3]
    th = obs_dm[idx + 4]

    c = cos(th)
    s = sin(th)

    ap = a + car_width/2 + safe_dis    # 外扩后的主半轴
    bp = b + car_width/2 + safe_dis    # 外扩后的次半轴

    # 在全局 x 轴上的半投影：|cosθ|*ap + |sinθ|*bp
    x_extent = fabs(c)*ap + fabs(s)*bp

    obs_parsed.append((xc, yc, ap, bp, c, s, x_extent))



for i in range(ns):
    x_val = x_star + i*dx

    #safe distance
    #g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(x_val, lt_coeff) + safe_dis
    #g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(x_val, rt_coeff) - safe_dis
    g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(f_up_x_fn(x_val,u,car_width), lt_coeff) + safe_dis
    g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(f_down_x_fn(x_val,u,car_width), rt_coeff) - safe_dis

    #curvature
    g_1.extend([g_1_up, g_1_down]) 
    g3 = 2 - K_fn(x_val,u)
    g_3.append(g3)

    # for j in range(n_obs):
    #     g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
    #     g_2.append(g2)

    x_min = x_val
    x_max = x_val + X_val
    y_val = y_fn(x_val, u)  

    for (xc, yc, ap, bp, c, s, x_extent) in obs_parsed:
        # 与窗口 [x_min, x_max] 是否有重叠（符号布尔）
        left_ok  = (xc + x_extent) >= x_min
        right_ok = (xc - x_extent) <= x_max
        overlap  = logic_and(left_ok, right_ok)

        # 窗口内=1，窗口外=0（符号条件，避免 Python if）
        w = if_else(overlap, 1.0, 0.0)

        # 旋转到障碍物坐标系
        xp = (x_val - xc)*c + (y_val - yc)*s
        yp = (x_val - xc)*s - (y_val - yc)*c

        # 椭圆外部约束：xp^2/ap^2 + yp^2/bp^2 >= 1
        g2 = (xp**2)/(ap**2) + (yp**2)/(bp**2) - 1.0

        # 仅在窗口内生效
        g_2.append(w * g2)


#obstacles avoidance
#g_2 = []
#for j in range(n_obs):
#    x_obs = obs[0+5*j]
#    y_obs = obs[1+5*j]
#    r_obs = obs[2+5*j]
#    x_vals = casadi.linspace(x_obs - r_obs, x_obs + r_obs, 10)  # 生成 CasADi 符号范围
#    x_vals_list = casadi.vertsplit(x_vals)  # 将符号矩阵拆分为列表
#    for x_val in x_vals_list:
#        g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
#        g_2.append(g2)
# g_2 = []
# for i in range(ns):
#     x_val = x_star + i*dx
#     for j in range(n_obs):
#         x_val = x_star + i*dx
#         g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
#         g_2.append(g2)


g_4 = y_fn(x_star,u) - y_star
g_5 = k_fn(x_star,u) - k_star



# nlp proble definieren
nlp = []
#nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1,*g_2,*g_3,g_4,g_5)} 
nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1, *g_2, *g_3,g_4,g_5)} 
opts = {}
ipopt_opts = {}
ipopt_opts["tol"] = 1e-4;
ipopt_opts["max_iter"] =50; #改了 原先是100
ipopt_opts["print_level"] = 1;
ipopt_opts["sb"] = "yes";
ipopt_opts["acceptable_tol"] = 1e-4;
ipopt_opts["acceptable_iter"] = 0;
# ipopt_opts["linear_solver"] = 'ma27';
#ipopt_opts["hessian_approximation"] = "limited-memory";
ipopt_opts["warm_start_init_point"] = "yes";
ipopt_opts["warm_start_bound_push"] = 1e-4;
ipopt_opts["warm_start_mult_bound_push"] = 1e-4;

# ipopt_opts["max_cpu_time"] = 0.12          # 120 ms,保证最大能达到6km/h的速度
# ipopt_opts["acceptable_tol"] = 1e-3
# ipopt_opts["acceptable_obj_change_tol"] = 1e-4
# ipopt_opts["acceptable_iter"] = 5

opts["expand"] = False
opts["print_time"] = 1;
opts["ipopt"] = ipopt_opts

# solver
# solver = nlpsol('solver', 'ipopt', nlp,opts)


compiler = "gcc"    # Linux
flags = ["-O3"] # Linux/OSX

#generate_c = True
generate_c = True
# # external c codegen
plugin_name = "geo_local_path_test"
if generate_c:
    solver = nlpsol('solver', 'ipopt', nlp,opts);
    solver.generate_dependencies(plugin_name+".c")

    import subprocess
    cmd_args = [compiler,"-fPIC","-shared"]+flags+[plugin_name+".c","-o",plugin_name+".so"]
    subprocess.run(cmd_args)

solver = nlpsol("solver", "ipopt", plugin_name+".so")
# nlp problem use
u_opt_all = []
u0 = [0, 0, 0, 0]


car_width_val = 1
global_coeff_val = [0,0,0,0]
lt_coeff_val =    [0,0,0,2.5]
rt_coeff_val =  [0,0,0,-2.5]

#zhangaiwu
safe_dis_val = 0.1
x_star_val = 0
y_star_val = 0
k_star_val = 0

#if len(obs_val)<n_obs:
#    n_fill = n_obs-len(obs_val)
#    obs_val = np.vstack([obs_val, np.tile([0, 0, 0, 0, 0], (n_fill, 1))])
obs_val_reshape = obs_val.reshape(-1,1) #改为列向量


params = vertcat(X_val, car_width_val, global_coeff_val, lt_coeff_val, rt_coeff_val, obs_val_reshape, safe_dis_val, x_star_val, y_star_val, k_star_val)

lbu = [-inf,-inf,-inf,-inf]
ubu = [inf,inf,inf,inf]
lbg = []
ubg = []

for i in range(3*ns+ns*n_obs):
    lbg.extend([0.0])  
    ubg.extend([np.inf])
lbg.extend([0.0, 0.0]) 
ubg.extend([0.0, 0.0])

# the first optimization
solve_times = []
total_time = 0.0
t0 = time.time()
sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
t1 = time.time()
dt = t1 - t0
solve_times.append(dt)
total_time += dt
u_opt = sol['x']
u_opt_all.append(u_opt)

# 创建主图，调整横向尺寸
fig, ax = plt.subplots(figsize=(9, 3.5))  # 使图像纵向尺寸更大

ax.axhline(y=0, color='red', linestyle='--',label='y_i,ref')
ax.axhline(y=2.5, color='green', linestyle='--',label='y_i,L')
ax.axhline(y=-2.5, color='blue', linestyle='--',label='y_i,R')

# 可视化障碍物
#fig, ax = plt.subplots()


# 优化路径绘制
for i in range(trav_dist):

    y_star_val = u_opt[0] * x_star_val ** 3 + u_opt[1] * x_star_val ** 2 + u_opt[2] * x_star_val + u_opt[3]
    k_star_val =  3 * u_opt[0] * x_star_val ** 2 + 2 * u_opt[1] * x_star_val + u_opt[2]
    params = vertcat(X_val, car_width_val, global_coeff_val, lt_coeff_val, rt_coeff_val, obs_val_reshape, safe_dis_val, x_star_val, y_star_val, k_star_val)

    t0 = time.time()
    sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
    t1 = time.time()
    dt = t1 - t0
    solve_times.append(dt)
    total_time += dt

    u_opt = sol['x']
    # 计算轨迹点
    x_max_plot = x_star_val + 0.2
    x_plot = np.linspace(x_star_val, x_max_plot, 100)
    f_values = []
    f_up_y_values = []
    f_down_y_values = []
    f_up_x_values = []
    f_down_x_values = []
    for x_val in x_plot:
        #f_values.append(y_fn(x_val, u_opt).full().flatten()[0])
        #f_up_y_values.append(f_up_y_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_down_y_values.append(f_down_y_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_up_x_values.append(f_up_x_fn(x_val, u_opt,car_width).full().flatten()[0])
        #f_down_x_values.append(f_down_x_fn(x_val, u_opt,car_width).full().flatten()[0])

        f_values.append(float(y_fn(x_val, u_opt).full().item()))
        f_up_y_values.append(float(f_up_y_fn(x_val, u_opt, car_width_val).full().item()))
        f_down_y_values.append(float(f_down_y_fn(x_val, u_opt, car_width_val).full().item()))
        f_up_x_values.append(float(f_up_x_fn(x_val, u_opt, car_width_val).full().item()))
        f_down_x_values.append(float(f_down_x_fn(x_val, u_opt, car_width_val).full().item()))


    #----Fig.1 Description of the driving environment----
    #----und----
    #----Fig.3,4,5 Optimal routing in case 1,2,3----
    
    if i == 0:

         ax.plot(x_plot, f_values, color="red", label="y_i,nav")
         ax.plot(f_up_x_values, f_up_y_values,color="green", label="y_i,l")
         #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
         ax.plot(f_down_x_values, f_down_y_values, color="blue", label="y_i,r")

    
    else:
         ax.plot(x_plot, f_values, color="red")
         ax.plot(f_up_x_values, f_up_y_values,color="green")
         #ax.plot(f_up_x_values, f_up_y_values,color=color)
         ax.plot(f_down_x_values, f_down_y_values, color="blue")
        

    x_star_val = x_star_val + x_stepl
    u_opt_all.append(u_opt)


# 绘制障碍物为圆
#for j in range(n_obs):
    #x_obs = obs_val[0 + 5 * j]
    #y_obs = obs_val[1 + 5 * j]
    #r_obs_x = obs_val[2 + 5 * j] 
    #r_obs_y = obs_val[3 + 5 * j]

    # 画椭圆的障碍物
    #circle = plt.Circle((x_obs, y_obs), max(r_obs_x, r_obs_y), color='green', alpha=0.5, label="Obstacle" if j == 0 else "")
    #ax.add_patch(circle)


from matplotlib.patches import Ellipse

for j in range(n_obs):
    x_obs = obs_val[0 + 5 * j]
    y_obs = obs_val[1 + 5 * j]
    r_obs_x = obs_val[2 + 5 * j] 
    r_obs_y = obs_val[3 + 5 * j]
    theta_obs = obs_val[4 + 5 * j] * 180 / np.pi  # 转换为角度

    # 创建椭圆障碍物
    ellipse = Ellipse((x_obs, y_obs), width=2*r_obs_x, height=2*r_obs_y,
                      angle=theta_obs, color='green', alpha=0.5, label="Obstacle" if j == 0 else "")
    ax.add_patch(ellipse)



ax.set_aspect('equal', adjustable='datalim')
ax.grid(True)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")

# 修改坐标轴字体大小
plt.rcParams['xtick.labelsize'] = 15  # x轴标签的字号
plt.rcParams['ytick.labelsize'] = 15  # y轴标签的字号

# 修改轴标签的字体大小
plt.rcParams['axes.labelsize'] = 13  # x轴和y轴标签的字号

# 设置刻度字体大小
ax.tick_params(axis='both', labelsize=13)

# 创建一个新的图形窗口来绘制单独的图例
#fig_legend = plt.figure()

# 获取主图的图例并绘制到新图
#fig_legend.legend(*ax.get_legend_handles_labels(), loc='center')

ax.legend(loc='center left', bbox_to_anchor=(1, 0.5),fontsize='11')
#ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
#ax.legend()

# 设置图例窗口的布局
plt.tight_layout()

# 设置x和y轴的显示范围
ax.set_xlim(0, 14)  # x轴范围
ax.set_ylim(-3, 3)  # y轴范围

# 设置 x 和 y 轴的刻度显示频率，每个刻度间隔1
ax.xaxis.set_major_locator(ticker.MultipleLocator(1))  # 设置 x 轴刻度间隔为 1
ax.yaxis.set_major_locator(ticker.MultipleLocator(1))  # 设置 y 轴刻度间隔为 1

# 保存为 .eps 格式
#fig.savefig('your_plot.eps', format='eps')

# 显示图形
#plt.tight_layout()

# 读取图片
img = mpimg.imread('/Users/liuyuanfang/Studium/24-25WS/Obstacle avoiding/Samples/Bildschirmfoto 2025-03-18 um 13.47.12.png')

# 创建绘图
#fig, ax = plt.subplots()

# 插入图片，设置图片显示的位置和尺寸 (这里以1x1的比例放在原点)
image = ax.imshow(img, extent=[-0.5, 0.5, -0.5, 0.5])

# # 设置旋转变换，顺时针旋转90度
transform = Affine2D().rotate_deg(-90) + ax.transData

# # 将变换应用到图片上
image.set_transform(transform)

# 设定坐标轴范围
#ax.set_xlim([-5, 5])
#ax.set_ylim([-5, 5])

# 显示图像
plt.show()

# 绘制耗时曲线
plt.figure()
plt.plot(solve_times, marker='o')
plt.xlabel("Optimization step")
plt.ylabel("Solve time (s)")
plt.title("Ipopt solve time per step")
plt.grid(True)
for i, t in enumerate(solve_times):
    plt.text(i, t, f"{t:.3f}", ha='center', va='bottom', fontsize=8, rotation=45)
plt.show()

# 假设 solve_times 已经填满了每次求解的时间（秒）
avg_time = np.mean(solve_times)          # 平均值
max_time = np.max(solve_times)           # 最大值
#min_time = np.min(solve_times)           # 最小值


print(f"Average solve time: {avg_time:.2f} s")
#print(f"Min solve time: {min_time*1000:.2f} ms")
print(f"Max solve time: {max_time:.2f} s")
print(f"Total solve time: {total_time:.2f} s")
