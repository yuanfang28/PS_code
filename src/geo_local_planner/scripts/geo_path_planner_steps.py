#!/usr/bin/env python3
from casadi import *
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.cm import get_cmap
from matplotlib.patches import Ellipse
import matplotlib.ticker as ticker


x  = MX.sym('x', 1)  # state
u  = MX.sym('u', 4)  # control

ns = 40  # number of point
n_obs = 5 # number of obstacles
#n_obs = MX.sym('n_obs',1)


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
K = fabs(k2) / ((1 + k ** 2) ** (3 / 2))#curvature
S = sqrt(1 + k ** 2)

f_up_y = y_local + (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_down_y = y_local - (car_width / 2) * (1 / sqrt(k ** 2 + 1))
f_up_x = x - (((car_width / 2) * k) / sqrt(k ** 2 + 1))
f_down_x = x + (((car_width / 2) * k) / sqrt(k ** 2 + 1))

# Create CasADi functions
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
g_3 = []

for i in range(ns):
    x_val = x_star + i*dx
    #g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(x_val, lt_coeff) + safe_dis
    #g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(x_val, rt_coeff) - safe_dis

    g_1_up =  -f_up_y_fn(f_up_x_fn(x_val,u,car_width), u, car_width) + y_lt_fn(f_up_x_fn(x_val,u,car_width), lt_coeff) + safe_dis
    g_1_down =  f_down_y_fn(f_down_x_fn(x_val,u,car_width), u, car_width) - y_rt_fn(f_down_x_fn(x_val,u,car_width), rt_coeff) - safe_dis
    g_1.extend([g_1_up, g_1_down]) 
    g3 = 2 - K_fn(x_val,u)
    g_3.append(g3)


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


g_2 = []
for i in range(ns):
    x_val = x_star + i*dx
    for j in range(n_obs):
        x_val = x_star + i*dx
        g2 = -1+(pow((x_val-obs[0+5*j])*cos(obs[4+5*j]) + (y_fn(x_val,u)-obs[1+5*j])*sin(obs[4+5*j]),2)/pow(obs[2+5*j]+car_width/2 + safe_dis,2) + pow((x_val-obs[0+5*j])*sin(obs[4+5*j]) - (y_fn(x_val,u)-obs[1+5*j])*cos(obs[4+5*j]),2)/pow(obs[3+5*j]+car_width/2 + safe_dis,2))
        g_2.append(g2)


g_4 = y_fn(x_star,u) - y_star
g_5 = k_fn(x_star,u) - k_star


# nlp problem definieren
nlp = []
#nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1,*g_2,*g_3,g_4,g_5)} 
nlp = {'x':u,  'p': vertcat(X, car_width, global_coeff,  lt_coeff, rt_coeff, obs, safe_dis, x_star, y_star, k_star),'f':J, 'g': vertcat(*g_1, *g_2, *g_3,g_4,g_5)} 
opts = {}
ipopt_opts = {}
ipopt_opts["tol"] = 1e-5;
ipopt_opts["max_iter"] = 100;
ipopt_opts["print_level"] = 1;
ipopt_opts["sb"] = "yes";
ipopt_opts["acceptable_tol"] = 1e-5;
ipopt_opts["acceptable_iter"] = 0;
#ipopt_opts["linear_solver"] = 'ma27';
# ipopt_opts["hessian_approximation"] = "limited-memory";
ipopt_opts["warm_start_init_point"] = "yes";
ipopt_opts["warm_start_bound_push"] = 1e-6;
ipopt_opts["warm_start_mult_bound_push"] = 1e-6;
opts["expand"] = False
opts["print_time"] = 0;
opts["ipopt"] = ipopt_opts

# solver
solver = nlpsol('solver', 'ipopt', nlp,opts)


compiler = "gcc"    # Linux
flags = ["-O3"] # Linux/OSX

#generate_c = True
generate_c = False
# external c codegen
plugin_name = "geo_local_path_"
if generate_c:
    solver = nlpsol('solver', 'ipopt', nlp,opts);
    solver.generate_dependencies(plugin_name+".c")

    import subprocess
    cmd_args = [compiler,"-fPIC","-shared"]+flags+[plugin_name+".c","-o",plugin_name+".so"]
    subprocess.run(cmd_args)

#solver = nlpsol("solver", "ipopt", plugin_name+".so")
# nlp problem use
u_opt_all = []
u0 = [0, 0, 0, 0]

X_val = 4
car_width_val = 1
global_coeff_val = [0,0,0,0]
lt_coeff_val =    [0,0,0,3]
rt_coeff_val =  [0,0,0,-3]
obs_val = np.array([4.2, 0.3,  0.7,  0.5 , np.pi/6,
                    7,  0.4,  0.7,  0.3, np.pi/4,
                    11.58, -0.81,  0.7,  0.4, np.pi/3,
                    10, 10, 0.01, 0.01,0,
                    10, 10, 0.01, 0.01,0])

#obs_val = np.array([-2.8, 2.1,  0.7,  0.5 , np.pi/6,
                    #0,  -2.1,  0.7,  0.3, np.pi/4,
                    #4.58, 2.1,  0.7,  0.4, np.pi/3,
                    #10, 10, 0.01, 0.01,0,
                    #10, 10, 0.01, 0.01,0])
                 
                 
safe_dis_val = 0.1
x_star_val = 1
y_star_val = 0
k_star_val = 0

#if len(obs_val)<n_obs:
#    n_fill = n_obs-len(obs_val)
#    obs_val = np.vstack([obs_val, np.tile([0, 0, 0, 0, 0], (n_fill, 1))])
obs_val_reshape = obs_val.reshape(-1,1)


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

sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
u_opt = sol['x']
u_opt_all.append(u_opt)


# 创建主图，调整横向尺寸
fig, ax = plt.subplots(figsize=(10, 7.5))  

ax.axhline(y=0, color='red', linestyle=':', label='The reference path (centerline of the sidewalk)',lw=2)
ax.axhline(y=2.5, color='green', linestyle=':', label='The left boundary of the sidewalk',lw=2)
ax.axhline(y=-2.5, color='blue', linestyle=':', label='The right boundary of the sidewalk',lw=2)

# 可视化障碍物
#fig, ax = plt.subplots()

# 优化路径绘制
for i in range(70):
    #x_star_val = x_star_val + 0.2
    y_star_val = u_opt[0] * x_star_val ** 3 + u_opt[1] * x_star_val ** 2 + u_opt[2] * x_star_val + u_opt[3]
    k_star_val =  3 * u_opt[0] * x_star_val ** 2 + 2 * u_opt[1] * x_star_val + u_opt[2]
    params = vertcat(X_val, car_width_val, global_coeff_val, lt_coeff_val, rt_coeff_val, obs_val_reshape, safe_dis_val, x_star_val, y_star_val, k_star_val)

    sol = solver(x0=u0, p=params, lbx=lbu, ubx=ubu, lbg=lbg, ubg=ubg)
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

        f_values.append(float(y_fn(x_val, u_opt).full()[0]))
        f_up_y_values.append(float(f_up_y_fn(x_val, u_opt, car_width_val).full()[0]))
        f_down_y_values.append(float(f_down_y_fn(x_val, u_opt, car_width_val).full()[0]))
        f_up_x_values.append(float(f_up_x_fn(x_val, u_opt, car_width_val).full()[0]))
        f_down_x_values.append(float(f_down_x_fn(x_val, u_opt, car_width_val).full()[0]))
    
    #Fig.2 The moving horizon strategy    
    #交替红蓝颜色
    color = "red" if i % 2 == 0 else "purple"
    color2 = "green" if i % 2 == 0 else "lightgreen"
    color3 = "blue" if i % 2 == 0 else "lightblue"

    if i == 0:

        ax.plot(x_plot, f_values, color=color, label="Navigation path")
        ax.plot(f_up_x_values, f_up_y_values,color=color2, label="The left boundary of the robot's trajectory")
        #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
        ax.plot(f_down_x_values, f_down_y_values, color=color3, label="The right boundary of the robot's trajectory")

        # 添加标注第一次优化范围 (1 到 5)
        ax.vlines(x=1, ymin=-3, ymax=3, colors="purple", linestyle="--", label="Optimization range (1 to 5)")
        ax.vlines(x=1, ymin=-3.2, ymax=3, colors="red", lw=1)
        ax.vlines(x=5, ymin=-3.2, ymax=3, colors="red", lw=1)
        ax.text(3, -2.8, "Opt.step i=1,x∈[1.0,5.0]", color="red", fontsize=12, ha="center")

        # 绘制一条直线（尺寸线）
        ax.plot([1, 5], [-3, -3], color="red", lw=2)  # 从 (1, 0) 到 (5, 0) 绘制尺寸线

        # 在两端添加箭头
        ax.annotate('', xy=(2, -3), xytext=(1, -3), arrowprops=dict(arrowstyle='<-', color="red",lw=2))  # 左箭头
        ax.annotate('', xy=(4, -3), xytext=(5, -3), arrowprops=dict(arrowstyle='<-', color="red",lw=2))  # 右箭头

         # 添加标注第一次优化范围 (1 到 5)，使用 transAxes 使虚线延伸到图表外
        #ax.vlines(x=1, ymin=-5, ymax=5, colors="purple", linestyle="--")
        #ax.vlines(x=5, ymin=-5, ymax=5, colors="purple", linestyle="--")
         # 使用 transAxes 放置文字在图表外
        #ax.text(0.5, 1.05, "Optimization range (1 to 5)", color="purple", fontsize=12, ha="center", transform=ax.transAxes, verticalalignment="bottom")
    
    elif i == 1:

        ax.plot(x_plot, f_values, color=color)
        ax.plot(f_up_x_values, f_up_y_values,color=color2)
        #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
        ax.plot(f_down_x_values, f_down_y_values, color=color3)

        #ax.vlines(x=1, ymin=-3, ymax=3, colors="purple", linestyle="--", label="Optimization range (1 to 5)")
        ax.vlines(x=1.2, ymin=-3.9, ymax=3, colors="purple", lw=1)
        ax.vlines(x=5.2, ymin=-3.9, ymax=3, colors="purple", lw=1)
        ax.text(3.2, -3.5, "Opt.step i=2,x∈[1.2,5.2]", color="purple", fontsize=12, ha="center")

        # 绘制一条直线（尺寸线）
        ax.plot([1.2, 5.2], [-3.7, -3.7], color="purple", lw=2)  # 从 (1, 0) 到 (5, 0) 绘制尺寸线

        # 在两端添加箭头
        ax.annotate('', xy=(2.2, -3.7), xytext=(1.2, -3.7), arrowprops=dict(arrowstyle='<-', color="purple",lw=2))  # 左箭头
        ax.annotate('', xy=(4.2, -3.7), xytext=(5.2, -3.7), arrowprops=dict(arrowstyle='<-', color="purple",lw=2))  # 右箭头

    elif i == 2:

        ax.plot(x_plot, f_values, color="red")
        ax.plot(f_up_x_values, f_up_y_values,color=color2)
        #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
        ax.plot(f_down_x_values, f_down_y_values, color=color3)

        #ax.vlines(x=1, ymin=-3, ymax=3, colors="purple", linestyle="--", label="Optimization range (1 to 5)")
        ax.vlines(x=1.4, ymin=-4.6, ymax=3, colors="red", lw=1)
        ax.vlines(x=5.4, ymin=-4.6, ymax=3, colors="red", lw=1)
        ax.text(3.4, -4.2, "Opt.step i=3,x∈[1.4,5.4]", color="red", fontsize=12, ha="center")

        # 绘制一条直线（尺寸线）
        ax.plot([1.4, 5.4], [-4.4, -4.4], color="red", lw=2)  # 从 (1, 0) 到 (5, 0) 绘制尺寸线

        # 在两端添加箭头
        ax.annotate('', xy=(2.4, -4.4), xytext=(1.4, -4.4), arrowprops=dict(arrowstyle='<-', color="red",lw=2))  # 左箭头
        ax.annotate('', xy=(4.4, -4.4), xytext=(5.4, -4.4), arrowprops=dict(arrowstyle='<-', color="red",lw=2))  # 右箭头

    elif i == 3:

        ax.plot(x_plot, f_values, color=color)
        ax.plot(f_up_x_values, f_up_y_values,color=color2)
        #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
        ax.plot(f_down_x_values, f_down_y_values, color=color3)

        #ax.vlines(x=1, ymin=-3, ymax=3, colors="purple", linestyle="--", label="Optimization range (1 to 5)")
        ax.vlines(x=1.6, ymin=-5.3, ymax=3, colors="purple", lw=1)
        ax.vlines(x=5.6, ymin=-5.3, ymax=3, colors="purple", lw=1)
        ax.text(3.6, -4.9, "Opt.step i=4 ...", color="purple", fontsize=12, ha="center")

        # 绘制一条直线（尺寸线）
        ax.plot([1.6, 5.6], [-5.1, -5.1], color="purple", lw=2)  # 从 (1, 0) 到 (5, 0) 绘制尺寸线

        # 在两端添加箭头
        ax.annotate('', xy=(2.6, -5.1), xytext=(1.6, -5.1), arrowprops=dict(arrowstyle='<-', color="purple",lw=2))  # 左箭头
        ax.annotate('', xy=(4.6, -5.1), xytext=(5.6, -5.1), arrowprops=dict(arrowstyle='<-', color="purple",lw=2))  # 右箭头


    else:
        ax.plot(x_plot, f_values, color=color)
        ax.plot(f_up_x_values, f_up_y_values,color=color2)
        #ax.plot(f_up_x_values, f_up_y_values,color=color)
        ax.plot(f_down_x_values, f_down_y_values, color=color3)


        
    
    #交替红蓝颜色
    #color = "red" if i % 2 == 0 else "blue"

    #ax.plot(x_plot, f_values, color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
    #ax.plot(f_up_x_values, f_up_y_values,color=colors[i], label="Iteration {}".format(i) if i % 10 == 0 else "")#使用渐变条
    #ax.plot(f_up_x_values, f_up_y_values,color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")
    #ax.plot(f_down_x_values, f_down_y_values, color=color, label="Schritt {}".format(i) if i % 10 == 0 else "")

    # 使用渐变色
    #选择 colormap (颜色渐变)
    #cmap = get_cmap("jet")  # 其他可选："viridis", "plasma", "coolwarm"

    #生成 70 种颜色
    #colors = [cmap(i / 70) for i in range(70)]

    #if i == 0 or i == 49:
        #ax.plot(x_plot, f_values, color=colors[i], label="Step {},navigation path".format(i+1))
        #ax.plot(f_up_x_values, f_up_y_values,color=colors[i])
        #ax.plot(f_down_x_values, f_down_y_values, color=colors[i])

    #else :
        #ax.plot(x_plot, f_values, color=colors[i])
        #ax.plot(f_up_x_values, f_up_y_values,color=colors[i])
        #ax.plot(f_down_x_values, f_down_y_values, color=colors[i])

   
    u_opt_all.append(u_opt)
    x_star_val = x_star_val + 0.2

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

# 添加颜色渐变条
#sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=70))
#cbar = plt.colorbar(sm)
#cbar = plt.colorbar(sm, ax=ax)
#cbar.set_label("optimization steps")

#添加图表标题
#ax.set_title("The continuity and smoothness between optinization steps ")

# 添加标签和图例

ax.set_aspect('equal', adjustable='datalim')
ax.grid(True)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")

# 修改坐标轴字体大小
plt.rcParams['xtick.labelsize'] = 18  # x轴标签的字号
plt.rcParams['ytick.labelsize'] = 18  # y轴标签的字号


# 修改轴标签的字体大小
plt.rcParams['axes.labelsize'] = 18  # x轴和y轴标签的字号

# 设置刻度字体大小
ax.tick_params(axis='both', labelsize=18)


# ----Legends----

# 创建一个新的图形窗口来绘制单独的图例
#fig_legend = plt.figure()

# 获取主图的图例并绘制到新图
#fig_legend.legend(*ax.get_legend_handles_labels(), loc='center')

# 图例放在图表外的右侧
#ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize='15')

# 图例放在图表外的下侧
ax.legend(loc='lower right',fontsize='12')

#ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
#ax.legend()

# 设置图例窗口的布局
plt.tight_layout()

# 设置x和y轴的显示范围
ax.set_xlim(1, 12)  
ax.set_ylim(-5.5, 3) 

# 设置 x 和 y 轴的刻度显示频率，每个刻度间隔1
ax.xaxis.set_major_locator(ticker.MultipleLocator(1))  # 设置 x 轴刻度间隔为 1
ax.yaxis.set_major_locator(ticker.MultipleLocator(1))  # 设置 y 轴刻度间隔为 1

# 保存为 .eps 格式
#fig.savefig('your_plot.eps', format='eps')

# 显示图形
#plt.tight_layout()
plt.show()

