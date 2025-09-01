import numpy as np
import matplotlib.pyplot as plt
from casadi import *
import random
#from car_path_plot import plot_function

class local_path_info:
    def __init__(self,global_x,global_y,ob_infor = np.array([[2,1,0.1,0.1]])):
        # Opti_info
        self.N = 1
        self.step = 1
        # car_info
        self.range_radar = 4
        self.car_width = 0.8
        self.car_length = 1
        self.safe_dis = 0.1
        self.ob_infor = ob_infor
        # linie_infor
        self.global_x = global_x
        self.global_y = global_y
        


    def path_fun(self, x, u, car_width):
        y_inter = u[0] * x ** 3 + u[1] * x ** 2 + u[2] * x + u[3]
        k = 3 * u[0] * x ** 2 + 2 * u[1] * x + u[2]
        k2 = 6 * u[0] * x + 2 * u[1]

        # Calculate curvature and arc length
        K = fabs(k2) / ((1 + k ** 2) ** (3 / 2))
        S = sqrt(1 + k ** 2)

        # Calculate obstacle avoidance functions
        f_up_y = y_inter + (car_width / 2) * (1 / sqrt(k ** 2 + 1))
        f_down_y = y_inter - (car_width / 2) * (1 / sqrt(k ** 2 + 1))
        f_up_x = x - (((car_width / 2) * k) / sqrt(k ** 2 + 1))
        f_down_x = x + (((car_width / 2) * k) / sqrt(k ** 2 + 1))

        # Create CasADi functions
        y_fn = Function('y_fn', [x, u], [y_inter])
        k_fn = Function('k_fn', [x, u], [k])
        k2_fn = Function('k2_fn', [x, u], [k2])
        K_fn = Function('K_fn', [x, u], [K])
        S_fn = Function('S_fn', [x, u], [S])
        f_up_y_fn = Function('f_up_y_fn', [x, u], [f_up_y])
        f_down_y_fn = Function('f_down_y_fn', [x, u], [f_down_y])
        f_up_x_fn = Function('f_up_x_fn', [x, u], [f_up_x])
        f_down_x_fn = Function('f_down_x_fn', [x, u], [f_down_x])

        return y_fn, k_fn, k2_fn, K_fn, S_fn, f_up_y_fn, f_down_y_fn, f_up_x_fn, f_down_x_fn, y_inter

    def trapezoidal_integral(self, f, x_min, x_max, u):

        x_data = self.global_x
        y_data = self.global_y
        dx = abs(x_data[1]-x_data[0])
        integral = 0.5 * ((f(x_min, u)- y_data[0])**2 + (f(x_max, u) - y_data[-1])**2)
        for i in range(1, len(x_data) - 1):
            x_i = x_data[i]
            integral += (f(x_i, u) - y_data[i])**2
        integral *= dx
        return integral

    def update_valid_obstacles(x_given, y_given, d_given, l_given, f_up_y_fn, f_down_y_fn, u_opt, x_star, x_max, valid_indices):

        x_new, y_new, d_new, l_new = [], [], [], []
        new_elements_added = False

        for i in range(len(x_given)):
            if x_given[i] - d_given[i] / 2 >= x_max or x_given[i] + d_given[i] / 2 <= x_star:
                continue

            if not ((y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i], u_opt) + self.safe_dis and
                    y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i] - d_given[i] / 2 - self.car_width / 2, u_opt) + self.safe_dis and
                    y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i] + d_given[i] / 2 + self.car_width / 2, u_opt) + self.safe_dis) or
                    (y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i], u_opt) - self.safe_dis and
                    y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i] - d_given[i] / 2 - self.car_width / 2, u_opt) - self.safe_dis and
                    y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i] + d_given[i] / 2 + self.car_width / 2, u_opt) - self.safe_dis)):
                if i not in valid_indices:
                    valid_indices.add(i)
                    new_elements_added = True

        x_new = [x_given[i] for i in valid_indices]
        y_new = [y_given[i] for i in valid_indices]
        d_new = [d_given[i] for i in valid_indices]
        l_new = [l_given[i] for i in valid_indices]

        return x_new, y_new, d_new, l_new, new_elements_added

    def constrain(self, num_Obstacles_new, opti, x_new, y_new, d_new, l_new, car_width, f_down_y_fn, f_up_y_fn,
                 safe_dis, binary_str, u, x_values, y_fn, k_fn, y_star, k_star, x_star, K_fn):
        for i in range(num_Obstacles_new):
            x_val_left = x_new[i] - d_new[i] / 2 - car_width / 2
            x_val_right = x_new[i] + d_new[i] / 2 + car_width / 2
            split_num = 1
            for n in range(split_num):
                if binary_str[i] == '0':  # Going up
                    opti.subject_to(f_down_y_fn(x_val_left + n * 0.1, u) >= (y_new[i] + (l_new[i] / 2) + safe_dis))
                    opti.subject_to(f_down_y_fn(x_val_right, u) >= (y_new[i] + (l_new[i] / 2) + safe_dis))
                    opti.subject_to(f_down_y_fn(x_new[i], u) >= (y_new[i] + (l_new[i] / 2) + safe_dis))
                else:  # Going down
                    opti.subject_to(f_up_y_fn(x_val_left + n * 0.1, u) <= (y_new[i] - (l_new[i] / 2) - safe_dis))
                    opti.subject_to(f_up_y_fn(x_val_right, u) <= (y_new[i] - (l_new[i] / 2) - safe_dis))
                    opti.subject_to(f_up_y_fn(x_new[i], u) <= (y_new[i] - (l_new[i] / 2) - safe_dis))

            for x_K in x_values:
                opti.subject_to(K_fn(x_K, u) < 2)
            for x_top in x_values:
                opti.subject_to(f_up_y_fn(x_top, u) <= 2 - safe_dis)
                opti.subject_to(f_down_y_fn(x_top, u) >= -2 + safe_dis)

            opti.subject_to(y_fn(x_star, u) == y_star)
            opti.subject_to(k_fn(x_star, u) == k_star)

    def compute_local_path(self):
        # Initialize variables
        x = MX.sym('x', 1)
        ob_infor = self.ob_infor
        x_given, y_given, d_given, l_given = ob_infor[:, 0], ob_infor[:, 1], ob_infor[:, 2], ob_infor[:, 3]
        num_obstacles = np.size(x_given)
        
        local_path_point = []  # Store (x, y) waypoints

        for m in range(self.N):
            x_star = self.step * m
            x_max = x_star + self.range_radar
            x_values = np.linspace(x_star, x_max, 20)
            
            x_star_plot = x_star
            x_max_plot = self.range_radar + self.step * m if m == self.N - 1 else (m + 1) * self.step
            y_star = 0 if m == 0 else y_fn(x_star, u_opt)
            k_star = 0 if m == 0 else k_fn(x_star, u_opt)
            
            t = 0
            all_solutions = []
            valid_indices = set()
            u_opt = None  # Initialize u_opt
            
            while True:
                new_elements_added = False
                if t == 0:
                    # Initialize optimization
                    opti = Opti()
                    u = opti.variable(4)
                    y_fn, k_fn, k2_fn, K_fn, S_fn, f_up_y_fn, f_down_y_fn, f_up_x_fn, f_down_x_fn, y_inter = self.path_fun(x, u, self.car_width)
                    integral_result = self.trapezoidal_integral(y_fn, x_star, x_max, u)
                    print(integral_result)
                    opti.minimize(integral_result)
                    opti.subject_to(y_fn(x_star, u) == y_star)
                    opti.subject_to(k_fn(x_star, u) == k_star)
                    opti.solver('ipopt')
                    sol = opti.solve()
                    u_opt = sol.value(u)

                else:
                    x_new, y_new, d_new, l_new = [], [], [], []
                    for i in range(num_obstacles):
                        if x_given[i] - d_given[i] / 2 >= x_max or x_given[i] + d_given[i] / 2 <= x_star:
                            continue
                        if not ((y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i], u_opt) + self.safe_dis and
                                 y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i] - d_given[i] / 2 - self.car_width / 2, u_opt) + self.safe_dis and
                                 y_given[i] - l_given[i] / 2 >= f_up_y_fn(x_given[i] + d_given[i] / 2 + self.car_width / 2, u_opt) + self.safe_dis) or
                                (y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i], u_opt) - self.safe_dis and
                                 y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i] - d_given[i] / 2 - self.car_width / 2, u_opt) - self.safe_dis and
                                 y_given[i] + l_given[i] / 2 <= f_down_y_fn(x_given[i] + d_given[i] / 2 + self.car_width / 2, u_opt) - self.safe_dis)):
                            if i not in valid_indices:
                                valid_indices.add(i)
                                new_elements_added = True

                    if not new_elements_added:
                        break
                    else:
                        x_new = [x_given[i] for i in valid_indices]
                        y_new = [y_given[i] for i in valid_indices]
                        d_new = [d_given[i] for i in valid_indices]
                        l_new = [l_given[i] for i in valid_indices]

                        for m in range(2 ** len(valid_indices)):
                            binary_str = f"{m:0{len(valid_indices)}b}"
                            opti = Opti()
                            u = opti.variable(4)
                            y_fn, k_fn, k2_fn, K_fn, S_fn, f_up_y_fn, f_down_y_fn, f_up_x_fn, f_down_x_fn, y_inter = self.path_fun(x, u, self.car_width)
                            integral_result = self.trapezoidal_integral(y_fn, x_star, x_max, u)
                            opti.minimize(integral_result)

                            self.constrain(len(valid_indices), opti, x_new, y_new, d_new, l_new, self.car_width, f_down_y_fn, f_up_y_fn, self.safe_dis, binary_str, u, x_values, y_fn, k_fn, y_star, k_star, x_star, K_fn)

                            try:
                                opti.solver("ipopt", {"expand": True}, {'mu_init': 1e-8, 'print_level': 0, 'mu_max': 1e+6})
                                sol = opti.solve()
                                u_all_opt = sol.value(u)
                                S_integral = self.trapezoidal_integral(S_fn, x_star, x_max, u_all_opt)
                                all_solutions.append((S_integral, u_all_opt))
                            except RuntimeError as e:
                                if 'Infeasible_Problem_Detected' in str(e):
                                    print(f"Skipping infeasible solution for configuration: {binary_str}")
                                else:
                                    raise e

                        feasible_solutions = [sol for sol in all_solutions if len(sol[1]) == 4]
                        if feasible_solutions:
                            best_solution = min(feasible_solutions, key=lambda x: x[0])
                            best_S_integral, u_opt = best_solution
                        else:
                            print("No feasible solutions found.")
                            break

                t += 1

            local_path_point.append([(x_val, y_fn(x_val, u_opt)) for x_val in x_values])
            plot_function(u_opt, x_star_plot, x_max_plot, y_fn, f_up_y_fn, f_down_y_fn, f_up_x_fn, f_down_x_fn, 
                          k2_fn, k_fn, K_fn, num_obstacles, x_given, y_given, d_given, l_given, m, self.N, self.step, self.range_radar)
            plt.show()
        return local_path_point


global_x = np.linspace(0, 4, 100)  # 从 0 到 4 生成 100 个均匀分布的 x 值
global_y = np.ones(100)
path_info = local_path_info(global_x,global_y)
local_path_points = path_info.compute_local_path()
