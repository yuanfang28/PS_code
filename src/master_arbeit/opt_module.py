from casadi import *

class discretization:
    def __init__(self, method, nx, f):
        """
        初始化离散化工具
        :param method: 离散化方法 (例如 "rk4")
        :param nx: 状态变量维度
        :param f: 动态方程 (CasADi 函数 f(x, u, param))
        """
        self.method = method
        self.nx = nx
        self.f = f

        if method != "rk4":
            raise ValueError("Only 'rk4' method is supported")

    def rk4(self, X0, U, param, DT):
        """
        使用 RK4 方法进行离散化
        :param X0: 当前状态
        :param U: 控制输入
        :param param: 动态方程参数 (例如 xICR)
        :param DT: 时间步长
        :return: 离散化后下一时刻的状态
        """
        k1 = self.f(X0, U, param)
        k2 = self.f(X0 + 0.5 * DT * k1, U, param)
        k3 = self.f(X0 + 0.5 * DT * k2, U, param)
        k4 = self.f(X0 + DT * k3, U, param)

        #X_next = X0 + (DT / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        X_next = X0 + DT * k1
        return X_next
