import numpy as np
from numpy.polynomial.polynomial import Polynomial

class ParametricCurve:
    def __init__(self, curve_type):
        """
        初始化曲线类型。
        :param curve_type: 曲线类型，例如 "Polynomial"。
        """
        self.curve_type = curve_type
        self.coefficients = None  # 存储拟合后的多项式系数

    def curve_fit(self, x_ref, y_ref, degree=3):
        """
        对参考点 (x_ref, y_ref) 进行多项式拟合。
        :param x_ref: 参考点的 x 坐标。
        :param y_ref: 参考点的 y 坐标。
        :param degree: 多项式的最高阶次。
        :return: 多项式系数。
        """
        if self.curve_type != "Polynomial":
            raise ValueError("Only 'Polynomial' curve type is supported")

        # 使用 numpy 的 least squares 方法进行拟合
        coeffs = np.polyfit(x_ref, y_ref, degree)

        # 系数存储为类属性
        self.coefficients = coeffs

        return coeffs

    def evaluate(self, x):
        """
        根据拟合曲线计算指定 x 的 y 值。
        :param x: 输入的 x 值。
        :return: 对应的 y 值。
        """
        if self.coefficients is None:
            raise ValueError("Curve is not fitted yet. Call curve_fit() first.")

        # Evaluate polynomial using the coefficients
        y = np.polyval(self.coefficients, x)
        return y

class Point:
    def __init__(self, x, y):
        """
        初始化点的坐标
        :param x: 点的 x 坐标
        :param y: 点的 y 坐标
        """
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Point({self.x}, {self.y})"


class Ellipse:
    def __init__(self, p1, p2):
        """
        初始化椭圆，通过两个点定义
        :param p1: 第一个点 (焦点之一)
        :param p2: 第二个点 (焦点之一)
        """
        self.p1 = p1
        self.p2 = p2

    def approximate_line_with_ellipse(self, inflation_rate):
        """
        用两点之间的直线拟合一个椭圆，返回中心、长半轴、短半轴、旋转角
        :param inflation_rate: 椭圆膨胀率，决定长半轴相对于焦点距离的伸展比例
        :return: 中心坐标 (cx, cy)、长半轴 (a)、短半轴 (b)、旋转角 (theta)
        """
        # 计算中心
        cx = (self.p1.x + self.p2.x) / 2
        cy = (self.p1.y + self.p2.y) / 2
        center = (cx, cy)

        # 焦点距离
        focal_distance = np.sqrt((self.p2.x - self.p1.x)**2 + (self.p2.y - self.p1.y)**2)

        # 长半轴和短半轴
        a = (1 + inflation_rate) * focal_distance / 2  # 膨胀后的长半轴
        b = a * np.sqrt(1 - (focal_distance / (2 * a))**2)  # 椭圆几何约束

        # 旋转角度
        theta = np.arctan2(self.p2.y - self.p1.y, self.p2.x - self.p1.x)

        return center, a, b, theta
