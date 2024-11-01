#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

/**
 * @brief OLS 类用于实现最小二乘法
 */
namespace tool::filter {
class OLS {
public:
    /**
     * @brief 构造函数，初始化最小二乘法对象
     * @param order 样本数量
     */
    explicit OLS(uint16_t order)
        : Order(order)
        , Count(0)
        , k(0)
        , b(0)
        , StandardDeviation(0) {
        x.resize(order);
        y.resize(order);
        t.resize(4);
        std::fill(x.begin(), x.end(), 0);
        std::fill(y.begin(), y.end(), 0);
        std::fill(t.begin(), t.end(), 0);
    }

    /**
     * @brief 更新最小二乘法对象的样本数据
     * @param deltax 信号新样本距上一个样本时间间隔
     * @param y 信号值
     */
    void Update(double deltax, double data) {
        double temp = x[1];
        for (uint16_t i = 0; i < Order - 1; ++i) {
            x[i] = x[i + 1] - temp;
            y[i] = y[i + 1];
        }
        x[Order - 1] = x[Order - 2] + deltax;
        y[Order - 1] = data;

        if (Count < Order) {
            Count++;
        }

        std::fill(t.begin(), t.end(), 0);
        for (uint16_t i = Order - Count; i < Order; ++i) {
            t[0] += x[i] * x[i];
            t[1] += x[i];
            t[2] += x[i] * y[i];
            t[3] += y[i];
        }
        auto toF32 = [](auto value) -> double { return static_cast<double>(value); };

        k = (t[2] * toF32(Order) - t[1] * t[3]) / (t[0] * toF32(Order) - t[1] * t[1]);
        b = (t[0] * t[3] - t[1] * t[2]) / (t[0] * toF32(Order) - t[1] * t[1]);

        StandardDeviation = 0;
        for (uint16_t i = Order - Count; i < Order; ++i) {
            StandardDeviation += std::fabs(k * x[i] + b - y[i]);
        }
        StandardDeviation /= toF32(Order);
    }

    /**
     * @brief 计算最小二乘法对象提取信号的微分
     * @param deltax 信号新样本距上一个样本时间间隔
     * @param y 信号值
     * @return 返回斜率 k
     */
    double Derivative(double deltax, double y) {
        Update(deltax, y);
        return k;
    }

    /**
     * @brief 平滑信号
     * @param deltax 信号新样本距上一个样本时间间隔
     * @param y 信号值
     * @return 返回平滑输出
     */
    double Smooth(double deltax, double y) {
        Update(deltax, y);
        return k * x[Order - 1] + b;
    }

    /**
     * @brief 获取平滑输出
     * @return 返回平滑输出
     */
    double GetSmooth() { return k * x[Order - 1] + b; }
    /**
     * @brief 获取最小二乘法提取信号微分
     * @retval 返回斜率k
     */
    [[nodiscard]] double GetDerivative() const { return k; }

private:
    uint16_t Order;          ///< 样本数量
    uint32_t Count;          ///< 实际样本数量
    std::vector<double> x;    ///< 存储样本时间间隔
    std::vector<double> y;    ///< 存储样本值
    double k;                 ///< 斜率
    double b;                 ///< 截距
    double StandardDeviation; ///< 标准差
    std::vector<double> t;    ///< 辅助计算中间变量
};
} // namespace tool::filter