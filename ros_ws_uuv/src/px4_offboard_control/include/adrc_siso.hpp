#pragma once

#include <cmath>
#include <algorithm>

struct AdrcParams
{
    float dt{0.02f};          // 控制周期 [s]
    float b0{1.0f};           // 名义通道增益

    // Tracking differentiator (TD)
    float td_r{20.0f};
    float td_zeta{1.0f};

    // ESO 带宽
    float omega_o{40.0f};

    // 状态误差反馈 (NLSEF 这里先用线性形式)
    float omega_c{10.0f};
    float zeta_c{0.9f};

    // 约束
    float u_min{-8.0f};
    float u_max{ 8.0f};
    float dead_zone{0.0f};
};

/**
 * 简化版 SISO LADRC 控制器
 * 对象输入: 参考量 ref, 测量输出 y
 * 输出: 控制量 u
 */
class AdrcSiso
{
public:
    AdrcSiso() = default;

    void configure(const AdrcParams &p)
    {
        params_ = p;

        beta1_ = 3.0f * params_.omega_o;
        beta2_ = 3.0f * params_.omega_o * params_.omega_o;
        beta3_ = params_.omega_o * params_.omega_o * params_.omega_o;

        k1_ = params_.omega_c * params_.omega_c;
        k2_ = 2.0f * params_.zeta_c * params_.omega_c;

        reset();
    }

    void reset()
    {
        v1_ = 0.0f;
        v2_ = 0.0f;
        z1_ = 0.0f;
        z2_ = 0.0f;
        z3_ = 0.0f;
        u_prev_ = 0.0f;
    }

    /// 单步更新，返回控制量
    float step(float ref, float y)
    {
        const float dt = params_.dt;

        // ---------- 1) TD：生成平滑参考 ----------
        float e_td = v1_ - ref;
        float dv1 = v2_;
        float dv2 = -params_.td_r * e_td
                    - 2.0f * params_.td_zeta * std::sqrt(params_.td_r) * v2_;

        v1_ += dt * dv1;
        v2_ += dt * dv2;

        // ---------- 2) 线性 ESO ----------
        float e_eso = y - z1_;
        float dz1 = z2_ + beta1_ * e_eso;
        float dz2 = z3_ + params_.b0 * u_prev_ + beta2_ * e_eso;
        float dz3 = beta3_ * e_eso;

        z1_ += dt * dz1;
        z2_ += dt * dz2;
        z3_ += dt * dz3;

        // ---------- 3) 状态误差反馈 NLSEF（这里先用线性） ----------
        float e1 = v1_ - z1_;
        float e2 = v2_ - z2_;
        float u0 = k1_ * e1 + k2_ * e2;

        // 通过 ESO 估计补偿扰动
        float u = (u0 - z3_) / params_.b0;

        // ---------- 饱和 + 死区 ----------
        u = std::clamp(u, params_.u_min, params_.u_max);

        if (std::fabs(u) < params_.dead_zone) {
            u = 0.0f;
        } else if (params_.dead_zone > 0.0f) {
            u -= (u > 0.0f ? 1.0f : -1.0f) * params_.dead_zone;
        }

        u_prev_ = u;
        return u;
    }

private:
    AdrcParams params_;

    // TD
    float v1_{0.0f};
    float v2_{0.0f};

    // ESO 状态
    float z1_{0.0f};
    float z2_{0.0f};
    float z3_{0.0f};

    // 上一次的控制输入
    float u_prev_{0.0f};

    // ESO & NLSEF 系数
    float beta1_{0.0f};
    float beta2_{0.0f};
    float beta3_{0.0f};
    float k1_{0.0f};
    float k2_{0.0f};
};
