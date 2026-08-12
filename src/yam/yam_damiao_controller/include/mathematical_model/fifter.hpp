// Created by piper on 2025-5-25
// 无时延低通滤波器
// 零相位延迟滤波器参考网上资料,首先使用3阶滤波器处理
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream"
#include "vector"
#include <stdbool.h> //for bool
//一阶低通滤波器 截止频率为角频率=2 * Pi * fc
class LPF1st {
    public:
        struct Impl {
            double cutoff_freq{ 0 };
            double last_value{ 0 };
            bool first_value{ true };
        };
    
        // 构造函数/析构函数
        inline LPF1st();
        inline ~LPF1st() = default;
    
        // 接口方法
        inline auto cutoffFreq() const -> double;
        inline auto setCutoffFreq(double freq) -> void;
        inline auto processing(double raw, double dt, double& processed) -> void;
        inline auto reset() -> void;
    
    private:
        Impl impl_;  // 使用对象实例而非指针
    };
    
    // 构造函数初始化列表
    LPF1st::LPF1st() : impl_{} {
    }
    
    // 成员函数实现
    auto LPF1st::cutoffFreq() const -> double {
        return impl_.cutoff_freq;
    }
    
    auto LPF1st::setCutoffFreq(double freq) -> void {
        impl_.cutoff_freq = freq;
    }
    
    auto LPF1st::processing(double raw, double dt, double& processed) -> void {
        if (impl_.first_value) {
            processed = raw;
            impl_.last_value = raw;
            impl_.first_value = false;
        } else {
            // 正确的一阶低通滤波器实现
            // const double alpha = dt / (1.0/(2*M_PI*impl_.cutoff_freq) + dt);
            // processed = impl_.last_value + alpha * (raw - impl_.last_value);
            // impl_.last_value = processed;

            // 这里的处理方式是基于一阶低通滤波器的公式
            processed = impl_.last_value + dt * (raw - impl_.last_value) * impl_.cutoff_freq;
            impl_.last_value = processed;
        }
    }
    
    auto LPF1st::reset() -> void {
        impl_.first_value = true;
    }

// class LPF3rd
// {
// public:
//     struct LPF3rd
//     {
//         double cutoff_freq{ 0 };
//         std::vector<double> last_value;
//         double A[3][3], B[3];
//         bool first_value{ true };
//     }imp_;

//     LPF3rd(const double cutoff_freq);
//     ~LPF3rd();
//     auto cutoffFreq()->double;
//     auto setCutoffFreq(double cutoff_freq)->void;
//     auto processing(const double raw, const double dt, double* processed)->void;
//     auto reset()->void;
// private:
// }

// auto LPF3rd::cutoffFreq()->double
// {
//     return imp_->cutoff_freq;
// }
// auto LPF3rd::setCutoffFreq(double cutoff_freq)->void
// {
//     imp_->cutoff_freq = cutoff_freq;
//     imp_->A[0][0] = 0; imp_->A[0][1] = 1; imp_->A[0][2] = 0;
//     imp_->A[1][0] = 0; imp_->A[1][1] = 0; imp_->A[1][2] = 1;
//     imp_->A[2][0] = -cutoff_freq * cutoff_freq * cutoff_freq;
//     imp_->A[2][1] = -2 * cutoff_freq * cutoff_freq;
//     imp_->A[2][2] = -2 * cutoff_freq;
//     imp_->B[0] = 0; imp_->B[1] = 0;
//     imp_->B[2] = -imp_->A[2][0];

// }
// auto LPF3rd::processing(const double raw, const double dt, double* processed)->void
// {
//     if (imp_->first_value)
//     {
//         processed[0] = raw; processed[1] = 0; processed[2] = 0;
//         imp_->first_value = false;
//         s_vc(3, processed, imp_->last_value.data());
//     }
//     else {
//         for(Size i=0;i<3;i++)
//             processed[i] = imp_->last_value[i] + dt * (imp_->A[i][0] * imp_->last_value[0] + imp_->A[i][1] * imp_->last_value[1] + imp_->A[i][2] * imp_->last_value[2] + imp_->B[i] * raw);
        
//         s_vc(3, processed, imp_->last_value.data());
//     }
// }
// auto LPF3rd::reset()->void
// {
//     imp_->first_value = true;
// }
// LPF3rd::~LPF3rd() = default;
// LPF3rd::LPF3rd(const double cutoff_freq)
// { 
//     imp_->cutoff_freq = cutoff_freq;
//     imp_->A[0][0] = 0; imp_->A[0][1] = 1; imp_->A[0][2] = 0;
//     imp_->A[1][0] = 0; imp_->A[1][1] = 0; imp_->A[1][2] = 1;
//     imp_->A[2][0] = -cutoff_freq * cutoff_freq * cutoff_freq;
//     imp_->A[2][1] = -2 * cutoff_freq * cutoff_freq;
//     imp_->A[2][2] = -2 * cutoff_freq;
//     imp_->B[0] = 0; imp_->B[1] = 0;
//     imp_->B[2] = -imp_->A[2][0];

//     imp_->last_value.resize(3);
// }
