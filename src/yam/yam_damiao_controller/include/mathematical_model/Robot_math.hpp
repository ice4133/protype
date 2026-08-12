
#include<stdbool.h> //for bool
#include "string.h"
#include "iostream"
typedef std::size_t Size;
auto inline constexpr next_r(Size at, Size ld)noexcept->Size { return at + ld; }
template<typename XType, typename YType>
auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = x[x_id]; }
template<typename XType, typename YType>
auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = alpha * x[x_id]; }
auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy_n(x, n, y); }
auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha * x[i]; }

auto inline s_nv(Size n, double alpha, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] *= alpha; }
template<typename XType, typename YType>
auto inline s_vv(Size n, const double *x, XType x_t, const double *y, YType y_t) noexcept->double { double ret{ 0 }; for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))ret += x[x_id] * y[y_id]; return ret; }
auto inline s_vv(Size n, const double *x, const double *y) noexcept->double { double ret{ 0 }; for (Size i = 0; i < n; ++i)ret += x[i] * y[i];	return ret; }

template<typename XType>
auto inline s_norm(Size n, const double *x, XType x_t) noexcept->double
{
    double norm = 0;
    for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t))norm += x[x_id] * x[x_id];
    return std::sqrt(norm);
}
auto inline s_norm(Size n, const double *x) noexcept->double { return s_norm(n, x, 1); }

inline auto s_pp2pm(const double *pp_in, double *pm_out) noexcept->double *
{
    // 正式开始计算 //
    pm_out[3] = pp_in[0];
    pm_out[7] = pp_in[1];
    pm_out[11] = pp_in[2];

    return pm_out;
}
inline auto s_rq2rm(const double *rq_in, double *rm_out = nullptr, Size rm_ld = 3) noexcept->double *;

inline auto s_rq2rm(const double *rq_in, double *rm_out, Size rm_ld) noexcept->double *
{
    // 正式开始计算 //
    rm_out[0 * rm_ld + 0] = 1 - 2 * rq_in[1] * rq_in[1] - 2 * rq_in[2] * rq_in[2];
    rm_out[0 * rm_ld + 1] = 2 * rq_in[0] * rq_in[1] - 2 * rq_in[3] * rq_in[2];
    rm_out[0 * rm_ld + 2] = 2 * rq_in[0] * rq_in[2] + 2 * rq_in[3] * rq_in[1];

    rm_out[1 * rm_ld + 0] = 2 * rq_in[0] * rq_in[1] + 2 * rq_in[3] * rq_in[2];
    rm_out[1 * rm_ld + 1] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[2] * rq_in[2];
    rm_out[1 * rm_ld + 2] = 2 * rq_in[1] * rq_in[2] - 2 * rq_in[3] * rq_in[0];

    rm_out[2 * rm_ld + 0] = 2 * rq_in[0] * rq_in[2] - 2 * rq_in[3] * rq_in[1];
    rm_out[2 * rm_ld + 1] = 2 * rq_in[1] * rq_in[2] + 2 * rq_in[3] * rq_in[0];
    rm_out[2 * rm_ld + 2] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[1] * rq_in[1];

    return rm_out;
}

auto inline s_pq2pm(const double *pq_in, double *pm_out) noexcept->double *
{

    // 正式开始计算 //
    s_pp2pm(pq_in, pm_out);
    s_rq2rm(pq_in + 3, pm_out,4);

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}

auto inline s_pm_dot_v3(double *pm,double *v3, double *v3_out)
{
    v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
    v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
    v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];
}

auto inline s_inv_pm_dot_v3(const double *inv_pm, const double *v3, double *v3_out) noexcept->double *
{
    v3_out[0] = inv_pm[0] * v3[0] + inv_pm[4] * v3[1] + inv_pm[8] * v3[2];
    v3_out[1] = inv_pm[1] * v3[0] + inv_pm[5] * v3[1] + inv_pm[9] * v3[2];
    v3_out[2] = inv_pm[2] * v3[0] + inv_pm[6] * v3[1] + inv_pm[10] * v3[2];
    return v3_out;
}




auto inline s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out) noexcept->double *
{
    pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
    pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
    pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
    pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

    pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
    pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
    pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
    pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

    pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
    pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
    pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
    pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}

auto inline s_pm2pp(const double *pm_in, double *pp_out) noexcept->double *
{
    // 正式开始计算 //
    pp_out[0] = pm_in[3];
    pp_out[1] = pm_in[7];
    pp_out[2] = pm_in[11];

    return pp_out;
}


auto inline s_rm2rq(const double *rm_in, double *rq_out, Size rm_ld) noexcept->double *
{

    // 正式开始计算 //
    static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
    static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
    static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

    double qt_square[4];

    qt_square[0] = (1 + rm_in[0] - rm_in[rm_ld + 1] - rm_in[2 * rm_ld + 2]) / 4;
    qt_square[1] = (1 + rm_in[rm_ld + 1] - rm_in[0] - rm_in[2 * rm_ld + 2]) / 4;
    qt_square[2] = (1 + rm_in[2 * rm_ld + 2] - rm_in[0] - rm_in[rm_ld + 1]) / 4;
    qt_square[3] = (1 + rm_in[0] + rm_in[rm_ld + 1] + rm_in[2 * rm_ld + 2]) / 4;

    int i = static_cast<int>(std::max_element(qt_square, qt_square + 4) - qt_square);
    rq_out[i] = std::sqrt(qt_square[i]);

    int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };
    for (auto m : jkl)rq_out[m] = (rm_in[P[i][m] * rm_ld + Q[i][m]] + T[i][m] * rm_in[Q[i][m] * rm_ld + P[i][m]]) / 4.0 / rq_out[i];

    // 将rq[3]置为正
    for (auto m = 0; m < 4; ++m)rq_out[m] = rq_out[3] < 0 ? -rq_out[m] : rq_out[m];

    return rq_out;
}
auto inline s_pm2rq(const double *pm_in, double *rq_out) noexcept->double * { return s_rm2rq(pm_in, rq_out, 4); }
auto inline s_pm2pq(const double *pm_in, double *pq_out) noexcept->double *
{

    // 正式开始计算 //
    s_pm2pp(pm_in, pq_out);
    s_pm2rq(pm_in, pq_out + 3);

    return pq_out;
}

//静力学
//tor=J.T*F
//F=J.-T*tor
//三次多项式插值轨迹规划
auto inline CubicSpline(int n, const double *x, const double *y, const double *z, double *p1, double *p2, double *p3)->void
{
    //std::cout <<" START" << std::endl;
    //std::cout << " n:"<<n << std::endl;
    for (int i = 0; i < n - 1; i++) {
        double T = x[i + 1] - x[i];
        p1[i] = z[i];
        p2[i] = (3 * y[i + 1] - 3 * y[i] - 2 * z[i] * T - z[i + 1] * T) / ::std::pow(T, 2);
        p3[i] = (2 * y[i] - 2 * y[i + 1] + z[i] * T + z[i + 1] * T) / ::std::pow(T, 3);
        //	std::cout << "p1 " << p1[i] << "  p2 " << p2[i] << "   p3 " << p3[i] << std::endl;
    }
}

auto inline CubicSpline_at(int n, const double *x, const double *y, const double *z, const double *p1, const double *p2, const double *p3, double xt)->double
{
    auto pos = std::upper_bound(x, x + n - 1, xt);
    std::size_t id = pos == x ? 0 : pos - x - 1;
    double w = xt - x[id];
    return ((w*p3[id] + p2[id])*w + p1[id])*w + y[id];
}

void inline q2dqt(double *ActPq, const double *DesPq, double *dqt, const double KPP)
	{
		double dQuar[4] = { 0 };
		//姿态误差2
		double cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];
		if (cos_theta < 0)
		{
			ActPq[3] = -ActPq[3];
			ActPq[4] = -ActPq[4];
			ActPq[5] = -ActPq[5];
			ActPq[6] = -ActPq[6];
		}
		cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];
		cos_theta = std::max(-1.0, cos_theta);
		cos_theta = std::min(1.0, cos_theta);
		double theta = std::acos(cos_theta);
		double sin_theta = std::sin(theta);

		if (theta < 0.03)
		{
			dQuar[0] = DesPq[3] - ActPq[3];
			dQuar[1] = DesPq[4] - ActPq[4];
			dQuar[2] = DesPq[5] - ActPq[5];
			dQuar[3] = DesPq[6] - ActPq[6];
		}
		else
		{
			dQuar[0] = -(DesPq[3] * cos_theta*(-theta) + theta * ActPq[3]) / sin_theta;
			dQuar[1] = -(DesPq[4] * cos_theta*(-theta) + theta * ActPq[4]) / sin_theta;
			dQuar[2] = -(DesPq[5] * cos_theta*(-theta) + theta * ActPq[5]) / sin_theta;
			dQuar[3] = -(DesPq[6] * cos_theta*(-theta) + theta * ActPq[6]) / sin_theta;
		}

		double norm_dQuar = std::max(1e-7, std::sqrt(s_vv(4, dQuar, dQuar)));
		double unit_dQuar[4] = { 0 };

		for (int i = 0; i < 4; i++)
			unit_dQuar[i] = dQuar[i] / norm_dQuar;

		for (int i = 0; i < 4; i++)
		{
			double dt = std::min(KPP*theta, 0.5);// protect angular velocity target, theta always positive
			dqt[i] = unit_dQuar[i] * dt;
		}
	}

auto inline s_wq2wa(const double *rq_in, const double *wq_in, double *wa_out) noexcept->double *
{
    // 正式开始计算 //
    double p11 = 2 * wq_in[0] * rq_in[0];
    double p22 = 2 * wq_in[1] * rq_in[1];
    double p33 = 2 * wq_in[2] * rq_in[2];
    double p12 = wq_in[0] * rq_in[1] + rq_in[0] * wq_in[1];
    double p13 = wq_in[0] * rq_in[2] + rq_in[0] * wq_in[2];
    double p23 = wq_in[1] * rq_in[2] + rq_in[1] * wq_in[2];
    double p41 = wq_in[3] * rq_in[0] + rq_in[3] * wq_in[0];
    double p42 = wq_in[3] * rq_in[1] + rq_in[3] * wq_in[1];
    double p43 = wq_in[3] * rq_in[2] + rq_in[3] * wq_in[2];

    double rm[3][3];
    s_rq2rm(rq_in, *rm);

    wa_out[0] = 2 * ((p13 - p42)*rm[1][0] + (p23 + p41)*rm[1][1] - (p11 + p22)*rm[1][2]);
    wa_out[1] = 2 * (-(p22 + p33)*rm[2][0] + (p12 - p43)*rm[2][1] + (p13 + p42)*rm[2][2]);
    wa_out[2] = 2 * ((p12 + p43)*rm[0][0] - (p11 + p33)*rm[0][1] + (p23 - p41)*rm[0][2]);

    return wa_out;
}

