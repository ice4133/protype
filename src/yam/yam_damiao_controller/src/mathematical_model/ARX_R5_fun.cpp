#include "mathematical_model/ARX_R5_fun.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace mr;

    ARX_R5_fun::ARX_R5_fun() {
        gravity_vector_ << 0, 0, -9.81; // 设置重力向量
        M_.resize(4, 4);
        Slist_.resize(6, 6);
        // 初始化M和Slist
        M_ << 1, 0, 0, 0.187,
                0, 1, 0, 0,
                0, 0, 1, 0.163,
                0, 0, 0, 1;

        Slist_<< 0,      0,         0,         0,        0,         1,         
            0,       1,        -1,       -1,        0,         0,       
            1,       0,        0,         0,        -1,         0,     
            0 ,      -0.104  ,      0.104   ,     0.163  ,         0    ,      0,
            0 ,           0  ,          0   ,         0  ,     0.075     ,   0.163,
            0  ,       0.02   ,     0.244    ,   -0.001  ,         0     ,       0;
        if (!isDynamicParamModel_)
        {
            std::string package_name = "yam_damiao_controller";
            std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
            std::string config_dir = package_share_dir + "/config/Y_parm_data.xml";
            Y_=loadVectorFromXml(config_dir);
        }
    }
    ARX_R5_fun::~ARX_R5_fun() {
        // 析构函数
    }
    bool allLessThan(const Eigen::VectorXd& vec, double threshold) {
        for (int i = 0; i < vec.size(); ++i) {
            if (vec(i) >= threshold) {
                return false;
            }
        }
        return true;
    }

Eigen::MatrixXd ARX_R5_fun::ARX_R5_FK(const Eigen::VectorXd thetaList){
    Eigen::MatrixXd FKCal = mr::FKinSpace(M_, Slist_, thetaList);
    return FKCal;
}

void ARX_R5_fun::updatArmState(const std::vector<float>& motor_pos, 
                              const std::vector<float>& motor_vel, 
                              const std::vector<float>& motor_tau) {
    if (motor_pos.size() != ARM_DOF || 
        motor_vel.size() != ARM_DOF || 
        motor_tau.size() != ARM_DOF) {
        throw std::invalid_argument("updatArmState: Input vectors must have size " + 
                                   std::to_string(ARM_DOF));
    }
    
    for (int i = 0; i < ARM_DOF; i++) {
        current_joint_pos[i] = motor_pos[i];
        current_joint_vel[i] = motor_vel[i];
        current_joint_tau[i] = motor_tau[i];
    }
}

void ARX_R5_fun::joint2motor(const std::vector<float>& q,
                            const std::vector<float>& dq,
                            const std::vector<float>& ddq,
                            const std::vector<float>& tau_joint,
                            std::vector<float>& motor_pos,
                            std::vector<float>& motor_vel,
                            std::vector<float>& motor_acc,
                            std::vector<float>& tau_motor) {
    // Validate input sizes
    if (q.size() != ARM_DOF || dq.size() != ARM_DOF || 
        ddq.size() != ARM_DOF || tau_joint.size() != ARM_DOF) {
        throw std::invalid_argument("joint2motor: Input vectors must have size " + 
                                   std::to_string(ARM_DOF));
    }
    
    // Resize output vectors
    motor_pos.resize(ARM_DOF);
    motor_vel.resize(ARM_DOF);
    motor_acc.resize(ARM_DOF);
    tau_motor.resize(ARM_DOF);
    
    // Convert joint space to motor space
    for (int i = 0; i < ARM_DOF; i++) {
        motor_pos[i] = q[i] * motor_direction[i];
        motor_vel[i] = dq[i] * motor_direction[i];
        motor_acc[i] = ddq[i] * motor_direction[i];
        tau_motor[i] = tau_joint[i] * motor_direction[i];
    }
}

//传统的逆解+角度限制约束+随机越出策略+迭代时间约束
Eigen::VectorXd ARX_R5_fun::ARX_R5_IK(Eigen::VectorXd thetalist,const Eigen::MatrixXd T){

    Eigen::VectorXd thetalist_now=thetalist;
    double eomg = 0.0005;
    double ev = 0.00005;//线速度0.1对应0.0001
    bool b_result = true;

    int i = 0;
    int maxiterations = 40;
    Eigen::MatrixXd Tfk = FKinSpace(M_, Slist_, thetalist);
    Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
    Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
    Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));
    Eigen::VectorXd q_min(6);
    q_min << -2.618, 0,    0,-1.745, -1.22, -2.0944;
    Eigen::VectorXd q_max(6);
    q_max << 2.618, M_PI, M_PI,    1.745, 1.22, 2.0944;


    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < maxiterations) {
        Js = JacobianSpace(Slist_, thetalist);
        thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
        //关节角度限制及随机种子都参考
        //关节角度限制约束TRAK-IK 的改进牛顿法 ChainIkSolverPos_TL
        for (int i = 0; i < 6; ++i){
            if(thetalist[i]<q_min[i]){
                double diffangle = std::fmod(q_min[i] -thetalist[i], 2 * M_PI);
                double curr_angle = q_min[i] - diffangle + 2 * M_PI;
                if (curr_angle > q_max[i])
                    thetalist[i] = q_min[i];
                else
                    thetalist[i] = curr_angle;
            }
            if(thetalist[i]>q_max[i]){
                // Find actual wrapped angle between limit and joint
                double diffangle = std::fmod(thetalist[i]-q_max[i], 2 * M_PI);
                // Subtract that angle from limit and go into the range by a
                // revolution
                double curr_angle = q_max[i] +diffangle - 2 * M_PI;
                if (curr_angle < q_min[i] )
                    thetalist[i]  = q_max[i] ;
                else
                    thetalist[i]  = curr_angle;
            }
        }
        //局部值陷入最优调出
        bool isFuzzyNull=allLessThan(thetalist-thetalist_now,0.0001);
        if (isFuzzyNull)
        {
            for (int i = 0; i < 6; ++i){
                thetalist[i]= q_min[i] + (double)(rand()) /RAND_MAX * (q_max[i] - q_min[i]);
            }
        }
        std::cout <<" "<<!err<<"  num  "<<i <<" isFuzzyNull "<<isFuzzyNull<<std::endl;
        i += 1;
        // iterate
        Tfk = FKinSpace(M_, Slist_, thetalist);
        Tdiff = TransInv(Tfk)*T;
        Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
        angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
        linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
        err = (angular.norm() > eomg || linear.norm() > ev);
        //暂未添加迭代时间约束
        std::cout << "angular.norm() :" << angular.norm() << std::endl;
        std::cout << "linear.norm() :" << linear.norm() << std::endl;
    }
    if (!err)
    {
        //std::cout << "IKinSpace success" << std::endl;
        // std::cout << "thetaList ans :" << thetalist << std::endl;
        return thetalist;
    }
    else
    {
        std::cout << "IKinSpace failed" << std::endl;
        return thetalist_now;
    }
}

//ARX_R5a模型的world雅可比矩阵
Eigen::MatrixXd ARX_R5_fun::ARX_R5_JacobianSpace(const Eigen::VectorXd thetaList){
    Eigen::MatrixXd result = mr::JacobianSpace(Slist_, thetaList);
    return result;
}

int sign(double value) {
    if (value == 0.0) return 0;
    return (value > 0.0) ? 1 : -1;
}

//方舟无限机械臂 ARX机械臂动力学参数辨识的观测矩阵 观测矩阵建立在TCP上
void ARX_Hb_code( double* H, const double* q, const double* dq, const double* ddq )
{
  double x0 = sin(q[1]);
  double x1 = -ddq[0];
  double x2 = cos(q[1]);
  double x3 = -dq[0];
  double x4 = x2*x3;
  double x5 = dq[1]*x4;
  double x6 = x0*x1 + x5;
  double x7 = -x6;
  double x8 = x0*x3;
  double x9 = dq[1]*x8;
  double x10 = -x9;
  double x11 = x10*x2;
  double x12 = x5 + x6;
  double x13 = -x2;
  double x14 = dq[0]*dq[1]*x0 + x1*x2;
  double x15 = -x0;
  double x16 = ((dq[1])*(dq[1]));
  double x17 = ((x8)*(x8));
  double x18 = x4*x8;
  double x19 = ddq[1] + x18;
  double x20 = -x16;
  double x21 = ((x4)*(x4));
  double x22 = -x18;
  double x23 = -x14;
  double x24 = x23 + x9;
  double x25 = -0.02*ddq[0];
  double x26 = sin(q[2]);
  double x27 = -x26;
  double x28 = cos(q[2]);
  double x29 = -x28;
  double x30 = x27*x8 + x29*x4;
  double x31 = dq[2]*x30 + x23*x26 + x28*x6;
  double x32 = x27*x4 + x28*x8;
  double x33 = -dq[1] + dq[2];
  double x34 = x32*x33;
  double x35 = x30*x33;
  double x36 = x31 + x35;
  double x37 = -x32;
  double x38 = dq[2]*x37 + x23*x28 + x26*x7;
  double x39 = -x34;
  double x40 = x38 + x39;
  double x41 = ((x32)*(x32));
  double x42 = -x41;
  double x43 = ((x33)*(x33));
  double x44 = x42 + x43;
  double x45 = x30*x32;
  double x46 = -ddq[1] + ddq[2];
  double x47 = x45 + x46;
  double x48 = x28*x47;
  double x49 = -x45;
  double x50 = x46 + x49;
  double x51 = -x43;
  double x52 = ((x30)*(x30));
  double x53 = x51 + x52;
  double x54 = -0.264*x24 + x25;
  double x55 = -x54;
  double x56 = -x38;
  double x57 = x34 + x56;
  double x58 = sin(q[3]);
  double x59 = cos(q[3]);
  double x60 = x32*x59 + x33*x58;
  double x61 = x33*x59 + x37*x58;
  double x62 = x60*x61;
  double x63 = -x62;
  double x64 = -x63;
  double x65 = dq[3]*x61 + x31*x59 + x46*x58;
  double x66 = dq[3] - x30;
  double x67 = x60*x66;
  double x68 = -x58;
  double x69 = x59*x65 + x67*x68;
  double x70 = x61*x66;
  double x71 = x65 + x70;
  double x72 = -x60;
  double x73 = dq[3]*x72 - x31*x58 + x46*x59;
  double x74 = -x67;
  double x75 = x73 + x74;
  double x76 = x59*x75 + x68*x71;
  double x77 = ((x60)*(x60));
  double x78 = ((x61)*(x61));
  double x79 = -x78;
  double x80 = x77 + x79;
  double x81 = -x80;
  double x82 = x65 - x70;
  double x83 = -x82;
  double x84 = -x77;
  double x85 = ((x66)*(x66));
  double x86 = x84 + x85;
  double x87 = ddq[3] + x56;
  double x88 = x62 + x87;
  double x89 = x59*x88;
  double x90 = x68*x86 + x89;
  double x91 = -x85;
  double x92 = x78 + x91;
  double x93 = x63 + x87;
  double x94 = x59*x92 + x68*x93;
  double x95 = x67 + x73;
  double x96 = -x95;
  double x97 = x59*x70 + x68*x74;
  double x98 = -x87;
  double x99 = x79 + x91;
  double x100 = x58*x99;
  double x101 = x42 + x51;
  double x102 = -((dq[0])*(dq[0]));
  double x103 = -0.02*x0*x102 - 9.81*x2;
  double x104 = x103 + 0.264*x19;
  double x105 = -9.81*x0 + 0.02*x102*x2;
  double x106 = -x21;
  double x107 = x105 + 0.264*x106 + 0.264*x20;
  double x108 = x104*x29 + x107*x27;
  double x109 = -x108;
  double x110 = 0.245*x101 + x109 - 0.06*x47;
  double x111 = -x110;
  double x112 = -0.245*x100 + x111*x68 - 0.245*x89;
  double x113 = -0.245*x36 + x54 + 0.06*x57;
  double x114 = -x52;
  double x115 = x114 + x51;
  double x116 = -x46;
  double x117 = x116 + x45;
  double x118 = x104*x27 + x107*x28;
  double x119 = 0.06*x115 - 0.245*x117 + x118;
  double x120 = x113*x59 + x119*x68;
  double x121 = -x120;
  double x122 = -0.06*x100 + x121 - 0.06*x89;
  double x123 = x100 + x89;
  double x124 = x84 + x91;
  double x125 = x124*x59;
  double x126 = x62 + x98;
  double x127 = x126*x58;
  double x128 = x110*x59 - 0.245*x125 - 0.245*x127;
  double x129 = x113*x58 + x119*x59;
  double x130 = -x129;
  double x131 = -0.06*x125 - 0.06*x127 - x130;
  double x132 = x125 + x127;
  double x133 = cos(q[4]);
  double x134 = -x133;
  double x135 = sin(q[4]);
  double x136 = -x66;
  double x137 = x133*x60 + x135*x136;
  double x138 = dq[4] + x61;
  double x139 = x137*x138;
  double x140 = x133*x136 + x135*x72;
  double x141 = dq[4]*x140 + x133*x65 + x135*x98;
  double x142 = -x135;
  double x143 = x134*x139 + x141*x142;
  double x144 = -x143;
  double x145 = x133*x141 + x139*x142;
  double x146 = x137*x140;
  double x147 = -x146;
  double x148 = x145*x59 + x147*x68;
  double x149 = -x139;
  double x150 = -x137;
  double x151 = dq[4]*x150 + x133*x98 - x135*x65;
  double x152 = x149 + x151;
  double x153 = x138*x140;
  double x154 = x141 + x153;
  double x155 = x134*x154 + x142*x152;
  double x156 = -x155;
  double x157 = ((x140)*(x140));
  double x158 = -x157;
  double x159 = ((x137)*(x137));
  double x160 = x158 + x159;
  double x161 = x133*x152 + x142*x154;
  double x162 = x160*x68 + x161*x59;
  double x163 = x141 - x153;
  double x164 = ddq[4] + x73;
  double x165 = x146 + x164;
  double x166 = -x159;
  double x167 = ((x138)*(x138));
  double x168 = x166 + x167;
  double x169 = x133*x165 + x142*x168;
  double x170 = x163*x68 + x169*x59;
  double x171 = x142*x165;
  double x172 = x134*x168 + x171;
  double x173 = -x172;
  double x174 = x147 + x164;
  double x175 = -x167;
  double x176 = x157 + x175;
  double x177 = x134*x174 + x142*x176;
  double x178 = -x177;
  double x179 = x133*x176 + x142*x174;
  double x180 = x139 + x151;
  double x181 = x179*x59 + x180*x68;
  double x182 = x133*x153 + x142*x149;
  double x183 = -x164;
  double x184 = x182*x59 + x183*x58;
  double x185 = x134*x149 + x142*x153;
  double x186 = -x185;
  double x187 = x111*x133 + x129*x142;
  double x188 = -x187;
  double x189 = x121*x142;
  double x190 = -x151;
  double x191 = x139 + x190;
  double x192 = x191*x59;
  double x193 = -0.245*x58;
  double x194 = x158 + x175;
  double x195 = x133*x194 + x171;
  double x196 = x188*x58 + x189*x59 - 0.245*x192 + x193*x195;
  double x197 = x121*x134;
  double x198 = x195*x58;
  double x199 = -0.06*x192 - x197 - 0.06*x198;
  double x200 = x192 + x198;
  double x201 = x120*x133;
  double x202 = x110*x142 + x129*x133;
  double x203 = -x202;
  double x204 = x154*x59;
  double x205 = x166 + x175;
  double x206 = x146 + x183;
  double x207 = x133*x206 + x142*x205;
  double x208 = x193*x207 + x201*x59 + x203*x68 - 0.245*x204;
  double x209 = x207*x58;
  double x210 = x120*x142;
  double x211 = -0.06*x204 - 0.06*x209 - x210;
  double x212 = x204 + x209;
  double x213 = cos(q[5]);
  double x214 = sin(q[5]);
  double x215 = x138*x213 + x150*x214;
  double x216 = dq[5]*x215 + x141*x213 + x164*x214;
  double x217 = -x214;
  double x218 = x137*x213 + x138*x214;
  double x219 = dq[5] - x140;
  double x220 = x218*x219;
  double x221 = x213*x216 + x217*x220;
  double x222 = x215*x218;
  double x223 = -x222;
  double x224 = -x223;
  double x225 = x134*x224 + x142*x221;
  double x226 = -x225;
  double x227 = x213*x220 + x214*x216;
  double x228 = x133*x221 + x142*x224;
  double x229 = x227*x68 + x228*x59;
  double x230 = x215*x219;
  double x231 = x216 + x230;
  double x232 = -x218;
  double x233 = dq[5]*x232 - x141*x214 + x164*x213;
  double x234 = -x220;
  double x235 = x233 + x234;
  double x236 = x213*x231 + x214*x235;
  double x237 = x213*x235 + x217*x231;
  double x238 = ((x218)*(x218));
  double x239 = ((x215)*(x215));
  double x240 = -x239;
  double x241 = x238 + x240;
  double x242 = -x241;
  double x243 = x133*x237 + x142*x242;
  double x244 = x236*x68 + x243*x59;
  double x245 = x134*x242 + x142*x237;
  double x246 = -x245;
  double x247 = x216 - x230;
  double x248 = -x247;
  double x249 = -x238;
  double x250 = ((x219)*(x219));
  double x251 = x249 + x250;
  double x252 = ddq[5] + x190;
  double x253 = x222 + x252;
  double x254 = x213*x253;
  double x255 = x217*x251 + x254;
  double x256 = x134*x248 + x142*x255;
  double x257 = -x256;
  double x258 = x133*x255 + x142*x248;
  double x259 = x213*x251 + x214*x253;
  double x260 = x258*x59 + x259*x68;
  double x261 = -x250;
  double x262 = x239 + x261;
  double x263 = x223 + x252;
  double x264 = x213*x262 + x217*x263;
  double x265 = x220 + x233;
  double x266 = -x265;
  double x267 = x133*x264 + x142*x266;
  double x268 = x213*x263 + x214*x262;
  double x269 = x267*x59 + x268*x68;
  double x270 = x134*x266 + x142*x264;
  double x271 = -x270;
  double x272 = -x252;
  double x273 = x213*x230 + x217*x234;
  double x274 = x134*x272 + x142*x273;
  double x275 = -x274;
  double x276 = x213*x234 + x214*x230;
  double x277 = x133*x273 + x142*x272;
  double x278 = x276*x68 + x277*x59;
  double x279 = -x233;
  double x280 = -x220 - x279;
  double x281 = x240 + x261;
  double x282 = x213*x281 + x217*x253;
  double x283 = x133*x282 + x142*x280;
  double x284 = x283*x58;
  double x285 = x120*x213 + x203*x214;
  double x286 = -x285;
  double x287 = -x188;
  double x288 = x217*x287;
  double x289 = x134*x286 + x142*x288;
  double x290 = x214*x281 + x254;
  double x291 = x290*x59;
  double x292 = -0.06*x284 - x289 - 0.06*x291;
  double x293 = x213*x287;
  double x294 = x133*x288 + x142*x286;
  double x295 = x193*x283 - 0.245*x291 + x293*x68 + x294*x59;
  double x296 = x284 + x291;
  double x297 = x249 + x261;
  double x298 = x222 + x272;
  double x299 = x213*x297 + x214*x298;
  double x300 = x299*x59;
  double x301 = -x231;
  double x302 = x213*x298 + x217*x297;
  double x303 = x133*x302 + x142*x301;
  double x304 = x303*x58;
  double x305 = x300 + x304;
  double x306 = x120*x214 + x202*x213;
  double x307 = -x306;
  double x308 = -x307;
  double x309 = x188*x213;
  double x310 = x133*x309 + x142*x308;
  double x311 = x188*x214;
  double x312 = x193*x303 - 0.245*x300 + x310*x59 + x311*x68;
  double x313 = x134*x308 + x142*x309;
  double x314 = -0.06*x300 - 0.06*x304 - x313;
  double x315 = cos(q[6]);
  double x316 = sin(q[6]);
  double x317 = x218*x315 + x219*x316;
  double x318 = dq[6] - x215;
  double x319 = x317*x318;
  double x320 = x219*x315 + x232*x316;
  double x321 = dq[6]*x320 + x216*x315 + x252*x316;
  double x322 = x315*x319 + x316*x321;
  double x323 = -x322;
  double x324 = x317*x320;
  double x325 = -x324;
  double x326 = -x325;
  double x327 = -x316;
  double x328 = x315*x321 + x319*x327;
  double x329 = x213*x328 + x217*x326;
  double x330 = x134*x323 + x142*x329;
  double x331 = -x330;
  double x332 = x213*x326 + x214*x328;
  double x333 = x133*x329 + x142*x323;
  double x334 = x332*x68 + x333*x59;
  double x335 = x318*x320;
  double x336 = x321 + x335;
  double x337 = -x317;
  double x338 = dq[6]*x337 - x216*x316 + x252*x315;
  double x339 = -x319;
  double x340 = x338 + x339;
  double x341 = x315*x340 + x327*x336;
  double x342 = ((x320)*(x320));
  double x343 = -x342;
  double x344 = ((x317)*(x317));
  double x345 = x343 + x344;
  double x346 = -x345;
  double x347 = x213*x341 + x217*x346;
  double x348 = x315*x336 + x316*x340;
  double x349 = -x348;
  double x350 = x134*x349 + x142*x347;
  double x351 = -x350;
  double x352 = x213*x346 + x214*x341;
  double x353 = x133*x347 + x142*x349;
  double x354 = x352*x68 + x353*x59;
  double x355 = x321 - x335;
  double x356 = -x355;
  double x357 = ddq[6] + x279;
  double x358 = x324 + x357;
  double x359 = x315*x358;
  double x360 = -x344;
  double x361 = ((x318)*(x318));
  double x362 = x360 + x361;
  double x363 = x327*x362 + x359;
  double x364 = x213*x356 + x214*x363;
  double x365 = x213*x363 + x217*x356;
  double x366 = x315*x362 + x316*x358;
  double x367 = -x366;
  double x368 = x133*x365 + x142*x367;
  double x369 = x364*x68 + x368*x59;
  double x370 = x134*x367 + x142*x365;
  double x371 = -x370;
  double x372 = x325 + x357;
  double x373 = -x361;
  double x374 = x342 + x373;
  double x375 = x315*x374 + x327*x372;
  double x376 = x319 + x338;
  double x377 = -x376;
  double x378 = x213*x375 + x217*x377;
  double x379 = x315*x372 + x316*x374;
  double x380 = -x379;
  double x381 = x134*x380 + x142*x378;
  double x382 = -x381;
  double x383 = x133*x378 + x142*x380;
  double x384 = x213*x377 + x214*x375;
  double x385 = x383*x59 + x384*x68;
  double x386 = x315*x335 + x327*x339;
  double x387 = -x357;
  double x388 = x213*x387 + x214*x386;
  double x389 = x213*x386 + x217*x387;
  double x390 = x315*x339 + x316*x335;
  double x391 = -x390;
  double x392 = x133*x389 + x142*x391;
  double x393 = x388*x68 + x392*x59;
  double x394 = x134*x391 + x142*x389;
  double x395 = -x394;
  double x396 = x343 + x373;
  double x397 = x315*x396 + x327*x358;
  double x398 = x319 - x338;
  double x399 = -x398;
  double x400 = x213*x399 + x214*x397;
  double x401 = x400*x59;
  double x402 = x213*x397 + x217*x399;
  double x403 = -x316*x396 - x359;
  double x404 = x133*x402 + x142*x403;
  double x405 = x404*x58;
  double x406 = x401 + x405;
  double x407 = -x286;
  double x408 = x327*x407;
  double x409 = x188*x315 + x307*x316;
  double x410 = -x409;
  double x411 = x213*x408 + x217*x410;
  double x412 = x315*x407;
  double x413 = -x412;
  double x414 = x133*x411 + x142*x413;
  double x415 = x213*x410 + x214*x408;
  double x416 = x193*x404 - 0.245*x401 + x414*x59 + x415*x68;
  double x417 = x134*x413 + x142*x411;
  double x418 = -0.06*x401 - 0.06*x405 - x417;
  double x419 = -x336;
  double x420 = x324 + x387;
  double x421 = x360 + x373;
  double x422 = x315*x420 + x327*x421;
  double x423 = x213*x422 + x217*x419;
  double x424 = -x315*x421 - x316*x420;
  double x425 = x133*x423 + x142*x424;
  double x426 = x286*x315;
  double x427 = x188*x316 + x306*x315;
  double x428 = -x427;
  double x429 = -x428;
  double x430 = x213*x429 + x214*x426;
  double x431 = x213*x419 + x214*x422;
  double x432 = x431*x59;
  double x433 = x213*x426 + x217*x429;
  double x434 = x286*x316;
  double x435 = -x434;
  double x436 = x133*x433 + x142*x435;
  double x437 = x193*x425 + x430*x68 - 0.245*x432 + x436*x59;
  double x438 = x134*x435 + x142*x433;
  double x439 = x425*x58;
  double x440 = -0.06*x432 - x438 - 0.06*x439;
  double x441 = x432 + x439;
  double x442 = cos(q[7]);
  double x443 = -x442;
  double x444 = sin(q[7]);
  double x445 = -x318;
  double x446 = x317*x442 + x444*x445;
  double x447 = dq[7] + x320;
  double x448 = x446*x447;
  double x449 = x337*x444 + x442*x445;
  double x450 = dq[7]*x449 + x321*x442 + x387*x444;
  double x451 = -x450;
  double x452 = x443*x448 + x444*x451;
  double x453 = -x452;
  double x454 = -x444;
  double x455 = x442*x450 + x448*x454;
  double x456 = x446*x449;
  double x457 = -x456;
  double x458 = x315*x455 + x327*x457;
  double x459 = x213*x453 + x214*x458;
  double x460 = x213*x458 + x217*x453;
  double x461 = x315*x457 + x316*x455;
  double x462 = -x461;
  double x463 = x133*x460 + x142*x462;
  double x464 = x459*x68 + x463*x59;
  double x465 = x134*x462 + x142*x460;
  double x466 = -x465;
  double x467 = ((x449)*(x449));
  double x468 = -x467;
  double x469 = ((x446)*(x446));
  double x470 = x468 + x469;
  double x471 = -x448;
  double x472 = -x446;
  double x473 = dq[7]*x472 - x321*x444 + x387*x442;
  double x474 = x471 + x473;
  double x475 = x447*x449;
  double x476 = x450 + x475;
  double x477 = x442*x474 + x454*x476;
  double x478 = x315*x477 + x327*x470;
  double x479 = x443*x476 + x454*x474;
  double x480 = -x479;
  double x481 = x213*x478 + x217*x480;
  double x482 = x315*x470 + x316*x477;
  double x483 = -x482;
  double x484 = x133*x481 + x142*x483;
  double x485 = x213*x480 + x214*x478;
  double x486 = x484*x59 + x485*x68;
  double x487 = x134*x483 + x142*x481;
  double x488 = -x487;
  double x489 = ddq[7] + x338;
  double x490 = x456 + x489;
  double x491 = x442*x490;
  double x492 = -x469;
  double x493 = ((x447)*(x447));
  double x494 = x492 + x493;
  double x495 = x454*x494 + x491;
  double x496 = x450 - x475;
  double x497 = x315*x495 + x327*x496;
  double x498 = x454*x490;
  double x499 = x443*x494 + x498;
  double x500 = -x499;
  double x501 = x213*x497 + x217*x500;
  double x502 = x315*x496 + x316*x495;
  double x503 = -x502;
  double x504 = x134*x503 + x142*x501;
  double x505 = -x504;
  double x506 = x133*x501 + x142*x503;
  double x507 = x213*x500 + x214*x497;
  double x508 = x506*x59 + x507*x68;
  double x509 = -x493;
  double x510 = x467 + x509;
  double x511 = x457 + x489;
  double x512 = x443*x511 + x454*x510;
  double x513 = -x512;
  double x514 = x448 + x473;
  double x515 = x442*x510 + x454*x511;
  double x516 = x315*x515 + x327*x514;
  double x517 = x213*x516 + x217*x513;
  double x518 = x315*x514 + x316*x515;
  double x519 = -x518;
  double x520 = x134*x519 + x142*x517;
  double x521 = -x520;
  double x522 = x133*x517 + x142*x519;
  double x523 = x213*x513 + x214*x516;
  double x524 = x522*x59 + x523*x68;
  double x525 = x442*x475 + x454*x471;
  double x526 = x315*x489 + x316*x525;
  double x527 = -x526;
  double x528 = x443*x471 + x454*x475;
  double x529 = -x528;
  double x530 = -x489;
  double x531 = x315*x525 + x316*x530;
  double x532 = x213*x531 + x217*x529;
  double x533 = x133*x532 + x142*x527;
  double x534 = x213*x529 + x214*x531;
  double x535 = x533*x59 + x534*x68;
  double x536 = x134*x527 + x142*x532;
  double x537 = -x536;
  double x538 = x468 + x509;
  double x539 = -x443*x490 - x454*x538;
  double x540 = -x473;
  double x541 = x448 + x540;
  double x542 = x442*x538 + x498;
  double x543 = x315*x542 + x327*x541;
  double x544 = x213*x543 + x217*x539;
  double x545 = -x315*x541 - x316*x542;
  double x546 = x133*x544 + x142*x545;
  double x547 = x546*x58;
  double x548 = 0.074*x358 + x409;
  double x549 = -x548;
  double x550 = x454*x549;
  double x551 = -x286 - 0.074*x398;
  double x552 = 0.074*x396 + x427;
  double x553 = x442*x551 + x454*x552;
  double x554 = 0.074*x444;
  double x555 = 0.074*x491 + x538*x554 + x553;
  double x556 = x315*x555 + x316*x550;
  double x557 = -x556;
  double x558 = x315*x550 + x327*x555;
  double x559 = x443*x549 + 0.074*x541;
  double x560 = -x559;
  double x561 = x213*x558 + x217*x560;
  double x562 = x134*x557 + x142*x561;
  double x563 = x213*x539 + x214*x543;
  double x564 = x563*x59;
  double x565 = -0.06*x547 - x562 - 0.06*x564;
  double x566 = x547 + x564;
  double x567 = x213*x560 + x214*x558;
  double x568 = x133*x561 + x142*x557;
  double x569 = x193*x546 - 0.245*x564 + x567*x68 + x568*x59;
  double x570 = x456 + x530;
  double x571 = x492 + x509;
  double x572 = -x443*x571 - x454*x570;
  double x573 = x442*x570 + x454*x571;
  double x574 = x315*x573 + x327*x476;
  double x575 = x213*x572 + x214*x574;
  double x576 = x575*x59;
  double x577 = x213*x574 + x217*x572;
  double x578 = -x315*x476 - x316*x573;
  double x579 = x133*x577 + x142*x578;
  double x580 = x579*x58;
  double x581 = x576 + x580;
  double x582 = x454*x548 + 0.074*x476;
  double x583 = -x582;
  double x584 = 0.074*x442;
  double x585 = x442*x552 + x444*x551;
  double x586 = -x585;
  double x587 = x554*x570 + x571*x584 + x586;
  double x588 = x442*x548;
  double x589 = x315*x588 + x327*x587;
  double x590 = x213*x583 + x214*x589;
  double x591 = x315*x587 + x316*x588;
  double x592 = -x591;
  double x593 = x213*x589 + x217*x583;
  double x594 = x133*x593 + x142*x592;
  double x595 = x193*x579 - 0.245*x576 + x59*x594 + x590*x68;
  double x596 = x134*x592 + x142*x593;
  double x597 = -0.06*x576 - 0.06*x580 - x596;
  double x598 = sin(q[8]);
  double x599 = -x598;
  double x600 = cos(q[8]);
  double x601 = x446*x600 + x447*x598;
  double x602 = dq[8] - x449;
  double x603 = x601*x602;
  double x604 = x447*x600 + x472*x598;
  double x605 = dq[8]*x604 + x450*x600 + x489*x598;
  double x606 = x599*x603 + x600*x605;
  double x607 = x601*x604;
  double x608 = -x607;
  double x609 = -x608;
  double x610 = x443*x609 + x454*x606;
  double x611 = -x610;
  double x612 = x442*x606 + x454*x609;
  double x613 = x598*x605 + x600*x603;
  double x614 = x315*x612 + x327*x613;
  double x615 = x213*x614 + x217*x611;
  double x616 = x315*x613 + x316*x612;
  double x617 = -x616;
  double x618 = x133*x615 + x142*x617;
  double x619 = x213*x611 + x214*x614;
  double x620 = x59*x618 + x619*x68;
  double x621 = x134*x617 + x142*x615;
  double x622 = -x621;
  double x623 = -x601;
  double x624 = dq[8]*x623 + x451*x598 + x489*x600;
  double x625 = -x603;
  double x626 = x624 + x625;
  double x627 = x602*x604;
  double x628 = x605 + x627;
  double x629 = x598*x626 + x600*x628;
  double x630 = x599*x628 + x600*x626;
  double x631 = ((x604)*(x604));
  double x632 = -x631;
  double x633 = ((x601)*(x601));
  double x634 = x632 + x633;
  double x635 = -x634;
  double x636 = x442*x630 + x454*x635;
  double x637 = x315*x636 + x327*x629;
  double x638 = x443*x635 + x454*x630;
  double x639 = -x638;
  double x640 = x213*x639 + x214*x637;
  double x641 = x213*x637 + x217*x639;
  double x642 = x315*x629 + x316*x636;
  double x643 = -x642;
  double x644 = x133*x641 + x142*x643;
  double x645 = x59*x644 + x640*x68;
  double x646 = x134*x643 + x142*x641;
  double x647 = -x646;
  double x648 = x605 - x627;
  double x649 = -x648;
  double x650 = ddq[8] + x540;
  double x651 = x607 + x650;
  double x652 = x600*x651;
  double x653 = -x633;
  double x654 = ((x602)*(x602));
  double x655 = x653 + x654;
  double x656 = x599*x655 + x652;
  double x657 = x443*x649 + x454*x656;
  double x658 = -x657;
  double x659 = x598*x651 + x600*x655;
  double x660 = x442*x656 + x454*x649;
  double x661 = x315*x660 + x327*x659;
  double x662 = x213*x658 + x214*x661;
  double x663 = x213*x661 + x217*x658;
  double x664 = x315*x659 + x316*x660;
  double x665 = -x664;
  double x666 = x133*x663 + x142*x665;
  double x667 = x59*x666 + x662*x68;
  double x668 = x134*x665 + x142*x663;
  double x669 = -x668;
  double x670 = x603 + x624;
  double x671 = -x670;
  double x672 = -x654;
  double x673 = x631 + x672;
  double x674 = x608 + x650;
  double x675 = x599*x674 + x600*x673;
  double x676 = x443*x671 + x454*x675;
  double x677 = -x676;
  double x678 = x598*x673 + x600*x674;
  double x679 = x442*x675 + x454*x671;
  double x680 = x315*x679 + x327*x678;
  double x681 = x213*x680 + x217*x677;
  double x682 = x315*x678 + x316*x679;
  double x683 = -x682;
  double x684 = x134*x683 + x142*x681;
  double x685 = -x684;
  double x686 = x133*x681 + x142*x683;
  double x687 = x213*x677 + x214*x680;
  double x688 = x59*x686 + x68*x687;
  double x689 = x598*x627 + x600*x625;
  double x690 = x599*x625 + x600*x627;
  double x691 = -x650;
  double x692 = x442*x690 + x454*x691;
  double x693 = x315*x689 + x316*x692;
  double x694 = -x693;
  double x695 = x315*x692 + x327*x689;
  double x696 = x443*x691 + x454*x690;
  double x697 = -x696;
  double x698 = x213*x695 + x217*x697;
  double x699 = x134*x694 + x142*x698;
  double x700 = -x699;
  double x701 = x133*x698 + x142*x694;
  double x702 = x213*x697 + x214*x695;
  double x703 = x59*x701 + x68*x702;
  double x704 = -0.112*x490 - x553;
  double x705 = -x704;
  double x706 = x599*x705;
  double x707 = x632 + x672;
  double x708 = x598*x707;
  double x709 = 0.112*x538 + x585;
  double x710 = 0.112*x541 + x548;
  double x711 = x599*x709 + x600*x710;
  double x712 = -x711;
  double x713 = -0.112*x652 - 0.112*x708 + x712;
  double x714 = x442*x706 + x454*x713;
  double x715 = -x624;
  double x716 = x603 + x715;
  double x717 = x600*x705 - 0.112*x716;
  double x718 = x599*x651 + x600*x707;
  double x719 = -x716;
  double x720 = x554*x718 + x584*x719 + x717;
  double x721 = x315*x720 + x316*x714;
  double x722 = -x721;
  double x723 = x315*x714 + x327*x720;
  double x724 = x652 + x708;
  double x725 = x443*x713 + x454*x706 + 0.074*x724;
  double x726 = -x725;
  double x727 = x213*x723 + x217*x726;
  double x728 = x133*x727 + x142*x722;
  double x729 = x442*x718 + x454*x719;
  double x730 = -x315*x724 - x316*x729;
  double x731 = -x443*x719 - x454*x718;
  double x732 = x315*x729 + x327*x724;
  double x733 = x213*x732 + x217*x731;
  double x734 = x133*x733 + x142*x730;
  double x735 = x213*x726 + x214*x723;
  double x736 = x213*x731 + x214*x732;
  double x737 = x59*x736;
  double x738 = x193*x734 + x59*x728 + x68*x735 - 0.245*x737;
  double x739 = x58*x734;
  double x740 = x737 + x739;
  double x741 = x134*x722 + x142*x727;
  double x742 = -0.06*x737 - 0.06*x739 - x741;
  double x743 = x607 + x691;
  double x744 = x653 + x672;
  double x745 = x599*x744 + x600*x743;
  double x746 = -x628;
  double x747 = x442*x745 + x454*x746;
  double x748 = x598*x743;
  double x749 = x600*x744;
  double x750 = x748 + x749;
  double x751 = -x315*x750 - x316*x747;
  double x752 = -x443*x746 - x454*x745;
  double x753 = x315*x747 + x327*x750;
  double x754 = x213*x753 + x217*x752;
  double x755 = x133*x754 + x142*x751;
  double x756 = x58*x755;
  double x757 = x213*x752 + x214*x753;
  double x758 = x59*x757;
  double x759 = x756 + x758;
  double x760 = x598*x704 - 0.112*x628;
  double x761 = x554*x745 + x584*x746 + x760;
  double x762 = x600*x704;
  double x763 = x598*x710 + x600*x709;
  double x764 = -x763;
  double x765 = -0.112*x748 - 0.112*x749 - x764;
  double x766 = x442*x762 + x454*x765;
  double x767 = x315*x766 + x327*x761;
  double x768 = x443*x765 + x454*x762 + 0.074*x750;
  double x769 = -x768;
  double x770 = x213*x769 + x214*x767;
  double x771 = x213*x767 + x217*x769;
  double x772 = x315*x761 + x316*x766;
  double x773 = -x772;
  double x774 = x133*x771 + x142*x773;
  double x775 = x193*x755 + x59*x774 + x68*x770 - 0.245*x758;
  double x776 = x134*x773 + x142*x771;
  double x777 = -0.06*x756 - 0.06*x758 - x776;
  double x778 = cos(q[9]);
  double x779 = sin(q[9]);
  double x780 = x602*x778 + x623*x779;
  double x781 = dq[9]*x780 + x605*x778 + x650*x779;
  double x782 = -x779;
  double x783 = x601*x778 + x602*x779;
  double x784 = dq[9] - x604;
  double x785 = x783*x784;
  double x786 = x778*x781 + x782*x785;
  double x787 = x780*x783;
  double x788 = -x787;
  double x789 = -x788;
  double x790 = x598*x786 + x600*x789;
  double x791 = x599*x789 + x600*x786;
  double x792 = x778*x785 + x779*x781;
  double x793 = -x792;
  double x794 = x442*x791 + x454*x793;
  double x795 = x315*x790 + x316*x794;
  double x796 = -x795;
  double x797 = x443*x793 + x454*x791;
  double x798 = -x797;
  double x799 = x315*x794 + x327*x790;
  double x800 = x213*x799 + x217*x798;
  double x801 = x133*x800 + x142*x796;
  double x802 = x213*x798 + x214*x799;
  double x803 = x59*x801 + x68*x802;
  double x804 = x134*x796 + x142*x800;
  double x805 = -x804;
  double x806 = x780*x784;
  double x807 = x781 + x806;
  double x808 = -x785;
  double x809 = -dq[9]*x783 - x605*x779 + x650*x778;
  double x810 = x808 + x809;
  double x811 = x778*x810 + x782*x807;
  double x812 = ((x780)*(x780));
  double x813 = -x812;
  double x814 = ((x783)*(x783));
  double x815 = x813 + x814;
  double x816 = -x815;
  double x817 = x598*x811 + x600*x816;
  double x818 = x599*x816 + x600*x811;
  double x819 = x778*x807 + x779*x810;
  double x820 = -x819;
  double x821 = x442*x818 + x454*x820;
  double x822 = x315*x821 + x327*x817;
  double x823 = x443*x820 + x454*x818;
  double x824 = -x823;
  double x825 = x213*x824 + x214*x822;
  double x826 = x213*x822 + x217*x824;
  double x827 = x315*x817 + x316*x821;
  double x828 = -x827;
  double x829 = x133*x826 + x142*x828;
  double x830 = x59*x829 + x68*x825;
  double x831 = x134*x828 + x142*x826;
  double x832 = -x831;
  double x833 = ddq[9] + x715;
  double x834 = x787 + x833;
  double x835 = x778*x834;
  double x836 = ((x784)*(x784));
  double x837 = -x814;
  double x838 = x836 + x837;
  double x839 = x782*x838 + x835;
  double x840 = x781 - x806;
  double x841 = -x840;
  double x842 = x598*x839 + x600*x841;
  double x843 = x599*x841 + x600*x839;
  double x844 = x778*x838 + x779*x834;
  double x845 = -x844;
  double x846 = x442*x843 + x454*x845;
  double x847 = x315*x842 + x316*x846;
  double x848 = -x847;
  double x849 = x315*x846 + x327*x842;
  double x850 = x443*x845 + x454*x843;
  double x851 = -x850;
  double x852 = x213*x849 + x217*x851;
  double x853 = x134*x848 + x142*x852;
  double x854 = -x853;
  double x855 = x213*x851 + x214*x849;
  double x856 = x133*x852 + x142*x848;
  double x857 = x59*x856 + x68*x855;
  double x858 = -x836;
  double x859 = x812 + x858;
  double x860 = x788 + x833;
  double x861 = x778*x859 + x782*x860;
  double x862 = x785 + x809;
  double x863 = -x862;
  double x864 = x599*x863 + x600*x861;
  double x865 = x778*x860 + x779*x859;
  double x866 = -x865;
  double x867 = x443*x866 + x454*x864;
  double x868 = -x867;
  double x869 = x442*x864 + x454*x866;
  double x870 = x598*x861 + x600*x863;
  double x871 = x315*x869 + x327*x870;
  double x872 = x213*x871 + x217*x868;
  double x873 = x315*x870 + x316*x869;
  double x874 = -x873;
  double x875 = x134*x874 + x142*x872;
  double x876 = -x875;
  double x877 = x133*x872 + x142*x874;
  double x878 = x213*x868 + x214*x871;
  double x879 = x59*x877 + x68*x878;
  double x880 = x778*x806 + x782*x808;
  double x881 = -x833;
  double x882 = x599*x881 + x600*x880;
  double x883 = x778*x808 + x779*x806;
  double x884 = -x883;
  double x885 = x442*x882 + x454*x884;
  double x886 = x598*x880 + x600*x881;
  double x887 = x315*x885 + x327*x886;
  double x888 = x443*x884 + x454*x882;
  double x889 = -x888;
  double x890 = x213*x887 + x217*x889;
  double x891 = x315*x886 + x316*x885;
  double x892 = -x891;
  double x893 = x134*x892 + x142*x890;
  double x894 = -x893;
  double x895 = x213*x889 + x214*x887;
  double x896 = x133*x890 + x142*x892;
  double x897 = x59*x896 + x68*x895;
  double x898 = x813 + x858;
  double x899 = x779*x898 + x835;
  double x900 = -x899;
  double x901 = -x785 + x809;
  double x902 = x778*x898 + x782*x834;
  double x903 = x599*x901 + x600*x902;
  double x904 = x442*x903 + x454*x900;
  double x905 = x600*x901;
  double x906 = x598*x902;
  double x907 = x905 + x906;
  double x908 = -x315*x907 - x316*x904;
  double x909 = -x443*x900 - x454*x903;
  double x910 = x315*x904 + x327*x907;
  double x911 = x213*x910 + x217*x909;
  double x912 = x133*x911 + x142*x908;
  double x913 = -x712;
  double x914 = x782*x913;
  double x915 = x704*x778 + x764*x779;
  double x916 = -x915;
  double x917 = x598*x914 + x600*x916 - 0.112*x899;
  double x918 = x554*x903 + x584*x900 + x917;
  double x919 = x778*x913;
  double x920 = -0.112*x905 - 0.112*x906 - x919;
  double x921 = x599*x916 + x600*x914;
  double x922 = x442*x921 + x454*x920;
  double x923 = x315*x918 + x316*x922;
  double x924 = -x923;
  double x925 = x315*x922 + x327*x918;
  double x926 = x443*x920 + x454*x921 + 0.074*x907;
  double x927 = -x926;
  double x928 = x213*x925 + x217*x927;
  double x929 = x133*x928 + x142*x924;
  double x930 = x213*x927 + x214*x925;
  double x931 = x213*x909 + x214*x910;
  double x932 = x59*x931;
  double x933 = x193*x912 + x59*x929 + x68*x930 - 0.245*x932;
  double x934 = x58*x912;
  double x935 = x134*x924 + x142*x928;
  double x936 = -0.06*x932 - 0.06*x934 - x935;
  double x937 = x932 + x934;
  double x938 = x837 + x858;
  double x939 = x787 + x881;
  double x940 = x778*x939 + x782*x938;
  double x941 = x598*x940;
  double x942 = -x807;
  double x943 = x600*x942;
  double x944 = x941 + x943;
  double x945 = x599*x942 + x600*x940;
  double x946 = x778*x938 + x779*x939;
  double x947 = -x946;
  double x948 = x442*x945 + x454*x947;
  double x949 = x315*x948 + x327*x944;
  double x950 = -x443*x947 - x454*x945;
  double x951 = x213*x950 + x214*x949;
  double x952 = x59*x951;
  double x953 = x213*x949 + x217*x950;
  double x954 = -x315*x944 - x316*x948;
  double x955 = x133*x953 + x142*x954;
  double x956 = x58*x955;
  double x957 = x952 + x956;
  double x958 = -x704*x779 - x763*x778;
  double x959 = -x958;
  double x960 = x712*x778;
  double x961 = x599*x959 + x600*x960;
  double x962 = x712*x779;
  double x963 = -0.112*x941 - 0.112*x943 - x962;
  double x964 = x442*x961 + x454*x963;
  double x965 = x598*x960 + x600*x959 - 0.112*x946;
  double x966 = x554*x945 + x584*x947 + x965;
  double x967 = x315*x964 + x327*x966;
  double x968 = x443*x963 + x454*x961 + 0.074*x944;
  double x969 = -x968;
  double x970 = x213*x967 + x217*x969;
  double x971 = x315*x966 + x316*x964;
  double x972 = -x971;
  double x973 = x133*x970 + x142*x972;
  double x974 = x213*x969 + x214*x967;
  double x975 = x193*x955 + x59*x973 + x68*x974 - 0.245*x952;
  double x976 = x134*x972 + x142*x970;
  double x977 = -0.06*x952 - 0.06*x956 - x976;
  double x978 = x114 + x41;
  double x979 = x31 - x35;
  double x980 = x34 + x38;
  double x981 = -0.264*x26;
  double x982 = -x118;
  double x983 = -0.264*x28;
  double x984 = x58*x65 + x59*x67;
  double x985 = x58*x75 + x59*x71;
  double x986 = x58*x88;
  double x987 = x59*x86 + x986;
  double x988 = x58*x92 + x59*x93;
  double x989 = x58*x70 + x59*x74;
  double x990 = x67 - x73;
  double x991 = x59*x99;
  double x992 = x111*x59 - 0.245*x986 - 0.06*x990 + 0.245*x991;
  double x993 = x126*x59;
  double x994 = x110*x58 + x124*x193 - 0.06*x71 + 0.245*x993;
  double x995 = x145*x58 + x147*x59;
  double x996 = x160*x59 + x161*x58;
  double x997 = x163*x59 + x169*x58;
  double x998 = x179*x58 + x180*x59;
  double x999 = x164*x59 + x182*x58;
  double x1000 = x195*x59;
  double x1001 = x134*x165 + x142*x194;
  double x1002 = 0.245*x1000 - 0.06*x1001 + x187*x59 + x189*x58 + x191*x193;
  double x1003 = x207*x59;
  double x1004 = x134*x205 + x142*x206;
  double x1005 = 0.245*x1003 - 0.06*x1004 + x154*x193 + x201*x58 + x203*x59;
  double x1006 = x227*x59 + x228*x58;
  double x1007 = x236*x59 + x243*x58;
  double x1008 = x258*x58 + x259*x59;
  double x1009 = x267*x58 + x268*x59;
  double x1010 = x276*x59 + x277*x58;
  double x1011 = x283*x59;
  double x1012 = x134*x280 + x142*x282;
  double x1013 = 0.245*x1011 - 0.06*x1012 + x193*x290 + x293*x59 + x294*x58;
  double x1014 = x303*x59;
  double x1015 = x134*x301 + x142*x302;
  double x1016 = 0.245*x1014 - 0.06*x1015 + x193*x299 + x310*x58 + x311*x59;
  double x1017 = x332*x59 + x333*x58;
  double x1018 = x352*x59 + x353*x58;
  double x1019 = x364*x59 + x368*x58;
  double x1020 = x383*x58 + x384*x59;
  double x1021 = x388*x59 + x392*x58;
  double x1022 = x134*x403 + x142*x402;
  double x1023 = x404*x59;
  double x1024 = -0.06*x1022 + 0.245*x1023 + x193*x400 + x414*x58 + x415*x59;
  double x1025 = x134*x424 + x142*x423;
  double x1026 = x425*x59;
  double x1027 = -0.06*x1025 + 0.245*x1026 + x193*x431 + x430*x59 + x436*x58;
  double x1028 = x459*x59 + x463*x58;
  double x1029 = x484*x58 + x485*x59;
  double x1030 = x506*x58 + x507*x59;
  double x1031 = x522*x58 + x523*x59;
  double x1032 = x533*x58 + x534*x59;
  double x1033 = x546*x59;
  double x1034 = x134*x545 + x142*x544;
  double x1035 = 0.245*x1033 - 0.06*x1034 + x193*x563 + x567*x59 + x568*x58;
  double x1036 = x579*x59;
  double x1037 = x134*x578 + x142*x577;
  double x1038 = 0.245*x1036 - 0.06*x1037 + x193*x575 + x58*x594 + x59*x590;
  double x1039 = x58*x618 + x59*x619;
  double x1040 = x58*x644 + x59*x640;
  double x1041 = x58*x666 + x59*x662;
  double x1042 = x58*x686 + x59*x687;
  double x1043 = x58*x701 + x59*x702;
  double x1044 = x59*x734;
  double x1045 = x134*x730 + x142*x733;
  double x1046 = 0.245*x1044 - 0.06*x1045 + x193*x736 + x58*x728 + x59*x735;
  double x1047 = x59*x755;
  double x1048 = x134*x751 + x142*x754;
  double x1049 = 0.245*x1047 - 0.06*x1048 + x193*x757 + x58*x774 + x59*x770;
  double x1050 = x58*x801 + x59*x802;
  double x1051 = x58*x829 + x59*x825;
  double x1052 = x58*x856 + x59*x855;
  double x1053 = x58*x877 + x59*x878;
  double x1054 = x58*x896 + x59*x895;
  double x1055 = x134*x908 + x142*x911;
  double x1056 = x59*x912;
  double x1057 = -0.06*x1055 + 0.245*x1056 + x193*x931 + x58*x929 + x59*x930;
  double x1058 = x134*x954 + x142*x953;
  double x1059 = x59*x955;
  double x1060 = -0.06*x1058 + 0.245*x1059 + x193*x951 + x58*x973 + x59*x974;
    //
  H[0] = ddq[0];
  H[1] = dq[0];
  H[2] = sign(dq[0]);
  H[3] = x0*x7 + x11;
  H[4] = x12*x13 + x15*(x10 + x14);
  H[5] = x13*(x16 - x17) + x15*x19;
  H[6] = x13*(ddq[1] + x22) + x15*(x20 + x21);
  H[7] = -x11 + x15*x5;
  H[8] = x13*x25 + 0.02*x24;
  H[9] = x0*x25 + 0.02*x12;
  H[10] = 0;
  H[11] = 0;
  H[12] = x13*(x27*x31 + x29*x34) + x15*(x27*x34 + x28*x31);
  H[13] = x13*(x27*x40 + x29*x36) + x15*(x27*x36 + x28*x40);
  H[14] = x13*(x27*x47 + x29*x44) + x15*(x27*x44 + x48);
  H[15] = x13*(x27*x53 + x29*x50) + x15*(x27*x50 + x28*x53);
  H[16] = x13*(x27*x35 + x29*x39) + x15*(x27*x39 + x28*x35);
  H[17] = x13*(x29*x55 + 0.264*x57) + x15*x27*x55 - 0.02*x57;
  H[18] = x13*(x27*x54 + 0.264*x36) + x15*x28*x54 - 0.02*x36;
  H[19] = 0;
  H[20] = 0;
  H[21] = x13*(x27*x69 + x29*x64) + x15*(x27*x64 + x28*x69);
  H[22] = x13*(x27*x76 + x29*x81) + x15*(x27*x81 + x28*x76);
  H[23] = x13*(x27*x90 + x29*x83) + x15*(x27*x83 + x28*x90);
  H[24] = x13*(x27*x94 + x29*x96) + x15*(x27*x96 + x28*x94);
  H[25] = x13*(x27*x97 + x29*x98) + x15*(x27*x98 + x28*x97);
  H[26] = -0.02*x123 + x13*(x112*x27 + x122*x29 + 0.264*x123) + x15*(x112*x28 + x122*x27);
  H[27] = x13*(x128*x27 + x131*x29 + 0.264*x132) - 0.02*x132 + x15*(x128*x28 + x131*x27);
  H[28] = 0;
  H[29] = 0;
  H[30] = x13*(x144*x29 + x148*x27) + x15*(x144*x27 + x148*x28);
  H[31] = x13*(x156*x29 + x162*x27) + x15*(x156*x27 + x162*x28);
  H[32] = x13*(x170*x27 + x173*x29) + x15*(x170*x28 + x173*x27);
  H[33] = x13*(x178*x29 + x181*x27) + x15*(x178*x27 + x181*x28);
  H[34] = x13*(x184*x27 + x186*x29) + x15*(x184*x28 + x186*x27);
  H[35] = x13*(x196*x27 + x199*x29 + 0.264*x200) + x15*(x196*x28 + x199*x27) - 0.02*x200;
  H[36] = x13*(x208*x27 + x211*x29 + 0.264*x212) + x15*(x208*x28 + x211*x27) - 0.02*x212;
  H[37] = 0;
  H[38] = 0;
  H[39] = x13*(x226*x29 + x229*x27) + x15*(x226*x27 + x229*x28);
  H[40] = x13*(x244*x27 + x246*x29) + x15*(x244*x28 + x246*x27);
  H[41] = x13*(x257*x29 + x260*x27) + x15*(x257*x27 + x260*x28);
  H[42] = x13*(x269*x27 + x271*x29) + x15*(x269*x28 + x27*x271);
  H[43] = x13*(x27*x278 + x275*x29) + x15*(x27*x275 + x278*x28);
  H[44] = x13*(x27*x295 + x29*x292 + 0.264*x296) + x15*(x27*x292 + x28*x295) - 0.02*x296;
  H[45] = x13*(x27*x312 + x29*x314 + 0.264*x305) + x15*(x27*x314 + x28*x312) - 0.02*x305;
  H[46] = 0;
  H[47] = 0;
  H[48] = x13*(x27*x334 + x29*x331) + x15*(x27*x331 + x28*x334);
  H[49] = x13*(x27*x354 + x29*x351) + x15*(x27*x351 + x28*x354);
  H[50] = x13*(x27*x369 + x29*x371) + x15*(x27*x371 + x28*x369);
  H[51] = x13*(x27*x385 + x29*x382) + x15*(x27*x382 + x28*x385);
  H[52] = x13*(x27*x393 + x29*x395) + x15*(x27*x395 + x28*x393);
  H[53] = x13*(x27*x416 + x29*x418 + 0.264*x406) + x15*(x27*x418 + x28*x416) - 0.02*x406;
  H[54] = x13*(x27*x437 + x29*x440 + 0.264*x441) + x15*(x27*x440 + x28*x437) - 0.02*x441;
  H[55] = 0;
  H[56] = 0;
  H[57] = x13*(x27*x464 + x29*x466) + x15*(x27*x466 + x28*x464);
  H[58] = x13*(x27*x486 + x29*x488) + x15*(x27*x488 + x28*x486);
  H[59] = x13*(x27*x508 + x29*x505) + x15*(x27*x505 + x28*x508);
  H[60] = x13*(x27*x524 + x29*x521) + x15*(x27*x521 + x28*x524);
  H[61] = x13*(x27*x535 + x29*x537) + x15*(x27*x537 + x28*x535);
  H[62] = x13*(x27*x569 + x29*x565 + 0.264*x566) + x15*(x27*x565 + x28*x569) - 0.02*x566;
  H[63] = x13*(x27*x595 + x29*x597 + 0.264*x581) + x15*(x27*x597 + x28*x595) - 0.02*x581;
  H[64] = 0;
  H[65] = 0;
  H[66] = x13*(x27*x620 + x29*x622) + x15*(x27*x622 + x28*x620);
  H[67] = x13*(x27*x645 + x29*x647) + x15*(x27*x647 + x28*x645);
  H[68] = x13*(x27*x667 + x29*x669) + x15*(x27*x669 + x28*x667);
  H[69] = x13*(x27*x688 + x29*x685) + x15*(x27*x685 + x28*x688);
  H[70] = x13*(x27*x703 + x29*x700) + x15*(x27*x700 + x28*x703);
  H[71] = x13*(x27*x738 + x29*x742 + 0.264*x740) + x15*(x27*x742 + x28*x738) - 0.02*x740;
  H[72] = x13*(x27*x775 + x29*x777 + 0.264*x759) + x15*(x27*x777 + x28*x775) - 0.02*x759;
  H[73] = 0;
  H[74] = 0;
  H[75] = x13*(x27*x803 + x29*x805) + x15*(x27*x805 + x28*x803);
  H[76] = x13*(x27*x830 + x29*x832) + x15*(x27*x832 + x28*x830);
  H[77] = x13*(x27*x857 + x29*x854) + x15*(x27*x854 + x28*x857);
  H[78] = x13*(x27*x879 + x29*x876) + x15*(x27*x876 + x28*x879);
  H[79] = x13*(x27*x897 + x29*x894) + x15*(x27*x894 + x28*x897);
  H[80] = x13*(x27*x933 + x29*x936 + 0.264*x937) + x15*(x27*x936 + x28*x933) - 0.02*x937;
  H[81] = x13*(x27*x975 + x29*x977 + 0.264*x957) + x15*(x27*x977 + x28*x975) - 0.02*x957;
  H[82] = 0;
  H[83] = 0;
  H[84] = 0;
  H[85] = 0;
  H[86] = 0;
  H[87] = x22;
  H[88] = x106 + x17;
  H[89] = -x5 + x6;
  H[90] = x14 + x9;
  H[91] = ddq[1];
  H[92] = x103;
  H[93] = -x105;
  H[94] = dq[1];
  H[95] = sign(dq[1]);
  H[96] = -x49;
  H[97] = -x978;
  H[98] = -x979;
  H[99] = -x980;
  H[100] = x116;
  H[101] = x109 + x115*x981 - 0.264*x48;
  H[102] = x101*x983 + x117*x981 - x982;
  H[103] = 0;
  H[104] = 0;
  H[105] = -x984;
  H[106] = -x985;
  H[107] = -x987;
  H[108] = -x988;
  H[109] = -x989;
  H[110] = x981*(x68*x88 + x991) - x983*x990 - x992;
  H[111] = -x71*x983 + x981*(x124*x68 + x993) - x994;
  H[112] = 0;
  H[113] = 0;
  H[114] = -x995;
  H[115] = -x996;
  H[116] = -x997;
  H[117] = -x998;
  H[118] = -x999;
  H[119] = -x1001*x983 - x1002 + x981*(x1000 + x191*x68);
  H[120] = -x1004*x983 - x1005 + x981*(x1003 + x154*x68);
  H[121] = 0;
  H[122] = 0;
  H[123] = -x1006;
  H[124] = -x1007;
  H[125] = -x1008;
  H[126] = -x1009;
  H[127] = -x1010;
  H[128] = -x1012*x983 - x1013 + x981*(x1011 + x290*x68);
  H[129] = -x1015*x983 - x1016 + x981*(x1014 + x299*x68);
  H[130] = 0;
  H[131] = 0;
  H[132] = -x1017;
  H[133] = -x1018;
  H[134] = -x1019;
  H[135] = -x1020;
  H[136] = -x1021;
  H[137] = -x1022*x983 - x1024 + x981*(x1023 + x400*x68);
  H[138] = -x1025*x983 - x1027 + x981*(x1026 + x431*x68);
  H[139] = 0;
  H[140] = 0;
  H[141] = -x1028;
  H[142] = -x1029;
  H[143] = -x1030;
  H[144] = -x1031;
  H[145] = -x1032;
  H[146] = -x1034*x983 - x1035 + x981*(x1033 + x563*x68);
  H[147] = -x1037*x983 - x1038 + x981*(x1036 + x575*x68);
  H[148] = 0;
  H[149] = 0;
  H[150] = -x1039;
  H[151] = -x1040;
  H[152] = -x1041;
  H[153] = -x1042;
  H[154] = -x1043;
  H[155] = -x1045*x983 - x1046 + x981*(x1044 + x68*x736);
  H[156] = -x1048*x983 - x1049 + x981*(x1047 + x68*x757);
  H[157] = 0;
  H[158] = 0;
  H[159] = -x1050;
  H[160] = -x1051;
  H[161] = -x1052;
  H[162] = -x1053;
  H[163] = -x1054;
  H[164] = -x1055*x983 - x1057 + x981*(x1056 + x68*x931);
  H[165] = -x1058*x983 - x1060 + x981*(x1059 + x68*x951);
  H[166] = 0;
  H[167] = 0;
  H[168] = 0;
  H[169] = 0;
  H[170] = 0;
  H[171] = 0;
  H[172] = 0;
  H[173] = 0;
  H[174] = 0;
  H[175] = 0;
  H[176] = 0;
  H[177] = 0;
  H[178] = 0;
  H[179] = 0;
  H[180] = x49;
  H[181] = x978;
  H[182] = x979;
  H[183] = x980;
  H[184] = x46;
  H[185] = x108;
  H[186] = x982;
  H[187] = dq[2];
  H[188] = sign(dq[2]);
  H[189] = x984;
  H[190] = x985;
  H[191] = x987;
  H[192] = x988;
  H[193] = x989;
  H[194] = x992;
  H[195] = x994;
  H[196] = 0;
  H[197] = 0;
  H[198] = x995;
  H[199] = x996;
  H[200] = x997;
  H[201] = x998;
  H[202] = x999;
  H[203] = x1002;
  H[204] = x1005;
  H[205] = 0;
  H[206] = 0;
  H[207] = x1006;
  H[208] = x1007;
  H[209] = x1008;
  H[210] = x1009;
  H[211] = x1010;
  H[212] = x1013;
  H[213] = x1016;
  H[214] = 0;
  H[215] = 0;
  H[216] = x1017;
  H[217] = x1018;
  H[218] = x1019;
  H[219] = x1020;
  H[220] = x1021;
  H[221] = x1024;
  H[222] = x1027;
  H[223] = 0;
  H[224] = 0;
  H[225] = x1028;
  H[226] = x1029;
  H[227] = x1030;
  H[228] = x1031;
  H[229] = x1032;
  H[230] = x1035;
  H[231] = x1038;
  H[232] = 0;
  H[233] = 0;
  H[234] = x1039;
  H[235] = x1040;
  H[236] = x1041;
  H[237] = x1042;
  H[238] = x1043;
  H[239] = x1046;
  H[240] = x1049;
  H[241] = 0;
  H[242] = 0;
  H[243] = x1050;
  H[244] = x1051;
  H[245] = x1052;
  H[246] = x1053;
  H[247] = x1054;
  H[248] = x1057;
  H[249] = x1060;
  H[250] = 0;
  H[251] = 0;
  H[252] = 0;
  H[253] = 0;
  H[254] = 0;
  H[255] = 0;
  H[256] = 0;
  H[257] = 0;
  H[258] = 0;
  H[259] = 0;
  H[260] = 0;
  H[261] = 0;
  H[262] = 0;
  H[263] = 0;
  H[264] = 0;
  H[265] = 0;
  H[266] = 0;
  H[267] = 0;
  H[268] = 0;
  H[269] = 0;
  H[270] = 0;
  H[271] = 0;
  H[272] = 0;
  H[273] = x63;
  H[274] = x80;
  H[275] = x82;
  H[276] = x95;
  H[277] = x87;
  H[278] = x120;
  H[279] = x130;
  H[280] = dq[3];
  H[281] = sign(dq[3]);
  H[282] = x143;
  H[283] = x155;
  H[284] = x172;
  H[285] = x177;
  H[286] = x185;
  H[287] = x197;
  H[288] = x210;
  H[289] = 0;
  H[290] = 0;
  H[291] = x225;
  H[292] = x245;
  H[293] = x256;
  H[294] = x270;
  H[295] = x274;
  H[296] = x289;
  H[297] = x313;
  H[298] = 0;
  H[299] = 0;
  H[300] = x330;
  H[301] = x350;
  H[302] = x370;
  H[303] = x381;
  H[304] = x394;
  H[305] = x417;
  H[306] = x438;
  H[307] = 0;
  H[308] = 0;
  H[309] = x465;
  H[310] = x487;
  H[311] = x504;
  H[312] = x520;
  H[313] = x536;
  H[314] = x562;
  H[315] = x596;
  H[316] = 0;
  H[317] = 0;
  H[318] = x621;
  H[319] = x646;
  H[320] = x668;
  H[321] = x684;
  H[322] = x699;
  H[323] = x741;
  H[324] = x776;
  H[325] = 0;
  H[326] = 0;
  H[327] = x804;
  H[328] = x831;
  H[329] = x853;
  H[330] = x875;
  H[331] = x893;
  H[332] = x935;
  H[333] = x976;
  H[334] = 0;
  H[335] = 0;
  H[336] = 0;
  H[337] = 0;
  H[338] = 0;
  H[339] = 0;
  H[340] = 0;
  H[341] = 0;
  H[342] = 0;
  H[343] = 0;
  H[344] = 0;
  H[345] = 0;
  H[346] = 0;
  H[347] = 0;
  H[348] = 0;
  H[349] = 0;
  H[350] = 0;
  H[351] = 0;
  H[352] = 0;
  H[353] = 0;
  H[354] = 0;
  H[355] = 0;
  H[356] = 0;
  H[357] = 0;
  H[358] = 0;
  H[359] = 0;
  H[360] = 0;
  H[361] = 0;
  H[362] = 0;
  H[363] = 0;
  H[364] = 0;
  H[365] = 0;
  H[366] = x147;
  H[367] = x160;
  H[368] = x163;
  H[369] = x180;
  H[370] = x164;
  H[371] = x187;
  H[372] = x203;
  H[373] = dq[4];
  H[374] = sign(dq[4]);
  H[375] = x227;
  H[376] = x236;
  H[377] = x259;
  H[378] = x268;
  H[379] = x276;
  H[380] = x293;
  H[381] = x311;
  H[382] = 0;
  H[383] = 0;
  H[384] = x332;
  H[385] = x352;
  H[386] = x364;
  H[387] = x384;
  H[388] = x388;
  H[389] = x415;
  H[390] = x430;
  H[391] = 0;
  H[392] = 0;
  H[393] = x459;
  H[394] = x485;
  H[395] = x507;
  H[396] = x523;
  H[397] = x534;
  H[398] = x567;
  H[399] = x590;
  H[400] = 0;
  H[401] = 0;
  H[402] = x619;
  H[403] = x640;
  H[404] = x662;
  H[405] = x687;
  H[406] = x702;
  H[407] = x735;
  H[408] = x770;
  H[409] = 0;
  H[410] = 0;
  H[411] = x802;
  H[412] = x825;
  H[413] = x855;
  H[414] = x878;
  H[415] = x895;
  H[416] = x930;
  H[417] = x974;
  H[418] = 0;
  H[419] = 0;
  H[420] = 0;
  H[421] = 0;
  H[422] = 0;
  H[423] = 0;
  H[424] = 0;
  H[425] = 0;
  H[426] = 0;
  H[427] = 0;
  H[428] = 0;
  H[429] = 0;
  H[430] = 0;
  H[431] = 0;
  H[432] = 0;
  H[433] = 0;
  H[434] = 0;
  H[435] = 0;
  H[436] = 0;
  H[437] = 0;
  H[438] = 0;
  H[439] = 0;
  H[440] = 0;
  H[441] = 0;
  H[442] = 0;
  H[443] = 0;
  H[444] = 0;
  H[445] = 0;
  H[446] = 0;
  H[447] = 0;
  H[448] = 0;
  H[449] = 0;
  H[450] = 0;
  H[451] = 0;
  H[452] = 0;
  H[453] = 0;
  H[454] = 0;
  H[455] = 0;
  H[456] = 0;
  H[457] = 0;
  H[458] = 0;
  H[459] = x223;
  H[460] = x241;
  H[461] = x247;
  H[462] = x265;
  H[463] = x252;
  H[464] = x285;
  H[465] = x307;
  H[466] = dq[5];
  H[467] = sign(dq[5]);
  H[468] = x322;
  H[469] = x348;
  H[470] = x366;
  H[471] = x379;
  H[472] = x390;
  H[473] = x412;
  H[474] = x434;
  H[475] = 0;
  H[476] = 0;
  H[477] = x461;
  H[478] = x482;
  H[479] = x502;
  H[480] = x518;
  H[481] = x526;
  H[482] = x556;
  H[483] = x591;
  H[484] = 0;
  H[485] = 0;
  H[486] = x616;
  H[487] = x642;
  H[488] = x664;
  H[489] = x682;
  H[490] = x693;
  H[491] = x721;
  H[492] = x772;
  H[493] = 0;
  H[494] = 0;
  H[495] = x795;
  H[496] = x827;
  H[497] = x847;
  H[498] = x873;
  H[499] = x891;
  H[500] = x923;
  H[501] = x971;
  H[502] = 0;
  H[503] = 0;
  H[504] = 0;
  H[505] = 0;
  H[506] = 0;
  H[507] = 0;
  H[508] = 0;
  H[509] = 0;
  H[510] = 0;
  H[511] = 0;
  H[512] = 0;
  H[513] = 0;
  H[514] = 0;
  H[515] = 0;
  H[516] = 0;
  H[517] = 0;
  H[518] = 0;
  H[519] = 0;
  H[520] = 0;
  H[521] = 0;
  H[522] = 0;
  H[523] = 0;
  H[524] = 0;
  H[525] = 0;
  H[526] = 0;
  H[527] = 0;
  H[528] = 0;
  H[529] = 0;
  H[530] = 0;
  H[531] = 0;
  H[532] = 0;
  H[533] = 0;
  H[534] = 0;
  H[535] = 0;
  H[536] = 0;
  H[537] = 0;
  H[538] = 0;
  H[539] = 0;
  H[540] = 0;
  H[541] = 0;
  H[542] = 0;
  H[543] = 0;
  H[544] = 0;
  H[545] = 0;
  H[546] = 0;
  H[547] = 0;
  H[548] = 0;
  H[549] = 0;
  H[550] = 0;
  H[551] = 0;
  H[552] = x325;
  H[553] = x345;
  H[554] = x355;
  H[555] = x376;
  H[556] = x357;
  H[557] = x409;
  H[558] = x428;
  H[559] = dq[6];
  H[560] = sign(dq[6]);
  H[561] = x452;
  H[562] = x479;
  H[563] = x499;
  H[564] = x512;
  H[565] = x528;
  H[566] = x559;
  H[567] = x582;
  H[568] = 0;
  H[569] = 0;
  H[570] = x610;
  H[571] = x638;
  H[572] = x657;
  H[573] = x676;
  H[574] = x696;
  H[575] = x725;
  H[576] = x768;
  H[577] = 0;
  H[578] = 0;
  H[579] = x797;
  H[580] = x823;
  H[581] = x850;
  H[582] = x867;
  H[583] = x888;
  H[584] = x926;
  H[585] = x968;
  H[586] = 0;
  H[587] = 0;
  H[588] = 0;
  H[589] = 0;
  H[590] = 0;
  H[591] = 0;
  H[592] = 0;
  H[593] = 0;
  H[594] = 0;
  H[595] = 0;
  H[596] = 0;
  H[597] = 0;
  H[598] = 0;
  H[599] = 0;
  H[600] = 0;
  H[601] = 0;
  H[602] = 0;
  H[603] = 0;
  H[604] = 0;
  H[605] = 0;
  H[606] = 0;
  H[607] = 0;
  H[608] = 0;
  H[609] = 0;
  H[610] = 0;
  H[611] = 0;
  H[612] = 0;
  H[613] = 0;
  H[614] = 0;
  H[615] = 0;
  H[616] = 0;
  H[617] = 0;
  H[618] = 0;
  H[619] = 0;
  H[620] = 0;
  H[621] = 0;
  H[622] = 0;
  H[623] = 0;
  H[624] = 0;
  H[625] = 0;
  H[626] = 0;
  H[627] = 0;
  H[628] = 0;
  H[629] = 0;
  H[630] = 0;
  H[631] = 0;
  H[632] = 0;
  H[633] = 0;
  H[634] = 0;
  H[635] = 0;
  H[636] = 0;
  H[637] = 0;
  H[638] = 0;
  H[639] = 0;
  H[640] = 0;
  H[641] = 0;
  H[642] = 0;
  H[643] = 0;
  H[644] = 0;
  H[645] = x457;
  H[646] = x470;
  H[647] = x496;
  H[648] = x514;
  H[649] = x489;
  H[650] = x553;
  H[651] = x586;
  H[652] = dq[7];
  H[653] = sign(dq[7]);
  H[654] = x613;
  H[655] = x629;
  H[656] = x659;
  H[657] = x678;
  H[658] = x689;
  H[659] = x717;
  H[660] = x760;
  H[661] = 0;
  H[662] = 0;
  H[663] = x790;
  H[664] = x817;
  H[665] = x842;
  H[666] = x870;
  H[667] = x886;
  H[668] = x917;
  H[669] = x965;
  H[670] = 0;
  H[671] = 0;
  H[672] = 0;
  H[673] = 0;
  H[674] = 0;
  H[675] = 0;
  H[676] = 0;
  H[677] = 0;
  H[678] = 0;
  H[679] = 0;
  H[680] = 0;
  H[681] = 0;
  H[682] = 0;
  H[683] = 0;
  H[684] = 0;
  H[685] = 0;
  H[686] = 0;
  H[687] = 0;
  H[688] = 0;
  H[689] = 0;
  H[690] = 0;
  H[691] = 0;
  H[692] = 0;
  H[693] = 0;
  H[694] = 0;
  H[695] = 0;
  H[696] = 0;
  H[697] = 0;
  H[698] = 0;
  H[699] = 0;
  H[700] = 0;
  H[701] = 0;
  H[702] = 0;
  H[703] = 0;
  H[704] = 0;
  H[705] = 0;
  H[706] = 0;
  H[707] = 0;
  H[708] = 0;
  H[709] = 0;
  H[710] = 0;
  H[711] = 0;
  H[712] = 0;
  H[713] = 0;
  H[714] = 0;
  H[715] = 0;
  H[716] = 0;
  H[717] = 0;
  H[718] = 0;
  H[719] = 0;
  H[720] = 0;
  H[721] = 0;
  H[722] = 0;
  H[723] = 0;
  H[724] = 0;
  H[725] = 0;
  H[726] = 0;
  H[727] = 0;
  H[728] = 0;
  H[729] = 0;
  H[730] = 0;
  H[731] = 0;
  H[732] = 0;
  H[733] = 0;
  H[734] = 0;
  H[735] = 0;
  H[736] = 0;
  H[737] = 0;
  H[738] = x608;
  H[739] = x634;
  H[740] = x648;
  H[741] = x670;
  H[742] = x650;
  H[743] = x711;
  H[744] = x764;
  H[745] = dq[8];
  H[746] = sign(dq[8]);
  H[747] = x792;
  H[748] = x819;
  H[749] = x844;
  H[750] = x865;
  H[751] = x883;
  H[752] = x919;
  H[753] = x962;
  H[754] = 0;
  H[755] = 0;
  H[756] = 0;
  H[757] = 0;
  H[758] = 0;
  H[759] = 0;
  H[760] = 0;
  H[761] = 0;
  H[762] = 0;
  H[763] = 0;
  H[764] = 0;
  H[765] = 0;
  H[766] = 0;
  H[767] = 0;
  H[768] = 0;
  H[769] = 0;
  H[770] = 0;
  H[771] = 0;
  H[772] = 0;
  H[773] = 0;
  H[774] = 0;
  H[775] = 0;
  H[776] = 0;
  H[777] = 0;
  H[778] = 0;
  H[779] = 0;
  H[780] = 0;
  H[781] = 0;
  H[782] = 0;
  H[783] = 0;
  H[784] = 0;
  H[785] = 0;
  H[786] = 0;
  H[787] = 0;
  H[788] = 0;
  H[789] = 0;
  H[790] = 0;
  H[791] = 0;
  H[792] = 0;
  H[793] = 0;
  H[794] = 0;
  H[795] = 0;
  H[796] = 0;
  H[797] = 0;
  H[798] = 0;
  H[799] = 0;
  H[800] = 0;
  H[801] = 0;
  H[802] = 0;
  H[803] = 0;
  H[804] = 0;
  H[805] = 0;
  H[806] = 0;
  H[807] = 0;
  H[808] = 0;
  H[809] = 0;
  H[810] = 0;
  H[811] = 0;
  H[812] = 0;
  H[813] = 0;
  H[814] = 0;
  H[815] = 0;
  H[816] = 0;
  H[817] = 0;
  H[818] = 0;
  H[819] = 0;
  H[820] = 0;
  H[821] = 0;
  H[822] = 0;
  H[823] = 0;
  H[824] = 0;
  H[825] = 0;
  H[826] = 0;
  H[827] = 0;
  H[828] = 0;
  H[829] = 0;
  H[830] = 0;
  H[831] = x788;
  H[832] = x815;
  H[833] = x840;
  H[834] = x862;
  H[835] = x833;
  H[836] = x915;
  H[837] = x958;
  H[838] = dq[9];
  H[839] = sign(dq[9]);

} 



//获取Piper手臂观测矩阵
Eigen::MatrixXd ARX_R5_fun::Get_ARX_Hb_fir(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist){
    
    //首先构建手臂DH参数模型，再利用SymPyBotics工具求解观测矩阵与最小惯性参数集
    //UR5手臂最小惯性参数集数量为84    考虑摩擦力情况下

    double q[10]{0};
    double dq[10]{0};
    double ddq[10]{0};
    double q_trans[10]{0};
    double dq_trans[10]{0};
    double ddq_trans[10]{0};
    //piper DH模型的关节偏移
    //建立DH模型与实际关节角度的对应关系
    //实际机械臂joint_dof=6  建立DHmdel的joint_dof=10 
    //q5对应joint4   q8对应joint5  q10对应joint5
    //关节偏移 3 7 9 轴offset 90°
    //实际电机角度与之对应关系
    //theta1=[joint1,joint2-pi,joint3+90/180*pi-pi,0,joint4,0,90/180*pi,joint5,90/180*pi,joint6];
    q_trans[0]=thetalist(0);
    q_trans[1]=thetalist(1);
    q_trans[2]=thetalist(2);

    q_trans[4]=thetalist(3);

    q_trans[7]=thetalist(4);

    q_trans[9]=thetalist(5);

    dq_trans[0]=dthetalist(0);
    dq_trans[1]=dthetalist(1);
    dq_trans[2]=dthetalist(2);
    dq_trans[4]=dthetalist(3);
    dq_trans[7]=dthetalist(4);
    dq_trans[9]=dthetalist(5);

    ddq_trans[0]=ddthetalist(0);
    ddq_trans[1]=ddthetalist(1);
    ddq_trans[2]=ddthetalist(2);
    ddq_trans[4]=ddthetalist(3);
    ddq_trans[7]=ddthetalist(4);
    ddq_trans[9]=ddthetalist(5);

    double q_offset[10]{0,-M_PI,-0.5*M_PI,0,0,0,0.5*M_PI,0.5*M_PI,0};
    for (size_t i = 0; i < 10; i++)
    {
        q[i]=q_trans[i]+q_offset[i];
        dq[i]=dq_trans[i];
        ddq[i]=ddq_trans[i];
    }
    double H[840]{0};
    ARX_Hb_code(H,q,dq,ddq);

    Eigen::MatrixXd Hb(10, 84);
    for (size_t i = 0; i < 10; i++)
    {
        for (size_t j = 0; j < 84; j++)
        {
            Hb(i,j)=H[i*84+j];
        }
    }
    // std::cout<<"Hb"<<std::endl;
    // std::cout<<Hb<<std::endl;
    return Hb;
}


//清除观测矩阵与关节力矩向量
void ARX_R5_fun::cleanVector() {
    // Hb_vector_.clear();
    // Joint_tor_vector_.clear();
    Hb_vector_ = std::vector<Eigen::MatrixXd>();
    Joint_tor_vector_ = std::vector<Eigen::VectorXd>();
}
//保存观测矩阵与关节力矩
void ARX_R5_fun::Get_ARX_Hb_Tor(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,const Eigen::VectorXd& joint_fit_tor){
   
    auto Hb=Get_ARX_Hb_fir(thetalist, dthetalist, ddthetalist);
    Hb_vector_.push_back(Hb);
    //模拟添加库伦及粘滞摩擦力
    Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(10);  // 关节力矩
    tau_ff(0)=joint_fit_tor(0);
    tau_ff(1)=joint_fit_tor(1);
    tau_ff(2)=joint_fit_tor(2);
    tau_ff(4)=joint_fit_tor(3);
    tau_ff(7)=joint_fit_tor(4);
    tau_ff(9)=joint_fit_tor(5);
    Joint_tor_vector_.push_back(tau_ff);
}


void ARX_R5_fun::saveVectorToXml(const Eigen::VectorXd& vec, const std::string& filename) {
    tinyxml2::XMLDocument doc;
    auto root = doc.NewElement("Vector");
    doc.InsertFirstChild(root);

    // 添加维度属性
    root->SetAttribute("size", vec.size());

    // 添加每个元素
    for (int i = 0; i < vec.size(); ++i) {
        auto elem = doc.NewElement("Element");
        elem->SetAttribute("index", i);
        elem->SetText(vec[i]);
        root->InsertEndChild(elem);
    }

    doc.SaveFile(filename.c_str());
}

Eigen::VectorXd ARX_R5_fun::loadVectorFromXml(const std::string& filename) {
    tinyxml2::XMLDocument doc;
    doc.LoadFile(filename.c_str());
    auto root = doc.FirstChildElement("Vector");

    // 获取向量维度
    int size = root->IntAttribute("size");
    Eigen::VectorXd vec(size);

    // 解析每个元素
    auto elem = root->FirstChildElement("Element");
    while (elem) {
        int index = elem->IntAttribute("index");
        vec[index] = elem->DoubleText();
        elem = elem->NextSiblingElement("Element");
    }
    return vec;
}

//SVD分解求解最小二乘
Eigen::VectorXd ARX_R5_fun::leastSquaresEstimation(
    const Eigen::MatrixXd& Y, 
    const Eigen::MatrixXd& tau) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(tau);
}

//最小二乘法求解
Eigen::VectorXd ARX_R5_fun::lessSquares(
    const Eigen::MatrixXd& Y, 
    const Eigen::MatrixXd& tau) {

    return (Y.transpose()*Y).inverse() * Y.transpose() * tau;
}

//迭代权重最小二乘法
Eigen::VectorXd ARX_R5_fun::lessSquaresfor(
    const Eigen::MatrixXd& Y, 
    const Eigen::MatrixXd& tau) {
    Eigen::MatrixXd X_temp=(Y.transpose()*Y).inverse() * Y.transpose() * tau;
    Eigen::MatrixXd error=Y*X_temp-tau;
    Eigen::MatrixXd W=Eigen::MatrixXd::Identity(Y.rows(),Y.rows());
    for (size_t i = 0; i < Y.rows(); i++)
    {
        //权重W矩阵,
        W(i,i)=1/(error(i,0)*error(i,0)+1e-6);
    }
   return (Y.transpose()*W*Y).inverse() * Y.transpose() *W* tau;
}


void ARX_R5_fun::Get_ARX_R5_Yparam(){
    int num=Hb_vector_.size();
    int Hb_joint_dof=10;
    Eigen::MatrixXd Hb_v(10*num, 84);
    Eigen::MatrixXd tor_v(10*num, 1);
    for (size_t i = 0; i < num; i++)
    {
      Hb_v.block(i*10, 0, 10, 84)=Hb_vector_[i];
      tor_v.block(i*10, 0, 10, 1)=Joint_tor_vector_[i];
    }
    auto Y_parm=leastSquaresEstimation(Hb_v,tor_v);
    // std::cout<<" leastSquaresEstimation "<<std::endl;
    // std::cout<<Y_ans<<std::endl;
    // auto Y_ans1=lessSquares(Hb_v,tor_v);
    for (size_t i = 0; i < 84; i++)
    {
        std::cout<<Y_parm(i)<<","<<std::endl;
    }
    Y_=Y_parm;
    //保存Y参数到xml文件
    saveVectorToXml(Y_parm, "./src/ARX_R5_ros2_V7/yam_damiao_controller/config/Y_parm_data.xml");
}

//InverseDynamics
// std::vector<double> ARX_R5_fun::InverseDynamics(const std::vector<double>& thetalist,const std::vector<double>& dthetalist,const std::vector<double>& ddthetalist ){
//     int dof=6;
//     Eigen::VectorXd thetalist_eigen(dof);
//     Eigen::VectorXd dthetalist_eigen(dof);
//     Eigen::VectorXd ddthetalist_eigen(dof);
//     for (size_t i = 0; i < dof; i++)
//     {
//         thetalist_eigen(i)=thetalist[i];
//         dthetalist_eigen(i)=dthetalist[i];
//         ddthetalist_eigen(i)=ddthetalist[i];
//     }
//     auto InvTor=InverseDynamics(thetalist_eigen, dthetalist_eigen, ddthetalist_eigen);
//     std::vector<double> InvTor_vec(6, 0.0);
//     for (size_t i = 0; i < dof; i++)
//     {
//         InvTor_vec[i]=InvTor(i,0);
//     }
//     return InvTor_vec;
// }
    

Eigen::MatrixXd ARX_R5_fun::InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist){
    auto Hb=Get_ARX_Hb_fir(thetalist, dthetalist, ddthetalist);
    return (Hb*Y_);
}

Eigen::MatrixXd ARX_R5_fun::GravityForces(const Eigen::VectorXd& thetalist){
    Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(6);  // 关节力矩
    auto Hb=Get_ARX_Hb_fir(thetalist, zero_v, zero_v);
    return (Hb*Y_);
}

//补偿重力及摩擦力
Eigen::MatrixXd ARX_R5_fun::GravityAndFirForces(const Eigen::VectorXd& thetalist,const Eigen::VectorXd& dthetalist){
    Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(6);  // 关节力矩
    
    auto Hb=Get_ARX_Hb_fir(thetalist, zero_v, zero_v);
    // std::cout<<"Y_:"<<Y_<<std::endl;
    Eigen::MatrixXd GravityForces=Hb*Y_;
    //重力项添加摩擦
    GravityForces(0,0)+=Y_[1]*dthetalist[0] +Y_[2] *sign(dthetalist[0]);
    GravityForces(1,0)+=Y_[10]*dthetalist[1]+Y_[11]*sign(dthetalist[1]);
    GravityForces(2,0)+=Y_[19]*dthetalist[2]+Y_[20]*sign(dthetalist[2]);

    GravityForces(3,0)+=Y_[37]*dthetalist[3]+Y_[38]*sign(dthetalist[3]);
    GravityForces(4,0)+=Y_[64]*dthetalist[4]+Y_[65]*sign(dthetalist[4]);
    GravityForces(5,0)+=Y_[82]*dthetalist[5]+Y_[83]*sign(dthetalist[5]);

    return GravityForces;
}

//静力学
//tor=J.T*F
//F=J.-T*tor

//求伪逆
Eigen::MatrixXd pinv(Eigen::MatrixXd A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	double  pinvtoler = 1.e-8; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = std::min(row, col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();//����ֵ
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());

	return X;
}


// 使用QR分解替代SVD
Eigen::MatrixXd computePseudoInverse(const Eigen::MatrixXd& matrix) {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(matrix);
    return qr.inverse();
}

// 使用阻尼最小二乘法（DLS）处理奇异位置
Eigen::MatrixXd computeDampedPseudoInverse(
    const Eigen::MatrixXd& matrix, 
    double lambda = 0.01) 
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(matrix.cols(), matrix.cols());
    return (matrix.transpose() * matrix + lambda * I).inverse() * matrix.transpose();
}


Eigen::MatrixXd ARX_R5_fun::Get_Ft_in_world(const Eigen::VectorXd& thetalist,const Eigen::VectorXd& tor){
    Eigen::MatrixXd JacobianSpace_ = mr::JacobianSpace(Slist_, thetalist);
    Eigen::MatrixXd JacobianT= JacobianSpace_.transpose();
    //return pinv(JacobianT) * tor;
    return computePseudoInverse(JacobianT) * tor;
}


//力旋量
void ARX_R5_fun::Force_to_Force(const Eigen::MatrixXd &T,const Eigen::MatrixXd &force_in_body, Eigen::MatrixXd &force_in_forword)
{
// //工具坐标系外力转换到世界坐标系
// Force_to_Force(pm_now, force_in_body, force_in_world);

// 将世界坐标系下的力转换到工具坐标系下
// Force_to_Force(pm_now.inverse(), force_in_world, force_in_body);

//参考  基于力传感器的工业机器人恒力磨抛系统研究_高培阳.pdf
Eigen::MatrixXd rot=T.block(0,0,3,3);
Eigen::Matrix3d S;
S << 0, -T(2,3), T(1,3),
    T(2,3), 0, -T(0,3),
    -T(1,3), T(0,3), 0;
Eigen::MatrixXd Tab=Eigen::MatrixXd::Zero(6,6);
Tab.block(0,0,3,3)=rot;
Tab.block(3,3,3,3)=rot;
Tab.block(3,0,3,3)=S*rot;
force_in_forword=Tab*force_in_body;
}

//力旋量转换
auto ARX_R5_fun::FT_in_world_to_FT_in_sensor(const Eigen::MatrixXd &T,const Eigen::MatrixXd &FT_in_world, Eigen::MatrixXd &FT_in_sensor)
{
    FT_in_sensor(0,0)=FT_in_world(0,0)*T(0,0) + FT_in_world(1,0)*T(1,0) + FT_in_world(2,0)*T(2,0);
    FT_in_sensor(1,0)=FT_in_world(0,0)*T(0,1) + FT_in_world(1,0)*T(1,1) + FT_in_world(2,0)*T(2,1);
    FT_in_sensor(2,0)=FT_in_world(0,0)*T(0,2) + FT_in_world(1,0)*T(1,2) + FT_in_world(2,0)*T(2,2);

    FT_in_sensor(3,0)=FT_in_world(3,0)*T(0,0) + FT_in_world(4,0)*T(0,1) + FT_in_world(5,0)*T(2,0);
    FT_in_sensor(4,0)=FT_in_world(3,0)*T(0,1) + FT_in_world(4,0)*T(1,1) + FT_in_world(5,0)*T(2,1);
    FT_in_sensor(5,0)=FT_in_world(3,0)*T(0,2) + FT_in_world(4,0)*T(1,2) + FT_in_world(5,0)*T(2,2);

}
auto ARX_R5_fun::FT_in_sensor_to_FT_in_world(const Eigen::MatrixXd &T,const Eigen::MatrixXd &FT_in_sensor, Eigen::MatrixXd &FT_in_world)
{
    FT_in_world(0,0)=FT_in_sensor(0,0)*T(0,0) + FT_in_sensor(1,0)*T(0,1) + FT_in_sensor(2,0)*T(0,2);
    FT_in_world(1,0)=FT_in_sensor(0,0)*T(1,0) + FT_in_sensor(1,0)*T(1,1) + FT_in_sensor(2,0)*T(1,2);
    FT_in_world(2,0)=FT_in_sensor(0,0)*T(2,0) + FT_in_sensor(1,0)*T(2,1) + FT_in_sensor(2,0)*T(2,2);

    FT_in_world(3,0)=FT_in_sensor(3,0)*T(0,0) + FT_in_sensor(4,0)*T(0,1) + FT_in_sensor(5,0)*T(0,2);
    FT_in_world(4,0)=FT_in_sensor(3,0)*T(1,0) + FT_in_sensor(4,0)*T(1,1) + FT_in_sensor(5,0)*T(1,2);
    FT_in_world(5,0)=FT_in_sensor(3,0)*T(2,0) + FT_in_sensor(4,0)*T(2,1) + FT_in_sensor(5,0)*T(2,2);

}


Eigen::MatrixXd ARX_R5_fun::admittanceForceControl(const Eigen::MatrixXd &pm_now, const Eigen::MatrixXd &Ft_in_sensor,const double dt_){
  //笛卡尔空间力控制
  int force_direction[6] = {1, 0, 0, 0, 0, 0}; 
  double force_gain[6] = {100, 100, 100, 100, 100, 100}; 
  double vel_limit[6] = {0.08, 0.05, 0.05, 0.05, 0.05, 0.05}; 
  double cart_vel[6] = {0, 0, 0, 0, 0, 0}; 
  double force_target[6] = {10, 0, 0, 0, 0, 0}; // 前三个是力，后三个是力矩
  for (int i = 0; i < 6; i++)
  {
      if (force_direction[i]) {
          double ft= force_target[i]-Ft_in_sensor(i);
          cart_vel[i] = ft * force_gain[i] ;
          cart_vel[i] = std::min(cart_vel[i], vel_limit[i]);
          cart_vel[i] = std::max(cart_vel[i], -vel_limit[i]);
      }
  }

  double p[3] = { 0 };
  for (int i = 0; i < 3; i++)
      p[i] = cart_vel[i] * dt_;

  double pq[6]{0};
  pq[0] = p[0];
  pq[1] = p[1];
  pq[2] = p[2];
  //姿态增量采用四元数，轴角转四元
  double w[3] = { 0 };
  for (int i = 3; i < 6; i++)
      w[i - 3] = cart_vel[i] * dt_;


  auto normv = s_norm(3, w);
  if (std::abs(normv) > 1e-10)
      s_nv(3, 1 / normv, w);
  auto theta = normv;

  pq[3] = w[0] * sin(theta / 2.0);
  pq[4] = w[1] * sin(theta / 2.0);
  pq[5] = w[2] * sin(theta / 2.0);
  pq[6] = cos(theta / 2.0);

  //原先的函数导纳直接计算出的当前位置与位置的增量
  double pm[16];
  s_pq2pm(pq, pm);
  for (int i = 0; i < 7; i++)
  {
    std::cout<<"pq:"<<pq[i]<<std::endl;
  }
  std::cout<<std::endl;

  Eigen::MatrixXd pm_dot= Eigen::MatrixXd::Zero(4, 4);
  for (int i = 0; i < 4; i++)
  {
      for (int j = 0; j < 4; j++)
      {
          pm_dot(i,j) = pm[i*4+j];
      }
  }
  //逆解期望位置是增量，用初始值确定
  Eigen::MatrixXd targetPm= Eigen::MatrixXd::Zero(4, 4);
  targetPm = pm_now*pm_dot;
  return targetPm;
}
