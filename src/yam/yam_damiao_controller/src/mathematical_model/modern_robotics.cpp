

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include "mathematical_model/modern_robotics.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
 /*
 说明：
 参考《现代机器人学》
!!!***特殊正交群SO3   3x3旋转矩阵***!!!
 满足 RtR=I    detR=1   R.iv * Rt=I
 封闭性；结合律；幺元；可逆   P41
 世界坐标系：左乘
 自身坐标系：右乘


!!!***so3 所有3X3反对称矩阵的集合***!!!
 角速度：w=w^* theta.dot   p45页
 R.dot=[wsXr1 wsXr2 wsXr3]=wsXR=[ws]R
 [ws]是反对称阵
 [x]=[0 -x3 x2
	  x3  0  -x1
	  -x2 x1  0]
 R为物体相对固定坐标系的姿态矩阵
 物体坐标系[wb]=R.inv*R.dot= 相对于静坐标系{b}的角速度
 固定坐标系[ws]=R.dot*R.inv
 D相对于C的旋转矩阵，则wc=Rcd*wd

 指数坐标系表示  w^*theta为三参数指数表示   单独写w^与theta为轴角   p47页
 单位转轴w^与绕该轴线的转角theta，通过连乘得到三维向量
 **三维向量p与单位转轴w^绕theta   p50
 p.dot=w^ X p=[w^]p =>微分方程 p(theta)=e^[w^]theta *p(0)
 罗德里格斯向量      p50页
 Rot(w^,theta)=e^[w^]theta=I+sin(theta)*[w^]+(1-cos(theta))*[w^]^2

 刚体转动的矩阵对数   p51页
 反对称阵[w^*theta]=[w^]*theta
 求R->[w^]*theta    P52页


 !!!***特殊欧式群SE3  刚体运动群、齐次变换矩阵  4x4旋转矩阵***!!!
 旋转变换矩阵特性  p54页
 T=[R    p
    0    1]
 T.inv=[R^T -R^T *p
		 0     1]

 物体运动旋量(物体坐标系上的速度) 
  T.inv * T.dot 为动坐标系相对当前与其瞬时重合的静坐标系{b}的线速度与角速度 p59页
 Vb=[wb
	 vb]
群se(3)  矩阵对数 反对称阵
 [vb]=[[wb] vb  =T.inv*T.dot
		 0   0]

 空间运动旋量(固定坐标系)		p61页
 Vs=[ws
	 vs]
 [vb]=[[ws] vs  =T.dot*T.inv
		 0   0]
 物体与空间旋量转换
 [ws = [R     0 * [wb
 vs]   [p]R  R]   vb]

 伴随变换矩阵    p61页
 [AdT]=[R     0
	   [p]R  R]
 Vs=[AdTsb]*Vb
 Vb=[AdTb]*Vs

 任意两个坐标系{c}与{d}运动旋量Vc与Vd关系为
 Vc=[AdTcd]*Vd
 Vd=[AdTdc]*Vc

 ！！！！！！ p62页！！！！！！！
 Vc=wc X (-rs)
 ！！！！！！！！！！！！！

 运动旋量的螺旋表达   p63页
 螺旋轴形式：{q.s^,h}  q为R3轴上任一点；s^为螺旋轴单位方向向量；
 h为螺距,大小为沿螺轴方向的线速度与角速度比值
 运动旋量V=[w  =[s^*theta.dot
			v]    -s^*theta.dot X q  +hs^theta.dot]

 螺旋轴S=[w
		  v]
 螺旋轴的矩阵表达形式
 [S]=[[w] v
	   0  0]
 [w] =[0 -w3 w2
	  w3  0  -w1
	  -w2 w1  0] 
 任意两个坐标系{a}与{b}螺旋轴的映射关系为
 Sa=[adTab]Sb


 任何刚体运动都可以通过绕空间某一固定的螺旋轴S的运动来实现
 螺旋轴S=(w,v)的节距为有限值，||w||=1,theta为绕螺旋轴旋转的角度
 螺旋轴S=(w,v)的节距为无限值，w=0,||v||=1,theta为绕螺旋轴移动的距离
 ！！刚体的指数坐标
 转动：||w||=1
	 e^([S]theta)=[e^([w]theta   (I*theta+(1-cos(theta)[w]+(theta-sin(theta)[w]^2))*v)
					 0                            1                                   ]
 移动：w=0,||v||=1
	 e^([S]theta)=[I v*theta
					0    1  ]

 ！！刚体的矩阵对数
 矩阵对数参考p65页   已知旋转矩阵求螺旋轴
 给的任意的（R,p) & SE(3)总能找到对应的螺旋轴S=（w,v)和标量theta满足
 e^([S]theta)=[R    p
			   0    1]
其中[S]=[[w] v
	      0  0]
 力矩ma=ra X fa
 空间力旋量:Fa=[ma  =[AdTba]^T*Fb      p67页
			   fa]


///*********正向运动学
基于基坐标系的螺旋轴
M SE(3)表述机器人处于初始位置的末端位姿
末端T= e^([Sn]thetan)M
Sn=（wn,vn)在基坐标系中的关节N的旋量坐标
其中：转动  vn=-wn X qn ，qn为关节轴上任一点
	  移动  wn=0,vn为关节正向单位向量，theta为移动距离

///*********微分运动学  p108页
机器人末端速度 x.dot=J(theta) *theta.dot
雅可比：末端速度相对
关节力矩与关节力矩关系
tor=J.inv(theta)*f
f=J.-inv(theta)*tor

vs=J1(theta)*theta(1).dot+J2(theta)*theta(2).dot+......
!!!!!!
给定位形theta，六维向量J(theta),其中雅可比矩阵的第i列，是当theta(i).dot=1,其他关节旋量为0的运动旋量V

空间雅可比 Vs=Js(theta) * theta().dot            p111页      
      
Jsi的每一列对应空间固定坐标系{s}中各关节螺旋轴的旋量坐标
Vs==[Vs]=T.dot*T.inv=Js1*theta(1).dot+Js2*theta(2).dot+....
注意定义5.1     p111页   求解J1  J2   J3.......

[vb]=[[ws] vs  =T.dot*T.inv
		 0   0]

物体雅可比 Vb=Jb(theta) * theta().dot    p114页
Jbi的每一列对应物体坐标系{s}中各关节螺旋轴的旋量坐标

空间雅可比与物体雅可比矩阵关系
Js=[AdTsb]*Jb

解析雅可比：广义坐标的时间导数q.dot=Ja * theta.dot     p117页
几何雅可比：关节速率与末端执行器速率关系

开链静力学
tor=Js.inv(theta)*fs
tor=Jb.inv(theta)*fb


///*********开链动力学 p165页
正向动力学：求解加速度
逆向动力学：求解力矩
拉格朗日动力学：基于动能与势能
整个系统的动能减势能为拉格朗日函数

动力学相关参数：
M对称正定的质量矩阵；C包含科里奥利力和向心力矩的向量；G重力矩
牛顿欧拉：刚体


 单刚体动力学    p173页
 Fb=[mb
	 fb]
 fb=mi(vb.dot+[wb]vb)      直线动力学
 mb=Ib wb.dot+[wb]*Ib*wb   旋转刚体的欧拉方程
 Ib为转动惯量矩阵  
 Ib=[Ixx  Ixy  Ixz
	 Ixy  Iyy  Iyz
	 Ixz  Iyz  Izz]
 惯性主轴：由Ib的特征向量与特征值给出
 一个主轴使关于通过质心的所有轴之间的转动惯量最大化，另一根主轴使得转动惯量最小化

 **两坐标系之间惯性矩阵转换     p175页
 Ic=Rbc^T*Tb*Rbc
 平行轴定理：刚体通过其质心的某轴线的标量惯量为Tcm,假设平行于该轴但与其相距为d的另一轴线标量惯量为Id
 Id=Icm+md^2

 运动旋量-力旋量方程   P176页
 空间惯量矩阵
 Gb=[Ib  0
	 0   mI]

 空间动量
 Pb=[Ib*wb =Gb*vb
	 m*vb]

 李括号（两个运动旋量V1与V2的运算）
 运动旋量v1=(w1,v1)  v2=(w2,v2)
 [adv1]v2=adv1(v2)=[[w1]    0   *[w2
					[v1]  [w1]]   v2]

 **单个刚体动力学方程
 Fb=Gb*vb.dot-[advb]^T*Gb*vb

 其他参考坐标系下的动力学方程  P178



 */




namespace mr {

	/* Function: Find if the value is negligible enough to consider 0
	 * Inputs: value to be checked as a double
	 * Returns: Boolean of true-ignore or false-can't ignore
	 */
	bool NearZero(const double val) {
		return (std::abs(val) < .000001);
	}

	//李括号，参考P177页
	/*
	 * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
	 * Input: Eigen::VectorXd (6x1)
	 * Output: Eigen::MatrixXd (6x6)
	 * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
	 */
	Eigen::MatrixXd ad(Eigen::VectorXd V) {
		Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		Eigen::MatrixXd result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
		result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
	}

	/* Function: Returns a normalized version of the input vector
	 * Input: Eigen::MatrixXd
	 * Output: Eigen::MatrixXd
	 * Note: MatrixXd is used instead of VectorXd for the case of row vectors
	 * 		Requires a copy
	 *		Useful because of the MatrixXd casting
	 */
	Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
		V.normalize();
		return V;
	}


	// //轴角转旋转矩阵
	// Eigen::MatrixXd rm1=mr::MatrixExp3(mr::VecToso3(vec));
	// std::cout<<"rm1:"<<rm1<<std::endl;
	// Eigen::MatrixXd pm=Eigen::MatrixXd::Identity(4, 4);
	// pm.block<3, 3>(0, 0)=rm1;
	// pm.block<3, 1>(0, 3)=pp;
	// std::cout<<"pm:"<<pm<<std::endl;

	//P46页 反对称矩阵
	/* Function: Returns the skew symmetric matrix representation of an angular velocity vector
	 * Input: Eigen::Vector3d 3x1 angular velocity vector
	 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
	 */
	Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
		Eigen::Matrix3d m_ret;
		m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
		return m_ret;
	}


	//P46页so3转向量
	/* Function: Returns angular velocity vector represented by the skew symmetric matrix
	 * Inputs: Eigen::MatrixXd 3x3 skew symmetric matrix
	 * Returns: Eigen::Vector3d 3x1 angular velocity
	 */
	Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
		Eigen::Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}


	/* Function: Translates an exponential rotation into it's individual components
	 * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
	 *				and the angle of rotation)
	 * Returns: The axis and angle of rotation as [x, y, z, theta]
	 */
	Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3) {
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

	//P50页罗德里格斯
	/* Function: Translates an exponential rotation into a rotation matrix
	 * Inputs: exponenential representation of a rotation
	 * Returns: Rotation matrix
	 *///(3.49)
	 //单位转轴与转角
	Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat) {
		Eigen::Vector3d omgtheta = so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (NearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}


	//p53页 旋转矩阵R转轴角算法
	/* Function: Computes the matrix logarithm of a rotation matrix
	 * Inputs: Rotation matrix
	 * Returns: matrix logarithm of a rotation
	 */
	Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}

	//p53页 旋转矩阵R求轴角算法
	double MatrixLog3_theta(const Eigen::Matrix3d& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1){
			double theta = 0;
			return theta;
		}
		else if (acosinput <= -1) {
			double theta = M_PI;
			return theta;
		}
		else {
			double theta = std::acos(acosinput);
			return theta;
		}
	}


	/* Function: Combines a rotation matrix and position vector into a single
	 * 				Special Euclidian Group (SE3) homogeneous transformation matrix
	 * Inputs: Rotation Matrix (R), Position Vector (p)
	 * Returns: Matrix of T = [ [R, p],
	 *						    [0, 1] ]
	 */
	Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
	}


	/* Function: Separates the rotation matrix and position vector from
	 *				the transfomation matrix representation
	 * Inputs: Homogeneous transformation matrix
	 * Returns: std::vector of [rotation matrix, position vector]
	 */
	std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> Rp_ret;
		Eigen::Matrix3d R_ret;
		// Get top left 3x3 corner
		R_ret = T.block<3, 3>(0, 0);

		Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));
		Rp_ret.push_back(R_ret);
		Rp_ret.push_back(p_ret);

		return Rp_ret;
	}


	/* Function: Translates a spatial velocity vector into a transformation matrix
	 * Inputs: Spatial velocity vector [angular velocity, linear velocity]
	 * Returns: Transformation matrix
	 */
	 //运动旋量
	Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V) {
		// Separate angular (exponential representation) and linear velocities
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));

		// Fill in values to the appropriate parts of the transformation matrix
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
			0, 0, 0, 0;

		return m_ret;
	}


	/* Function: Translates a transformation matrix into a spatial velocity vector
	 * Inputs: Transformation matrix
	 * Returns: Spatial velocity vector [angular velocity, linear velocity]
	 */
	Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& T) {
		Eigen::VectorXd m_ret(6);
		m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

		return m_ret;
	}


	//伴随矩阵求解
	/* Function: Provides the adjoint representation of a transformation matrix
	 *			 Used to change the frame of reference for spatial velocity vectors
	 * Inputs: 4x4 Transformation matrix SE(3)
	 * Returns: 6x6 Adjoint Representation of the matrix
	 *///[AdT]  3.76   P61页
	Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> R = TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		ad_ret = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
			VecToso3(R[1]) * R[0], R[0];
		return ad_ret;
	}


	/* Function: Rotation expanded for screw axis
	 * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
	 * Returns: 6x6 Matrix representing the rotation
	 */
	 //刚体运动指数坐标 p65页
	Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			//塞入线速度
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			//单位化P63
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}

	}

	//刚体运动旋转矩阵转矩阵对数P65页 [S]theta   se(3)
	Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = mr::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (NearZero(omgmat.norm())) {
			m_ret << zeros3d, rp.at(1),
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((rp.at(0).trace() - 1) / 2.0);//p53
			Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*rp.at(1),
				0, 0, 0, 0;
		}
		return m_ret;
	}


	/* Function: Compute end effector frame (used for current spatial position calculation)
	 * Inputs: Home configuration (position and orientation) of end-effector
	 *		   The joint screw axes in the space frame when the manipulator
	 *             is at the home position
	 * 		   A list of joint coordinates.
	 * Returns: Transfomation matrix representing the end-effector frame when the joints are
	 *				at the specified coordinates
	 * Notes: FK means Forward Kinematics
	 */
	Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}

	/*
	 * Function: Compute end effector frame (used for current body position calculation)
	 * Inputs: Home configuration (position and orientation) of end-effector
	 *		   The joint screw axes in the body frame when the manipulator
	 *             is at the home position
	 * 		   A list of joint coordinates.
	 * Returns: Transfomation matrix representing the end-effector frame when the joints are
	 *				at the specified coordinates
	 * Notes: FK means Forward Kinematics
	 */
	Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}


	/* Function: Gives the space Jacobian
	 * Inputs: Screw axis in home position, joint configuration
	 * Returns: 6xn Spatial Jacobian
	 */
	Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Js = Slist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd sListTemp(Slist.col(0).size());
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}

		return Js;
	}

	/*
	 * Function: Gives the body Jacobian
	 * Inputs: Screw axis in BODY position, joint configuration
	 * Returns: 6xn Bobdy Jacobian
	 */
	Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Jb = Blist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd bListTemp(Blist.col(0).size());
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}

	Eigen::MatrixXd TransInv(const Eigen::MatrixXd& transform) {
		auto rp = mr::TransToRp(transform);
		auto Rt = rp.at(0).transpose();
		auto t = -(Rt * rp.at(1));
		Eigen::MatrixXd inv(4, 4);
		inv = Eigen::MatrixXd::Zero(4, 4);
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}

	Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix) {
		return rotMatrix.transpose();
	}

	Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q, Eigen::Vector3d s, double h) {
		Eigen::VectorXd axis(6);
		axis.segment(0, 3) = s;
		axis.segment(3, 3) = q.cross(s) + (h * s);
		return axis;
	}

	Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6) {
		Eigen::VectorXd v_ret(7);
		double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
		if (NearZero(theta))
			theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
		v_ret << expc6 / theta, theta;
		return v_ret;
	}

	Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd& M) {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0)
			// In this case the result may be far from M; reverse sign of 3rd column
			R.col(2) *= -1;
		return R;
	}

	Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd& M) {
		Eigen::Matrix3d R = M.block<3, 3>(0, 0);
		Eigen::Vector3d t = M.block<3, 1>(0, 3);
		Eigen::MatrixXd T = RpToTrans(ProjectToSO3(R), t);
		return T;
	}

	double DistanceToSO3(const Eigen::Matrix3d& M) {
		if (M.determinant() > 0)
			return (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
		else
			return 1.0e9;
	}

	double DistanceToSE3(const Eigen::Matrix4d& T) {
		Eigen::Matrix3d matR = T.block<3, 3>(0, 0);
		if (matR.determinant() > 0) {
			Eigen::Matrix4d m_ret;
			m_ret << matR.transpose()*matR, Eigen::Vector3d::Zero(3),
				T.row(3);
			m_ret = m_ret - Eigen::Matrix4d::Identity();
			return m_ret.norm();
		}
		else
			return 1.0e9;
	}

	bool TestIfSO3(const Eigen::Matrix3d& M) {
		return std::abs(DistanceToSO3(M)) < 1e-3;
	}

	bool TestIfSE3(const Eigen::Matrix4d& T) {
		return std::abs(DistanceToSE3(T)) < 1e-3;
	}


	//P138页
	bool IKinBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
		Eigen::VectorXd& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinBody(M, Blist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
		Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Jb;
		while (err && i < maxiterations) {
			Jb = JacobianBody(Blist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}
	//数值解（牛顿欧拉法进行迭代求解）
	bool IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
		Eigen::VectorXd& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
		Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Js;
		while (err && i < maxiterations) {
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	/*
	* Function: This function uses forward-backward Newton-Euler iterations to solve the
	* equation:
	* taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
	*           + g(thetalist) + Jtr(thetalist) * Ftip
	* Inputs:
	*  thetalist: n-vector of joint variables
	*  dthetalist: n-vector of joint rates
	*  ddthetalist: n-vector of joint accelerations
	*  g: Gravity vector g
	*  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
	*  Mlist: List of link frames {i} relative to {i-1} at the home position
	*  Glist: Spatial inertia matrices Gi of the links
	*  Slist: Screw axes Si of the joints in a space frame, in the format
	*         of a matrix with the screw axes as the columns.
	*
	* Outputs:
	*  taulist: The n-vector of required joint forces/torques
	*
	*/
	Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
		const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
		const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		// the size of the lists
		int n = thetalist.size();

		Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
		Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6, n);
		std::vector<Eigen::MatrixXd> AdTi;
		for (int i = 0; i < n + 1; i++) {
			AdTi.push_back(Eigen::MatrixXd::Zero(6, 6));
		}
		Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6, n + 1);    // velocity
		Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6, n + 1);   // acceleration

		Vdi.block(3, 0, 3, 1) = -g;
		AdTi[n] = mr::Adjoint(mr::TransInv(Mlist[n]));
		//std::cout<<"iter_T[DOF] "<<AdTi[n]<<std::endl;
		Eigen::VectorXd Fi = Ftip;

		Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);

		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = mr::Adjoint(mr::TransInv(Mi))*Slist.col(i);

			AdTi[i] = mr::Adjoint(mr::MatrixExp6(mr::VecTose3(Ai.col(i)*-thetalist(i)))* mr::TransInv(Mlist[i]));

			Vi.col(i + 1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i + 1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
				+ ad(Vi.col(i + 1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}

		// backward pass
		for (int i = n - 1; i >= 0; i--) {
			Fi = AdTi[i + 1].transpose() * Fi + Glist[i] * Vdi.col(i + 1)
				- ad(Vi.col(i + 1)).transpose() * (Glist[i] * Vi.col(i + 1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}

	/*
	 * Function: This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
	 *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation
	 * Inputs:
	 *  thetalist: n-vector of joint variables
	 *  g: Gravity vector g
	 *  Mlist: List of link frames {i} relative to {i-1} at the home position
	 *  Glist: Spatial inertia matrices Gi of the links
	 *  Slist: Screw axes Si of the joints in a space frame, in the format
	 *         of a matrix with the screw axes as the columns.
	 *
	 * Outputs:
	 *  grav: The 3-vector showing the effect force of gravity to the dynamics
	 *
	 */
	Eigen::VectorXd GravityForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& g,
		const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd grav = mr::InverseDynamics(thetalist, dummylist, dummylist, g,
			dummyForce, Mlist, Glist, Slist);
		return grav;
	}

	/*
	 * Function: This function calls InverseDynamics n times, each time passing a
	 * ddthetalist vector with a single element equal to one and all other
	 * inputs set to zero. Each call of InverseDynamics generates a single
	 * column, and these columns are assembled to create the inertia matrix.
	 *
	 * Inputs:
	 *  thetalist: n-vector of joint variables
	 *  Mlist: List of link frames {i} relative to {i-1} at the home position
	 *  Glist: Spatial inertia matrices Gi of the links
	 *  Slist: Screw axes Si of the joints in a space frame, in the format
	 *         of a matrix with the screw axes as the columns.
	 *
	 * Outputs:
	 *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
	 *     chain at the given configuration thetalist.
	 */
	Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist,
		const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
		for (int i = 0; i < n; i++) {
			Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
			ddthetalist(i) = 1;
			M.col(i) = mr::InverseDynamics(thetalist, dummylist, ddthetalist,
				dummyg, dummyforce, Mlist, Glist, Slist);
		}
		return M;
	}

	/*
	 * Function: This function calls InverseDynamics with g = 0, Ftip = 0, and
	 * ddthetalist = 0.
	 *
	 * Inputs:
	 *  thetalist: n-vector of joint variables
	 *  dthetalist: A list of joint rates
	 *  Mlist: List of link frames {i} relative to {i-1} at the home position
	 *  Glist: Spatial inertia matrices Gi of the links
	 *  Slist: Screw axes Si of the joints in a space frame, in the format
	 *         of a matrix with the screw axes as the columns.
	 *
	 * Outputs:
	 *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
	 *     terms for a given thetalist and dthetalist.
	 */
	Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
		const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd c = mr::InverseDynamics(thetalist, dthetalist, dummylist,
			dummyg, dummyforce, Mlist, Glist, Slist);
		return c;
	}

	/*
	 * Function: This function calls InverseDynamics with g = 0, dthetalist = 0, and
	 * ddthetalist = 0.
	 *
	 * Inputs:
	 *  thetalist: n-vector of joint variables
	 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
	 *  Mlist: List of link frames {i} relative to {i-1} at the home position
	 *  Glist: Spatial inertia matrices Gi of the links
	 *  Slist: Screw axes Si of the joints in a space frame, in the format
	 *         of a matrix with the screw axes as the columns.
	 *
	 * Outputs:
	 *  JTFtip: The joint forces and torques required only to create the
	 *     end-effector force Ftip.
	 */
	Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& Ftip,
		const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);

		Eigen::VectorXd JTFtip = mr::InverseDynamics(thetalist, dummylist, dummylist,
			dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}

	/*
	 * Function: This function computes ddthetalist by solving:
	 * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist)
	 *                                  - g(thetalist) - Jtr(thetalist) * Ftip
	 * Inputs:
	 *  thetalist: n-vector of joint variables
	 *  dthetalist: n-vector of joint rates
	 *  taulist: An n-vector of joint forces/torques
	 *  g: Gravity vector g
	 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
	 *  Mlist: List of link frames {i} relative to {i-1} at the home position
	 *  Glist: Spatial inertia matrices Gi of the links
	 *  Slist: Screw axes Si of the joints in a space frame, in the format
	 *         of a matrix with the screw axes as the columns.
	 *
	 * Outputs:
	 *  ddthetalist: The resulting joint accelerations
	 *
	 */
	Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& taulist,
		const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
		const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {

		Eigen::VectorXd totalForce = taulist - mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
			- mr::GravityForces(thetalist, g, Mlist, Glist, Slist)
			- mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

		Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

		// Use LDLT since M is positive definite
		Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);

		return ddthetalist;
	}

	void EulerStep(Eigen::VectorXd& thetalist, Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist, double dt) {
		thetalist += dthetalist * dt;
		dthetalist += ddthetalist * dt;
		return;
	}

	Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat, const Eigen::MatrixXd& dthetamat, const Eigen::MatrixXd& ddthetamat,
		const Eigen::VectorXd& g, const Eigen::MatrixXd& Ftipmat, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist) {
		Eigen::MatrixXd thetamatT = thetamat.transpose();
		Eigen::MatrixXd dthetamatT = dthetamat.transpose();
		Eigen::MatrixXd ddthetamatT = ddthetamat.transpose();
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();

		int N = thetamat.rows();  // trajectory points
		int dof = thetamat.cols();
		Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(dof, N);
		for (int i = 0; i < N; ++i) {
			taumatT.col(i) = InverseDynamics(thetamatT.col(i), dthetamatT.col(i), ddthetamatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
		}
		Eigen::MatrixXd taumat = taumatT.transpose();
		return taumat;
	}

	std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::MatrixXd& taumat,
		const Eigen::VectorXd& g, const Eigen::MatrixXd& Ftipmat, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, double dt, int intRes) {
		Eigen::MatrixXd taumatT = taumat.transpose();
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
		int N = taumat.rows();  // force/torque points
		int dof = taumat.cols();
		Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(dof, N);
		Eigen::MatrixXd dthetamatT = Eigen::MatrixXd::Zero(dof, N);
		thetamatT.col(0) = thetalist;
		dthetamatT.col(0) = dthetalist;
		Eigen::VectorXd thetacurrent = thetalist;
		Eigen::VectorXd dthetacurrent = dthetalist;
		Eigen::VectorXd ddthetalist;
		for (int i = 0; i < N - 1; ++i) {
			for (int j = 0; j < intRes; ++j) {
				ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taumatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
				EulerStep(thetacurrent, dthetacurrent, ddthetalist, 1.0*dt / intRes);
			}
			thetamatT.col(i + 1) = thetacurrent;
			dthetamatT.col(i + 1) = dthetacurrent;
		}
		std::vector<Eigen::MatrixXd> JointTraj_ret;
		JointTraj_ret.push_back(thetamatT.transpose());
		JointTraj_ret.push_back(dthetamatT.transpose());
		return JointTraj_ret;
	}

	Eigen::VectorXd ComputedTorque(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& eint,
		const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalistd, const Eigen::VectorXd& dthetalistd, const Eigen::VectorXd& ddthetalistd,
		double Kp, double Ki, double Kd) {

		Eigen::VectorXd e = thetalistd - thetalist;  // position err
		Eigen::VectorXd tau_feedforward = MassMatrix(thetalist, Mlist, Glist, Slist)*(Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

		Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd tau_inversedyn = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

		Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
		return tau_computed;
	}

	double CubicTimeScaling(double Tf, double t) {
		double timeratio = 1.0*t / Tf;
		double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
		return st;
	}

	double QuinticTimeScaling(double Tf, double t) {
		double timeratio = 1.0*t / Tf;
		double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
		return st;
	}

	Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd& thetastart, const Eigen::VectorXd& thetaend, double Tf, int N, int method) {
		double timegap = Tf / (N - 1);
		Eigen::MatrixXd trajT = Eigen::MatrixXd::Zero(thetastart.size(), N);
		double st;
		for (int i = 0; i < N; ++i) {
			if (method == 3)
				st = CubicTimeScaling(Tf, timegap*i);
			else
				st = QuinticTimeScaling(Tf, timegap*i);
			trajT.col(i) = st * thetaend + (1 - st)*thetastart;
		}
		Eigen::MatrixXd traj = trajT.transpose();
		return traj;
	}
	std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd& Xstart, const Eigen::MatrixXd& Xend, double Tf, int N, int method) {
		double timegap = Tf / (N - 1);
		std::vector<Eigen::MatrixXd> traj(N);
		double st;
		for (int i = 0; i < N; ++i) {
			if (method == 3)
				st = CubicTimeScaling(Tf, timegap*i);
			else
				st = QuinticTimeScaling(Tf, timegap*i);
			Eigen::MatrixXd Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
			traj.at(i) = Xstart * MatrixExp6(Ttemp*st);
		}
		return traj;
	}

	std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd& Xstart, const Eigen::MatrixXd& Xend, double Tf, int N, int method) {
		double timegap = Tf / (N - 1);
		std::vector<Eigen::MatrixXd> traj(N);
		std::vector<Eigen::MatrixXd> Rpstart = TransToRp(Xstart);
		std::vector<Eigen::MatrixXd> Rpend = TransToRp(Xend);
		Eigen::Matrix3d Rstart = Rpstart[0]; Eigen::Vector3d pstart = Rpstart[1];
		Eigen::Matrix3d Rend = Rpend[0]; Eigen::Vector3d pend = Rpend[1];
		double st;
		for (int i = 0; i < N; ++i) {
			if (method == 3)
				st = CubicTimeScaling(Tf, timegap*i);
			else
				st = QuinticTimeScaling(Tf, timegap*i);
			Eigen::Matrix3d Ri = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend)*st);
			Eigen::Vector3d pi = st * pend + (1 - st)*pstart;
			Eigen::MatrixXd traji(4, 4);
			traji << Ri, pi,
				0, 0, 0, 1;
			traj.at(i) = traji;
		}
		return traj;
	}
	std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& g,
		const Eigen::MatrixXd& Ftipmat, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetamatd, const Eigen::MatrixXd& dthetamatd, const Eigen::MatrixXd& ddthetamatd,
		const Eigen::VectorXd& gtilde, const std::vector<Eigen::MatrixXd>& Mtildelist, const std::vector<Eigen::MatrixXd>& Gtildelist,
		double Kp, double Ki, double Kd, double dt, int intRes) {
		Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
		Eigen::MatrixXd thetamatdT = thetamatd.transpose();
		Eigen::MatrixXd dthetamatdT = dthetamatd.transpose();
		Eigen::MatrixXd ddthetamatdT = ddthetamatd.transpose();
		int m = thetamatdT.rows(); int n = thetamatdT.cols();
		Eigen::VectorXd thetacurrent = thetalist;
		Eigen::VectorXd dthetacurrent = dthetalist;
		Eigen::VectorXd eint = Eigen::VectorXd::Zero(m);
		Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(m, n);
		Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(m, n);
		Eigen::VectorXd taulist;
		Eigen::VectorXd ddthetalist;
		for (int i = 0; i < n; ++i) {
			taulist = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, Mtildelist, Gtildelist, Slist, thetamatdT.col(i),
				dthetamatdT.col(i), ddthetamatdT.col(i), Kp, Ki, Kd);
			for (int j = 0; j < intRes; ++j) {
				ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, FtipmatT.col(i), Mlist, Glist, Slist);
				EulerStep(thetacurrent, dthetacurrent, ddthetalist, dt / intRes);
			}
			taumatT.col(i) = taulist;
			thetamatT.col(i) = thetacurrent;
			eint += dt * (thetamatdT.col(i) - thetacurrent);
		}
		std::vector<Eigen::MatrixXd> ControlTauTraj_ret;
		ControlTauTraj_ret.push_back(taumatT.transpose());
		ControlTauTraj_ret.push_back(thetamatT.transpose());
		return ControlTauTraj_ret;
	}
}