#include "pch.h"
#include <cmath>
#include "../include/libquat.h"

// コンストラクタ
quaternion::quaternion()
{
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	w_ = 1.0f;
}

quaternion::quaternion(float x, float y, float z, float w)
{
	x_ = x;
	y_ = y; 
	z_ = z; 
	w_ = w;
}

quaternion::quaternion(const vector3 &v, float arg)
{
	float co = cosf(0.5f * arg);
	float si = sinf(0.5f * arg);

	x_ = si * v.x();
	y_ = si * v.y();
	z_ = si * v.z();
	w_ = co;
}

// デストラクタ
quaternion::~quaternion()
{
}

quaternion quaternion::operator*(float f) const
{
	return quaternion(x_ * f, y_ * f, z_ * f, w_ * f);
}

quaternion quaternion::operator/(float f) const
{
	return quaternion(x_ / f, y_ / f, z_ / f, w_ / f);
}

vector3 quaternion::operator*(const vector3& v) const
{
	// todo: 実装して下さい
	quaternion v_qua(v.x(), v.y(), v.z(), 0);

	quaternion q_conj = this->conjugate();

	quaternion q_ret = *this * v_qua * q_conj;

	return vector3(q_ret.x_, q_ret.y_, q_ret.z_);
}

quaternion quaternion::operator*(const quaternion& rhs) const
{
	// todo: 実装して下さい
	return quaternion(
	w_*rhs.x_ + x_*rhs.w_ + y_*rhs.z_ - z_*rhs.y_,//x
	w_*rhs.y_ - x_*rhs.z_ + y_*rhs.w_ + z_*rhs.x_,//y
	w_*rhs.z_ + x_*rhs.y_ - y_*rhs.x_ + z_*rhs.w_,//z
	w_*rhs.w_ - x_*rhs.x_ - y_*rhs.y_ - z_*rhs.z_//w
	);
}

quaternion quaternion::operator+(const quaternion& rhs) const
{
	return quaternion(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_, w_ + rhs.w_);
}


// 単位元にする
quaternion &quaternion::identity()
{
	// todo: 実装して下さい
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	w_ = 1.0f;
	return *this;
}

// 正規化する
quaternion &quaternion::normalize()
{
	// todo: 実装して下さい
	float length = std::sqrt(this->length_sq());
	*this = *this / length;
	return *this;
}

// 大きさの2乗
float quaternion::length_sq() const
{
	return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_;
}

// 共役を返す
quaternion quaternion::conjugate() const
{
	// todo: 実装して下さい
	return quaternion(-x_, -y_, -z_, w_);
}

// 逆元を返す
quaternion quaternion::inverse() const
{
	// todo: 実装して下さい
	float norm = this->length_sq();

	quaternion ret = this->conjugate() / norm;

	return ret;
}

// 球面線形補間
quaternion quaternion::slerp(const quaternion& q0, const quaternion& q1, float t)
{
	// todo: 実装して下さい
	const float threshold_theta = 0.0005f;//同じクォータニオンだった場合の角度のしきい値


	//内積
	float dot = q0.w_ * q1.w_ + q0.x_ * q1.x_ + q0.y_ * q1.y_ + q0.z_ * q1.z_;

	//内積が負の時は角度が180度以上になる->一番近い(小さい)角度になるように
	if (dot < 0)
	{
		return slerp(q0, q1*-1, t);
	}

	//角度
	float theta = acosf(dot);

	quaternion q_ret;//返すクォータニオン

	//ほぼ同じ方向のクォータニオンを入れてしまうと角度が0になる->sin0は0なので0除算が起こる
	//それを止めるためにほぼ同じ方向の場合はLerp
	if (theta <= threshold_theta)
	{
		q_ret = q0 * (1 - t) + q1 * t;
		return q_ret;
	}

	float sin_theta = sinf(theta);

	float sin_q0=sinf((1-t)*theta)/sin_theta;
	float sin_q1 = sinf(t * theta) / sin_theta;

	q_ret = q0 * sin_q0 + q1 * sin_q1;
	return q_ret;
}

