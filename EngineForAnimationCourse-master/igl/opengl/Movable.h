#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>
#include <iostream>

class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void Movable::ResetTranslation();
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d &rot);
	void Movable::SetCenterOfRotation(Eigen::Vector3d amt);
	void MyScale(Eigen::Vector3d amt);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);
	void Movable::TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }
	void RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle);
	Eigen::Vector3d Movable::GetCenterOfRotation();
	Eigen::Vector3d Movable::GetCenter()
	{
		Eigen::Vector3d amt;
		Eigen::Matrix4d tempMat = MakeTransd();
		return Eigen::Vector3d(tempMat(0, 3), tempMat(1, 3), tempMat(2, 3));
	}

	virtual ~Movable() {}
private:
	Eigen::Affine3d Tout,Tin;
};

