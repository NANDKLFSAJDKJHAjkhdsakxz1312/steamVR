#ifndef VR_2_EIGEN_TRANSFORMS_H
#define VR_2_EIGEN_TRANSFORMS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <openvr.h>
#include <cmath>

namespace VR2EigenTransforms {

	inline Eigen::Matrix4d vrMatrixToEigen(const vr::HmdMatrix34_t& vrMatrix) {
		Eigen::Matrix4d eigenMatrix = Eigen::Matrix4d::Identity();
		for (int row = 0; row < 3; ++row) {
			for (int col = 0; col < 4; ++col) {
				eigenMatrix(row, col) = vrMatrix.m[row][col];
			}
		}
		return eigenMatrix;
	}

	inline Eigen::Vector3d getPosition(const Eigen::Matrix4d& transform) {
		return transform.block<3, 1>(0, 3);
	}

	inline Eigen::Matrix3d getRotationMatrix(const Eigen::Matrix4d& transform) {
		return transform.block<3, 3>(0, 0);
	}

	inline Eigen::Quaterniond getQuaternion(const Eigen::Matrix4d& transform) {
		Eigen::Matrix3d rotationMatrix = getRotationMatrix(transform);
		return Eigen::Quaterniond(rotationMatrix);
	}


	inline Eigen::Quaterniond getQuaternionFromVRMatrix(const vr::HmdMatrix34_t& vrMatrix) {
		Eigen::Matrix4d eigenMatrix = vrMatrixToEigen(vrMatrix);
		return getQuaternion(eigenMatrix);
	}

	inline Eigen::Vector3d getPositionFromVRMatrix(const vr::HmdMatrix34_t& vrMatrix) {
		return Eigen::Vector3d(vrMatrix.m[0][3], vrMatrix.m[1][3], vrMatrix.m[2][3]);
	}

	inline Eigen::Vector3d quaternionToEulerXYZ(const Eigen::Quaterniond& q) {
		return q.toRotationMatrix().canonicalEulerAngles(0, 1, 2); // XYZ order
	}

	inline Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& euler) {
		return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
	}

	inline Eigen::Matrix4d createTransform(const Eigen::Vector3d& position,
		const Eigen::Quaterniond& quaternion) {
		Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
		transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
		transform.block<3, 1>(0, 3) = position;
		return transform;
	}

	inline Eigen::Matrix4d computeRelativeTransform(const Eigen::Matrix4d& from,
		const Eigen::Matrix4d& to) {
		return from.inverse() * to;
	}

	inline Eigen::Vector3d filterPosition(const Eigen::Vector3d& current,
		const Eigen::Vector3d& previous,
		double alpha) {
		return alpha * current + (1.0 - alpha) * previous;
	}

	inline Eigen::Quaterniond filterQuaternion(const Eigen::Quaterniond& current,
		const Eigen::Quaterniond& previous,
		double alpha) {
		return previous.slerp(alpha, current);
	}

	inline bool isPositionChangeReasonable(const Eigen::Vector3d& currentPos,
		const Eigen::Vector3d& previousPos,
		double maxDistance) {
		return (currentPos - previousPos).norm() <= maxDistance;
	}

	inline Eigen::Vector3d computeVelocity(const Eigen::Vector3d& currentPos,
		const Eigen::Vector3d& previousPos,
		double deltaTime) {
		if (deltaTime <= 0.0) return Eigen::Vector3d::Zero();
		return (currentPos - previousPos) / deltaTime;
	}

	inline Eigen::Vector3d rotateVector(const Eigen::Vector3d& vector,
		const Eigen::Quaterniond& quaternion) {
		return quaternion * vector;
	}

} // namespace VR2EigenTransforms

#endif // VR_2_EIGEN_TRANSFORMS_H
