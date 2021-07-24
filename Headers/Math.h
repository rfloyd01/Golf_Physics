#pragma once

#include <vector>
#include "glm.h"

struct Vector_3d
{
	double x = 0;
	double y = 0;
	double z = 0;
};

float DotProduct(std::vector<float> vec1, std::vector<float> vec2);
std::vector<float> CrossProduct(std::vector<float> vec1, std::vector<float> vec2);

float Magnitude(std::vector<float> vec);
void Normalize(glm::quat& q);
void Normalize(std::vector<float> &vec);

void QuatRotate(glm::quat q, std::vector<float>& data);
void QuatRotate(glm::dquat q, std::vector<double>& data);
void QuatRotate(glm::dquat q, Vector_3d& data);
void QuatRotate(glm::quat q, glm::vec3& data);
void QuatRotate(glm::quat q1, glm::quat& q2);
glm::quat QuaternionMultiply(glm::quat q1, glm::quat q2);
glm::quat GetRotationQuaternion(std::vector<float> vec1, std::vector<float> vec2);
glm::quat GetRotationQuaternion(float angle, std::vector<float> vec);
glm::quat Conjugate(glm::quat q);

void matrixMultiply(float* m1, int rows1, int columns1, float* m2, int rows2, int columns2, float* prod);

//Vector_3d functions
double DotProduct(Vector_3d A, Vector_3d B);
Vector_3d CrossProduct(Vector_3d A, Vector_3d B);
Vector_3d VectorAdd(Vector_3d A, Vector_3d B);
void VectorMult(Vector_3d& A, double s);
double VectorMagnitude(Vector_3d A);
Vector_3d VectorProjection(Vector_3d A, Vector_3d B);
