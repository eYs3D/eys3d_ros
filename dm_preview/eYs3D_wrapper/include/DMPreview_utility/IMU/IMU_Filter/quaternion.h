#ifndef QUATERNION_H
#define QUATERNION_H

#include <vector>

struct EulerAngles
{
    double yaw;
    double pitch;
    double roll;
};

std::vector<double> quaternionProduct(const std::vector<double> &a, const std::vector<double> &b);

std::vector<double> quaternionConjugate(const std::vector<double> &q);

EulerAngles quaternionToEulerAngles(const std::vector<double> &q);

#endif // QUATERNION_H
