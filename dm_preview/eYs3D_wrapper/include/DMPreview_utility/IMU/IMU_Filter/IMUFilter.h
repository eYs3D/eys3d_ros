#ifndef IMUFILTER_H
#define IMUFILTER_H

class IMUFilter
{
public:
    IMUFilter() = default;
    virtual ~IMUFilter() = default;
public:
    virtual void getOrientation(double& q0, double& q1, double& q2, double& q3) = 0;
    virtual void setOrientation(double q0, double q1, double q2, double q3) = 0;

    virtual void update(double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double mx, double my, double mz,
                        double dt) = 0;
    virtual void update(double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double dt) = 0;
};

#endif // IMUFILTER_H
