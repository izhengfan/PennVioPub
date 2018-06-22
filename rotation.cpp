// Allrights reserved.
// Author: github.com/izhengfan (ZHENG, Fan)
/// \brief calculating a quaterion from a rotation matrix

#include <iostream>
#include <Eigen/Geometry>

using namespace std;

int main()
{
    Eigen::Matrix3d mat;

    mat << 0.99997679, 0.00434699, -0.00524601,
            -0.00435932, 0.99998776, -0.00234207,
            0.00523576, 0.00236488, 0.9999835;
    Eigen::Quaterniond q(mat);
    cout << q.x() << endl;
    cout << q.y() << endl;
    cout << q.z() << endl;
    cout << q.w() << endl;

    return 0;
}
