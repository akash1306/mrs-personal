#include<math_utils.hpp>



UtilMath::UtilMath(){
    std::cout<<"Initialized"<<std::endl;

}
double UtilMath::getMaxValue(const double a, const double b) {
    return (a > b) ? a : b;
  }

  //}

  /* heading2Rad() //{ */

  double UtilMath::heading2Rad(const double a) {
    if (a > 180) {
      return (a - 180) * M_PI / 180 - M_PI;
    } else {
      return a * M_PI / 180;
    }
  }

  //}

  /* heading2Deg() //{ */

  double UtilMath::heading2Deg(const double a) {
    if (a < 0) {
      return 180 + 180 * (M_PI + a) / M_PI;
    } else {
      return 180 * a / M_PI;
    }
  }
  
  //}

  /* headingDiff() //{ */

  double UtilMath::headingDiff(const double a, const double b) {
    double a_deg = heading2Deg(a), b_deg = heading2Deg(b);
    double d = fmod(abs(a_deg - b_deg), 360);
    double dif = a_deg - b_deg;
    double r;

    if (d > 180) {
      r = 360 - d;
    } else {
      r = d;
    }

    if ((dif >= 0 && dif <= 180) || (dif <= -180 && dif >= -360)) {
      r *= -1;
    }

    return heading2Rad(r);
  }

  //}

  /* relativeBearing() //{ */

  double UtilMath::relativeBearing(const double focalRbt_x, const double focalRbt_y, const double focalRbt_heading, const double neighborRbt_x,
                         const double neighborRbt_y) {
    double angle_between = atan2(neighborRbt_y - focalRbt_y, neighborRbt_x - focalRbt_x);

    return headingDiff(focalRbt_heading, angle_between);
  }

  //}

  /* inclination() //{ */

  double UtilMath::inclination(const double focalRbt_x, const double focalRbt_y, const double focalRbt_z, const double neighborRbt_x, const double neighborRbt_y, const double neighborRbt_z) {
    double x = neighborRbt_x - focalRbt_x;
    double y = neighborRbt_y - focalRbt_y;
    double z = neighborRbt_z - focalRbt_z;

    if (z == 0) {
      return 0;
    } else {
      return atan2(sqrt(pow(x, 2) + pow(y, 2)), z); 
    }
  }

int main(){
    UtilMath math_obj;
    std::cout << math_obj.getMaxValue(5.0,6.0)<<std::endl;
    return 0;
}