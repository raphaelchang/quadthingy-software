#ifndef _UTILS_H_
#define _UTILS_H_

#define M_PI 3.14159265358979328

class Utils
{
public:
    static inline float atan2(float y, float x) {
        float abs_y = fabsf(y) + 1e-10;
        float res;

        if (x >= 0) {
            float r = (x - abs_y) / (x + abs_y);
            float rsq = r * r;
            res = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
        } else {
            float r = (x + abs_y) / (abs_y - x);
            float rsq = r * r;
            res = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
        }

        if (y < 0) {
            return(-res);
        } else {
            return(res);
        }
    }
    static inline void sincos(float angle, float *sin, float *cos) {
        //always wrap input angle to -PI..PI
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }

        while (angle >  M_PI) {
            angle -= 2.0 * M_PI;
        }

        //compute sine
        if (angle < 0.0) {
            *sin = 1.27323954 * angle + 0.405284735 * angle * angle;

            if (*sin < 0.0) {
                *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
            } else {
                *sin = 0.225 * (*sin * *sin - *sin) + *sin;
            }
        } else {
            *sin = 1.27323954 * angle - 0.405284735 * angle * angle;

            if (*sin < 0.0) {
                *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
            } else {
                *sin = 0.225 * (*sin * *sin - *sin) + *sin;
            }
        }

        // compute cosine: sin(x + PI/2) = cos(x)
        angle += 0.5 * M_PI;
        if (angle >  M_PI) {
            angle -= 2.0 * M_PI;
        }

        if (angle < 0.0) {
            *cos = 1.27323954 * angle + 0.405284735 * angle * angle;

            if (*cos < 0.0) {
                *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
            } else {
                *cos = 0.225 * (*cos * *cos - *cos) + *cos;
            }
        } else {
            *cos = 1.27323954 * angle - 0.405284735 * angle * angle;

            if (*cos < 0.0) {
                *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
            } else {
                *cos = 0.225 * (*cos * *cos - *cos) + *cos;
            }
        }
    }
};

#endif /* _UTILS_H_ */
