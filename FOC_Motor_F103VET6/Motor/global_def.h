#pragma once

#ifndef PI
#define PI 3.14159265358979323846
#endif // PI

#define deg2rad(deg) ((deg) * (PI / 180.0))
#define rad2deg(rad) ((rad) * (180.0 / PI))

#define max(a, b) (( (a) > (b) ) ? (a) : (b))
#define min(a, b) (( (a) < (b) ) ? (a) : (b))