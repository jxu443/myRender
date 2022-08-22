#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>
#include "tgaimage.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const float PI = 3.14159265358;
const int width  = 300;
const int height = 300;

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green  = TGAColor(0,   255, 0,   255);
const TGAColor blue   = TGAColor(0,   0,   255, 255);
const TGAColor yellow = TGAColor(255, 255, 0,   255);

template <class t> struct Vec2 {
	union {
		struct {t u, v;};
		struct {t x, y;};
		t raw[2];
	};
	Vec2() : u(0), v(0) {}
	Vec2(t _u, t _v) : u(_u),v(_v) {}
	inline Vec2<t> operator +(const Vec2<t> &V) const { return Vec2<t>(u+V.u, v+V.v); }
	inline Vec2<t> operator -(const Vec2<t> &V) const { return Vec2<t>(u-V.u, v-V.v); }
	inline Vec2<t> operator *(float f)          const { return Vec2<t>(u*f, v*f); }
	inline t       operator *(const Vec2<t> &V) const { return x*V.x + y*V.y; }

	// vector
	float vlen() const {return x*x+y*y;}
	float norm () const { return std::sqrt(x*x+y*y);}
	Vec2<t> & normalize(t l=1) { *this = (*this)*(l/norm()); return *this; }

	Vec2<t> R() {return Vec2<t>(-y, x);}
};

template <class t> struct Vec3 {
	union {
		struct {t x, y, z;};
		struct { t ivert, iuv, inorm; };
		t raw[3];
	};
	Vec3() : x(0), y(0), z(0) {}
	Vec3(t _x, t _y, t _z) : x(_x),y(_y),z(_z) {}
	inline Vec3<t> operator ^(const Vec3<t> &v) const { return Vec3<t>(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x); } // cross
	inline Vec3<t> operator +(const Vec3<t> &v) const { return Vec3<t>(x+v.x, y+v.y, z+v.z); }
	inline Vec3<t> operator -(const Vec3<t> &v) const { return Vec3<t>(x-v.x, y-v.y, z-v.z); }
	inline Vec3<t> operator *(float f)          const { return Vec3<t>(x*f, y*f, z*f); } // scale
	inline t       operator *(const Vec3<t> &v) const { return x*v.x + y*v.y + z*v.z; } // dot 
	
	float vlen() const {return x*x+y*y+z*z;}
	float norm () const { return std::sqrt(x*x+y*y+z*z); }
	Vec3<t> & normalize(t l=1) { *this = (*this)*(l/norm()); return *this; }
};

typedef Vec2<float> Vec2f;
typedef Vec2<int>   Vec2i;
typedef Vec3<float> Vec3f;
typedef Vec3<int>   Vec3i;

//Line
void line(Vec3f p00, Vec3f p11, TGAImage &image, TGAColor color);

//Triangle
float triangleArea(Vec2i A, Vec2i B, Vec2i C);

template <typename T> 
Vec3f barycentric(T *pts, T p) {
    float area = triangleArea(pts[0],pts[1], pts[2]);
    float k0 = triangleArea(p, pts[1], pts[2])/area;
    float k1 = triangleArea(p, pts[0], pts[2])/area;
    float k2 = triangleArea(p, pts[0], pts[1])/area; 
    return Vec3f(k0, k1, k2);
}

#endif //__GEOMETRY_H__
