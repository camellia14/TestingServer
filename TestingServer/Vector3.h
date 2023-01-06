#pragma once

template <typename T>
class Vector3
{
public:
	T x{}, y{}, z{};
	Vector3() {}
	Vector3(T x, T y, T z) :x(x), y(y), z(z) {}

	inline Vector3<T> operator +(Vector3<T> v) const
	{
		return Vector3<T>(x + v.x, y + v.y, z + v.z);
	}
	inline Vector3<T> operator -(Vector3<T> v) const
	{
		return Vector3<T>(x - v.x, y - v.y, z - v.z);
	}
	inline Vector3<T> operator *(Vector3<T> v) const
	{
		return Vector3<T>(x * v.x, y * v.y, z * v.z);
	}
	inline Vector3<T> operator /(Vector3<T> v) const
	{
		return Vector3<T>(x / v.x, y / v.y, z / v.z);
	}
	inline Vector3<T> operator +(T v) const
	{
		return Vector3<T>(x + v, y + v, z + v);
	}
	inline Vector3<T> operator -(T v) const
	{
		return Vector3<T>(x - v, y - v, z - v);
	}
	inline Vector3<T> operator *(T v) const
	{
		return Vector3<T>(x * v, y * v, z * v);
	}
	inline Vector3<T> operator /(T v) const
	{
		return Vector3<T>(x / v, y / v, z / v);
	}
	inline void operator +=(Vector3<T> v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}
	inline void operator -=(Vector3<T> v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}
	inline void operator *=(Vector3<T> v)
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}
	inline void operator /=(Vector3<T> v)
	{
		x /= v.x;
		y /= v.y;
		z /= v.z;
	}
	inline void operator +=(T v)
	{
		x += v;
		y += v;
		z += v;
	}
	inline void operator -=(T v)
	{
		x -= v;
		y -= v;
		z -= v;
	}
	inline void operator *=(T v)
	{
		x *= x;
		y *= y;
		z *= z;
	}
	inline void operator /=(T v)
	{
		x /= x;
		y /= y;
		z /= z;
	}

	inline bool operator==(const Vector3<T>& v) const {
		return x == v.x && y == v.y && z == v.z;
	}
};
