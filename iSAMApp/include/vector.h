#ifndef __VECTOR
#define __VECTOR

class Vector2
{
public:
    double d[2];

public:
	Vector2(double a = 0.0, double b = 0.0) 
	{
		Set(a, b);
	}

	void Set(double a, double b)
	{
		d[0] = a;
		d[1] = b;
	}

	Vector2 operator +(const Vector2& v2)
	{
		Vector2 Result;
		for (int i = 0; i < 2; i++)
			Result.d[i] = d[i] + v2.d[i];

		return Result;
	}

	Vector2 operator -(const Vector2& v2)
	{
		Vector2 Result;
		for (int i = 0; i < 2; i++)
			Result.d[i] = d[i] - v2.d[i];

		return Result;
	}

	void operator +=(const Vector2& v2)
	{
		for (int i = 0; i < 2; i++)
			d[i] += v2.d[i];
	}

	void operator -=(const Vector2& v2)
	{
		for (int i = 0; i < 2; i++)
			d[i] -= v2.d[i];
	}
};

class Vector3
{
public:
    double d[3];

public:
	Vector3(double a = 0.0,  double b = 0.0, double c = 0.0)
	{
		Set(a, b, c);
	}

	void Set(double a, double b, double c)
	{
		d[0] = a;
		d[1] = b;
		d[2] = c;
	}

	Vector3 operator +(const Vector3& v2)
	{
		Vector3 Result;
		for (int i = 0; i < 3; i++)
			Result.d[i] = d[i] + v2.d[i];

		return Result;
	}

	Vector3 operator -(const Vector3& v2)
	{
		Vector3 Result;
		for (int i = 0; i < 3; i++)
			Result.d[i] = d[i] - v2.d[i];

		return Result;
	}

	void operator +=(const Vector3& v2)
	{
		for (int i = 0; i < 3; i++)
			d[i] += v2.d[i];
	}

	void operator -=(const Vector3& v2)
	{
		for (int i = 0; i < 3; i++)
			d[i] -= v2.d[i];
	}

	//
	// multiplies a with each vector element.
	//
	void operator *= (double a)
	{
		d[0] *= a;
		d[1] *= a;
		d[2] *= a;
	}

	//
	// Returns transpose(v1) * v2
	//
	double operator * (const Vector3& v2)
	{
		return (d[0]*v2.d[0] + d[1]*v2.d[1] + d[2]*v2.d[2]);
	}

};
#endif
