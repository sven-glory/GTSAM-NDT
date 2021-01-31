#ifndef SCANSTUDIO_MATRIX_H
#define SCANSTUDIO_MATRIX_H

#include "ZTypes.h"
#include "misc.h"
#include "Vector.h"

#define LARGE_VAL 1e20

#define GETDATA2(m) \
        a = m->d[0][0]; \
        b = m->d[0][1]; \
        c = m->d[1][0]; \
        d = m->d[1][1];

#define GETDATA3() \
        _a = d[0][0]; \
        _b = d[0][1]; \
        _c = d[0][2]; \
        _d = d[1][0]; \
        _e = d[1][1]; \
        _f = d[1][2]; \
        _g = d[2][0]; \
        _h = d[2][1]; \
        _i = d[2][2];


/* error codes */
enum
{
    MATERR_MEM = 1,
    MATERR_NOMATRIX,
    MATERR_NOSQUAREMATRIX,
    MATERR_SIZE,
    MATERR_SINGULAR
};

class Matrix2
{
public:
	float d[2][2];

public:
	Matrix2(float _a = 0.0, float _b = 0.0, float _c = 0.0, float _d = 0.0)
	{
		Set(_a, _b, _c, _d);
	}

	void Set(float _a, float _b, float _c, float _d)
	{
		d[0][0] = _a;
		d[0][1] = _b;
		d[1][0] = _c;
		d[1][1] = _d;
	}

	// Inverts 2x2 Matrix m. Returns FALSE if matrix is singular.
	BOOL Inverse();

	// Transposes 2x2 Matrix m.
	void Transpose();

	// Adds A and B, result is stored in C. A and B are not altered.
	Matrix2 operator + (Matrix2& B);

	void operator += (Matrix2& B);

	// Multiplies matrix with vector x. Result is stored in result, x are not altered
	void MultVector(const Vector2& x, Vector2& result);

	// Multiplies A * B, result is stored in C. A and B are not altered.
	Matrix2 operator * (Matrix2& B);

	void operator *= (Matrix2& B);


};

class Matrix3
{
public:
    float d[3][3];

public:
	// Fills in matrix structure
	void Set(float _a, float _b, float _c, float _d, float _e, float _f,
		float _g, float _h, float _i);

	// clears angular covariance fields
	void Magic();

	// Returns determinante of matrix m
	float Determinante();

	// Returns the trace of matrix m
	float Trace();

	// Inverts 3x3 Matrix m. Returns FALSE if matrix is singular
	BOOL Inverse();

	// Transposes 3x3 Matrix m
	void Transpose();

	// Adds A and B, result is stored in C. A and B are not altered
	Matrix3 operator + (Matrix3& B);

	void operator += (Matrix3& B);

	// multiplies a with each matrix element
	void operator *= (float a);

	// Multiplies matrix with vector x. Result is stored in result, m and x are not altered.
	Vector3 operator * (Vector3& x); 

	// Multiplies A * B, result is stored in C. A and B are not altered.
	Matrix3 operator * (Matrix3& B);



};

typedef float elem_t;

/* generic matrix and vector */
struct Matrix
{
    int n, m;           /* rows and cols */
    elem_t **d;     /* pointer to rows (which itself points to elements) */
};

extern Vector3 Vector3Zero;
extern Matrix3 Matrix3Zero, Matrix3One;


void Matrix2GetEllipseParams(const Matrix2 *m, 
    float *a, float *b, float *alpha);
/*
** calculates main axis a and b and angle to x-axis.
*/

void Matrix2DrawEllipse(long cx, long cy, long a, long b, float alpha,
    void (*drawLine)(long x1, long y1, long x2, long y2));
/*
** Draws an ellipse at (cx, cy) with axis a and b and angle to x-axis
** alpha by approximating it with line segments and calling the
** supplied drawLine function.
*/


void Matrix3GetEllipseParams(const Matrix3 *m, 
    float *a, float *b, float *alpha);
/*
** calculates main axis a and b and angle to x-axis for 2x2 sub-matrix
*/

void Matrix3Print(const Matrix3 *m);
/*
** Prints matrix to stdout.
*/

/* Generic matrix functions */

struct Matrix *MatrixAlloc(int n, int m);
/*
** Allocates matrix structure and data space.
** Initial element values are undefined.
*/

void MatrixFree(struct Matrix *m);
/* Frees matrix structure and data space. */

void MatrixZero(struct Matrix *m);
/* Sets all elements to 0.0. */

int MatrixDeterminante(const struct Matrix *m, elem_t *det);
/* Returns determinante of matrix m. */

int MatrixInverse(struct Matrix *m, struct Matrix *res);
/*
** Calculates inverse of matrix m.
** Result is stored in matrix res (allocated by you!).
** Warning: m will be modified!
*/

int MatrixMult(const struct Matrix *A, const struct Matrix *B, 
    struct Matrix *C);
/*
** Calculates A * B (matrix multiplication and stores the result in C.
** C must be allocated by you!
*/

void MatrixPrint(const struct Matrix *m, FILE *file);
/* Prints matrix to file */

char *MatError(int errnum);
/* returns error string to given error number */

#endif
