#include "stdafx.h"
#include "misc.h"
#include "matrix.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


Vector3 Vector3Zero;

Matrix3 Matrix3Zero =
{
	{ 
		{0.0, 0.0, 0.0,},
		{0.0, 0.0, 0.0,},
		{0.0, 0.0, 0.0,},
	}
};

Matrix3 Matrix3One =
{
	{
		{1.0, 0.0, 0.0,},
		{0.0, 1.0, 0.0,},
		{0.0, 0.0, 1.0,},
	}
};

///////////////////////////////////////////////////////////////////////////////

//
// Inverts 2x2 Matrix m. Returns FALSE if matrix is singular.
//
BOOL Matrix2::Inverse()
{
	float d00 = d[0][0];
	float d01 = d[0][1];
	float d10 = d[1][0];
	float d11 = d[1][1];

	float det = d00*d11 - d01*d10;
	if (NEAR_ZERO(det))
	{
		d[0][0] = d[1][1] = LARGE_VAL;
		d[1][0] = d[0][1] = 0.0;
		return FALSE;
	}
	
	d[0][0] = d11 / det;
	d[0][1] = -d01 / det;
	d[1][0] = -d10 / det;
	d[1][1] = d00 / det;

	return TRUE;
}

//
//   Transposes 2x2 Matrix m.
//
void Matrix2::Transpose()
{
	float d01 = d[0][1];

	d[0][1] = d[1][0];
	d[1][0] = d01;
}

//
//   Adds A and B, result is stored in C. A and B are not altered.
//
Matrix2 Matrix2::operator + (Matrix2& B)
{
	Matrix2 Result;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			Result.d[i][j] = d[i][j] + B.d[i][j];

	return Result;
}

void Matrix2::operator += (Matrix2& B)
{
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			d[i][j] += B.d[i][j];
}

/*
** multiplies matrix with vector x. Result is stored in result,
** m and x are not altered.
*/
//
//   Multiplies matrix with vector x. Result is stored in result, x are not altered.
//
void Matrix2::MultVector(const Vector2& x, Vector2& result)
{
	for (int i = 0; i < 2; i++)
	{
		result.d[i] = 0.0;

		for (int j = 0; j < 2; j++)
			result.d[i] += d[i][j] * x.d[j];
	}
}

//
//   Multiplies A * B, result is stored in C. A and B are not altered.
//
Matrix2 Matrix2::operator * (Matrix2& B)
{
	Matrix2 result;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
		{
			float val = 0.0;
			for (int k = 0; k < 2; k++)
				val += d[i][k] * B.d[k][j];

			result.d[i][j] = val;
		}

	return result;
}

void Matrix2::operator *= (Matrix2& B)
{
	Matrix2 m = *this;
	*this = m * B;
}

static void GetEllipseParams(float a, float b, float c, float d,
    float *pa, float *pb, float *palpha)
{
	static const float mahaDist = 1.0;
	float h, lambda1, lambda2;

	/* get eigen-values */
	h = (a - d)*(a - d)/4.0 + b*c;

	if (h > 0.0) 
	{
		h = sqrt(h);
	}
	else 
	{
		if (h < -1e-8) 
		{
			fprintf(stderr, "GetEllipseParams(%g %g %g %g): det (%g) < 0!\n",
				a, b, c, d, h);
		}
		h = 0.0;
	}

	lambda1 = (a + d)/2.0 + h;
	lambda2 = (a + d)/2.0 - h;

	if (lambda1 < 0.0) 
	{
		fprintf(stderr, "GetEllipseParams(%g %g %g %g): lambda1 (%g) < 0!\n",
	            a, b, c, d, lambda1);
		lambda1 = 0.0;
	}

	if (lambda2 < 0.0) 
	{
		fprintf(stderr, "GetEllipseParams(%g %g %g %g): lambda2 (%g) < 0!\n",
                a, b, c, d, lambda2);
		lambda2 = 0.0;
	}

	if (pa) 
		*pa = sqrt(mahaDist * lambda1);

	if (pb) 
		*pb = sqrt(mahaDist * lambda2);

	if (palpha) 
		*palpha = atan2(lambda1 - a, b);
}

void Matrix2GetEllipseParams(const Matrix2 *m, 
    float *pa, float *pb, float *palpha)
{
	if (m) 
		GetEllipseParams(m->d[0][0], m->d[0][1], m->d[1][0], m->d[1][1], pa, pb, palpha);
}

void Matrix2DrawEllipse(long cx, long cy, long a, long b, float alpha,
    void (*drawLine)(long x1, long y1, long x2, long y2))
{
	float sina, cosa;
	long p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y;
	long qx, qy;
	float x, y;
	float step;
	int help;

	if (drawLine == NULL)
		return;

	if (b > a)
	{
		help = a;
		a = b;
		b = help;
		alpha += PI/2;
	}

	sina = sin(alpha);
	cosa = cos(alpha);

	if (a == 0)
	{
		/* ellipse is only a point */

		drawLine(cx, cy, cx, cy);
		return;
	}

	p1x = p2x = cx - a * cosa;
	p1y = p2y = cy - a * sina;
	p3x = p4x = cx + a * cosa;
	p3y = p4y = cy + a * sina;

	step = a / 50.0;
	if (step < 1.0)
	step = 1.0;

	for (x = -a; ; x = MIN(x + step, 0.0))
	{
		y = sqrt(b*b * (1.0 - (x/a)*(x/a)));

		qx = cx + x*cosa - y*sina;
		qy = cy + x*sina + y*cosa;
		drawLine(p1x, p1y, qx, qy);
		p1x = qx;
		p1y = qy;

		qx = cx + x*cosa + y*sina;
		qy = cy + x*sina - y*cosa;
		drawLine(p2x, p2y, qx, qy);
		p2x = qx;
		p2y = qy;

		qx = cx - x*cosa - y*sina;
		qy = cy - x*sina + y*cosa;
		drawLine(p3x, p3y, qx, qy);
		p3x = qx;
		p3y = qy;

		qx = cx - x*cosa + y*sina;
		qy = cy - x*sina - y*cosa;
		drawLine(p4x, p4y, qx, qy);
		p4x = qx;
		p4y = qy;

		if (x >= 0.0)
			break;
	}
}

//
//   Fills in matrix structure.
//
void Matrix3::Set(float _a, float _b, float _c, float _d, float _e, 
						float _f, float _g, float _h, float _i)
{
	d[0][0] = _a;
	d[0][1] = _b;
	d[0][2] = _c;
	d[1][0] = _d;
	d[1][1] = _e;
	d[1][2] = _f;
	d[2][0] = _g;
	d[2][1] = _h;
	d[2][2] = _i;
}

//
//   clears angular covariance fields.
//
void Matrix3::Magic()
{
#if 1  /* it seems that these values can cause bad effects, linearization?. */
	d[0][2] = 0.0;
	d[1][2] = 0.0;
	d[2][0] = 0.0;
	d[2][1] = 0.0;
#endif
}


//
//   Returns determinante of matrix m.
//
float Matrix3::Determinante()
{
	/*  
	** we use the bottom line since for position covarianve
	** matries there are often extreme numbers here.
	*/
	float det = d[2][0] * (d[0][1]*d[1][2] - d[0][2]*d[1][1]) - 
	             d[2][1] * (d[0][0]*d[1][2] - d[0][2]*d[1][0]) + 
	             d[2][2] * (d[0][0]*d[1][1] - d[0][1]*d[1][0]);
	return det;
}

/*
        a = m->d[0][0]; \
        b = m->d[0][1]; \
        c = m->d[0][2]; \
        d = m->d[1][0]; \
        e = m->d[1][1]; \
        f = m->d[1][2]; \
        g = m->d[2][0]; \
        h = m->d[2][1]; \
        i = m->d[2][2];
*/
//
// Returns the trace of matrix m
//
float Matrix3::Trace()
{
	float trace = d[0][0] * d[1][1] * d[2][2];
	return trace;
}

//
//   Inverts 3x3 Matrix m. Returns FALSE if matrix is singular.
//
BOOL Matrix3::Inverse()
{
	float _a, _b, _c, _d, _e, _f, _g, _h, _i;
	float det = Determinante();

	GETDATA3();                /* loads a - i */

	if (NEAR_ZERO(det))
	{
		Set(
			1.0 / _a, 0.0, 0.0,
			0.0, 1.0 / _e, 0.0,
			0.0, 0.0, 1.0 / _i);
		return TRUE;
	}
	else if (isnan(det))
	{
		*this = Matrix3Zero;
		return FALSE;
	}

	d[0][0] = +(_e * _i - _f * _h) / det;
	d[0][1] = -(_b * _i - _c * _h) / det;
	d[0][2] = +(_b * _f - _c * _e) / det;
	d[1][0] = -(_d * _i - _f * _g) / det;
	d[1][1] = +(_a * _i - _c * _g) / det;
	d[1][2] = -(_a * _f - _c * _d) / det;
	d[2][0] = +(_d * _h - _e * _g) / det;
	d[2][1] = -(_a * _h - _b * _g) / det;
	d[2][2] = +(_a * _e - _b * _d) / det;

	return TRUE;
}

//
//   Transposes 3x3 Matrix m.
//
void Matrix3::Transpose()
{
	float _a, _b, _c, _d, _e, _f, _g, _h, _i;

	GETDATA3();                /* loads a - i */

	d[0][1] = _d;
	d[0][2] = _g;
	d[1][0] = _b;
	d[1][2] = _h;
	d[2][0] = _c;
	d[2][1] = _f;
}

//
//   Adds A and B, result is stored in C. A and B are not altered.
//
Matrix3 Matrix3::operator + (Matrix3& B)
{
	Matrix3 result;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			result.d[i][j] = d[i][j] + B.d[i][j];

	return result;
}

void Matrix3::operator += (Matrix3& B)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			d[i][j] += B.d[i][j];
}

//
//   multiplies a with each matrix element.
//
void Matrix3::operator *= (float a)
{
	if (a != 1.0)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				d[i][j] *= a;
	}
}

//
//   Multiplies matrix with vector x. Result is stored in result, m and x are not altered.
//
Vector3 Matrix3::operator * (Vector3& x)
{
	Vector3 result;

	result.d[0] = d[0][0] * x.d[0] + d[0][1]*x.d[1] + d[0][2]*x.d[2];
	result.d[1] = d[1][0] * x.d[0] + d[1][1]*x.d[1] + d[1][2]*x.d[2];
	result.d[2] = d[2][0] * x.d[0] + d[2][1]*x.d[1] + d[2][2]*x.d[2];

	return result;
}

//
//   Multiplies A * B, result is stored in C. A and B are not altered.
//
Matrix3 Matrix3::operator * (Matrix3& B)
{
	Matrix3 result;

	result.d[0][0] = d[0][0] * B.d[0][0] + d[0][1] * B.d[1][0] + d[0][2] * B.d[2][0];
	result.d[1][0] = d[1][0] * B.d[0][0] + d[1][1] * B.d[1][0] + d[1][2] * B.d[2][0];
	result.d[2][0] = d[2][0] * B.d[0][0] + d[2][1] * B.d[1][0] + d[2][2] * B.d[2][0];
	result.d[0][1] = d[0][0] * B.d[0][1] + d[0][1] * B.d[1][1] + d[0][2] * B.d[2][1];
	result.d[1][1] = d[1][0] * B.d[0][1] + d[1][1] * B.d[1][1] + d[1][2] * B.d[2][1];
	result.d[2][1] = d[2][0] * B.d[0][1] + d[2][1] * B.d[1][1] + d[2][2] * B.d[2][1];
	result.d[0][2] = d[0][0] * B.d[0][2] + d[0][1] * B.d[1][2] + d[0][2] * B.d[2][2];
	result.d[1][2] = d[1][0] * B.d[0][2] + d[1][1] * B.d[1][2] + d[1][2] * B.d[2][2];
	result.d[2][2] = d[2][0] * B.d[0][2] + d[2][1] * B.d[1][2] + d[2][2] * B.d[2][2];

	return result;
}

void Matrix3GetEllipseParams(const Matrix3 *m, 
    float *pa, float *pb, float *palpha)
{
	if (m) {
	GetEllipseParams(m->d[0][0], m->d[0][1], m->d[1][0], m->d[1][1],
		pa, pb, palpha);
	}
}

void Matrix3Print(const Matrix3 *m)
{
	int i, j;

	if (m != NULL)
	{
		for (i=0; i < 3; i++)
		{
			for (j=0; j < 3; j++)
				printf("%15g\t", m->d[i][j]);
			printf("\n");
		}
		printf("\n");
	}
}

/*
** Generic matrix functions
*/

void MatrixFree(struct Matrix *matrix)
{
	if (matrix != NULL)
	{
		int i;

		if (matrix->d != NULL)
		{
			for (i=0; i < matrix->n; i++)
				if (matrix->d[i] != NULL)
					SSfree(matrix->d[i]);
			SSfree(matrix->d);
		}
		SSfree(matrix);
	}
}

struct Matrix *MatrixAlloc(int n, int m)
{
	struct Matrix *matrix = NULL;
	int i;

	matrix = (Matrix *)SSmalloc(sizeof(struct Matrix));
	if (matrix == NULL)
		return(NULL);

	matrix->n = n;
	matrix->m = m;
	matrix->d = (elem_t **)SSmalloc(n * sizeof(elem_t *));
	if (matrix->d == NULL)
	{
		MatrixFree(matrix);
		return(NULL);
	}
	for (i=0; i<n; i++)
		matrix->d[i] = NULL;

	for (i=0; i<n; i++)
	{
		matrix->d[i] = (elem_t *)SSmalloc(m * sizeof(elem_t));
		if (matrix->d[i] == NULL)
		{
			MatrixFree(matrix);
			return(NULL);
		}
	}
	return(matrix);
}

void MatrixZero(struct Matrix *matrix)
{
	if (matrix != NULL)
	{
		int i, j;

		for (i=0; i < matrix->n; i++)
			for (j=0; j < matrix->m; j++)
				matrix->d[i][j] = 0.0;
	}
}

void MatrixOne(struct Matrix *matrix)
{
	if (matrix != NULL)
	{
		int i, n = MIN(matrix->n, matrix->m);

		MatrixZero(matrix);
		for (i=0; i < n; i++)
			matrix->d[i][i] = 1.0;
	}
}

void MatrixAdd(const struct Matrix *A, const struct Matrix *B, struct Matrix *C)
{
	int i, j, n, m;

	if (A == NULL || B == NULL || C == NULL)
		return;
	if (A->n != B->n || A->n != C->n)
	return;
	if (A->m != B->m || A->n != C->m)
	return;

	n = A->n;
	m = A->m;

	for (i=0; i<n; i++)
		for (j=0; j<m; j++)
			C->d[i][j] = A->d[i][j] + B->d[i][j];
}

static bool finditem(int item, int *list, int size)
{
	int i;

	for (i=0; i < size; i++, list++)
		if (*list == item)
			return(TRUE);
	return(FALSE);
}

static elem_t MatrixSubDeterminante(const struct Matrix *m, int n,
		int *deli, int *delj)
/*
** Calculates determinante of a sub matrix of m which has the size size n x n.
** deli specifies the rows to ignore, delj the cols to ignore.
** (There are exactly (m->n - n) rows and cols to ignore.
**
** Reference: U. Stammbach: Lineare Algebra, p. 136
*/
{
	int i, j, deln = m->n - n;
	elem_t sum = 0.0, sign = 1.0;

	if (n < 1)				   /* end of recursion */
		return(1.0);

	/*
	** Recursive part.
	** First thing is to determine a free row for the
	** determinante evaluation.
	*/
	for (i=0; i < m->n; i++)
		if (!finditem(i, deli, deln))
			break;

	/*
	** Now loop through the row.
	*/
	for (j=0; j < m->n; j++)
	{
		if (!finditem(j, delj, deln))
		{
			deli[deln] = i;
			delj[deln] = j;
			sum += sign * m->d[i][j] * MatrixSubDeterminante(m, n-1, deli, delj);
			sign = -sign;
		}
	}

	return(sum);
}

int MatrixDeterminante(const struct Matrix *m, elem_t *det)
{
	int *deli = NULL, *delj = NULL;
	int n, err = OK;
	elem_t rc;

	if (m == NULL)
		return(MATERR_NOMATRIX);
	if (m->n != m->m)
		return(MATERR_NOSQUAREMATRIX);

	n = m->n;
	deli = (int *)SSmalloc(n * sizeof(int));
	delj = (int *)SSmalloc(n * sizeof(int));

	if (deli != NULL && delj != NULL)
	{
		rc = MatrixSubDeterminante(m, n, deli, delj);
		if (det != NULL)
			*det = rc;
	}
	else
		err = MATERR_MEM;

	if (delj != NULL)
		SSfree(delj);
	if (deli != NULL)
		SSfree(deli);

	return(err);
}

#define LUDECOMPOSE

#ifdef LUDECOMPOSE
/* 
** Algorithm from Recipes in C.
** Modified for our purpose (index from 0...(n-1), types)
*/

static int ludcmp(elem_t **a, int n, int *indx, elem_t *d)
{
	int i, imax, j, k; 
	elem_t big, dum, sum, temp;
	elem_t *vv;

	/* vv stores the implicit scaling of each row. */
	vv = (elem_t *)SSmalloc(n * sizeof(elem_t));  
	if (vv == NULL)
	return(MATERR_MEM);
   
	*d = 1.0;		   /* no row interchanges yet. */

	/* Loop over rows to get the implicit scaling information. */
	for (i=0; i < n; i++)
	{
	big = 0.0;
	for (j=0; j < n; j++)
		if ((temp = fabs(a[i][j])) > big)
		big = temp;
	if (big == 0.0)
	{
		SSfree(vv);
		return(MATERR_SINGULAR);
	}
	vv[i] = 1.0 / big;	  /* save the scaling. */
	}

	for (j=0; j < n; j++)		/* loop over columns */
	{
	for (i=0; i < j; i++)		
	{
		sum = a[i][j];
		for (k=0; k < i; k++)
		sum -= a[i][k] * a[k][j];
		a[i][j] = sum;
	}
	big = 0.0;		  /* search for largest pivot element */
	imax = j;
	for (i=j; i < n; i++)
	{
		sum = a[i][j];
		for (k=0; k < j; k++)
		sum -= a[i][k] * a[k][j];
		a[i][j] = sum;
		if ((dum = vv[i] * fabs(sum)) >= big)
		{
		/* figure of merit for pivot is better than the best so far */
		big = dum;
		imax = i;
		}
	}
	if (j != imax)	   /* do we need to interchange rows? */
	{
		for (k=0; k < n; k++)
		{
		dum = a[imax][k];
		a[imax][k] = a[j][k];
		a[j][k] = dum;
		}
		*d = -(*d);	 /* change the parity of d. */
		vv[imax] = vv[j];   /* also interchange the scale factor. */
	}
	indx[j] = imax;
	if (a[j][j] == 0.0)
	{
		SSfree(vv);
		return(MATERR_SINGULAR);
	}

	if (j != n-1)
	{
		dum = 1.0 / a[j][j];
		for (i=j+1; i < n; i++)
		a[i][j] *= dum;
	}
	}
	SSfree(vv);
	return(0);
}

static void lubksb(elem_t **a, int n, int *indx, elem_t *b)
{
	int i, ii=-1, ip, j;
	elem_t sum;

	for (i=0; i < n; i++)
	{
	ip = indx[i];
	sum = b[ip];
	b[ip] = b[i];
	if (ii >= 0)
		for (j=ii; j <= i-1; j++)
		sum -= a[i][j] * b[j];
	else if (sum)
		ii = i;
	b[i] = sum;
	}
	for (i=n-1; i>=0; i--)
	{
	sum = b[i];
	for (j=i+1; j < n; j++)
		sum -= a[i][j] * b[j];
	b[i] = sum / a[i][i];
	}
}

int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
	int n, rc = 0;
	struct Matrix *copy = NULL;
	elem_t d, *col = NULL;
	int i, j, *indx = NULL;

	if (m == NULL || res == NULL)
		return(MATERR_NOMATRIX);
	if (m->n != m->m || res->n != res->m)
		return(MATERR_NOSQUAREMATRIX);
	if (m->n != res->n)
		return(MATERR_SIZE);

	n = m->n;

	copy = MatrixAlloc(n, n);
	col = (elem_t *)SSmalloc(n * sizeof(elem_t));
	indx = (int *)SSmalloc(n * sizeof(int));
	if (copy == NULL || indx == NULL)
	rc = MATERR_MEM;

	if (rc == 0)
	{
	MatrixZero(copy);
	MatrixAdd(copy, m, copy);

	rc = ludcmp(copy->d, n, indx, &d);
	}

	if (rc == 0)
	{
	for (j=0; j < n; j++)
	{
		for (i=0; i < n; i++)
		col[i] = 0.0;
		col[j] = 1.0;
		lubksb(copy->d, n, indx, col);
		for (i=0; i < n; i++)
		res->d[i][j] = col[i];
	}
	}

	if (indx != NULL)
	SSfree(indx);
	if (col != NULL)
	SSfree(col);
	MatrixFree(copy);
	
	return(rc);
}

#elif (defined GAUSS_ELIMINATION_WITH_SIMPLE_PIVOTING)
int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
	int n, i, j, k, pivot;
	elem_t *help, fac, pivot_val;

	if (m == NULL || res == NULL)
		return(MATERR_NOMATRIX);
	if (m->n != m->m || res->n != res->m)
		return(MATERR_NOSQUAREMATRIX);
	if (m->n != res->n)
		return(MATERR_SIZE);

	n = m->n;
	MatrixOne(res);

	/*				  /1 * *\
	** bring matrix on  |0 1 *| form
	**				  \0 0 1/
	*/
	for (j=0; j < n; j++)				/* loop through cols */
	{
		/* find row with maximum value in col j (pivot) */
		pivot_val = 0.0;
		for (i=j; i < n; i++)
			if (fabs(m->d[i][j]) > pivot_val)
			{
				pivot_val = fabs(m->d[i][j]);
				pivot = i;
			}
		if (NEAR_ZERO(pivot_val))
			return(MATERR_SINGULAR);	/* can't calculate inverse matrix */
		if (pivot != j)
		{
			/* exchange rows */
			help = m->d[pivot];
			m->d[pivot] = m->d[j];
			m->d[j] = help;
			help = res->d[pivot];
			res->d[pivot] = res->d[j];
			res->d[j] = help;
		}

		/* Divide row by pivot element */
		fac = 1.0 / m->d[j][j];
		for (k=0; k < n; k++)
		{
			m->d[j][k] *= fac;
			res->d[j][k] *= fac;
		}

		/*
		** no we can zero the j-th element of all rows below row j
		** by basic row operations.
		*/
		for (i=j+1; i < n; i++)
		{
			if (m->d[i][j] == 0.0)
				continue;			   /* already zero */
			fac = m->d[i][j];
			for (k=0; k < n; k++)
			{
				m->d[i][k] -= m->d[j][k] * fac;
				res->d[i][k] -= res->d[j][k] * fac;
			}
		}
	}

    /*                  /1 0 0\
    ** bring matrix on  |0 1 0| form
    **                  \0 0 1/
    */
    for (j=n-1; j >= 0; j--)                /* loop through cols */
    {
		/* check col j */
		if (NEAR_ZERO(m->d[j][j]))
			return(MATERR_SINGULAR);	/* can't calculate inverse matrix */

		/*
		** no we can zero the j-th element of all rows above row j
		** by basic row operations.
		*/
		for (i=j-1; i >= 0; i--)
		{
			if (m->d[i][j] == 0.0)
				continue;			   /* already zero */
			fac = m->d[i][j];
			for (k=0; k < n; k++)
			{
				m->d[i][k] -= m->d[j][k] * fac;
				res->d[i][k] -= res->d[j][k] * fac;
			}
		}
	}

	return(OK);
}
#else
/*
** Thanks to Ralf Corsepius for the following piece of code:
*/
int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
  /*
   * This routine calculates a dim x dim inverse matrix.  It uses Gaussian
   * elimination on the system [matrix][inverse] = [identity].  The values
   * of the matrix then become the coefficients of the system of equations
   * that evolves from this equation.  The system is then solved for the
   * values of [inverse].  The algorithm solves for each column of [inverse]
   * in turn.  Partial pivoting is also done to reduce the numerical error
   * involved in the computations.  If singularity is detected, the routine
   * ends and returns an error.
   *
   * (See Numerical Analysis, L.W. Johnson and R.D.Riess, 1982).
   */

	int ipivot = 0, h, i, j, k;
	struct Matrix *aug;
	elem_t pivot, q;
	int dim;
	
	if (m == NULL || res == NULL)
		return(MATERR_NOMATRIX);
	if (m->n != m->m || res->n != res->m)
		return(MATERR_NOSQUAREMATRIX);
	if (m->n != res->n)
		return(MATERR_SIZE);

	dim = m->n;

	aug = MatrixAlloc(dim, dim+1);
	if (aug == NULL)
	return(MATERR_MEM);
	  
	for (h = 0; h < dim; h++)   /* solve column by column */
	{
		/*
	 * Set up the augmented matrix for [matrix][inverse] = [identity]
	 */

		for (i = 0; i < dim; i++)
		{
		memcpy(aug->d[i], m->d[i], dim * sizeof(elem_t));
			aug->d[i][dim] = (h == i) ? 1.0 : 0.0;
		}

		/*
	 * Search for the largest entry in column i, rows i through dim-1.
		 * ipivot is the row index of the largest entry.
	 */

		for (i = 0; i < dim-1; i++)
		{
			pivot = 0.0;

			for (j = i; j < dim; j++)
			{
			elem_t temp ;
		
			temp = ABS (aug->d[j][i]);
			if (pivot < temp)
			{
				pivot = temp;
				ipivot = j;
			}
			}
			if (NEAR_ZERO (pivot))		  /* singularity check */
			{
			MatrixFree(aug);
			return(MATERR_SINGULAR);
		}

			/* interchange rows i and ipivot */

			if (ipivot != i)
			{
		elem_t *temp ;
		
		temp = aug->d[i];
		aug->d[i] = aug->d[ipivot];
		aug->d[ipivot] = temp;
		}

		/* put augmented matrix in echelon form */

		for (k = i + 1; k < dim; k++)
		{
		q = -aug->d[k][i] / aug->d[i][i];
		aug->d[k][i] = 0.0;
		for (j = i + 1; j < dim+1; j++)
		{
			aug->d[k][j] = q * aug->d[i][j] + aug->d[k][j];
		}
		}
	}

	if (NEAR_ZERO (aug->d[dim-1][dim-1]))   /* singularity check */
	{
		MatrixFree(aug);
		return(MATERR_SINGULAR);
		}

		/* backsolve to obtain values for inverse matrix */

		res->d[dim-1][h] = aug->d[dim-1][dim] / aug->d[dim-1][dim-1];

		for (k = 1; k < dim; k++)
		{
			q = 0.0;
			for (j = 1; j <= k; j++)
			{
			q += aug->d[dim - 1 - k][dim - j] * res->d[dim - j][h];
			}
			res->d[dim-1 - k][h] = 
		(aug->d[dim - 1 - k][dim] - q) / 
		aug->d[dim - 1 - k][dim - 1 - k];
		}
	}

	MatrixFree(aug);
	return (OK);
}
#endif

int MatrixMult(const struct Matrix *A, const struct Matrix *B, struct Matrix *C)
{
	int i, j, k;
	elem_t val;

	if (A == NULL || B == NULL || C == NULL)
		return(MATERR_NOMATRIX);
	if (A->m != B->n || A->n != C->n || B->m != C->m)
		return(MATERR_SIZE);

	for (i=0; i < C->n; i++)
		for (j=0; j < C->m; j++)
		{
			val = 0.0;
			for (k=0; k < A->m; k++)
				val += A->d[i][k] * B->d[k][j];
			C->d[i][j] = val;
		}
	return(OK);
}

void MatrixPrint(const struct Matrix *m, FILE *file)
{
	int i, j;

	if (m != NULL && file != NULL)
	{
		for (i=0; i < m->n; i++)
		{
			for (j=0; j < m->m; j++)
				fprintf(file, "%15g\t", (float)m->d[i][j]);
			fprintf(file, "\n");
		}
	}
}

static char *err_strings[] =
{
	"No error!",
	"Out of memory!",
	"No matrix!",
	"No square matrix!",
	"Wrong size!",
	"Matrix singular!"
};

char *MatError(int errnum)
{

	return(err_strings[errnum]);
}
