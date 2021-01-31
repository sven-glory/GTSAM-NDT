//                             - TOOLS.H -
//
//   The interface of some commonly used mathematical functions.
//
//   Applicable environment:
//     - VRTXsa86 only
//
//   Author: Zhang Lei
//   Date: 2000. 10. 26
//

#ifndef __Tools
#define __Tools

#include "ZTypes.h"

#define FLOAT_GATE      1e-5

//////////////////////////////////////////////////////////////////////////////

#if _MSC_VER >= 1000      // Version is above Visual C++ 4.0
	// Template function is supported by the compiler

	// Limit "x" within "[-LimitVal, LimitVal]"
	template <class Type> Type Limit(Type x, Type LimitVal)
	{
		if (x > LimitVal)
			return LimitVal;

		else if (x > -LimitVal)
			return x;

		else
			return -LimitVal;
	}

	// Limit "x" within "[LimitLowVal, LimitUpVal]"
	template <class Type> Type LimitBetween(Type x, Type LimitLowVal, Type LimitUpVal)
	{
		if (x > LimitUpVal)
			x = LimitUpVal;

		if (x < LimitLowVal)
			x = LimitLowVal;

		return x;
	}

	// Caculate the square of a value
	template <class Type> Type Square(Type x)
	{
		return (Type)(x * x);
	}

	//
	// Let "from" approaches "to" at the increament of "step".
	//
	// Note:
	//    1. "from" and "to" can be any value, but "fStep" should be positive.
	//    2. If "from" reaches "to", TRUE is returned.
	//
	template <class Type>
	BOOL Approach(Type& from, Type to, Type step)
	{
		if (from + step < to)
		{
			from += step;
			return FALSE;
		}

		else if (from - step > to)
		{
			from -= step;
			return FALSE;
		}

		else
		{
			from = to;
			return TRUE;
		}
	}

DllExport SHORT HexStrToInt(CString& str);
DllExport SHORT DecStrToInt(CString& str);
DllExport CString FloatToStr(float f);

#else      // Version is below Visual C++ 4.0
	// Template function not supported by the compiler

	#define Limit(x, LimitVal)    (x)//(((x)>(LimitVal)) ? (LimitVal) :	(((x)>(-(Limit))) ? (x):(-(LimitVal))))
	#define Square(x)             ((x)*(x))

	BOOL Approach(float& from, float to, float step);
	BOOL Approach(int& from, int to, int step);

	#if _MFC_VER
		CArchive& operator >> (CArchive& ar, char& Obj);
		CArchive& operator << (CArchive& ar, char& Obj);
		CArchive& operator >> (CArchive& ar, SHORT& Obj);
		CArchive& operator << (CArchive& ar, SHORT& Obj);
	#endif      // _MFC_VER

#endif      // _MSC_VER >= 1000

// Estimate covering area
DllExport float EstimateArea(float fStartValue, float fEndValue, float fSlope);

// Estimate end value
DllExport float EstimateEndValue(float fStartValue, float fArea, float fSlope);

// Check if 2 floats are approximately equal
DllExport BOOL ApprEqual(float x, float y, float fGate = FLOAT_GATE);

// Swab the high/low bytes of the specified word
DllExport void SwabWord(USHORT &uWord);

// Reverse the order of bits in a word data
DllExport void ReverseBitsOrder(USHORT& uWord);

// Search for "1" in the mask byte
DllExport SHORT FindBit(USHORT Mask, SHORT nFromBit = 0);

// Generate mask byte for the specified bit
DllExport USHORT FindMaskWord(SHORT nBit);

#if _MSC_VER < 12
// Get the nearest integer
DllExport int round(double d);
#endif

DllExport void CharToHexStr(UCHAR chAscii, UCHAR *pchHex);
DllExport BOOL HexStrToChar(UCHAR *pchHex, UCHAR& chAscii);
DllExport void CharToOctStr(UCHAR chAscii, UCHAR *pchOct);

#define BIT(x)                 (((USHORT) 1) << (x))
#define TestBit(x, i)          ((x & BIT(i)) != 0)
#define sign(x)                ((x>=0) ? 1 : -1)     // The sign of variable x

// Defines PI
#if !defined PI
#define PI                     ((float)3.14159265)
#endif

#define TO_DEGREE(x)           (x/PI*180.0f)
#define TO_RADIAN(x)           (x/180.0f*PI)

DllExport BOOL IsFloat(char* str, float* pFloat = NULL);
DllExport void UpdateActiveTime();
DllExport BOOL CheckInactive(int nLimitSec);
DllExport UCHAR GetCMosVar(USHORT uIndex);
DllExport void SetCMosVar(USHORT uIndex, UCHAR uchVar);

#ifdef _WIN32_WCE
DllExport time_t time(time_t *timer);
double difftime(time_t timer1, time_t timer0);

DllExport CArchive& operator >> (CArchive& ar, char* lzStr);
DllExport CArchive& operator << (CArchive& ar, char* lzStr);

#endif

#endif
