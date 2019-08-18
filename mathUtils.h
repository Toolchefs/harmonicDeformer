#pragma once

#include <limits>
#include <cmath>
#include <iostream>

namespace tc
{
	class Vector
	{
	public:

		//-------------------
		// Access to elements
		//-------------------

		double			x, y, z;

		double &			operator [] (int i);
		const double &		operator [] (int i) const;


		//-------------
		// Constructors
		//-------------

		Vector();			   // no initialization
		explicit Vector(double a);           // (a a a)
		Vector(double a, double b, double c);	   // (a b c)


		//---------------------------------
		// Copy constructors and assignment
		//---------------------------------

		Vector(const Vector &v);

		const Vector &	operator = (const Vector &v);



		//---------
		// Equality
		//---------

		bool		operator == (const Vector &v) const;

		bool		operator != (const Vector &v) const;

		//-----------------------------------------------------------------------
		// Compare two vectors and test if they are "approximately equal":
		//
		// equalWithAbsError (v, e)
		//
		//	    Returns true if the coefficients of this and v are the same with
		//	    an absolute error of no more than e, i.e., for all i
		//
		//      abs (this[i] - v[i]) <= e
		//
		// equalWithRelError (v, e)
		//
		//	    Returns true if the coefficients of this and v are the same with
		//	    a relative error of no more than e, i.e., for all i
		//
		//      abs (this[i] - v[i]) <= e * abs (this[i])
		//-----------------------------------------------------------------------

		bool		equalWithAbsError(const Vector &v, double e) const;

		//------------
		// Dot product
		//------------

		double			dot(const Vector &v) const;
		double			operator ^ (const Vector &v) const;


		//---------------------------
		// Right-handed cross product
		//---------------------------

		Vector		cross(const Vector &v) const;
		const Vector &	operator %= (const Vector &v);
		Vector		operator % (const Vector &v) const;


		//------------------------
		// Component-wise addition
		//------------------------

		const Vector &	operator += (const Vector &v);
		Vector		operator + (const Vector &v) const;


		//---------------------------
		// Component-wise subtraction
		//---------------------------

		const Vector &	operator -= (const Vector &v);
		Vector		operator - (const Vector &v) const;


		//------------------------------------
		// Component-wise multiplication by -1
		//------------------------------------

		Vector		operator - () const;
		const Vector &	negate();


		//------------------------------
		// Component-wise multiplication
		//------------------------------

		const Vector &	operator *= (const Vector &v);
		const Vector &	operator *= (double a);
		Vector		operator * (const Vector &v) const;
		Vector		operator * (double a) const;


		//------------------------
		// Component-wise division
		//------------------------

		const Vector &	operator /= (const Vector &v);
		const Vector &	operator /= (double a);
		Vector		operator / (const Vector &v) const;
		Vector		operator / (double a) const;


		//----------------------------------------------------------------
		// Length and normalization:  If v.length() is 0.0, v.normalize()
		// and v.normalized() produce a null vector; v.normalizeExc() and
		// v.normalizedExc() throw a NullVecExc.
		// v.normalizeNonNull() and v.normalizedNonNull() are slightly
		// faster than the other normalization routines, but if v.length()
		// is 0.0, the result is undefined.
		//----------------------------------------------------------------

		double			length() const;
		double			length2() const;

		const Vector &	normalize();           // modifies *this
		const Vector &	normalizeNonNull();

		Vector		normalized() const;	// does not modify *this
		Vector		normalizedNonNull() const;


		//--------------------------------------------------------
		// Number of dimensions, i.e. number of elements in a Vector
		//--------------------------------------------------------

		static unsigned int	dimensions() { return 3; }


		//-------------------------------------------------
		// Limitations of type double (see also class limits<double>)
		//-------------------------------------------------
		/*
		static double		baseTypeMin()		{ return std::numeric_limits<double>::min(); }
		static double		baseTypeMax()		{ return std::numeric_limits<double>::max(); }
		static double		baseTypeSmallest()	{ return std::numeric_limits<double>::min(); }
		static double		baseTypeEpsilon()	{ return std::numeric_limits<double>::epsilon(); }

		*/
		//--------------------------------------------------------------
		// Base type -- in templates, which accept a parameter, V, which
		// could be either a Vec2<double>, a Vector<double>, or a Vec4<double> you can 
		// refer to double as V::BaseType
		//--------------------------------------------------------------

		typedef double		BaseType;

	private:

		double			lengthTiny() const;
	};


	std::ostream &	operator << (std::ostream &s, const Vector &v);
}