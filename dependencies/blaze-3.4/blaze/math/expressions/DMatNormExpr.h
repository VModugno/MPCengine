//=================================================================================================
/*!
//  \file blaze/math/expressions/DMatNormExpr.h
//  \brief Header file for the dense matrix norm expression
//
//  Copyright (C) 2012-2018 Klaus Iglberger - All Rights Reserved
//
//  This file is part of the Blaze library. You can redistribute it and/or modify it under
//  the terms of the New (Revised) BSD License. Redistribution and use in source and binary
//  forms, with or without modification, are permitted provided that the following conditions
//  are met:
//
//  1. Redistributions of source code must retain the above copyright notice, this list of
//     conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright notice, this list
//     of conditions and the following disclaimer in the documentation and/or other materials
//     provided with the distribution.
//  3. Neither the names of the Blaze development group nor the names of its contributors
//     may be used to endorse or promote products derived from this software without specific
//     prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
//  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
//  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
//  DAMAGE.
*/
//=================================================================================================

#ifndef _BLAZE_MATH_EXPRESSIONS_DMATNORMEXPR_H_
#define _BLAZE_MATH_EXPRESSIONS_DMATNORMEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <utility>
#include <blaze/math/Aliases.h>
#include <blaze/math/expressions/DenseMatrix.h>
#include <blaze/math/functors/Abs.h>
#include <blaze/math/functors/Cbrt.h>
#include <blaze/math/functors/L1Norm.h>
#include <blaze/math/functors/L2Norm.h>
#include <blaze/math/functors/L3Norm.h>
#include <blaze/math/functors/L4Norm.h>
#include <blaze/math/functors/LpNorm.h>
#include <blaze/math/functors/Noop.h>
#include <blaze/math/functors/Pow2.h>
#include <blaze/math/functors/Pow3.h>
#include <blaze/math/functors/Pow4.h>
#include <blaze/math/functors/Qdrt.h>
#include <blaze/math/functors/Sqrt.h>
#include <blaze/math/functors/UnaryPow.h>
#include <blaze/math/shims/Evaluate.h>
#include <blaze/math/shims/Invert.h>
#include <blaze/math/shims/IsZero.h>
#include <blaze/math/SIMD.h>
#include <blaze/math/traits/MultTrait.h>
#include <blaze/math/typetraits/HasSIMDAdd.h>
#include <blaze/math/typetraits/IsPadded.h>
#include <blaze/math/typetraits/UnderlyingBuiltin.h>
#include <blaze/system/Optimizations.h>
#include <blaze/util/Assert.h>
#include <blaze/util/FalseType.h>
#include <blaze/util/FunctionTrace.h>
#include <blaze/util/mpl/And.h>
#include <blaze/util/mpl/Bool.h>
#include <blaze/util/mpl/If.h>
#include <blaze/util/StaticAssert.h>
#include <blaze/util/Template.h>
#include <blaze/util/TrueType.h>
#include <blaze/util/TypeList.h>
#include <blaze/util/Types.h>
#include <blaze/util/typetraits/HasMember.h>
#include <blaze/util/typetraits/RemoveReference.h>


namespace blaze {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Auxiliary helper struct for the dense matrix norms.
// \ingroup dense_matrix
*/
template< typename MT       // Type of the dense matrix
        , typename Abs      // Type of the abs operation
        , typename Power >  // Type of the power operation
struct DMatNormHelper
{
   //**Type definitions****************************************************************************
   //! Composite type of the dense matrix expression.
   using CT = RemoveReference_t< CompositeType_t<MT> >;
   //**********************************************************************************************

   //**SIMD support detection**********************************************************************
   //! Definition of the HasSIMDEnabled type trait.
   BLAZE_CREATE_HAS_DATA_OR_FUNCTION_MEMBER_TYPE_TRAIT( HasSIMDEnabled, simdEnabled );

   //! Definition of the HasLoad type trait.
   BLAZE_CREATE_HAS_DATA_OR_FUNCTION_MEMBER_TYPE_TRAIT( HasLoad, load );

   //! Helper structure for the detection of the SIMD capabilities of the given custom operation.
   struct UseSIMDEnabledFlag {
      static constexpr bool value = Power::BLAZE_TEMPLATE simdEnabled< ElementType_t<MT> >();
   };
   //**********************************************************************************************

   //**********************************************************************************************
   static constexpr bool value =
      ( useOptimizedKernels &&
        CT::simdEnabled &&
        If_t< HasSIMDEnabled_v<Abs> && HasSIMDEnabled_v<Power>
            , UseSIMDEnabledFlag
            , And< HasLoad<Abs>, HasLoad<Power> > >::value &&
        HasSIMDAdd_v< ElementType_t<CT>, ElementType_t<CT> > );
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Default backend implementation of the norm of a row-major dense matrix.
// \ingroup dense_matrix
//
// \param dm The given row-major dense matrix for the norm computation.
// \param abs The functor for the abs operation.
// \param power The functor for the power operation.
// \param root The functor for the root operation.
// \return The norm of the given matrix.
//
// This function implements the performance optimized norm of a row-major dense matrix. Due to
// the explicit application of the SFINAE principle, this function can only be selected by the
// compiler in case vectorization cannot be applied.
*/
template< typename MT      // Type of the dense matrix
        , typename Abs     // Type of the abs opertaion
        , typename Power   // Type of the power operation
        , typename Root >  // Type of the root operation
inline decltype(auto) norm_backend( const DenseMatrix<MT,false>& dm, Abs abs, Power power, Root root, FalseType )
{
   using CT = CompositeType_t<MT>;
   using ET = ElementType_t<MT>;
   using RT = decltype( evaluate( root( std::declval<ET>() ) ) );

   if( (~dm).rows() == 0UL || (~dm).columns() == 0UL ) return RT();

   CT tmp( ~dm );

   const size_t M( tmp.rows()    );
   const size_t N( tmp.columns() );

   ET norm( power( abs( tmp(0UL,0UL) ) ) );

   {
      size_t j( 1UL );

      for( ; (j+4UL) <= N; j+=4UL ) {
         norm += power( abs( tmp(0UL,j    ) ) ) + power( abs( tmp(0UL,j+1UL) ) ) +
                 power( abs( tmp(0UL,j+2UL) ) ) + power( abs( tmp(0UL,j+3UL) ) );
      }
      for( ; (j+2UL) <= N; j+=2UL ) {
         norm += power( abs( tmp(0UL,j) ) ) + power( abs( tmp(0UL,j+1UL) ) );
      }
      for( ; j<N; ++j ) {
         norm += power( abs( tmp(0UL,j) ) );
      }
   }

   for( size_t i=1UL; i<M; ++i )
   {
      size_t j( 0UL );

      for( ; (j+4UL) <= N; j+=4UL ) {
         norm += power( abs( tmp(i,j    ) ) ) + power( abs( tmp(i,j+1UL) ) ) +
                 power( abs( tmp(i,j+2UL) ) ) + power( abs( tmp(i,j+3UL) ) );
      }
      for( ; (j+2UL) <= N; j+=2UL ) {
         norm += power( abs( tmp(i,j) ) ) + power( abs( tmp(i,j+1UL) ) );
      }
      for( ; j<N; ++j ) {
         norm += power( abs( tmp(i,j) ) );
      }
   }

   return evaluate( root( norm ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Default backend implementation of the norm of a column-major dense matrix.
// \ingroup dense_matrix
//
// \param dm The given column-major dense matrix for the norm computation.
// \param abs The functor for the abs operation.
// \param power The functor for the power operation.
// \param root The functor for the root operation.
// \return The norm of the given matrix.
//
// This function implements the performance optimized norm of a column-major dense matrix. Due
// to the explicit application of the SFINAE principle, this function can only be selected by
// the compiler in case vectorization cannot be applied.
*/
template< typename MT      // Type of the dense matrix
        , typename Abs     // Type of the abs operation
        , typename Power   // Type of the power operation
        , typename Root >  // Type of the root operation
inline decltype(auto) norm_backend( const DenseMatrix<MT,true>& dm, Abs abs, Power power, Root root, FalseType )
{
   using CT = CompositeType_t<MT>;
   using ET = ElementType_t<MT>;
   using RT = decltype( evaluate( root( std::declval<ET>() ) ) );

   if( (~dm).rows() == 0UL || (~dm).columns() == 0UL ) return RT();

   CT tmp( ~dm );

   const size_t M( tmp.rows()    );
   const size_t N( tmp.columns() );

   ET norm( power( abs( tmp(0UL,0UL) ) ) );

   {
      size_t i( 1UL );

      for( ; (i+4UL) <= M; i+=4UL ) {
         norm += power( abs( tmp(i    ,0UL) ) ) + power( abs( tmp(i+1UL,0UL) ) ) +
                 power( abs( tmp(i+2UL,0UL) ) ) + power( abs( tmp(i+3UL,0UL) ) );
      }
      for( ; (i+2UL) <= M; i+=2UL ) {
         norm += power( abs( tmp(i,0UL) ) ) + power( abs( tmp(i+1UL,0UL) ) );
      }
      for( ; i<M; ++i ) {
         norm += power( abs( tmp(i,0UL) ) );
      }
   }

   for( size_t j=1UL; j<N; ++j )
   {
      size_t i( 0UL );

      for( ; (i+4UL) <= M; i+=4UL ) {
         norm += power( abs( tmp(i    ,j) ) ) + power( abs( tmp(i+1UL,j) ) ) +
                 power( abs( tmp(i+2UL,j) ) ) + power( abs( tmp(i+3UL,j) ) );
      }
      for( ; (i+2UL) <= M; i+=2UL ) {
         norm += power( abs( tmp(i,j) ) ) + power( abs( tmp(i+1UL,j) ) );
      }
      for( ; i<M; ++i ) {
         norm += power( abs( tmp(i,j) ) );
      }
   }

   return evaluate( root( norm ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief SIMD optimized backend implementation of the norm of a row-major dense matrix.
// \ingroup dense_matrix
//
// \param dm The given row-major dense matrix for the norm computation.
// \param abs The functor for the abs operation.
// \param power The functor for the power operation.
// \param root The functor for the root operation.
// \return The norm of the given matrix.
//
// This function implements the performance optimized norm of a row-major dense matrix. Due to
// the explicit application of the SFINAE principle, this function can only be selected by the
// compiler in case vectorization can be applied.
*/
template< typename MT      // Type of the dense matrix
        , typename Abs     // Type of the abs operation
        , typename Power   // Type of the power operation
        , typename Root >  // Type of the root operation
inline decltype(auto) norm_backend( const DenseMatrix<MT,false>& dm, Abs abs, Power power, Root root, TrueType )
{
   using CT = CompositeType_t<MT>;
   using ET = ElementType_t<MT>;
   using RT = decltype( evaluate( root( std::declval<ET>() ) ) );

   static constexpr size_t SIMDSIZE = SIMDTrait<ET>::size;

   if( (~dm).rows() == 0UL || (~dm).columns() == 0UL ) return RT();

   CT tmp( ~dm );

   const size_t M( tmp.rows()    );
   const size_t N( tmp.columns() );

   constexpr bool remainder( !usePadding || !IsPadded_v< RemoveReference_t<CT> > );

   const size_t jpos( ( remainder )?( N & size_t(-SIMDSIZE) ):( N ) );
   BLAZE_INTERNAL_ASSERT( !remainder || ( N - ( N % SIMDSIZE ) ) == jpos, "Invalid end calculation" );

   SIMDTrait_t<ET> xmm1, xmm2, xmm3, xmm4;
   ET norm{};

   for( size_t i=0UL; i<M; ++i )
   {
      size_t j( 0UL );

      for( ; (j+SIMDSIZE*3UL) < jpos; j+=SIMDSIZE*4UL ) {
         xmm1 += power( abs( tmp.load(i,j             ) ) );
         xmm2 += power( abs( tmp.load(i,j+SIMDSIZE    ) ) );
         xmm3 += power( abs( tmp.load(i,j+SIMDSIZE*2UL) ) );
         xmm4 += power( abs( tmp.load(i,j+SIMDSIZE*3UL) ) );
      }
      for( ; (j+SIMDSIZE) < jpos; j+=SIMDSIZE*2UL ) {
         xmm1 += power( abs( tmp.load(i,j         ) ) );
         xmm2 += power( abs( tmp.load(i,j+SIMDSIZE) ) );
      }
      for( ; j<jpos; j+=SIMDSIZE ) {
         xmm1 += power( abs( tmp.load(i,j) ) );
      }
      for( ; remainder && j<N; ++j ) {
         norm += power( abs( tmp(i,j) ) );
      }
   }

   norm += sum( xmm1 + xmm2 + xmm3 + xmm4 );

   return evaluate( root( norm ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief SIMD optimized backend implementation of the norm of a column-major dense matrix.
// \ingroup dense_matrix
//
// \param dm The given column-major dense matrix for the norm computation.
// \param abs The functor for the abs operation.
// \param power The functor for the power operation.
// \param root The functor for the root operation.
// \return The norm of the given matrix.
//
// This function implements the performance optimized norm of a column-major dense matrix. Due
// to the explicit application of the SFINAE principle, this function can only be selected by
// the compiler in case vectorization can be applied.
*/
template< typename MT      // Type of the dense matrix
        , typename Abs     // Type of the abs operation
        , typename Power   // Type of the power operation
        , typename Root >  // Type of the root operation
inline decltype(auto) norm_backend( const DenseMatrix<MT,true>& dm, Abs abs, Power power, Root root, TrueType )
{
   using CT = CompositeType_t<MT>;
   using ET = ElementType_t<MT>;
   using RT = decltype( evaluate( root( std::declval<ET>() ) ) );

   static constexpr size_t SIMDSIZE = SIMDTrait<ET>::size;

   if( (~dm).rows() == 0UL || (~dm).columns() == 0UL ) return RT();

   CT tmp( ~dm );

   const size_t M( tmp.rows()    );
   const size_t N( tmp.columns() );

   constexpr bool remainder( !usePadding || !IsPadded_v< RemoveReference_t<CT> > );

   const size_t ipos( ( remainder )?( M & size_t(-SIMDSIZE) ):( M ) );
   BLAZE_INTERNAL_ASSERT( !remainder || ( M - ( M % SIMDSIZE ) ) == ipos, "Invalid end calculation" );

   SIMDTrait_t<ET> xmm1, xmm2, xmm3, xmm4;
   ET norm{};

   for( size_t j=0UL; j<N; ++j )
   {
      size_t i( 0UL );

      for( ; (i+SIMDSIZE*3UL) < ipos; i+=SIMDSIZE*4UL ) {
         xmm1 += power( abs( tmp.load(i             ,j) ) );
         xmm2 += power( abs( tmp.load(i+SIMDSIZE    ,j) ) );
         xmm3 += power( abs( tmp.load(i+SIMDSIZE*2UL,j) ) );
         xmm4 += power( abs( tmp.load(i+SIMDSIZE*3UL,j) ) );
      }
      for( ; (i+SIMDSIZE) < ipos; i+=SIMDSIZE*2UL ) {
         xmm1 += power( abs( tmp.load(i         ,j) ) );
         xmm2 += power( abs( tmp.load(i+SIMDSIZE,j) ) );
      }
      for( ; i<ipos; i+=SIMDSIZE ) {
         xmm1 += power( abs( tmp.load(i,j) ) );
      }
      for( ; remainder && i<M; ++i ) {
         norm += power( abs( tmp(i,j) ) );
      }
   }

   norm += sum( xmm1 + xmm2 + xmm3 + xmm4 );

   return evaluate( root( norm ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Computes a custom norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \param abs The functor for the abs operation.
// \param power The functor for the power operation.
// \param root The functor for the root operation.
// \return The norm of the given dense matrix.
//
// This function computes a custom norm of the given dense matrix by means of the given functors.
// The following example demonstrates the computation of the L2 norm by means of the blaze::Pow2
// and blaze::Sqrt functors:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l2 = norm( A, blaze::Pow2(), blaze::Sqrt() );
   \endcode
*/
template< typename MT      // Type of the dense matrix
        , bool SO          // Storage order
        , typename Abs     // Type of the abs operation
        , typename Power   // Type of the power operation
        , typename Root >  // Type of the root operation
decltype(auto) norm_backend( const DenseMatrix<MT,SO>& dm, Abs abs, Power power, Root root )
{
   return norm_backend( ~dm, abs, power, root, Bool< DMatNormHelper<MT,Abs,Power>::value >() );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the L2 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The L2 norm of the given dense matrix.
//
// This function computes the L2 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l2 = norm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) norm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Noop(), Pow2(), Sqrt() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the squared L2 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The squared L2 norm of the given dense matrix.
//
// This function computes the squared L2 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l2 = sqrNorm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) sqrNorm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Noop(), Pow2(), Noop() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the L1 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The L1 norm of the given dense matrix.
//
// This function computes the L1 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l1 = l1Norm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) l1Norm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Abs(), Noop(), Noop() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the L2 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The L2 norm of the given dense matrix.
//
// This function computes the L2 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l2 = l2Norm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) l2Norm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Noop(), Pow2(), Sqrt() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the L3 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The L3 norm of the given dense matrix.
//
// This function computes the L3 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l3 = l3Norm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) l3Norm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Abs(), Pow3(), Cbrt() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the L4 norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The L4 norm of the given dense matrix.
//
// This function computes the L4 norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double l4 = l4Norm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) l4Norm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return norm_backend( ~dm, Noop(), Pow4(), Qdrt() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the Lp norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \param p The norm parameter (p > 0).
// \return The Lp norm of the given dense matrix.
//
// This function computes the Lp norm of the given dense matrix, where the norm is specified by
// the runtime argument \a p:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double lp = lpNorm( A, 2.3 );
   \endcode

// \note The norm parameter \a p is expected to be larger than 0. This precondition is only checked
// by a user assertion.
*/
template< typename MT    // Type of the dense matrix
        , bool SO        // Storage order
        , typename ST >  // Type of the norm parameter
decltype(auto) lpNorm( const DenseMatrix<MT,SO>& dm, ST p )
{
   BLAZE_FUNCTION_TRACE;

   BLAZE_USER_ASSERT( !isZero( p ), "Invalid p for Lp norm detected" );

   using ScalarType = MultTrait_t< UnderlyingBuiltin_t<MT>, decltype( inv( p ) ) >;
   return norm_backend( ~dm, Abs(), UnaryPow<ScalarType>( p ), UnaryPow<ScalarType>( inv( p ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the Lp norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The Lp norm of the given dense matrix.
//
// This function computes the Lp norm of the given dense matrix, where the norm is specified by
// the runtime argument \a P:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double lp = lpNorm<2>( A );
   \endcode

// \note The norm parameter \a P is expected to be larger than 0. A value of 0 results in a
// compile time error!.
*/
template< size_t P     // Compile time norm parameter
        , typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
inline decltype(auto) lpNorm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_STATIC_ASSERT_MSG( P > 0UL, "Invalid norm parameter detected" );

   using Norms = TypeList< L1Norm, L2Norm, L3Norm, L4Norm, LpNorm<P> >;
   using Norm  = typename TypeAt< Norms, min( P-1UL, 4UL ) >::Type;

   return Norm()( ~dm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the maximum norm for the given dense matrix.
// \ingroup dense_matrix
//
// \param dm The given dense matrix for the norm computation.
// \return The maximum norm of the given dense matrix.
//
// This function computes the maximum norm of the given dense matrix:

   \code
   blaze::DynamicMatrix<double> A;
   // ... Resizing and initialization
   const double max = maxNorm( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
decltype(auto) maxNorm( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   return max( abs( ~dm ) );
}
//*************************************************************************************************

} // namespace blaze

#endif
