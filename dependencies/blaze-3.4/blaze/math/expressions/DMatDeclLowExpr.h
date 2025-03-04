//=================================================================================================
/*!
//  \file blaze/math/expressions/DMatDeclLowExpr.h
//  \brief Header file for the dense matrix lower declaration expression
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

#ifndef _BLAZE_MATH_EXPRESSIONS_DMATDECLLOWEXPR_H_
#define _BLAZE_MATH_EXPRESSIONS_DMATDECLLOWEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iterator>
#include <blaze/math/adaptors/lowermatrix/BaseTemplate.h>
#include <blaze/math/Aliases.h>
#include <blaze/math/constraints/DenseMatrix.h>
#include <blaze/math/constraints/Lower.h>
#include <blaze/math/constraints/MatMatMultExpr.h>
#include <blaze/math/constraints/StorageOrder.h>
#include <blaze/math/constraints/UniUpper.h>
#include <blaze/math/Exception.h>
#include <blaze/math/expressions/Declaration.h>
#include <blaze/math/expressions/DeclLowExpr.h>
#include <blaze/math/expressions/DenseMatrix.h>
#include <blaze/math/expressions/Forward.h>
#include <blaze/math/simd/SIMDTrait.h>
#include <blaze/math/sparse/Forward.h>
#include <blaze/math/traits/DeclLowTrait.h>
#include <blaze/math/typetraits/HasConstDataAccess.h>
#include <blaze/math/typetraits/IsAligned.h>
#include <blaze/math/typetraits/IsExpression.h>
#include <blaze/math/typetraits/IsHermitian.h>
#include <blaze/math/typetraits/IsLower.h>
#include <blaze/math/typetraits/IsStrictlyLower.h>
#include <blaze/math/typetraits/IsStrictlyUpper.h>
#include <blaze/math/typetraits/IsSymmetric.h>
#include <blaze/math/typetraits/IsUniLower.h>
#include <blaze/math/typetraits/IsUniUpper.h>
#include <blaze/math/typetraits/IsUpper.h>
#include <blaze/math/typetraits/RequiresEvaluation.h>
#include <blaze/util/Assert.h>
#include <blaze/util/DisableIf.h>
#include <blaze/util/EnableIf.h>
#include <blaze/util/FunctionTrace.h>
#include <blaze/util/IntegralConstant.h>
#include <blaze/util/InvalidType.h>
#include <blaze/util/mpl/If.h>
#include <blaze/util/TrueType.h>
#include <blaze/util/Types.h>
#include <blaze/util/typetraits/GetMemberType.h>


namespace blaze {

//=================================================================================================
//
//  CLASS DMATDECLLOWEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for the explicit lower declaration of dense matrices.
// \ingroup dense_matrix_expression
//
// The DMatDeclLowExpr class represents the compile time expression for the explicit lower
// declaration of a dense matrix.
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
class DMatDeclLowExpr
   : public DeclLowExpr< DenseMatrix< DMatDeclLowExpr<MT,SO>, SO > >
   , public Declaration<MT>
{
 private:
   //**Type definitions****************************************************************************
   using RT = ResultType_t<MT>;  //!< Result type of the dense matrix expression.

   //! Definition of the GetConstIterator type trait.
   BLAZE_CREATE_GET_TYPE_MEMBER_TYPE_TRAIT( GetConstIterator, ConstIterator, INVALID_TYPE );
   //**********************************************************************************************

   //**Serial evaluation strategy******************************************************************
   //! Compilation switch for the serial evaluation strategy of the lower declaration expression.
   /*! The \a useAssign compile time constant expression represents a compilation switch for
       the serial evaluation strategy of the lower declaration expression. In case the given
       dense matrix expression of type \a MT requires an intermediate evaluation, \a useAssign
       will be set to 1 and the lower declaration expression will be evaluated via the \a assign
       function family. Otherwise \a useAssign will be set to 0 and the expression will be
       evaluated via the subscript operator. */
   static constexpr bool useAssign = RequiresEvaluation_v<MT>;

   /*! \cond BLAZE_INTERNAL */
   //! Helper variable template for the explicit application of the SFINAE principle.
   template< typename MT2 >
   static constexpr bool UseAssign_v = useAssign;
   /*! \endcond */
   //**********************************************************************************************

   //**Parallel evaluation strategy****************************************************************
   /*! \cond BLAZE_INTERNAL */
   //! Helper variable template for the explicit application of the SFINAE principle.
   /*! This variable template is a helper for the selection of the parallel evaluation strategy.
       In case the target matrix is SMP assignable and the dense matrix operand requires an
       intermediate evaluation, the variable is set to 1 and the expression specific evaluation
       strategy is selected. Otherwise the variable is set to 0 and the default strategy is
       chosen. */
   template< typename MT2 >
   static constexpr bool UseSMPAssign_v = ( MT2::smpAssignable && useAssign );
   /*! \endcond */
   //**********************************************************************************************

 public:
   //**Type definitions****************************************************************************
   using This          = DMatDeclLowExpr<MT,SO>;       //!< Type of this DMatDeclLowExpr instance.
   using ResultType    = DeclLowTrait_t<RT>;           //!< Result type for expression template evaluations.
   using OppositeType  = OppositeType_t<ResultType>;   //!< Result type with opposite storage order for expression template evaluations.
   using TransposeType = TransposeType_t<ResultType>;  //!< Transpose type for expression template evaluations.
   using ElementType   = ElementType_t<MT>;            //!< Resulting element type.
   using ReturnType    = ReturnType_t<MT>;             //!< Return type for expression template evaluations.

   //! Data type for composite expression templates.
   using CompositeType = If_t< RequiresEvaluation_v<MT>, const ResultType, const DMatDeclLowExpr& >;

   //! Iterator over the elements of the dense matrix.
   using ConstIterator = GetConstIterator_t<MT>;

   //! Composite data type of the dense matrix expression.
   using Operand = If_t< IsExpression_v<MT>, const MT, const MT& >;
   //**********************************************************************************************

   //**Compilation flags***************************************************************************
   //! Compilation switch for the expression template evaluation strategy.
   static constexpr bool simdEnabled = MT::simdEnabled;

   //! Compilation switch for the expression template assignment strategy.
   static constexpr bool smpAssignable = MT::smpAssignable;
   //**********************************************************************************************

   //**SIMD properties*****************************************************************************
   //! The number of elements packed within a single SIMD element.
   static constexpr size_t SIMDSIZE = SIMDTrait<ElementType>::size;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatDeclLowExpr class.
   //
   // \param dm The dense matrix operand of the decllow expression.
   */
   explicit inline DMatDeclLowExpr( const MT& dm ) noexcept
      : dm_( dm )  // Dense matrix of the decllow expression
   {
      BLAZE_INTERNAL_ASSERT( isSquare( ~dm ), "Non-square matrix detected" );
   }
   //**********************************************************************************************

   //**Access operator*****************************************************************************
   /*!\brief 2D-access to the matrix elements.
   //
   // \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
   // \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
   // \return The resulting value.
   */
   inline ReturnType operator()( size_t i, size_t j ) const {
      BLAZE_INTERNAL_ASSERT( i < dm_.rows()   , "Invalid row access index"    );
      BLAZE_INTERNAL_ASSERT( j < dm_.columns(), "Invalid column access index" );
      return dm_(i,j);
   }
   //**********************************************************************************************

   //**At function*********************************************************************************
   /*!\brief Checked access to the matrix elements.
   //
   // \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
   // \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
   // \return The resulting value.
   // \exception std::out_of_range Invalid matrix access index.
   */
   inline ReturnType at( size_t i, size_t j ) const {
      if( i >= dm_.rows() ) {
         BLAZE_THROW_OUT_OF_RANGE( "Invalid row access index" );
      }
      if( j >= dm_.columns() ) {
         BLAZE_THROW_OUT_OF_RANGE( "Invalid column access index" );
      }
      return (*this)(i,j);
   }
   //**********************************************************************************************

   //**Load function*******************************************************************************
   /*!\brief Access to the SIMD elements of the matrix.
   //
   // \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
   // \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
   // \return Reference to the accessed values.
   */
   BLAZE_ALWAYS_INLINE auto load( size_t i, size_t j ) const noexcept {
      BLAZE_INTERNAL_ASSERT( i < dm_.columns(), "Invalid row access index"    );
      BLAZE_INTERNAL_ASSERT( j < dm_.rows()   , "Invalid column access index" );
      BLAZE_INTERNAL_ASSERT( !SO || ( i % SIMDSIZE == 0UL ), "Invalid row access index"    );
      BLAZE_INTERNAL_ASSERT( SO  || ( j % SIMDSIZE == 0UL ), "Invalid column access index" );
      return dm_.load(i,j);
   }
   //**********************************************************************************************

   //**Low-level data access***********************************************************************
   /*!\brief Low-level data access to the matrix elements.
   //
   // \return Pointer to the internal element storage.
   */
   inline const ElementType* data() const noexcept {
      return dm_.data();
   }
   //**********************************************************************************************

   //**Begin function******************************************************************************
   /*!\brief Returns an iterator to the first non-zero element of row/column \a i.
   //
   // \param i The row/column index.
   // \return Iterator to the first non-zero element of row/column \a i.
   */
   inline ConstIterator begin( size_t i ) const {
      return ConstIterator( dm_.begin(i) );
   }
   //**********************************************************************************************

   //**End function********************************************************************************
   /*!\brief Returns an iterator just past the last non-zero element of row/column \a i.
   //
   // \param i The row/column index.
   // \return Iterator just past the last non-zero element of row/column \a i.
   */
   inline ConstIterator end( size_t i ) const {
      return ConstIterator( dm_.end(i) );
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const noexcept {
      return dm_.rows();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const noexcept {
      return dm_.columns();
   }
   //**********************************************************************************************

   //**Operand access******************************************************************************
   /*!\brief Returns the dense matrix operand.
   //
   // \return The dense matrix operand.
   */
   inline Operand operand() const noexcept {
      return dm_;
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression can alias with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case the expression can alias, \a false otherwise.
   */
   template< typename T >
   inline bool canAlias( const T* alias ) const noexcept {
      return dm_.canAlias( alias );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression is aliased with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case an alias effect is detected, \a false otherwise.
   */
   template< typename T >
   inline bool isAliased( const T* alias ) const noexcept {
      return dm_.isAliased( alias );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the operands of the expression are properly aligned in memory.
   //
   // \return \a true in case the operands are aligned, \a false if not.
   */
   inline bool isAligned() const noexcept {
      return dm_.isAligned();
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression can be used in SMP assignments.
   //
   // \return \a true in case the expression can be used in SMP assignments, \a false if not.
   */
   inline bool canSMPAssign() const noexcept {
      return dm_.canSMPAssign();
   }
   //**********************************************************************************************

 private:
   //**Member variables****************************************************************************
   Operand dm_;  //!< Dense matrix of the decllow expression.
   //**********************************************************************************************

   //**Assignment to dense matrices****************************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix decllow
   // expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      assign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      assign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse matrices***************************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix decllow
   // expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      assign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      assign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense matrices*******************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix
   // decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      addAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      addAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse matrices******************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix
   // decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      addAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      addAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to dense matrices****************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      subAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      subAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse matrices***************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      subAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      subAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Schur product assignment to dense matrices**************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Schur product assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression for the Schur product.
   // \return void
   //
   // This function implements the performance optimized Schur product assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      schurAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      schurAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Schur product assignment to sparse matrices*************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Schur product assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression for the Schur product.
   // \return void
   //
   // This function implements the performance optimized Schur product assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      schurAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      schurAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to dense matrices*************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Multiplication assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      multAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      multAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse matrices************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief Multiplication assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseAssign_v<MT2> >
      multAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      multAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP assignment to dense matrices************************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized SMP assignment of a dense matrix
   // decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP assignment to sparse matrices***********************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized SMP assignment of a dense matrix
   // decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP addition assignment to dense matrices***************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP addition assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be added.
   // \return void
   //
   // This function implements the performance optimized SMP addition assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpAddAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpAddAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP addition assignment to sparse matrices**************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP addition assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be added.
   // \return void
   //
   // This function implements the performance optimized SMP addition assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpAddAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpAddAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP subtraction assignment to dense matrices************************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP subtraction assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized SMP subtraction assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpSubAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpSubAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP subtraction assignment to sparse matrices***********************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP subtraction assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized SMP subtraction assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpSubAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpSubAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP Schur product assignment to dense matrices**********************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP Schur product assignment of a dense matrix decllow expression to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression for the Schur product.
   // \return void
   //
   // This function implements the performance optimized SMP Schur product assignment of a dense
   // matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpSchurAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpSchurAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP Schur product assignment to sparse matrices*********************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP Schur product assignment of a dense matrix decllow expression to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression for the Schur product.
   // \return void
   //
   // This function implements the performance optimized SMP Schur product assignment of a dense
   // matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpSchurAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpSchurAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP multiplication assignment to dense matrices*********************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP multiplication assignment of a dense matrix decllow expression to a dense
   //        matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side decllow expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized SMP multiplication assignment of a
   // dense matrix decllow expression to a dense matrix.
   */
   template< typename MT2  // Type of the target dense matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpMultAssign( DenseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpMultAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**SMP multiplication assignment to sparse matrices********************************************
   /*! \cond BLAZE_INTERNAL */
   /*!\brief SMP multiplication assignment of a dense matrix decllow expression to a sparse
   //        matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side decllow expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized SMP multiplication assignment of a
   // dense matrix decllow expression to a sparse matrix.
   */
   template< typename MT2  // Type of the target sparse matrix
           , bool SO2 >    // Storage order of the target dense matrix
   friend inline EnableIf_t< UseSMPAssign_v<MT2> >
      smpMultAssign( SparseMatrix<MT2,SO2>& lhs, const DMatDeclLowExpr& rhs )
   {
      BLAZE_FUNCTION_TRACE;

      BLAZE_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      BLAZE_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      smpMultAssign( ~lhs, rhs.dm_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond BLAZE_INTERNAL */
   BLAZE_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
   BLAZE_CONSTRAINT_MUST_BE_MATRIX_WITH_STORAGE_ORDER( MT, SO );
   BLAZE_CONSTRAINT_MUST_NOT_BE_LOWER_MATRIX_TYPE( MT );
   BLAZE_CONSTRAINT_MUST_NOT_BE_UNIUPPER_MATRIX_TYPE( MT );
   BLAZE_CONSTRAINT_MUST_NOT_BE_MATMATMULTEXPR_TYPE( MT );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Declares the given dense matrix expression \a dm as lower.
// \ingroup dense_matrix
//
// \param dm The input matrix.
// \return The redeclared dense matrix.
//
// This function declares the given dense matrix expression \a dm as lower. The function
// returns an expression representing the operation.
*/
template< typename MT  // Type of the dense matrix
        , bool SO      // Storage order
        , typename = DisableIf_t< IsLower_v<MT> || IsUniUpper_v<MT> > >
inline const DMatDeclLowExpr<MT,SO> decllow_backend( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   BLAZE_INTERNAL_ASSERT( isSquare( ~dm ), "Non-square matrix detected" );

   return DMatDeclLowExpr<MT,SO>( ~dm );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Declares the given uniupper dense matrix expression \a dm as lower.
// \ingroup dense_matrix
//
// \param dm The input matrix.
// \return The redeclared dense matrix.
//
// This function declares the given uniupper dense matrix expression \a dm as lower. The
// function returns an identity matrix.
*/
template< typename MT  // Type of the dense matrix
        , bool SO      // Storage order
        , typename = EnableIf_t< !IsLower_v<MT> && IsUniUpper_v<MT> > >
inline const IdentityMatrix<ElementType_t<MT>,SO> decllow_backend( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   BLAZE_INTERNAL_ASSERT( isSquare( ~dm ), "Non-square matrix detected" );

   return IdentityMatrix<ElementType_t<MT>,SO>( (~dm).rows() );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Redeclares the given lower dense matrix expression \a dm as lower.
// \ingroup dense_matrix
//
// \param dm The input matrix.
// \return The redeclared dense matrix.
//
// This function redeclares the given lower dense matrix expression \a dm as lower. The
// function returns a reference to the already lower matrix expression.
*/
template< typename MT  // Type of the dense matrix
        , bool SO      // Storage order
        , typename = EnableIf_t< IsLower_v<MT> > >
inline const MT& decllow_backend( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   BLAZE_INTERNAL_ASSERT( isSquare( ~dm ), "Non-square matrix detected" );

   return ~dm;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declares the given dense matrix expression \a dm as lower.
// \ingroup dense_matrix
//
// \param dm The input matrix.
// \return The redeclared dense matrix.
// \exception std::invalid_argument Invalid lower matrix specification.
//
// The \a decllow function declares the given dense matrix expression \a dm as lower. In case
// the given matrix is not a square matrix, a \a std::invalid_argument exception is thrown.\n
// The following example demonstrates the use of the \a decllow function:

   \code
   blaze::DynamicMatrix<double> A, B;
   // ... Resizing and initialization
   B = decllow( A );
   \endcode
*/
template< typename MT  // Type of the dense matrix
        , bool SO >    // Storage order
inline decltype(auto) decllow( const DenseMatrix<MT,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   if( !isSquare( ~dm ) ) {
      BLAZE_THROW_INVALID_ARGUMENT( "Invalid lower matrix specification" );
   }

   return decllow_backend( ~dm );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL RESTRUCTURING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
/*!\brief Declares the given non-lower dense matrix-scalar multiplication expression as lower.
// \ingroup dense_matrix
//
// \param dm The input dense matrix-scalar multiplication expression.
// \return The redeclared expression.
// \exception std::invalid_argument Invalid lower matrix specification.
//
// This function implements the application of the decllow() operation on a dense matrix-
// scalar multiplication. It restructures the expression \f$ A=decllow(B*s1) \f$ to the
// expression \f$ A=decllow(B)*s1 \f$. In case the given matrix is not a square matrix,
// a \a std::invalid_argument exception is thrown.
*/
template< typename MT  // Type of the left-hand side dense matrix
        , typename ST  // Type of the right-hand side scalar value
        , bool SO      // Storage order
        , typename = DisableIf_t< IsLower_v<MT> > >
inline decltype(auto) decllow( const DMatScalarMultExpr<MT,ST,SO>& dm )
{
   BLAZE_FUNCTION_TRACE;

   if( !isSquare( ~dm ) ) {
      BLAZE_THROW_INVALID_ARGUMENT( "Invalid lower matrix specification" );
   }

   return decllow( dm.leftOperand() ) * dm.rightOperand();
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  HASCONSTDATAACCESS SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct HasConstDataAccess< DMatDeclLowExpr<MT,SO> >
   : public HasConstDataAccess<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISALIGNED SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsAligned< DMatDeclLowExpr<MT,SO> >
   : public IsAligned<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISSYMMETRIC SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsSymmetric< DMatDeclLowExpr<MT,SO> >
   : public IsSymmetric<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISHERMITIAN SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsHermitian< DMatDeclLowExpr<MT,SO> >
   : public IsHermitian<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISLOWER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsLower< DMatDeclLowExpr<MT,SO> >
   : public TrueType
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISUNILOWER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsUniLower< DMatDeclLowExpr<MT,SO> >
   : public IsUniLower<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISSTRICTLYLOWER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsStrictlyLower< DMatDeclLowExpr<MT,SO> >
   : public IsStrictlyLower<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISUPPER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsUpper< DMatDeclLowExpr<MT,SO> >
   : public BoolConstant< IsSymmetric_v<MT> || IsHermitian_v<MT> || IsUpper_v<MT> >
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISUNIUPPER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsUniUpper< DMatDeclLowExpr<MT,SO> >
   : public IsUniUpper<MT>
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  ISSTRICTLYUPPER SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond BLAZE_INTERNAL */
template< typename MT, bool SO >
struct IsStrictlyUpper< DMatDeclLowExpr<MT,SO> >
   : public IsStrictlyUpper<MT>
{};
/*! \endcond */
//*************************************************************************************************

} // namespace blaze

#endif
