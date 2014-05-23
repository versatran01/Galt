//-----------------------------------------------------------------------------
#ifndef file_ptrH
#define file_ptrH file_ptrH
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <common/System/Platform.h>

#ifdef _MSC_VER // is Microsoft compiler?
#   pragma warning( push )
#   if _MSC_VER > 1300 // is VC 2005 compiler?
#       pragma warning( disable : 4996 ) // '<function> was declared deprecated'
#   endif // #if _MSC_VER > 1300
#endif // #ifdef _MSC_VER

//-----------------------------------------------------------------------------
/// \brief replaces the C-type FILE*
///
/// Automatically closes the file when leaving the scope of the object (even if
/// an exception is thrown). Behaves (almost) exactly like a 'normal' FILE-pointer otherwise.
///
/// Limitations:
///
/// Comparisons must be done against '0'
/// \verbatim
/// CFilePtr fp( "a.out", "w" );
/// if( fp == 0 )
/// {
///    // Some error occurred
///    return errorCode;
/// }
/// \endverbatim
/// might not work if certain stdio-functions are defined via macros
class CFilePtr
//-----------------------------------------------------------------------------
{
    mutable FILE* m_p;
    CFilePtr( const CFilePtr& );        // do not allow copy constructor, if needed this must be implemented correctly
    CFilePtr& operator=( const CFilePtr& ); // do not allow assignments, if needed this must be implemented correctly
public:
    CFilePtr( const char* pName, const char* pMode ) : m_p( fopen( pName, pMode ) ) {}
    CFilePtr( FILE* p ) : m_p( p ) {}
    virtual ~CFilePtr()
    {
        if( m_p )
        {
            fclose( m_p );
        }
    }
    operator FILE* ()
    {
        return m_p;
    }
    bool operator==( const mv_IntPtr i ) const // allows to compare with 0
    {
        return m_p == ( FILE* )i;
    }
    /// \brief Returns the size of the file in bytes
    ///
    /// \return The size of the file in bytes or -1 if no file is associated with the current object
    mv_int32 Size( void ) const
    {
        if( !m_p )
        {
            return -1;
        }
        const mv_int32 cur = ftell( m_p );
        fseek( m_p, 0, SEEK_END );
        const mv_int32 size = ftell( m_p );
        fseek( m_p, cur, SEEK_SET );
        return size;
    }
};

//-----------------------------------------------------------------------------
inline bool fileExists( const char* pFileName )
//-----------------------------------------------------------------------------
{
    CFilePtr fp( pFileName, "r" );
    return ( fp.operator FILE * () != 0 );
}

#ifdef _MSC_VER // is Microsoft compiler?
// restore old warning level
#   pragma warning( pop )
#endif // #ifdef _MSC_VER

#endif // file_ptrH
