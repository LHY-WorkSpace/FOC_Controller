/******************************************************************************
* ÎÄµµ: transform_3D.h
* ×÷Õß: Wu LianWei
******************************************************************************/
#ifndef _TRANSFORM_3D_H_
#define _TRANSFORM_3D_H_



typedef struct 
{
    int x; int y;
}_2D;
typedef struct 
{
    float x; float y; float z;
}_3D;
//---------------------------------------
_3D VEC_MultMatrix( _3D Source, float mat[4][4] );
//---------------------------------------
void Identity_3D( float mat[4][4] );
void Translate_3D( float mat[4][4], int tx, int ty, int tz );
void Scale_3D( float mat[4][4], float sx, float sy, float sz );
void Rotate_3D( float mat[4][4], float ax, float ay, float az );



#endif


