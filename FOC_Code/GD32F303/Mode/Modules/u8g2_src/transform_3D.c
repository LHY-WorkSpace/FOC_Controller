/*******************************************************************************
* 文档: transform_3D.c
* 作者: Wu LianWei
* 描述: 1.基于矩阵计算的3D图形变换算法
*       2.纯算法
*******************************************************************************/
#include "u8g2_UserGUI.h"
#include "DataType.h"
#include "gd32f30x.h"
#include "transform_3D.h"




/******************************************************************************/
/* 矩阵计算                                                                   */
/******************************************************************************/

/***************************************
函数: MAT_Copy
功能: 矩阵拷贝
说明: source(源矩阵),dest(目标矩阵)
***************************************/
void MAT_Copy( float source[4][4], float dest[4][4] )
{
    u8 i,j;
    for(i=0; i<4; i++)
        for(j=0; j<4; j++)
            dest[i][j]=source[i][j];
}

/***************************************
函数: MAT_Mult
功能: 矩阵相乘
说明: mat1(矩阵1),mat2(矩阵2),dest(相乘后的新矩阵)
***************************************/
void MAT_Mult( float mat1[4][4], float mat2[4][4], float dest[4][4] )
{
    u8 i,j;
    for(i=0; i<4; i++)
        for(j=0; j<4; j++)
            dest[i][j]=mat1[i][0]*mat2[0][j]+mat1[i][1]*mat2[1][j]+mat1[i][2]*mat2[2][j]+mat1[i][3]*mat2[3][j];
}

/***************************************
函数: VEC_MultMatrix
功能: 矢量与矩阵相乘
输入: Source(源矢量(坐标)),mat(变换矩阵)
输出: 变换后的新矢量(坐标)
说明: 计算过程和矩阵相乘一致
***************************************/
_3D VEC_MultMatrix( _3D Source, float mat[4][4] )
{
    _3D  Dest;
    Dest.x = Source.x*mat[0][0] + Source.y*mat[1][0] + Source.z*mat[2][0] + mat[3][0];
    Dest.y = Source.x*mat[0][1] + Source.y*mat[1][1] + Source.z*mat[2][1] + mat[3][1];
    Dest.z = Source.x*mat[0][2] + Source.y*mat[1][2] + Source.z*mat[2][2] + mat[3][2];
    return Dest;
}



/******************************************************************************/
/* 变换矩阵                                                                   */
/******************************************************************************/

/***************************************
函数: Identity_3D
功能: 单位矩阵
说明: 把一个数组构造成单位矩阵
***************************************/
void Identity_3D( float mat[4][4] )
{
    mat[0][0]=1.0; mat[0][1]=0.0; mat[0][2]=0.0; mat[0][3]=0.0;
    mat[1][0]=0.0; mat[1][1]=1.0; mat[1][2]=0.0; mat[1][3]=0.0;
    mat[2][0]=0.0; mat[2][1]=0.0; mat[2][2]=1.0; mat[2][3]=0.0;
    mat[3][0]=0.0; mat[3][1]=0.0; mat[3][2]=0.0; mat[3][3]=1.0;
}

/***************************************
函数: Translate_3D
功能: 平移变换矩阵
说明: 1.tx,ty,tz(平移参数)
      2.矩阵mat根据tx,ty,tz进行平移变换
***************************************/
void Translate_3D( float mat[4][4], int tx, int ty, int tz )
{
    float  lmat[4][4];
    float  tmat[4][4];
    //---------------------
    tmat[0][0]=1;  tmat[0][1]=0;  tmat[0][2]=0;  tmat[0][3]=0;
    tmat[1][0]=0;  tmat[1][1]=1;  tmat[1][2]=0;  tmat[1][3]=0;
    tmat[2][0]=0;  tmat[2][1]=0;  tmat[2][2]=1;  tmat[2][3]=0;
    tmat[3][0]=tx; tmat[3][1]=ty; tmat[3][2]=tz; tmat[3][3]=1;
    //---------------------
    MAT_Mult(mat,tmat,lmat);	//矩阵乘法
    MAT_Copy(lmat,mat);			//复制矩阵
}

/***************************************
函数: Scale_3D
功能: 比例变换矩阵
说明: 1.相对于原点的比例变换
      2.sx,sy,sz(x轴/y轴/z轴上的缩放量)
      3.矩阵mat根据sx,sy,sz进行比例变换
***************************************/
void Scale_3D( float mat[4][4], float sx, float sy, float sz )
{
    float  lmat[4][4];
    float  smat[4][4];
    //---------------------
    smat[0][0]=sx; smat[0][1]=0;  smat[0][2]=0;  smat[0][3]=0;
    smat[1][0]=0;  smat[1][1]=sy; smat[1][2]=0;  smat[1][3]=0;
    smat[2][0]=0;  smat[2][1]=0;  smat[2][2]=sz; smat[2][3]=0;
    smat[3][0]=0;  smat[3][1]=0;  smat[3][2]=0;  smat[3][3]=1;
    //---------------------
    MAT_Mult(mat,smat,lmat);
    MAT_Copy(lmat,mat);
}

/***************************************
函数: Rotate_3D
功能: 旋转变换矩阵
说明: 1.绕z轴转az度角
      2.az(旋转的角度量,赋正值为顺时针转)
      3.矩阵mat根据a度角进行旋转变换
***************************************/
void Rotate_3D( float mat[4][4], float ax, float ay, float az )
{
    float  mat1[4][4];
    float  mat2[4][4];
    float  xmat[4][4];
    float  ymat[4][4];
    float  zmat[4][4];

    float SinX = FastSin(DEGTORAD(ax));
    float CosX = FastCos(DEGTORAD(ax));

    float SinY = FastSin(DEGTORAD(ay));
    float CosY = FastCos(DEGTORAD(ay));

    float SinZ = FastSin(DEGTORAD(az));
    float CosZ = FastCos(DEGTORAD(az));

    //---------------------
    xmat[0][0]=1;        xmat[0][1]=0;        xmat[0][2]=0;        xmat[0][3]=0;
    xmat[1][0]=0;        xmat[1][1]=CosX;     xmat[1][2]=SinX;     xmat[1][3]=0;
    xmat[2][0]=0;        xmat[2][1]=-SinX;    xmat[2][2]=CosX;     xmat[2][3]=0;
    xmat[3][0]=0;        xmat[3][1]=0;        xmat[3][2]=0;        xmat[3][3]=1;
    
    ymat[0][0]=CosY;     ymat[0][1]=0;        ymat[0][2]=-SinY;     ymat[0][3]=0;
    ymat[1][0]=0;        ymat[1][1]=1;        ymat[1][2]=0;         ymat[1][3]=0;
    ymat[2][0]=SinY;     ymat[2][1]=0;        ymat[2][2]=CosY;      ymat[2][3]=0;
    ymat[3][0]=0;        ymat[3][1]=0;        ymat[3][2]=0;         ymat[3][3]=1;
    
    zmat[0][0]=CosZ;     zmat[0][1]=SinZ;     zmat[0][2]=0;        zmat[0][3]=0;
    zmat[1][0]=-SinZ;    zmat[1][1]=CosZ;     zmat[1][2]=0;        zmat[1][3]=0;
    zmat[2][0]=0;        zmat[2][1]=0;        zmat[2][2]=1;        zmat[2][3]=0;
    zmat[3][0]=0;        zmat[3][1]=0;        zmat[3][2]=0;        zmat[3][3]=1;
    //---------------------
    MAT_Mult(mat,xmat,mat1);
    MAT_Mult(mat1,ymat,mat2);
    MAT_Mult(mat2,zmat,mat);
}
