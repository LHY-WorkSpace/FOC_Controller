/*******************************************************************************
* �ĵ�: transform_3D.c
* ����: Wu LianWei
* ����: 1.���ھ�������3Dͼ�α任�㷨
*       2.���㷨
*******************************************************************************/
#include "u8g2_UserGUI.h"
#include "DataType.h"
#include "stm32f10x.h"
#include "transform_3D.h"




/******************************************************************************/
/* �������                                                                   */
/******************************************************************************/

/***************************************
����: MAT_Copy
����: ���󿽱�
˵��: source(Դ����),dest(Ŀ�����)
***************************************/
void MAT_Copy( float source[4][4], float dest[4][4] )
{
    u8 i,j;
    for(i=0; i<4; i++)
        for(j=0; j<4; j++)
            dest[i][j]=source[i][j];
}

/***************************************
����: MAT_Mult
����: �������
˵��: mat1(����1),mat2(����2),dest(��˺���¾���)
***************************************/
void MAT_Mult( float mat1[4][4], float mat2[4][4], float dest[4][4] )
{
    u8 i,j;
    for(i=0; i<4; i++)
        for(j=0; j<4; j++)
            dest[i][j]=mat1[i][0]*mat2[0][j]+mat1[i][1]*mat2[1][j]+mat1[i][2]*mat2[2][j]+mat1[i][3]*mat2[3][j];
}

/***************************************
����: VEC_MultMatrix
����: ʸ����������
����: Source(Դʸ��(����)),mat(�任����)
���: �任�����ʸ��(����)
˵��: ������̺;������һ��
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
/* �任����                                                                   */
/******************************************************************************/

/***************************************
����: Identity_3D
����: ��λ����
˵��: ��һ�����鹹��ɵ�λ����
***************************************/
void Identity_3D( float mat[4][4] )
{
    mat[0][0]=1.0; mat[0][1]=0.0; mat[0][2]=0.0; mat[0][3]=0.0;
    mat[1][0]=0.0; mat[1][1]=1.0; mat[1][2]=0.0; mat[1][3]=0.0;
    mat[2][0]=0.0; mat[2][1]=0.0; mat[2][2]=1.0; mat[2][3]=0.0;
    mat[3][0]=0.0; mat[3][1]=0.0; mat[3][2]=0.0; mat[3][3]=1.0;
}

/***************************************
����: Translate_3D
����: ƽ�Ʊ任����
˵��: 1.tx,ty,tz(ƽ�Ʋ���)
      2.����mat����tx,ty,tz����ƽ�Ʊ任
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
    MAT_Mult(mat,tmat,lmat);	//����˷�
    MAT_Copy(lmat,mat);			//���ƾ���
}

/***************************************
����: Scale_3D
����: �����任����
˵��: 1.�����ԭ��ı����任
      2.sx,sy,sz(x��/y��/z���ϵ�������)
      3.����mat����sx,sy,sz���б����任
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
����: Rotate_3D
����: ��ת�任����
˵��: 1.��z��תaz�Ƚ�
      2.az(��ת�ĽǶ���,����ֵΪ˳ʱ��ת)
      3.����mat����a�Ƚǽ�����ת�任
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
