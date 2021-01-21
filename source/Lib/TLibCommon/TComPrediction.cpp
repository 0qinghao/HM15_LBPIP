/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
    : m_pLumaRecBuffer(0), m_iLumaRecStride(0)
{
    m_piYuvExt = NULL;
}

TComPrediction::~TComPrediction()
{

    delete[] m_piYuvExt;

    m_acYuvPred[0].destroy();
    m_acYuvPred[1].destroy();

    m_cYuvPredTemp.destroy();

    if (m_pLumaRecBuffer)
    {
        delete[] m_pLumaRecBuffer;
    }

    Int i, j;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            m_filteredBlock[i][j].destroy();
        }
        m_filteredBlockTmp[i].destroy();
    }
}

Void TComPrediction::initTempBuff()
{
    if (m_piYuvExt == NULL)
    {
        Int extWidth = MAX_CU_SIZE + 16;
        Int extHeight = MAX_CU_SIZE + 1;
        Int i, j;
        for (i = 0; i < 4; i++)
        {
            m_filteredBlockTmp[i].create(extWidth, extHeight + 7);
            for (j = 0; j < 4; j++)
            {
                m_filteredBlock[i][j].create(extWidth, extHeight);
            }
        }
        m_iYuvExtHeight = ((MAX_CU_SIZE + 2) << 4);
        m_iYuvExtStride = ((MAX_CU_SIZE + 8) << 4);
        m_piYuvExt = new Int[m_iYuvExtStride * m_iYuvExtHeight];

        // new structure
        m_acYuvPred[0].create(MAX_CU_SIZE, MAX_CU_SIZE);
        m_acYuvPred[1].create(MAX_CU_SIZE, MAX_CU_SIZE);

        m_cYuvPredTemp.create(MAX_CU_SIZE, MAX_CU_SIZE);
    }

    if (m_iLumaRecStride != (MAX_CU_SIZE >> 1) + 1)
    {
        m_iLumaRecStride = (MAX_CU_SIZE >> 1) + 1;
        if (!m_pLumaRecBuffer)
        {
            m_pLumaRecBuffer = new Pel[m_iLumaRecStride * m_iLumaRecStride];
        }
    }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
Pel TComPrediction::predIntraGetPredValDC(Int *pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft)
{
    assert(iWidth > 0 && iHeight > 0);
    Int iInd, iSum = 0;
    Pel pDcVal;

    if (bAbove)
    {
        for (iInd = 0; iInd < iWidth; iInd++)
        {
            iSum += pSrc[iInd - iSrcStride];
        }
    }
    if (bLeft)
    {
        for (iInd = 0; iInd < iHeight; iInd++)
        {
            iSum += pSrc[iInd * iSrcStride - 1];
        }
    }

    if (bAbove && bLeft)
    {
        pDcVal = (iSum + iWidth) / (iWidth + iHeight);
    }
    // 完全没进过下面这些分支
    else if (bAbove)
    {
        pDcVal = (iSum + iWidth / 2) / iWidth;
    }
    else if (bLeft)
    {
        pDcVal = (iSum + iHeight / 2) / iHeight;
    }
    else
    {
        pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
    }

    return pDcVal;
}
Pel TComPrediction::predIntraGetPredValDCLP(Int *pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, Bool bAbove, Bool bLeft, UInt uiPredDstSize)
{
    assert(iWidth > 0 && iHeight > 0 && uiPredDstSize > 0);
    Int iInd, iSum = 0;
    Pel pDcVal;

    if (bAbove)
    {
        for (iInd = 0; iInd < uiPredDstSize; iInd++)
        {
            iSum += pSrc[iInd - iSrcStride];
        }
    }
    if (bLeft)
    {
        for (iInd = 0; iInd < uiPredDstSize; iInd++)
        {
            iSum += pSrc[iInd * iSrcStride - 1];
        }
    }

    if (bAbove && bLeft)
    {
        pDcVal = (iSum + uiPredDstSize) / (uiPredDstSize + uiPredDstSize);
    }
    else
    {
        assert(0);
    }
    return pDcVal;
}
// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
// TODO: 原来这个 rpDst 是引用指针, 引用导致没法在调用的时候操作该参数, 改完发现没影响. 是真的没影响吗
Void TComPrediction::xPredIntraAng(Int bitDepth, Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter)
{
    Int k, l;
    Int blkSize = width;
    Pel *pDst = rpDst;

    // Map the mode index to main prediction direction and angle
    assert(dirMode > 0); //no planar
    Bool modeDC = dirMode < 2;
    Bool modeHor = !modeDC && (dirMode < 18);
    Bool modeVer = !modeDC && !modeHor;
    // intraPredAngle 记录当前模式同水平/垂直模式之间的差
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng = abs(intraPredAngle);
    Int signAng = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9] = {0, 2, 5, 9, 13, 17, 21, 26, 32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle = invAngTable[absAng];
    absAng = angTable[absAng];
    intraPredAngle = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable);

        for (k = 0; k < blkSize; k++)
        {
            for (l = 0; l < blkSize; l++)
            {
                pDst[k * dstStride + l] = dcval;
            }
        }
    }

    // Do angular predictions
    else
    {
        Pel *refMain;
        Pel *refSide;
        Pel refAbove[2 * MAX_CU_SIZE + 1];
        Pel refLeft[2 * MAX_CU_SIZE + 1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {
            for (k = 0; k < blkSize + 1; k++)
            {
                refAbove[k + blkSize - 1] = pSrc[k - srcStride - 1];
            }
            for (k = 0; k < blkSize + 1; k++)
            {
                refLeft[k + blkSize - 1] = pSrc[(k - 1) * srcStride - 1];
            }
            refMain = (modeVer ? refAbove : refLeft) + (blkSize - 1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize - 1);

            // Extend the Main reference to the left.
            Int invAngleSum = 128; // rounding for (shift by 8)
            for (k = -1; k > blkSize * intraPredAngle >> 5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum >> 8];
            }
        }
        else
        {
            for (k = 0; k < 2 * blkSize + 1; k++)
            {
                refAbove[k] = pSrc[k - srcStride - 1];
            }
            for (k = 0; k < 2 * blkSize + 1; k++)
            {
                refLeft[k] = pSrc[(k - 1) * srcStride - 1];
            }
            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft : refAbove;
        }

        if (intraPredAngle == 0)
        {
            for (k = 0; k < blkSize; k++)
            {
                for (l = 0; l < blkSize; l++)
                {
                    pDst[k * dstStride + l] = refMain[l + 1];
                }
            }

            if (bFilter)
            {
                for (k = 0; k < blkSize; k++)
                {
                    pDst[k * dstStride] = Clip3(0, (1 << bitDepth) - 1, pDst[k * dstStride] + ((refSide[k + 1] - refSide[0]) >> 1));
                }
            }
        }
        else
        {
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            Int refMainIndex;

            for (k = 0; k < blkSize; k++)
            {
                deltaPos += intraPredAngle;
                deltaInt = deltaPos >> 5;
                deltaFract = deltaPos & (32 - 1);

                if (deltaFract)
                {
                    // Do linear filtering
                    for (l = 0; l < blkSize; l++)
                    {
                        refMainIndex = l + deltaInt + 1;
                        pDst[k * dstStride + l] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                    }
                }
                else
                {
                    // Just copy the integer samples
                    for (l = 0; l < blkSize; l++)
                    {
                        pDst[k * dstStride + l] = refMain[l + deltaInt + 1];
                    }
                }
            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            Pel tmp;
            for (k = 0; k < blkSize - 1; k++)
            {
                for (l = k + 1; l < blkSize; l++)
                {
                    tmp = pDst[k * dstStride + l];
                    pDst[k * dstStride + l] = pDst[l * dstStride + k];
                    pDst[l * dstStride + k] = tmp;
                }
            }
        }
    }
}
Int TComPrediction::getModebound(int x, Bool isrow)
{
    //x是在当前行中第几个（从1开始）或者在列中第几个
    //width是当前层宽
    //widthf是块宽
    //dirMode是
    //isrow表示是行像素还是列像素
    //适用于下方像素也存在的情况
    // assert(x <= 32);
    // Int i;
    // Int Modebound = 0;
    // Int Rx[32] = {100, 34, 33, 31, 30, 30, 30, 30, 30, 29, 29, 29, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 27};
    // Int Ry[32] = {0, 2, 3, 5, 6, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9};
    // Modebound = isrow ? Ry[x - 1] : Rx[x - 1];
    // return Modebound;

    // Int getModebound(int x, int width, int widthf, int isrow)
    //x是在当前行中第几个（从1开始）或者在列中第几个
    //width是当前层宽
    //widthf是块宽
    //dirMode是
    //isrow表示是行像素还是列像素
    //适用于下方像素也存在的情况
    // Int blkSizef = widthf; //16
    // int blkSize = width;
    int i;
    int tang[9] = {0, 16, 40, 72, 104, 136, 168, 208, 256}; //当前角度正切值
    int Modebound = 0;
    int Modeboundrow = 0;
    int Modeboundcol;
    int Modebound1 = 0;
    int Modebound2 = 0;
    int Modebound3 = 0;

    float curang1 = 256.0 / x; //黑色
                               //	float	curang2 = 256.0 *(blkSizef - blkSize + 1) / (blkSize - x + blkSizef);//红色
                               //	float	curang3 = 256.0 / (blkSize - x);//绿色

    for (i = 0; i <= 8; i++) //根据每个点位置信息算出了需要反向的模式边界
    {
        if (curang1 > tang[i] && curang1 <= tang[i + 1]) //步骤1
        // Modebound1 = 10 - (i + 1);
        {
            Modebound1 = isrow ? (10 - (i + 1)) : (26 + i + 1);
            break;
        }
    }

    return Modebound1;
}
Void TComPrediction::xPredIntraAngLP(Int bitDepth, Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter, UInt uiPredDstSize)
{
    Int k, l;
    Int blkSize = uiPredDstSize;
    Pel *pDst = rpDst;
    //pSrc定为上一层参考像素的指针
    //y从0开始
    //输入中还需加入最大块对应的参考像素指针定为 pSrcf
    //当前层位于当前块的第y行(从0开始)
    //srcStridef表示最大块(16)的stride
    // Map the mode index to main prediction direction and angle
    assert(dirMode > 0); //no planar
    Bool modeDC = dirMode < 2;
    Bool modeHor = !modeDC && (dirMode < 18);
    Bool modeVer = !modeDC && !modeHor;
    Int intraPredAngle = modeVer ? (Int)dirMode - 26 : modeHor ? -((Int)dirMode - 10) : 0;
    Int absAng = abs(intraPredAngle);
    Int signAng = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9] = {0, 2, 5, 9, 13, 17, 21, 26, 32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle(0,2,5,9这种)
    Int invAngle = invAngTable[absAng];                                 // 当前角度的缩放余切值
    absAng = angTable[absAng];                                          // 当前角度偏移值绝对值
    intraPredAngle = signAng * absAng;                                  // 当前角度偏移值

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDCLP(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable, uiPredDstSize);
        // Pel dcval = 2000;
        for (l = 0; l < blkSize; l++)
        {
            pDst[l] = dcval;
        }
        for (k = 0; k < blkSize; k++)
        {
            pDst[k * dstStride] = dcval;
        }
    }

    // Do angular predictions
    else
    {
        Pel *refMain;                      // 主参考像素，垂直类模式即为上方的参考像素
        Pel *refSide;                      // 侧边参考像素，垂直类模式即为左侧的参考像素
        Pel refAbove[2 * MAX_CU_SIZE + 1]; // 上方参考像素
        Pel refLeft[2 * MAX_CU_SIZE + 1];  // 左侧参考像素

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {
            // 对于偏移值小于零的角度，垂直类模式需要将左侧的参考像素投影到上方参考像素的左侧，水平类反之
            for (k = 0; k < blkSize + 1; k++)
            {
                refAbove[k + blkSize - 1] = pSrc[k - srcStride - 1]; //blkSize-1是为了主参考像素向右偏移
            }
            for (k = 0; k < blkSize + 1; k++)
            {
                refLeft[k + blkSize - 1] = pSrc[(k - 1) * srcStride - 1]; // 填充左侧参考像素
            }

            refMain = (modeVer ? refAbove : refLeft) + (blkSize - 1); // 确定主参考像素
            refSide = (modeVer ? refLeft : refAbove) + (blkSize - 1); // 确定侧参考像素

            // Extend the Main reference to the left.
            //进行参考像素的投影
            Int invAngleSum = 128; // rounding for (shift by 8)
                                   // intraPredAngle>>5即正切值 当前块高度乘以角度正切值，得到所需投影的最大数量
            for (k = -1; k > blkSize * intraPredAngle >> 5; k--)
            {
                invAngleSum += invAngle;                // 每次加缩放余切值，相当于invAngleSum=invAngle*abs(k),开始的128为了四舍五入，不用在意
                refMain[k] = refSide[invAngleSum >> 8]; // 右移8位，相当于除以256，把余切值缩放抵消。因此得到的坐标相当于是abs(k)乘以余切值，正好是投影
            }
        }
        else
        {
            // 角度偏移值大于等于零的不需要投影
            for (k = 0; k < 2 * blkSize + 1; k++)
            {
                refAbove[k] = pSrc[k - srcStride - 1];      // 填充上方参考像素
                refLeft[k] = pSrc[(k - 1) * srcStride - 1]; // 填充左侧参考像素
            }

            refMain = modeVer ? refAbove : refLeft; // 确定主参考像素
            refSide = modeVer ? refLeft : refAbove; // 确定侧参考像素
        }

        if (intraPredAngle == 0) // pure vertical or pure horizontal
        {
            //竖直模式或水平模式
            for (l = 0; l < blkSize; l++)
            {
                pDst[l] = refMain[l + 1];
            }
            for (k = 1; k < blkSize; k++)
            {
                pDst[k * dstStride] = refMain[1];
            }

            if (bFilter)
            {
                for (k = 0; k < blkSize; k++)
                {
                    pDst[k * dstStride] = Clip3(0, (1 << bitDepth) - 1, pDst[k * dstStride] + ((refSide[k + 1] - refSide[0]) >> 1));
                }
            }
        }

        else if (intraPredAngle < 0)
        {
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            Int refMainIndex;

            {
                deltaPos += intraPredAngle;
                deltaInt = deltaPos >> 5;
                deltaFract = deltaPos & (32 - 1);
                if (deltaFract)
                {
                    // Do linear filtering
                    for (l = 0; l < blkSize; l++)
                    {
                        refMainIndex = l + deltaInt + 1;
                        pDst[l] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                    }
                }
                else
                {
                    // Just copy the integer samples
                    for (l = 0; l < blkSize; l++)
                    {
                        pDst[l] = refMain[l + deltaInt + 1];
                    }
                }
            }
            for (k = 1; k < blkSize; k++)
            {
                deltaPos += intraPredAngle;
                deltaInt = deltaPos >> 5;
                deltaFract = deltaPos & (32 - 1);
                if (deltaFract)
                {
                    // Do linear filtering
                    refMainIndex = deltaInt + 1;
                    pDst[k * dstStride] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                }
                else
                {
                    // Just copy the integer samples
                    pDst[k * dstStride] = refMain[deltaInt + 1];
                }
            }
        }

        else //剩>0的情况
        {
            // 非竖直或水平模式
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            Int refMainIndex;

            Int Modebound;

            //模式为垂直类和水平类对反向的计算起始一样，只是水平类最后翻转一下即可

            deltaPos += intraPredAngle;
            deltaInt = deltaPos >> 5;
            deltaFract = deltaPos & (32 - 1);
            if (deltaFract)
            {
                // Do linear filtering
                for (l = 0; l < blkSize; l++) //行的情况不变
                {
                    refMainIndex = l + deltaInt + 1;
                    pDst[l] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                }
            }
            else
            {
                // Just copy the integer samples
                for (l = 0; l < blkSize; l++)
                {
                    pDst[l] = refMain[l + deltaInt + 1];
                }
            }

            for (k = 1; k < blkSize; k++) //列的情况
            {
                Modebound = getModebound(k + 1, modeHor);
                if (modeVer ? (dirMode < Modebound) : (dirMode > Modebound)) //说明不要反向
                {
                    deltaPos += intraPredAngle;
                    deltaInt = deltaPos >> 5;
                    deltaFract = deltaPos & (32 - 1);
                    if (deltaFract)
                    {
                        // Do linear filtering
                        refMainIndex = deltaInt + 1;
                        pDst[k * dstStride] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                    }
                    else
                    {
                        // Just copy the integer samples
                        pDst[k * dstStride] = refMain[deltaInt + 1];
                    }
                }
                else //需要反向的点
                {
                    //由于是从垂直模式反向水平直模式，原来的偏移值intraPredAngle指的是当前模式距离垂直模式的偏移
                    deltaPos = 32;                                                   //deltaPos为行高(这里即为1)，32除以该值即为参考像素相对当前像素所在行的反向偏移值
                    deltaInt = deltaPos / intraPredAngle;                            //整数部分
                    deltaFract = 32 * ((float)deltaPos / intraPredAngle - deltaInt); //余数部分

                    if (deltaFract)
                    {
                        refMainIndex = k + deltaInt + 1; // 左侧参考像素
                        pDst[k * dstStride] = (Pel)(((32 - deltaFract) * refSide[refMainIndex] + deltaFract * refSide[refMainIndex + 1] + 16) >> 5);
                    }
                    else
                    {
                        pDst[k * dstStride] = refSide[k + deltaInt + 1]; //copy
                    }
                }
            }
        }
        if (modeHor)
        {
            Pel tmp;
            k = 0;
            {
                for (l = k + 1; l < blkSize; l++)
                {
                    tmp = pDst[k * dstStride + l];
                    pDst[k * dstStride + l] = pDst[l * dstStride + k];
                    pDst[l * dstStride + k] = tmp;
                }
            }
        }
    }
}

Void TComPrediction::xPredIntraAng3x3(Int bitDepth, Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, Bool bFilter, UInt uiPredDstSize)
{
    assert(uiPredDstSize == 3);
    Int k, l;
    Int blkSize = uiPredDstSize;
    Pel *pDst = rpDst;

    // Map the mode index to main prediction direction and angle
    assert(dirMode > 0); //no planar
    Bool modeDC = dirMode < 2;
    Bool modeHor = !modeDC && (dirMode < 18);
    Bool modeVer = !modeDC && !modeHor;
    // intraPredAngle 记录当前模式同水平/垂直模式之间的差
    Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
    Int absAng = abs(intraPredAngle);
    Int signAng = intraPredAngle < 0 ? -1 : 1;

    // Set bitshifts and scale the angle parameter to block size
    Int angTable[9] = {0, 2, 5, 9, 13, 17, 21, 26, 32};
    Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle = invAngTable[absAng];
    absAng = angTable[absAng];
    intraPredAngle = signAng * absAng;

    // Do the DC prediction
    if (modeDC)
    {
        Pel dcval = predIntraGetPredValDCLP(pSrc, srcStride, width, height, blkAboveAvailable, blkLeftAvailable, uiPredDstSize);

        for (k = 0; k < blkSize; k++)
        {
            for (l = 0; l < blkSize; l++)
            {
                pDst[k * dstStride + l] = dcval;
            }
        }
    }

    // Do angular predictions
    else
    {
        Pel *refMain;
        Pel *refSide;
        Pel refAbove[2 * MAX_CU_SIZE + 1];
        Pel refLeft[2 * MAX_CU_SIZE + 1];

        // Initialise the Main and Left reference array.
        if (intraPredAngle < 0)
        {
            for (k = 0; k < blkSize + 1; k++)
            {
                refAbove[k + blkSize - 1] = pSrc[k - srcStride - 1];
            }
            for (k = 0; k < blkSize + 1; k++)
            {
                refLeft[k + blkSize - 1] = pSrc[(k - 1) * srcStride - 1];
            }
            refMain = (modeVer ? refAbove : refLeft) + (blkSize - 1);
            refSide = (modeVer ? refLeft : refAbove) + (blkSize - 1);

            // Extend the Main reference to the left.
            Int invAngleSum = 128; // rounding for (shift by 8)
            for (k = -1; k > blkSize * intraPredAngle >> 5; k--)
            {
                invAngleSum += invAngle;
                refMain[k] = refSide[invAngleSum >> 8];
            }
        }
        else
        {
            for (k = 0; k < 2 * blkSize + 1; k++)
            { // 不管是 refAbove 还是 refLeft 都是从左上角参考点开始存储, 也就是说 refAbove[0] 和 refLeft[0] 是一样的, 重复了
                refAbove[k] = pSrc[k - srcStride - 1];
            }
            for (k = 0; k < 2 * blkSize + 1; k++)
            {
                refLeft[k] = pSrc[(k - 1) * srcStride - 1];
            }
            refMain = modeVer ? refAbove : refLeft;
            refSide = modeVer ? refLeft : refAbove;
        }

        if (intraPredAngle == 0)
        {
            for (k = 0; k < blkSize; k++)
            {
                for (l = 0; l < blkSize; l++)
                {
                    pDst[k * dstStride + l] = refMain[l + 1];
                }
            }

            if (bFilter)
            {
                for (k = 0; k < blkSize; k++)
                {
                    pDst[k * dstStride] = Clip3(0, (1 << bitDepth) - 1, pDst[k * dstStride] + ((refSide[k + 1] - refSide[0]) >> 1));
                }
            }
        }
        else
        {
            Int deltaPos = 0;
            Int deltaInt;
            Int deltaFract;
            Int refMainIndex;

            for (k = 0; k < blkSize; k++)
            {
                deltaPos += intraPredAngle;
                deltaInt = deltaPos >> 5;
                deltaFract = deltaPos & (32 - 1);

                if (deltaFract)
                {
                    // Do linear filtering
                    for (l = 0; l < blkSize; l++)
                    {
                        refMainIndex = l + deltaInt + 1;
                        pDst[k * dstStride + l] = (Pel)(((32 - deltaFract) * refMain[refMainIndex] + deltaFract * refMain[refMainIndex + 1] + 16) >> 5);
                    }
                }
                else
                {
                    // Just copy the integer samples
                    for (l = 0; l < blkSize; l++)
                    {
                        pDst[k * dstStride + l] = refMain[l + deltaInt + 1];
                    }
                }
            }
        }

        // Flip the block if this is the horizontal mode
        if (modeHor)
        {
            Pel tmp;
            for (k = 0; k < blkSize - 1; k++)
            {
                for (l = k + 1; l < blkSize; l++)
                {
                    tmp = pDst[k * dstStride + l];
                    pDst[k * dstStride + l] = pDst[l * dstStride + k];
                    pDst[l * dstStride + k] = tmp;
                }
            }
        }
    }
}
Void TComPrediction::predIntraLumaAng(TComPattern *pcTComPattern, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
    Pel *pDst = piPred;
    // ptrSrc 指向当前的参考像素
    Int *ptrSrc;

    assert(g_aucConvertToBit[iWidth] >= 0); //   4x  4
    assert(g_aucConvertToBit[iWidth] <= 5); // 128x128
    assert(iWidth == iHeight);

    ptrSrc = pcTComPattern->getPredictorPtr(uiDirMode, g_aucConvertToBit[iWidth] + 2, m_piYuvExt);

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;

    // Create the prediction
    if (uiDirMode == PLANAR_IDX)
    {
        // PLANAR 模式预测
        xPredIntraPlanar(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
    }
    else
    {
        if ((iWidth > 16) || (iHeight > 16))
        {
            xPredIntraAng(g_bitDepthY, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false);
        }
        else
        {
            // 角度模式预测(包括 DC 在内), 块小于 16 时需要滤波
            xPredIntraAng(g_bitDepthY, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true);

            // DC 模式下预测结果的滤波
            if ((uiDirMode == DC_IDX) && bAbove && bLeft)
            {
                xDCPredFiltering(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
            }
        }
    }
}
UInt TComPrediction::predIntraLumaAngLP(TComPattern *pcTComPattern, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft, UInt mask, UInt uiPredDstSize)
{
    Pel *pDst = piPred;
    // ptrSrc 指向当前的参考像素
    Int *ptrSrc;

    // assert(g_aucConvertToBit[iWidth] >= 0); //   4x  4
    // assert(g_aucConvertToBit[iWidth] <= 5); // 128x128
    assert(iWidth == iHeight);

    ptrSrc = pcTComPattern->getPredictorPtrLP(m_piYuvExt);

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;
    Int srcoffset = sw * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);
    Int dstoffset = uiStride * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);

    // Create the prediction
    if (uiDirMode == PLANAR_IDX)
    {
        // PLANAR 模式预测
        // xPredIntraPlanarLP(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
        xPredIntraPlanarLPnew(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
    }
    else
    {
        // TODO: 环状预测时 尺寸是连续变化的 可以尝试不设定 16 作为滤波的界限
        // if ((iWidth > 16) || (iHeight > 16))
        if (uiPredDstSize > 8)
        {
            xPredIntraAngLP(g_bitDepthY, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false, uiPredDstSize);
        }
        else
        {
            // 角度模式预测(包括 DC 在内), 块小于 16 时需要滤波
            xPredIntraAngLP(g_bitDepthY, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true, uiPredDstSize);

            // DC 模式下预测结果的滤波
            if ((uiDirMode == DC_IDX) && bAbove && bLeft)
            {
                // xDCPredFiltering(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight);
                xDCPredFiltering(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, uiPredDstSize, uiPredDstSize);
            }
        }
    }

    // TODO: 可以尝试用误差平方和作为环状决定预测角度的依据
    UInt uiSAE = 0;
    Pel *pred = pDst + dstoffset;
    Int *src = ptrSrc + sw + 1 + srcoffset;
    switch (mask)
    {
    case 0b1111:
    case 0b1110:
        for (Int i = 0; i < uiPredDstSize; i++)
        {
            uiSAE += abs(src[i] - pred[i]);
            // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
        }
        for (Int j = 1; j < uiPredDstSize; j++)
        {
            uiSAE += abs(src[j * sw] - pred[j * uiStride]);
            // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
        }
        break;
    case 0b1011:
        if (uiPredDstSize <= iWidth / 2)
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        else
        {
            for (Int i = 0; i < uiPredDstSize - iWidth / 2; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        break;
    case 0b1101:
        if (uiPredDstSize <= iWidth / 2)
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        else
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize - iWidth / 2; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        break;
    default:
        assert(0);
    }

    return uiSAE;
}
UInt TComPrediction::predIntraLumaAng3x3(TComPattern *pcTComPattern, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft, UInt mask, UInt uiPredDstSize)
{
    Pel *pDst = piPred;
    // ptrSrc 指向当前的参考像素
    Int *ptrSrc;

    // assert(g_aucConvertToBit[iWidth] >= 0); //   4x  4
    // assert(g_aucConvertToBit[iWidth] <= 5); // 128x128
    assert(iWidth == iHeight);

    ptrSrc = pcTComPattern->getPredictorPtrLP(m_piYuvExt);

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;
    Int srcoffset = sw * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);
    Int dstoffset = uiStride * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);

    // Create the prediction
    if (uiDirMode == PLANAR_IDX)
    {
        // PLANAR 模式预测
        xPredIntraPlanar3x3(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
    }
    else
    {
        // if ((iWidth > 16) || (iHeight > 16))
        // if (uiPredDstSize > 8)
        // {
        //     xPredIntraAng3x3(g_bitDepthY, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false, uiPredDstSize);
        // }
        // else
        {
            // 角度模式预测(包括 DC 在内), 块小于 16 时需要滤波
            xPredIntraAng3x3(g_bitDepthY, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, true, uiPredDstSize);

            // DC 模式下预测结果的滤波
            if ((uiDirMode == DC_IDX) && bAbove && bLeft)
            {
                xDCPredFiltering(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, uiPredDstSize, uiPredDstSize);
            }
        }
    }

    // TODO: 可以尝试用误差平方和作为环状决定预测角度的依据
    UInt uiSAE = 0;
    Pel *pred = pDst + dstoffset;
    Int *src = ptrSrc + sw + 1 + srcoffset;
    // switch (mask)
    // {
    // case 0b1111:
    for (Int i = 0; i < uiPredDstSize; i++)
    {
        for (Int j = 0; j < uiPredDstSize; j++)
        {
            uiSAE += abs(src[i * sw + j] - pred[i * uiStride + j]);
            // uiSAE += (src[i * sw + j] - pred[i * uiStride + j]) * (src[i * sw + j] - pred[i * uiStride + j]);
        }
    }
    //     break;
    // default:
    //     break;
    // }

    return uiSAE;
}
Void TComPrediction::FillRefChromaLP(Int *piRef, Pel *piOrg, UInt uiWidth, UInt mask)
{

    UInt uiStrideRef = uiWidth * 2 + 1;
    // 色差部分不同的地方
    // UInt uiStrideOrg = (uiWidth == 4) ? 8 : uiWidth;
    UInt uiStrideOrg = uiWidth;
    piRef += uiStrideRef + 1;

    Int *piRefbak = piRef;
    Pel *piOrgbak = piOrg;
    Int i, j, k;

    for (i = 0; i < uiWidth; i++)
    {
        for (j = 0; j < uiWidth; j++)
        {
            piRef[j] = piOrg[j];
        }
        for (j = uiWidth; j < 2 * uiWidth; j++)
        {
            piRef[j] = piOrg[uiWidth - 1];
        }

        piRef += uiStrideRef;
        piOrg += uiStrideOrg;
    }
    piOrg -= uiStrideOrg;
    for (i = 0; i < uiWidth; i++)
    {
        for (j = 0; j < uiWidth; j++)
        {
            piRef[j] = piOrg[j];
        }
        piRef += uiStrideRef;
    }

    piRef = piRefbak;
    piOrg = piOrgbak;
    if (uiWidth >= 8)
    {
        switch (mask)
        {
        case 0b1011:
            piRef += uiWidth / 2;
            piOrg += uiWidth / 2;
            for (i = 0; i < uiWidth / 2 - 1; i++)
            {
                for (j = 0; j < uiWidth / 2 * 3; j++)
                {
                    piRef[j] = piOrg[-1];
                }
                piRef += uiStrideRef;
                piOrg += uiStrideOrg;
            }
            break;
        case 0b1101:
            piRef += uiWidth / 2 * uiStrideRef;
            piOrg += uiWidth / 2 * uiStrideOrg;
            for (i = 0; i < uiWidth / 2 * 3; i++)
            {
                for (j = 0; j < uiWidth / 2 - 1; j++)
                {
                    piRef[j] = *(piOrg + j - uiStrideOrg);
                }
                piRef += uiStrideRef;
            }
            break;
        default:
            break;
        }
    }
}
Void TComPrediction::FillRefLP(Int *piRef, Pel *piOrg, UInt uiWidth, UInt mask)
{

    UInt uiStrideRef = uiWidth * 2 + 1;
    UInt uiStrideOrg = (uiWidth == 4) ? 8 : uiWidth;
    piRef += uiStrideRef + 1;

    Int *piRefbak = piRef;
    Pel *piOrgbak = piOrg;
    Int i, j, k;

    for (i = 0; i < uiWidth; i++)
    {
        for (j = 0; j < uiWidth; j++)
        {
            piRef[j] = piOrg[j];
        }
        for (j = uiWidth; j < 2 * uiWidth; j++)
        {
            piRef[j] = piOrg[uiWidth - 1];
        }

        piRef += uiStrideRef;
        piOrg += uiStrideOrg;
    }
    piOrg -= uiStrideOrg;
    for (i = 0; i < uiWidth; i++)
    {
        for (j = 0; j < uiWidth; j++)
        {
            piRef[j] = piOrg[j];
        }
        piRef += uiStrideRef;
    }

    piRef = piRefbak;
    piOrg = piOrgbak;
    switch (mask)
    {
    case 0b1011:
        piRef += uiWidth / 2;
        piOrg += uiWidth / 2;
        for (i = 0; i < uiWidth / 2 - 1; i++)
        {
            for (j = 0; j < (uiWidth / 2) * 3; j++)
            {
                piRef[j] = piOrg[-1];
            }
            piRef += uiStrideRef;
            piOrg += uiStrideOrg;
        }
        break;
    case 0b1101:
        piRef += uiWidth / 2 * uiStrideRef;
        piOrg += uiWidth / 2 * uiStrideOrg;
        for (i = 0; i < uiWidth / 2 * 3; i++)
        {
            for (j = 0; j < uiWidth / 2 - 1; j++)
            {
                piRef[j] = *(piOrg + j - uiStrideOrg);
            }
            piRef += uiStrideRef;
        }
        break;
    default:
        break;
    }
}
// Angular chroma
Void TComPrediction::predIntraChromaAng(Int *piSrc, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft)
{
    Pel *pDst = piPred;
    Int *ptrSrc = piSrc;

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;

    if (uiDirMode == PLANAR_IDX)
    {
        xPredIntraPlanar(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
    }
    else
    {
        // Create the prediction
        xPredIntraAng(g_bitDepthC, ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false);
    }
}
UInt TComPrediction::predIntraChromaAng3x3(Int *piSrc, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft, UInt mask, UInt uiPredDstSize)
{
    Pel *pDst = piPred;
    Int *ptrSrc = piSrc;

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;
    Int srcoffset = sw * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);
    Int dstoffset = uiStride * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);

    if (uiDirMode == PLANAR_IDX)
    {
        xPredIntraPlanar3x3(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
    }
    else
    {
        // Create the prediction
        xPredIntraAng3x3(g_bitDepthC, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false, uiPredDstSize);
    }

    // TODO: 可以尝试用误差平方和作为环状决定预测角度的依据
    UInt uiSAE = 0;
    Pel *pred = pDst + dstoffset;
    Int *src = ptrSrc + sw + 1 + srcoffset;
    for (Int i = 0; i < uiPredDstSize; i++)
    {
        for (Int j = 0; j < uiPredDstSize; j++)
        {
            uiSAE += abs(src[i * sw + j] - pred[i * uiStride + j]);
            // uiSAE += (src[i * sw + j] - pred[i * uiStride + j]) * (src[i * sw + j] - pred[i * uiStride + j]);
        }
    }
    return uiSAE;
}
UInt TComPrediction::predIntraChromaAngLP(Int *piSrc, UInt uiDirMode, Pel *piPred, UInt uiStride, Int iWidth, Int iHeight, Bool bAbove, Bool bLeft, UInt mask, UInt uiPredDstSize)
{
    Pel *pDst = piPred;
    Int *ptrSrc = piSrc;

    // get starting pixel in block
    Int sw = 2 * iWidth + 1;
    Int srcoffset = sw * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);
    Int dstoffset = uiStride * (iWidth - uiPredDstSize) + (iWidth - uiPredDstSize);

    if (uiDirMode == PLANAR_IDX)
    {
        // xPredIntraPlanarLP(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
        xPredIntraPlanarLPnew(ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiPredDstSize);
    }
    else
    {
        // Create the prediction
        xPredIntraAngLP(g_bitDepthC, ptrSrc + sw + 1 + srcoffset, sw, pDst + dstoffset, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft, false, uiPredDstSize);
    }

    // TODO: 可以尝试用误差平方和作为环状决定预测角度的依据
    UInt uiSAE = 0;
    Pel *pred = pDst + dstoffset;
    Int *src = ptrSrc + sw + 1 + srcoffset;
    switch (mask)
    {
    case 0b1111:
    case 0b1110:
        for (Int i = 0; i < uiPredDstSize; i++)
        {
            uiSAE += abs(src[i] - pred[i]);
            // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
        }
        for (Int j = 1; j < uiPredDstSize; j++)
        {
            uiSAE += abs(src[j * sw] - pred[j * uiStride]);
            // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
        }
        break;
    case 0b1011:
        if (uiPredDstSize <= iWidth / 2)
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        else
        {
            for (Int i = 0; i < uiPredDstSize - iWidth / 2; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        break;
    case 0b1101:
        if (uiPredDstSize <= iWidth / 2)
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        else
        {
            for (Int i = 0; i < uiPredDstSize; i++)
            {
                uiSAE += abs(src[i] - pred[i]);
                // uiSAE += (src[i] - pred[i]) * (src[i] - pred[i]);
            }
            for (Int j = 1; j < uiPredDstSize - iWidth / 2; j++)
            {
                uiSAE += abs(src[j * sw] - pred[j * uiStride]);
                // uiSAE += (src[j * sw] - pred[j * uiStride]) * (src[j * sw] - pred[j * uiStride]);
            }
        }
        break;
    default:
        assert(0);
    }

    return uiSAE;
}
/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion(TComDataCU *pcCU, UInt PartAddr)
{
    if (pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred())
    {
        if (pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
        {
            Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
            Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
            if (RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
            {
                return true;
            }
        }
    }
    return false;
}

Void TComPrediction::motionCompensation(TComDataCU *pcCU, TComYuv *pcYuvPred, RefPicList eRefPicList, Int iPartIdx)
{
    Int iWidth;
    Int iHeight;
    UInt uiPartAddr;

    if (iPartIdx >= 0)
    {
        pcCU->getPartIndexAndSize(iPartIdx, uiPartAddr, iWidth, iHeight);
        if (eRefPicList != REF_PIC_LIST_X)
        {
            if (pcCU->getSlice()->getPPS()->getUseWP())
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true);
            }
            else
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred);
            }
            if (pcCU->getSlice()->getPPS()->getUseWP())
            {
                xWeightedPredictionUni(pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred);
            }
        }
        else
        {
            if (xCheckIdenticalMotion(pcCU, uiPartAddr))
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred);
            }
            else
            {
                xPredInterBi(pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred);
            }
        }
        return;
    }

    for (iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++)
    {
        pcCU->getPartIndexAndSize(iPartIdx, uiPartAddr, iWidth, iHeight);

        if (eRefPicList != REF_PIC_LIST_X)
        {
            if (pcCU->getSlice()->getPPS()->getUseWP())
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true);
            }
            else
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred);
            }
            if (pcCU->getSlice()->getPPS()->getUseWP())
            {
                xWeightedPredictionUni(pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred);
            }
        }
        else
        {
            if (xCheckIdenticalMotion(pcCU, uiPartAddr))
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred);
            }
            else
            {
                xPredInterBi(pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred);
            }
        }
    }
    return;
}

Void TComPrediction::xPredInterUni(TComDataCU *pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv *&rpcYuvPred, Bool bi)
{
    Int iRefIdx = pcCU->getCUMvField(eRefPicList)->getRefIdx(uiPartAddr);
    assert(iRefIdx >= 0);
    TComMv cMv = pcCU->getCUMvField(eRefPicList)->getMv(uiPartAddr);
    pcCU->clipMv(cMv);
    xPredInterLumaBlk(pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi);
    xPredInterChromaBlk(pcCU, pcCU->getSlice()->getRefPic(eRefPicList, iRefIdx)->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi);
}

Void TComPrediction::xPredInterBi(TComDataCU *pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv *&rpcYuvPred)
{
    TComYuv *pcMbYuv;
    Int iRefIdx[2] = {-1, -1};

    for (Int iRefList = 0; iRefList < 2; iRefList++)
    {
        RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
        iRefIdx[iRefList] = pcCU->getCUMvField(eRefPicList)->getRefIdx(uiPartAddr);

        if (iRefIdx[iRefList] < 0)
        {
            continue;
        }

        assert(iRefIdx[iRefList] < pcCU->getSlice()->getNumRefIdx(eRefPicList));

        pcMbYuv = &m_acYuvPred[iRefList];
        if (pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiPartAddr) >= 0)
        {
            xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true);
        }
        else
        {
            if ((pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE) ||
                (pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE))
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true);
            }
            else
            {
                xPredInterUni(pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv);
            }
        }
    }

    if (pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE)
    {
        xWeightedPredictionBi(pcCU, &m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred);
    }
    else if (pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE)
    {
        xWeightedPredictionUni(pcCU, &m_acYuvPred[0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred);
    }
    else
    {
        xWeightedAverage(&m_acYuvPred[0], &m_acYuvPred[1], iRefIdx[0], iRefIdx[1], uiPartAddr, iWidth, iHeight, rpcYuvPred);
    }
}

/**
 * \brief Generate motion-compensated luma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterLumaBlk(TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi)
{
    Int refStride = refPic->getStride();
    Int refOffset = (mv->getHor() >> 2) + (mv->getVer() >> 2) * refStride;
    Pel *ref = refPic->getLumaAddr(cu->getAddr(), cu->getZorderIdxInCU() + partAddr) + refOffset;

    Int dstStride = dstPic->getStride();
    Pel *dst = dstPic->getLumaAddr(partAddr);

    Int xFrac = mv->getHor() & 0x3;
    Int yFrac = mv->getVer() & 0x3;

    if (yFrac == 0)
    {
        m_if.filterHorLuma(ref, refStride, dst, dstStride, width, height, xFrac, !bi);
    }
    else if (xFrac == 0)
    {
        m_if.filterVerLuma(ref, refStride, dst, dstStride, width, height, yFrac, true, !bi);
    }
    else
    {
        Int tmpStride = m_filteredBlockTmp[0].getStride();
        Short *tmp = m_filteredBlockTmp[0].getLumaAddr();

        Int filterSize = NTAPS_LUMA;
        Int halfFilterSize = (filterSize >> 1);

        m_if.filterHorLuma(ref - (halfFilterSize - 1) * refStride, refStride, tmp, tmpStride, width, height + filterSize - 1, xFrac, false);
        m_if.filterVerLuma(tmp + (halfFilterSize - 1) * tmpStride, tmpStride, dst, dstStride, width, height, yFrac, false, !bi);
    }
}

/**
 * \brief Generate motion-compensated chroma block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */
Void TComPrediction::xPredInterChromaBlk(TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi)
{
    Int refStride = refPic->getCStride();
    Int dstStride = dstPic->getCStride();

    Int refOffset = (mv->getHor() >> 3) + (mv->getVer() >> 3) * refStride;

    Pel *refCb = refPic->getCbAddr(cu->getAddr(), cu->getZorderIdxInCU() + partAddr) + refOffset;
    Pel *refCr = refPic->getCrAddr(cu->getAddr(), cu->getZorderIdxInCU() + partAddr) + refOffset;

    Pel *dstCb = dstPic->getCbAddr(partAddr);
    Pel *dstCr = dstPic->getCrAddr(partAddr);

    Int xFrac = mv->getHor() & 0x7;
    Int yFrac = mv->getVer() & 0x7;
    UInt cxWidth = width >> 1;
    UInt cxHeight = height >> 1;

    Int extStride = m_filteredBlockTmp[0].getStride();
    Short *extY = m_filteredBlockTmp[0].getLumaAddr();

    Int filterSize = NTAPS_CHROMA;

    Int halfFilterSize = (filterSize >> 1);

    if (yFrac == 0)
    {
        m_if.filterHorChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, xFrac, !bi);
        m_if.filterHorChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, xFrac, !bi);
    }
    else if (xFrac == 0)
    {
        m_if.filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, !bi);
        m_if.filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, !bi);
    }
    else
    {
        m_if.filterHorChroma(refCb - (halfFilterSize - 1) * refStride, refStride, extY, extStride, cxWidth, cxHeight + filterSize - 1, xFrac, false);
        m_if.filterVerChroma(extY + (halfFilterSize - 1) * extStride, extStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, false, !bi);

        m_if.filterHorChroma(refCr - (halfFilterSize - 1) * refStride, refStride, extY, extStride, cxWidth, cxHeight + filterSize - 1, xFrac, false);
        m_if.filterVerChroma(extY + (halfFilterSize - 1) * extStride, extStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, false, !bi);
    }
}

Void TComPrediction::xWeightedAverage(TComYuv *pcYuvSrc0, TComYuv *pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv *&rpcYuvDst)
{
    if (iRefIdx0 >= 0 && iRefIdx1 >= 0)
    {
        rpcYuvDst->addAvg(pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight);
    }
    else if (iRefIdx0 >= 0 && iRefIdx1 < 0)
    {
        pcYuvSrc0->copyPartToPartYuv(rpcYuvDst, uiPartIdx, iWidth, iHeight);
    }
    else if (iRefIdx0 < 0 && iRefIdx1 >= 0)
    {
        pcYuvSrc1->copyPartToPartYuv(rpcYuvDst, uiPartIdx, iWidth, iHeight);
    }
}

// AMVP
Void TComPrediction::getMvPredAMVP(TComDataCU *pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv &rcMvPred)
{
    AMVPInfo *pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
    if (pcAMVPInfo->iN <= 1)
    {
        rcMvPred = pcAMVPInfo->m_acMvCand[0];

        pcCU->setMVPIdxSubParts(0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts(pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
        return;
    }

    assert(pcCU->getMVPIdx(eRefPicList, uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList, uiPartAddr)];
    return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
Void TComPrediction::xPredIntraPlanar(Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height)
{
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    UInt blkSize = width;
    UInt offset2D = width;
    UInt shift1D = g_aucConvertToBit[width] + 2;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    for (k = 0; k < blkSize + 1; k++)
    {
        topRow[k] = pSrc[k - srcStride];
        leftColumn[k] = pSrc[k * srcStride - 1];
    }

    // Prepare intermediate variables used in interpolation
    bottomLeft = leftColumn[blkSize];
    topRight = topRow[blkSize];
    for (k = 0; k < blkSize; k++)
    {
        bottomRow[k] = bottomLeft - topRow[k];
        rightColumn[k] = topRight - leftColumn[k];
        topRow[k] <<= shift1D;
        leftColumn[k] <<= shift1D;
    }

    // Generate prediction signal
    for (k = 0; k < blkSize; k++)
    {
        horPred = leftColumn[k] + offset2D;
        for (l = 0; l < blkSize; l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k * dstStride + l] = ((horPred + topRow[l]) >> shift2D);
        }
    }
}
Void TComPrediction::xPredIntraPlanarLP(Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt uiPredDstSize)
{
    assert(width == height);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    // UInt blkSize = width;
    // UInt offset2D = width;
    UInt blkSize = uiPredDstSize;
    UInt offset2D = uiPredDstSize;
    // UInt shift1D = g_aucConvertToBit[width] + 2;
    // UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    for (k = 0; k < blkSize + 1; k++)
    {
        topRow[k] = pSrc[k - srcStride];
        leftColumn[k] = pSrc[k * srcStride - 1];
    }

    // Prepare intermediate variables used in interpolation
    bottomLeft = leftColumn[blkSize];
    topRight = topRow[blkSize];
    for (k = 0; k < blkSize; k++)
    {
        bottomRow[k] = bottomLeft - topRow[k];
        rightColumn[k] = topRight - leftColumn[k];
        topRow[k] = topRow[k] * uiPredDstSize;
        leftColumn[k] = leftColumn[k] * uiPredDstSize;
        // topRow[k] <<= shift1D;
        // leftColumn[k] <<= shift1D;
    }

    horPred = leftColumn[0] + offset2D;
    for (l = 0; l < blkSize; l++)
    {
        horPred += rightColumn[0];
        topRow[l] += bottomRow[l];
        rpDst[l] = ((horPred + topRow[l]) / (2 * uiPredDstSize));
    }
    for (k = 1; k < blkSize; k++)
    {
        horPred = leftColumn[k] + offset2D;
        horPred += rightColumn[k];
        topRow[0] += bottomRow[0];
        rpDst[k * dstStride] = ((horPred + topRow[0]) / (2 * uiPredDstSize));
    }
    // Generate prediction signal
    // for (k = 0; k < blkSize; k++)
    // {
    //     horPred = leftColumn[k] + offset2D;
    //     for (l = 0; l < blkSize; l++)
    //     {
    //         horPred += rightColumn[k];
    //         topRow[l] += bottomRow[l];
    //         rpDst[k * dstStride + l] = ((horPred + topRow[l]) >> shift2D);
    //     }
    // }
}

Void TComPrediction::xPredIntraPlanarLPnew(Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt uiPredDstSize)
{
    enum PlanarType
    {
        HEVC,
        V_Planar,
        H_Planar,
        HV_Planar,
        Planar_3p,
        SAP_E,
        Tendency,
        Tendency_C,
        Weight121,
        SAP_E_LP,
    };
    PlanarType SelectedType = SAP_E;

    assert(width == height);
    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    UInt blkSize = uiPredDstSize;
    UInt offset2D = uiPredDstSize;
    // Get left and above reference column and row
    for (k = 0; k < blkSize + 1; k++)
    {
        topRow[k] = pSrc[k - srcStride];
        leftColumn[k] = pSrc[k * srcStride - 1];
    }
    // Prepare intermediate variables used in interpolation
    bottomLeft = leftColumn[blkSize];
    topRight = topRow[blkSize];

    switch (SelectedType)
    {
    case HEVC:
    {
        for (k = 0; k < blkSize; k++)
        {
            bottomRow[k] = bottomLeft - topRow[k];
            rightColumn[k] = topRight - leftColumn[k];
            topRow[k] = topRow[k] * uiPredDstSize;
            leftColumn[k] = leftColumn[k] * uiPredDstSize;
            // topRow[k] <<= shift1D;
            // leftColumn[k] <<= shift1D;
        }
        horPred = leftColumn[0] + offset2D;
        for (l = 0; l < blkSize; l++)
        {
            horPred += rightColumn[0];
            topRow[l] += bottomRow[l];
            rpDst[l] = ((horPred + topRow[l] + uiPredDstSize) / (2 * uiPredDstSize));
        }
        for (k = 1; k < blkSize; k++)
        {
            horPred = leftColumn[k] + offset2D;
            horPred += rightColumn[k];
            topRow[0] += bottomRow[0];
            rpDst[k * dstStride] = ((horPred + topRow[0] + uiPredDstSize) / (2 * uiPredDstSize));
        }
        break;
    }
    case V_Planar:
    {
        for (l = 0; l < blkSize; l++)
        {
            rpDst[l] = (topRow[l] * uiPredDstSize + bottomLeft * 1) / (uiPredDstSize + 1);
        }
        for (k = 1; k < blkSize; k++)
        {
            rpDst[k * dstStride] = ((topRow[0] * (uiPredDstSize - k) + bottomLeft * (k + 1)) / (uiPredDstSize + 1));
        }
        break;
    }
    case H_Planar:
    {
        for (l = 0; l < blkSize; l++)
        {
            rpDst[l] = (leftColumn[0] * (uiPredDstSize - l) + topRight * (l + 1)) / (uiPredDstSize + 1);
        }
        for (k = 1; k < blkSize; k++)
        {
            rpDst[k * dstStride] = ((leftColumn[k] * (uiPredDstSize) + topRight * 1)) / (uiPredDstSize + 1);
        }
        break;
    }
    case HV_Planar:
    {
        for (l = 0; l < blkSize; l++)
        {
            rpDst[l] = (leftColumn[0] * (uiPredDstSize - l) + topRight * (l + 1)) / (uiPredDstSize + 1);
        }
        for (k = 1; k < blkSize; k++)
        {
            rpDst[k * dstStride] = (topRow[0] * (uiPredDstSize - k) + bottomLeft * (k + 1)) / (uiPredDstSize + 1);
        }
        break;
    }
    case Planar_3p:
    {
        for (l = 0; l < blkSize; l++)
        {
            rpDst[l] = (leftColumn[0] * (uiPredDstSize - l) + topRight * (l + 1)) / (uiPredDstSize + 1);
            rpDst[l] = (rpDst[l] + topRow[l] + 1) / 2;
        }
        for (k = 1; k < blkSize; k++)
        {
            rpDst[k * dstStride] = (topRow[0] * (uiPredDstSize - k) + bottomLeft * (k + 1)) / (uiPredDstSize + 1);
            rpDst[k * dstStride] = (rpDst[k * dstStride] + leftColumn[k] + 1) / 2;
        }
        break;
    }
    case SAP_E:
    {
        for (l = 0; l < blkSize; l++)
        {
            Pel left = pSrc[l - 1];
            Pel top = pSrc[l - srcStride];
            Pel lefttop = pSrc[l - srcStride - 1];
            if (lefttop >= max(left, top))
            {
                rpDst[l] = min(left, top);
            }
            else if (lefttop <= min(left, top))
            {
                rpDst[l] = max(left, top);
            }
            else
            {
                rpDst[l] = left + top - lefttop;
            }
        }
        for (k = 1; k < blkSize; k++)
        {
            Pel left = pSrc[k * srcStride - 1];
            Pel top = pSrc[(k - 1) * srcStride];
            Pel lefttop = pSrc[(k - 1) * srcStride - 1];
            if (lefttop >= max(left, top))
            {
                rpDst[k * dstStride] = min(left, top);
            }
            else if (lefttop <= min(left, top))
            {
                rpDst[k * dstStride] = max(left, top);
            }
            else
            {
                rpDst[k * dstStride] = left + top - lefttop;
            }
        }
        break;
    }
    case Tendency:
    {
        for (l = 0; l < blkSize; l++)
        {
            Pel far = (l == 0) ? pSrc[-1] : pSrc[l - srcStride - 2];
            Pel mid = pSrc[l - srcStride - 1];
            Pel near = pSrc[l - srcStride];
            if ((far >= mid && mid >= near) || (far <= mid && mid <= near))
            {
                rpDst[l] = near;
            }
            else
            {
                rpDst[l] = ((mid - far + mid) + near) / 2;
            }
        }
        for (k = 1; k < blkSize; k++)
        {
            Pel far = pSrc[(k - 2) * srcStride - 1];
            Pel mid = pSrc[(k - 1) * srcStride - 1];
            Pel near = pSrc[k * srcStride - 1];
            if ((far >= mid && mid >= near) || (far <= mid && mid <= near))
            {
                rpDst[k * dstStride] = near;
            }
            else
            {
                rpDst[k * dstStride] = ((mid - far + mid) + near) / 2;
            }
        }
        break;
    }
    case Tendency_C:
    {
        for (l = 0; l < blkSize; l++)
        {
            Pel far1 = pSrc[l - srcStride - 1];
            Pel near = pSrc[l - srcStride];
            Pel far2 = pSrc[l - srcStride + 1];
            if ((far1 >= near && near >= far2) || (far1 <= near && near <= far2))
            {
                rpDst[l] = near;
            }
            else
            {
                rpDst[l] = (far1 + far2 + near * 2) / 4;
            }

            // else if (abs(far1 - near) > abs(far2 - near))
            // {
            //     rpDst[l] = far2;
            // }
            // else
            // {
            //     rpDst[l] = far1;
            // }
        }
        for (k = 1; k < blkSize; k++)
        {
            Pel far1 = pSrc[(k - 1) * srcStride - 1];
            Pel near = pSrc[k * srcStride - 1];
            Pel far2 = pSrc[(k + 1) * srcStride - 1];
            if ((far1 >= near && near >= far2) || (far1 <= near && near <= far2))
            {
                rpDst[k * dstStride] = near;
            }
            else
            {
                rpDst[k * dstStride] = (far1 + far2 + near * 2) / 4;
            }

            // else if (abs(far1 - near) > abs(far2 - near))
            // {
            //     rpDst[k * dstStride] = far2;
            // }
            // else
            // {
            //     rpDst[k * dstStride] = far1;
            // }
        }
        break;
    }
    case Weight121:
    {
        for (l = 0; l < blkSize; l++)
        {
            Pel far1 = pSrc[l - srcStride - 1];
            Pel near = pSrc[l - srcStride];
            Pel far2 = pSrc[l - srcStride + 1];
            rpDst[l] = (far1 + far2 + near * 2) / 4;
        }
        for (k = 1; k < blkSize; k++)
        {
            Pel far1 = pSrc[(k - 1) * srcStride - 1];
            Pel near = pSrc[k * srcStride - 1];
            Pel far2 = pSrc[(k + 1) * srcStride - 1];
            rpDst[k * dstStride] = (far1 + far2 + near * 2) / 4;
        }
        break;
    }
    case SAP_E_LP:
    {
        for (l = 0; l < blkSize; l++)
        {
            Pel mid = pSrc[l - srcStride - 1];
            Pel near = pSrc[l - srcStride];
            Pel far = (l == 0) ? pSrc[-1] : pSrc[l - srcStride - 2];
            if (far > max(mid, near))
            {
                rpDst[l] = min(mid, near);
            }
            else if (far < min(mid, near))
            {
                rpDst[l] = max(mid, near);
            }
            else
            {
                rpDst[l] = mid + near - far;
            }
        }
        for (k = 1; k < blkSize; k++)
        {
            Pel mid = pSrc[k * srcStride - 1];
            Pel near = pSrc[(k - 1) * srcStride];
            Pel far = pSrc[(k - 1) * srcStride - 1];
            if (far > max(mid, near))
            {
                rpDst[k * dstStride] = min(mid, near);
            }
            else if (far < min(mid, near))
            {
                rpDst[k * dstStride] = max(mid, near);
            }
            else
            {
                rpDst[k * dstStride] = mid + near - far;
            }
        }
        break;
    }
    default:
        assert(0);
        break;
    }
}
Void TComPrediction::xPredIntraPlanar3x3(Int *pSrc, Int srcStride, Pel *rpDst, Int dstStride, UInt width, UInt height, UInt uiPredDstSize)
{
    assert(width == height);
    assert(uiPredDstSize == 3);

    Int k, l, bottomLeft, topRight;
    Int horPred;
    Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
    // UInt blkSize = width;
    // UInt offset2D = width;
    UInt blkSize = uiPredDstSize;
    UInt offset2D = uiPredDstSize;
    // UInt shift1D = g_aucConvertToBit[width] + 2;
    // UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    for (k = 0; k < blkSize + 1; k++)
    {
        topRow[k] = pSrc[k - srcStride];
        leftColumn[k] = pSrc[k * srcStride - 1];
    }

    // Prepare intermediate variables used in interpolation
    bottomLeft = leftColumn[blkSize];
    topRight = topRow[blkSize];
    for (k = 0; k < blkSize; k++)
    {
        bottomRow[k] = bottomLeft - topRow[k];
        rightColumn[k] = topRight - leftColumn[k];
        topRow[k] = topRow[k] * uiPredDstSize;
        leftColumn[k] = leftColumn[k] * uiPredDstSize;
        // topRow[k] <<= shift1D;
        // leftColumn[k] <<= shift1D;
    }

    // horPred = leftColumn[0] + offset2D;
    // for (l = 0; l < blkSize; l++)
    // {
    //     horPred += rightColumn[0];
    //     topRow[l] += bottomRow[l];
    //     rpDst[l] = ((horPred + topRow[l]) / (2 * uiPredDstSize));
    // }
    // for (k = 1; k < blkSize; k++)
    // {
    //     horPred = leftColumn[k] + offset2D;
    //     horPred += rightColumn[k];
    //     topRow[0] += bottomRow[0];
    //     rpDst[k * dstStride] = ((horPred + topRow[0]) / (2 * uiPredDstSize));
    // }
    // Generate prediction signal
    for (k = 0; k < blkSize; k++)
    {
        horPred = leftColumn[k] + offset2D;
        for (l = 0; l < blkSize; l++)
        {
            horPred += rightColumn[k];
            topRow[l] += bottomRow[l];
            rpDst[k * dstStride + l] = ((horPred + topRow[l]) / (2 * uiPredDstSize));
        }
    }
}
/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering(Int *pSrc, Int iSrcStride, Pel *rpDst, Int iDstStride, Int iWidth, Int iHeight)
{
    Pel *pDst = rpDst;
    Int x, y, iDstStride2, iSrcStride2;

    // boundary pixels processing
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    for (x = 1; x < iWidth; x++)
    {
        pDst[x] = (Pel)((pSrc[x - iSrcStride] + 3 * pDst[x] + 2) >> 2);
    }

    for (y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride - 1; y < iHeight; y++, iDstStride2 += iDstStride, iSrcStride2 += iSrcStride)
    {
        pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }

    return;
}
//! \}
