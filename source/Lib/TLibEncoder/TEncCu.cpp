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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"

#include <cmath>
#include <algorithm>
using namespace std;

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight)
{
    Int i;

    m_uhTotalDepth = uhTotalDepth + 1;
    m_ppcBestCU = new TComDataCU *[m_uhTotalDepth - 1];
    m_ppcTempCU = new TComDataCU *[m_uhTotalDepth - 1];

    m_ppcPredYuvBest = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcResiYuvBest = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcRecoYuvBest = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcPredYuvTemp = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcResiYuvTemp = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcRecoYuvTemp = new TComYuv *[m_uhTotalDepth - 1];
    m_ppcOrigYuv = new TComYuv *[m_uhTotalDepth - 1];

    UInt uiNumPartitions;
    for (i = 0; i < m_uhTotalDepth - 1; i++)
    {
        uiNumPartitions = 1 << ((m_uhTotalDepth - i - 1) << 1);
        UInt uiWidth = uiMaxWidth >> i;
        UInt uiHeight = uiMaxHeight >> i;

        m_ppcBestCU[i] = new TComDataCU;
        m_ppcBestCU[i]->create(uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1));
        m_ppcTempCU[i] = new TComDataCU;
        m_ppcTempCU[i]->create(uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1));

        m_ppcPredYuvBest[i] = new TComYuv;
        m_ppcPredYuvBest[i]->create(uiWidth, uiHeight);
        m_ppcResiYuvBest[i] = new TComYuv;
        m_ppcResiYuvBest[i]->create(uiWidth, uiHeight);
        m_ppcRecoYuvBest[i] = new TComYuv;
        m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight);

        m_ppcPredYuvTemp[i] = new TComYuv;
        m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight);
        m_ppcResiYuvTemp[i] = new TComYuv;
        m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight);
        m_ppcRecoYuvTemp[i] = new TComYuv;
        m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight);

        m_ppcOrigYuv[i] = new TComYuv;
        m_ppcOrigYuv[i]->create(uiWidth, uiHeight);
    }

    m_bEncodeDQP = false;

    // initialize partition order.
    UInt *piTmp = &g_auiZscanToRaster[0];
    initZscanToRaster(m_uhTotalDepth, 1, 0, piTmp);
    initRasterToZscan(uiMaxWidth, uiMaxHeight, m_uhTotalDepth);

    // initialize conversion matrix from partition index to pel
    initRasterToPelXY(uiMaxWidth, uiMaxHeight, m_uhTotalDepth);
}

Void TEncCu::destroy()
{
    Int i;

    for (i = 0; i < m_uhTotalDepth - 1; i++)
    {
        if (m_ppcBestCU[i])
        {
            m_ppcBestCU[i]->destroy();
            delete m_ppcBestCU[i];
            m_ppcBestCU[i] = NULL;
        }
        if (m_ppcTempCU[i])
        {
            m_ppcTempCU[i]->destroy();
            delete m_ppcTempCU[i];
            m_ppcTempCU[i] = NULL;
        }
        if (m_ppcPredYuvBest[i])
        {
            m_ppcPredYuvBest[i]->destroy();
            delete m_ppcPredYuvBest[i];
            m_ppcPredYuvBest[i] = NULL;
        }
        if (m_ppcResiYuvBest[i])
        {
            m_ppcResiYuvBest[i]->destroy();
            delete m_ppcResiYuvBest[i];
            m_ppcResiYuvBest[i] = NULL;
        }
        if (m_ppcRecoYuvBest[i])
        {
            m_ppcRecoYuvBest[i]->destroy();
            delete m_ppcRecoYuvBest[i];
            m_ppcRecoYuvBest[i] = NULL;
        }
        // TODO: 小心内存问题
        // if (m_ppcPredYuvTemp[i])
        // {
        //     m_ppcPredYuvTemp[i]->destroy();
        //     delete m_ppcPredYuvTemp[i];
        //     m_ppcPredYuvTemp[i] = NULL;
        // }
        if (m_ppcResiYuvTemp[i])
        {
            m_ppcResiYuvTemp[i]->destroy();
            delete m_ppcResiYuvTemp[i];
            m_ppcResiYuvTemp[i] = NULL;
        }
        // TODO: 小心内存问题
        // if (m_ppcRecoYuvTemp[i])
        // {
        //     m_ppcRecoYuvTemp[i]->destroy();
        //     delete m_ppcRecoYuvTemp[i];
        //     m_ppcRecoYuvTemp[i] = NULL;
        // }
        if (m_ppcOrigYuv[i])
        {
            m_ppcOrigYuv[i]->destroy();
            delete m_ppcOrigYuv[i];
            m_ppcOrigYuv[i] = NULL;
        }
    }
    if (m_ppcBestCU)
    {
        delete[] m_ppcBestCU;
        m_ppcBestCU = NULL;
    }
    if (m_ppcTempCU)
    {
        delete[] m_ppcTempCU;
        m_ppcTempCU = NULL;
    }

    if (m_ppcPredYuvBest)
    {
        delete[] m_ppcPredYuvBest;
        m_ppcPredYuvBest = NULL;
    }
    if (m_ppcResiYuvBest)
    {
        delete[] m_ppcResiYuvBest;
        m_ppcResiYuvBest = NULL;
    }
    if (m_ppcRecoYuvBest)
    {
        delete[] m_ppcRecoYuvBest;
        m_ppcRecoYuvBest = NULL;
    }
    if (m_ppcPredYuvTemp)
    {
        delete[] m_ppcPredYuvTemp;
        m_ppcPredYuvTemp = NULL;
    }
    if (m_ppcResiYuvTemp)
    {
        delete[] m_ppcResiYuvTemp;
        m_ppcResiYuvTemp = NULL;
    }
    if (m_ppcRecoYuvTemp)
    {
        delete[] m_ppcRecoYuvTemp;
        m_ppcRecoYuvTemp = NULL;
    }
    if (m_ppcOrigYuv)
    {
        delete[] m_ppcOrigYuv;
        m_ppcOrigYuv = NULL;
    }
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init(TEncTop *pcEncTop)
{
    m_pcEncCfg = pcEncTop;
    m_pcPredSearch = pcEncTop->getPredSearch();
    m_pcTrQuant = pcEncTop->getTrQuant();
    m_pcBitCounter = pcEncTop->getBitCounter();
    m_pcRdCost = pcEncTop->getRdCost();

    m_pcEntropyCoder = pcEncTop->getEntropyCoder();
    m_pcCavlcCoder = pcEncTop->getCavlcCoder();
    m_pcSbacCoder = pcEncTop->getSbacCoder();
    m_pcBinCABAC = pcEncTop->getBinCABAC();

    m_pppcRDSbacCoder = pcEncTop->getRDSbacCoder();
    m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();

    m_pcRateCtrl = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU(TComDataCU *&rpcCU)
{
    // initialize CU data
    m_ppcBestCU[0]->initCU(rpcCU->getPic(), rpcCU->getAddr());
    m_ppcTempCU[0]->initCU(rpcCU->getPic(), rpcCU->getAddr());

    // analysis of CU
    // 编码核心入口
    // 相当于 CU 的根节点
    xCompressCU(m_ppcBestCU[0], m_ppcTempCU[0], 0);

#if ADAPTIVE_QP_SELECTION
    if (m_pcEncCfg->getUseAdaptQpSelect())
    {
        if (rpcCU->getSlice()->getSliceType() != I_SLICE) //IIII
        {
            xLcuCollectARLStats(rpcCU);
        }
    }
#endif
}
/** \param  pcCU  pointer of CU data class
 */
Void TEncCu::encodeCU(TComDataCU *pcCU)
{
    if (pcCU->getSlice()->getPPS()->getUseDQP())
    {
        setdQPFlag(true);
    }

    // Encode CU data
    xEncodeCU(pcCU, 0, 0);
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void 
*/
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP(TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP(TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
    if (rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
    {
        bTestAMP_Hor = true;
    }
    else if (rpcBestCU->getPartitionSize(0) == SIZE_Nx2N)
    {
        bTestAMP_Ver = true;
    }
    else if (rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false)
    {
        bTestAMP_Hor = true;
        bTestAMP_Ver = true;
    }

#if AMP_MRG
    //! Utilizing the partition size of parent PU
    if (eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N)
    {
        bTestMergeAMP_Hor = true;
        bTestMergeAMP_Ver = true;
    }

    if (eParentPartSize == SIZE_NONE) //! if parent is intra
    {
        if (rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
        {
            bTestMergeAMP_Hor = true;
        }
        else if (rpcBestCU->getPartitionSize(0) == SIZE_Nx2N)
        {
            bTestMergeAMP_Ver = true;
        }
    }

    if (rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false)
    {
        bTestMergeAMP_Hor = true;
        bTestMergeAMP_Ver = true;
    }

    if (rpcBestCU->getWidth(0) == 64)
    {
        bTestAMP_Hor = false;
        bTestAMP_Ver = false;
    }
#else
    //! Utilizing the partition size of parent PU
    if (eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N)
    {
        bTestAMP_Hor = true;
        bTestAMP_Ver = true;
    }

    if (eParentPartSize == SIZE_2Nx2N)
    {
        bTestAMP_Hor = false;
        bTestAMP_Ver = false;
    }
#endif
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/
// 非对称运动划分 与帧内无关
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth, PartSize eParentPartSize)
#else
Void TEncCu::xCompressCU(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
#endif
{
    TComPic *pcPic = rpcBestCU->getPic();

    // get Original YUV data from picture
    m_ppcOrigYuv[uiDepth]->copyFromPicYuv(pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU());

    // variable for Early CU determination
    Bool bSubBranch = true;

    // variable for Cbf fast mode PU decision
    Bool doNotBlockPu = true;
    Bool earlyDetectionSkipMode = false;

    Bool bBoundary = false;
    // X 水平坐标 Y 垂直坐标
    UInt uiLPelX = rpcBestCU->getCUPelX();
    UInt uiRPelX = uiLPelX + rpcBestCU->getWidth(0) - 1;
    UInt uiTPelY = rpcBestCU->getCUPelY();
    UInt uiBPelY = uiTPelY + rpcBestCU->getHeight(0) - 1;

    Int iBaseQP = xComputeQP(rpcBestCU, uiDepth);
    Int iMinQP;
    Int iMaxQP;
    Bool isAddLowestQP = false;

    if ((g_uiMaxCUWidth >> uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize())
    {
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3(-rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP - idQP);
        iMaxQP = Clip3(-rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP + idQP);
    }
    else
    {
        iMinQP = rpcTempCU->getQP(0);
        iMaxQP = rpcTempCU->getQP(0);
    }

    if (m_pcEncCfg->getUseRateCtrl())
    {
        iMinQP = m_pcRateCtrl->getRCQP();
        iMaxQP = m_pcRateCtrl->getRCQP();
    }

    // transquant-bypass (TQB) processing loop variable initialisation ---

    const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

    if ((rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()))
    {
        isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
        iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
        if (m_pcEncCfg->getCUTransquantBypassFlagForceValue())
        {
            iMaxQP = iMinQP;
        }
    }

    // If slice start or slice end is within this cu...
    TComSlice *pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcTempCU->getSCUAddr() && pcSlice->getSliceSegmentCurStartCUAddr() < rpcTempCU->getSCUAddr() + rpcTempCU->getTotalNumPart();
    Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr() > rpcTempCU->getSCUAddr() && pcSlice->getSliceSegmentCurEndCUAddr() < rpcTempCU->getSCUAddr() + rpcTempCU->getTotalNumPart());
    Bool bInsidePicture = (uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples()) && (uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples());
    // We need to split, so don't try these modes.
    if (!bSliceEnd && !bSliceStart && bInsidePicture)
    {
        for (Int iQP = iMinQP; iQP <= iMaxQP; iQP++)
        {
            const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

            if (bIsLosslessMode)
            {
                iQP = lowestQP;
            }

            rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

            // do inter modes, SKIP and 2Nx2N
            if (rpcBestCU->getSlice()->getSliceType() != I_SLICE)
            {
                // 2Nx2N
                if (m_pcEncCfg->getUseEarlySkipDetection())
                {
                    xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2Nx2N);
                    rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode); //by Competition for inter_2Nx2N
                }
                // SKIP
                xCheckRDCostMerge2Nx2N(rpcBestCU, rpcTempCU, &earlyDetectionSkipMode); //by Merge for inter_2Nx2N
                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

                if (!m_pcEncCfg->getUseEarlySkipDetection())
                {
                    // 2Nx2N, NxN
                    xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2Nx2N);
                    rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                    if (m_pcEncCfg->getUseCbfFastMode())
                    {
                        doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                    }
                }
            }

            if (bIsLosslessMode)
            {
                iQP = iMinQP;
            }
        }

        if (!earlyDetectionSkipMode)
        {
            for (Int iQP = iMinQP; iQP <= iMaxQP; iQP++)
            {
                const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

                if (bIsLosslessMode)
                {
                    iQP = lowestQP;
                }
                // 既然不可能进入 inter modes, 自然不需要这一步初始化. 前面已经做过一次
                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

                // do inter modes, NxN, 2NxN, and Nx2N
                if (rpcBestCU->getSlice()->getSliceType() != I_SLICE)
                {
                    // 2Nx2N, NxN
                    if (!((rpcBestCU->getWidth(0) == 8) && (rpcBestCU->getHeight(0) == 8)))
                    {
                        if (uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
                        {
                            xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_NxN);
                            rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        }
                    }

                    // 2NxN, Nx2N
                    if (doNotBlockPu)
                    {
                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_Nx2N);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N)
                        {
                            doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                        }
                    }
                    if (doNotBlockPu)
                    {
                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxN);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
                        {
                            doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                        }
                    }

#if 1
                    //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
                    if (pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth))
                    {
#if AMP_ENC_SPEEDUP
                        Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
                        Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

                        deriveTestModeAMP(rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
                        deriveTestModeAMP(rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

                        //! Do horizontal AMP
                        if (bTestAMP_Hor)
                        {
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnU);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnD);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                        }
#if AMP_MRG
                        else if (bTestMergeAMP_Hor)
                        {
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnU, true);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnD, true);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                        }
#endif

                        //! Do horizontal AMP
                        if (bTestAMP_Ver)
                        {
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nLx2N);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nRx2N);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                            }
                        }
#if AMP_MRG
                        else if (bTestMergeAMP_Ver)
                        {
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nLx2N, true);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                                if (m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N)
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf(0) != 0;
                                }
                            }
                            if (doNotBlockPu)
                            {
                                xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nRx2N, true);
                                rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                            }
                        }
#endif

#else
                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnU);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_2NxnD);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nLx2N);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

                        xCheckRDCostInter(rpcBestCU, rpcTempCU, SIZE_nRx2N);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

#endif
                    }
#endif
                }

                // do normal intra modes
                // speedup for inter frames
                if (rpcBestCU->getSlice()->getSliceType() == I_SLICE ||
                    rpcBestCU->getCbf(0, TEXT_LUMA) != 0 ||
                    rpcBestCU->getCbf(0, TEXT_CHROMA_U) != 0 ||
                    rpcBestCU->getCbf(0, TEXT_CHROMA_V) != 0) // avoid very complex intra if it is unlikely
                {
                    // 计算当前层该块的 RD
                    xCheckRDCostIntra(rpcBestCU, rpcTempCU, SIZE_2Nx2N);
                    rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                    if (uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth)
                    { // 8x8 层 CU 继续往下分 4 个 PU 计算
                        if (rpcTempCU->getWidth(0) > (1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize()))
                        {
                            xCheckRDCostIntra(rpcBestCU, rpcTempCU, SIZE_NxN);
                            rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                        }
                    }
                }

                // 不尝试 PCM
                // test PCM
                if (pcPic->getSlice(0)->getSPS()->getUsePCM() && rpcTempCU->getWidth(0) <= (1 << pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize()) && rpcTempCU->getWidth(0) >= (1 << pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()))
                {
                    UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * rpcBestCU->getWidth(0) * rpcBestCU->getHeight(0) / 2;
                    UInt uiBestBits = rpcBestCU->getTotalBits();
                    if ((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
                    {
                        xCheckIntraPCM(rpcBestCU, rpcTempCU);
                        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);
                    }
                }
                if (bIsLosslessMode)
                {
                    iQP = iMinQP;
                }
            }
        }

        m_pcEntropyCoder->resetBits();
        // m_pcEntropyCoder->encodeSplitFlag(rpcBestCU, 0, uiDepth, true);
        // rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        // rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
        // rpcBestCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion()); // 更新 BestCU 的代价
        // 针对 L 分块模式, 加上分块标志重新计算总代价. 因为 L 分块视为 split=1, 和传统一样在向上回归时计算 split 代价, 在向下计算时暂时不处理
        // Bool bCurrSplitFlag = rpcBestCU->getDepth(0) > uiDepth;
        // if (bCurrSplitFlag == true)
        // {
        //     rpcBestCU->getTotalCostnpLpart(0b0111) += m_pcEntropyCoder->getNumberOfWrittenBits();
        //     rpcBestCU->getTotalCostnpLpart(0b1011) += m_pcEntropyCoder->getNumberOfWrittenBits();
        //     rpcBestCU->getTotalCostnpLpart(0b1101) += m_pcEntropyCoder->getNumberOfWrittenBits();
        //     rpcBestCU->getTotalCostnpLpart(0b1110) += m_pcEntropyCoder->getNumberOfWrittenBits();
        // }

        // Early CU determination
        if (m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0))
        {
            bSubBranch = false;
        }
        else
        {
            bSubBranch = true;
        }
    }
    else if (!(bSliceEnd && bInsidePicture))
    {
        bBoundary = true;
    }

    // 项目中不考虑 PCM
    // copy orginal YUV samples to PCM buffer
    // if (rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
    // {
    //     xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
    // }
    if ((g_uiMaxCUWidth >> uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize())
    {
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3(-rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP - idQP);
        iMaxQP = Clip3(-rpcTempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP + idQP);
    }
    else if ((g_uiMaxCUWidth >> uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize())
    {
        iMinQP = iBaseQP;
        iMaxQP = iBaseQP;
    }
    else
    {
        Int iStartQP;
        if (pcPic->getCU(rpcTempCU->getAddr())->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
        {
            iStartQP = rpcTempCU->getQP(0);
        }
        else
        {
            UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
            iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
        }
        iMinQP = iStartQP;
        iMaxQP = iStartQP;
    }
    if (m_pcEncCfg->getUseRateCtrl())
    {
        iMinQP = m_pcRateCtrl->getRCQP();
        iMaxQP = m_pcRateCtrl->getRCQP();
    }

    if (m_pcEncCfg->getCUTransquantBypassFlagForceValue())
    {
        iMaxQP = iMinQP; // If all blocks are forced into using transquant bypass, do not loop here.
    }

    for (Int iQP = iMinQP; iQP <= iMaxQP; iQP++)
    {
        const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
        rpcTempCU->initEstData(uiDepth, iQP, bIsLosslessMode);

        // CU 继续分割
        // bSubBranch 标识是否设置继续分割, 如果提前结束会设为 False
        // uiDepth 反映最大分割深度的设置
        // further split
        if (bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth)
        {
            UChar uhNextDepth = uiDepth + 1;
            TComDataCU *pcSubBestPartCU = m_ppcBestCU[uhNextDepth];
            TComDataCU *pcSubTempPartCU = m_ppcTempCU[uhNextDepth];
            Double dCostPre;

            for (UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++)
            {
                pcSubBestPartCU->initSubCU(rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP); // clear sub partition datas or init.
                pcSubTempPartCU->initSubCU(rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP); // clear sub partition datas or init.

                Bool bInSlice = pcSubBestPartCU->getSCUAddr() + pcSubBestPartCU->getTotalNumPart() > pcSlice->getSliceSegmentCurStartCUAddr() && pcSubBestPartCU->getSCUAddr() < pcSlice->getSliceSegmentCurEndCUAddr();
                if (bInSlice && (pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples()))
                {
                    if (0 == uiPartUnitIdx) //initialize RD with previous depth buffer
                    {
                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
                    }
                    else
                    {
                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
                    }

#if AMP_ENC_SPEEDUP
                    if (rpcBestCU->isIntra(0))
                    {
                        xCompressCU(pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, SIZE_NONE);
                    }
                    else
                    {
                        xCompressCU(pcSubBestPartCU, pcSubTempPartCU, uhNextDepth, rpcBestCU->getPartitionSize(0));
                    }
#else
                    // 递归调用 xCompressCU 实现四叉树分割编码
                    xCompressCU(pcSubBestPartCU, pcSubTempPartCU, uhNextDepth);
#endif
                    // 4个划分的最优的信息的累加，以便和分割前的CU的最优的预测模式的RD-cost进行比较也就是m_ppcBestCU进行比较
                    rpcTempCU->copyPartFrom(pcSubBestPartCU, uiPartUnitIdx, uhNextDepth);
                    // TODO: 在这里记录/计算 8 16 层的 1/4 块状部分总 Cost, 后面上层的 BestCU L Cost 加上这里对应的 1/4 得到新分块方法的 Cost
                    switch (uiPartUnitIdx)
                    {
                    case 0:
                        rpcTempCU->dBestCostQuarPartLT = pcSubBestPartCU->getTotalBits();
                        // rpcTempCU->dBestCostQuarPartLT = rpcTempCU->getTotalBits();
                        // dCostPre = rpcTempCU->getTotalBits();
                        break;
                    case 1:
                        rpcTempCU->dBestCostQuarPartRT = pcSubBestPartCU->getTotalBits();
                        // rpcTempCU->dBestCostQuarPartRT = rpcTempCU->getTotalBits() - dCostPre;
                        // dCostPre = rpcTempCU->getTotalBits();
                        break;
                    case 2:
                        rpcTempCU->dBestCostQuarPartLB = pcSubBestPartCU->getTotalBits();
                        // rpcTempCU->dBestCostQuarPartLB = rpcTempCU->getTotalBits() - dCostPre;
                        // dCostPre = rpcTempCU->getTotalBits();
                        break;
                    case 3:
                        rpcTempCU->dBestCostQuarPartRB = pcSubBestPartCU->getTotalBits();
                        // rpcTempCU->dBestCostQuarPartRB = rpcTempCU->getTotalBits() - dCostPre;
                        // dCostPre = rpcTempCU->getTotalBits();
                        break;
                    default:
                        assert(0);
                    }
                    xCopyYuv2Tmp(pcSubBestPartCU->getTotalNumPart() * uiPartUnitIdx, uhNextDepth);
                }
                else if (bInSlice)
                {
                    pcSubBestPartCU->copyToPic(uhNextDepth);
                    rpcTempCU->copyPartFrom(pcSubBestPartCU, uiPartUnitIdx, uhNextDepth);
                }
            }

            if (!bBoundary)
            {
                m_pcEntropyCoder->resetBits();
                // 不处理 split 标志, 丢到 xCheckBestMode 里面模拟计算
                // m_pcEntropyCoder->encodeSplitFlag(rpcTempCU, 0, uiDepth, true);

                // rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
                // rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
            }
            rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion());

            // 不会进入该分支
            if ((g_uiMaxCUWidth >> uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
            {
                Bool hasResidual = false;
                for (UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx++)
                {
                    if ((pcPic->getCU(rpcTempCU->getAddr())->getSliceSegmentStartCU(uiBlkIdx + rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr()) &&
                        (rpcTempCU->getCbf(uiBlkIdx, TEXT_LUMA) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_U) || rpcTempCU->getCbf(uiBlkIdx, TEXT_CHROMA_V)))
                    {
                        hasResidual = true;
                        break;
                    }
                }

                UInt uiTargetPartIdx;
                if (pcPic->getCU(rpcTempCU->getAddr())->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr())
                {
                    uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
                }
                else
                {
                    uiTargetPartIdx = 0;
                }
                if (hasResidual)
                {
#if !RDO_WITHOUT_DQP_BITS
                    m_pcEntropyCoder->resetBits();
                    m_pcEntropyCoder->encodeQP(rpcTempCU, uiTargetPartIdx, false);
                    rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
                    rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
                    rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion());
#endif

                    Bool foundNonZeroCbf = false;
                    rpcTempCU->setQPSubCUs(rpcTempCU->getRefQP(uiTargetPartIdx), rpcTempCU, 0, uiDepth, foundNonZeroCbf);
                    assert(foundNonZeroCbf);
                }
                else
                {
                    rpcTempCU->setQPSubParts(rpcTempCU->getRefQP(uiTargetPartIdx), 0, uiDepth); // set QP to default QP
                }
            }

            m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

            Bool isEndOfSlice = rpcBestCU->getSlice()->getSliceMode() == FIXED_NUMBER_OF_BYTES && (rpcBestCU->getTotalBits() > rpcBestCU->getSlice()->getSliceArgument() << 3);
            Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES && (rpcBestCU->getTotalBits() > rpcBestCU->getSlice()->getSliceSegmentArgument() << 3);
            if (isEndOfSlice || isEndOfSliceSegment)
            {
                rpcBestCU->getTotalCost() = rpcTempCU->getTotalCost() + 1;
            }
            // xCheckBestMode 向上回归时判断 CU 分割前后哪个好 (8与16比 16与32比)
            xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth); // RD compare current larger prediction
        }                                                  // with sub partitioned prediction.
    }

    rpcBestCU->copyToPic(uiDepth); // Copy Best data to Picture for next partition prediction.

    xCopyYuv2Pic(rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY); // Copy Yuv data to picture Yuv
    if (bBoundary || (bSliceEnd && bInsidePicture))
    {
        return;
    }

    // Assert if Best prediction mode is NONE
    // Selected mode's RD-cost must be not MAX_DOUBLE.
    assert(rpcBestCU->getPartitionSize(0) != SIZE_NONE);
    assert(rpcBestCU->getPredictionMode(0) != MODE_NONE);
    assert(rpcBestCU->getTotalCost() != MAX_DOUBLE);
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::finishCU(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiDepth)
{
    TComPic *pcPic = pcCU->getPic();
    TComSlice *pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

    //Calculate end address
    UInt uiCUAddr = pcCU->getSCUAddr() + uiAbsPartIdx;

    UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr() - 1) % pcPic->getNumPartInCU();
    UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr() - 1) / pcPic->getNumPartInCU();
    UInt uiPosX = (uiExternalAddress % pcPic->getFrameWidthInCU()) * g_uiMaxCUWidth + g_auiRasterToPelX[g_auiZscanToRaster[uiInternalAddress]];
    UInt uiPosY = (uiExternalAddress / pcPic->getFrameWidthInCU()) * g_uiMaxCUHeight + g_auiRasterToPelY[g_auiZscanToRaster[uiInternalAddress]];
    UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
    UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
    while (uiPosX >= uiWidth || uiPosY >= uiHeight)
    {
        uiInternalAddress--;
        uiPosX = (uiExternalAddress % pcPic->getFrameWidthInCU()) * g_uiMaxCUWidth + g_auiRasterToPelX[g_auiZscanToRaster[uiInternalAddress]];
        uiPosY = (uiExternalAddress / pcPic->getFrameWidthInCU()) * g_uiMaxCUHeight + g_auiRasterToPelY[g_auiZscanToRaster[uiInternalAddress]];
    }
    uiInternalAddress++;
    if (uiInternalAddress == pcCU->getPic()->getNumPartInCU())
    {
        uiInternalAddress = 0;
        uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress) + 1);
    }
    UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress * pcPic->getNumPartInCU() + uiInternalAddress);

    // Encode slice finish
    Bool bTerminateSlice = false;
    if (uiCUAddr + (pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1)) == uiRealEndAddress)
    {
        bTerminateSlice = true;
    }
    UInt uiGranularityWidth = g_uiMaxCUWidth;
    uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdx]];
    uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdx]];
    Bool granularityBoundary = ((uiPosX + pcCU->getWidth(uiAbsPartIdx)) % uiGranularityWidth == 0 || (uiPosX + pcCU->getWidth(uiAbsPartIdx) == uiWidth)) && ((uiPosY + pcCU->getHeight(uiAbsPartIdx)) % uiGranularityWidth == 0 || (uiPosY + pcCU->getHeight(uiAbsPartIdx) == uiHeight));

    if (granularityBoundary)
    {
        // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
        if (!bTerminateSlice)
            m_pcEntropyCoder->encodeTerminatingBit(bTerminateSlice ? 1 : 0);
    }

    Int numberOfWrittenBits = 0;
    if (m_pcBitCounter)
    {
        numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    }

    // Calculate slice end IF this CU puts us over slice bit size.
    UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
    Int iGranularityEnd = ((pcCU->getSCUAddr() + uiAbsPartIdx) / iGranularitySize) * iGranularitySize;
    if (iGranularityEnd <= pcSlice->getSliceSegmentCurStartCUAddr())
    {
        iGranularityEnd += max(iGranularitySize, (pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1)));
    }
    // Set slice end parameter
    if (pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES && !pcSlice->getFinalized() && pcSlice->getSliceBits() + numberOfWrittenBits > pcSlice->getSliceArgument() << 3)
    {
        pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
        pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
        return;
    }
    // Set dependent slice end parameter
    if (pcSlice->getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES && !pcSlice->getFinalized() && pcSlice->getSliceSegmentBits() + numberOfWrittenBits > pcSlice->getSliceSegmentArgument() << 3)
    {
        pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
        return;
    }
    if (granularityBoundary)
    {
        pcSlice->setSliceBits((UInt)(pcSlice->getSliceBits() + numberOfWrittenBits));
        pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits() + numberOfWrittenBits);
        if (m_pcBitCounter)
        {
            m_pcEntropyCoder->resetBits();
        }
    }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP(TComDataCU *pcCU, UInt uiDepth)
{
    Int iBaseQp = pcCU->getSlice()->getSliceQp();
    Int iQpOffset = 0;
    if (m_pcEncCfg->getUseAdaptiveQP())
    {
        TEncPic *pcEPic = dynamic_cast<TEncPic *>(pcCU->getPic());
        UInt uiAQDepth = min(uiDepth, pcEPic->getMaxAQDepth() - 1);
        TEncPicQPAdaptationLayer *pcAQLayer = pcEPic->getAQLayer(uiAQDepth);
        UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
        UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
        UInt uiAQUStride = pcAQLayer->getAQPartStride();
        TEncQPAdaptationUnit *acAQU = pcAQLayer->getQPAdaptationUnit();

        Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange() / 6.0);
        Double dAvgAct = pcAQLayer->getAvgActivity();
        Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
        Double dNormAct = (dMaxQScale * dCUAct + dAvgAct) / (dCUAct + dMaxQScale * dAvgAct);
        Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
        iQpOffset = Int(floor(dQpOffset + 0.49999));
    }
    return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQp + iQpOffset);
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::xEncodeCU(TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiDepth)
{
    TComPic *pcPic = pcCU->getPic();

    Bool bBoundary = false;
    UInt uiLPelX = pcCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdx]];
    UInt uiRPelX = uiLPelX + (g_uiMaxCUWidth >> uiDepth) - 1;
    UInt uiTPelY = pcCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdx]];
    UInt uiBPelY = uiTPelY + (g_uiMaxCUHeight >> uiDepth) - 1;

    TComSlice *pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
    // If slice start is within this cu...
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx &&
                       pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx + (pcPic->getNumPartInCU() >> (uiDepth << 1));
    // We need to split, so don't try these modes.
    if (!bSliceStart && (uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
    {
        // （1）调用encodeSplitFlag对分割标志进行编码（最终调用TEncSbac::codeSplitFlag）
        m_pcEntropyCoder->encodeSplitFlag(pcCU, uiAbsPartIdx, uiDepth);
    }
    else
    {
        bBoundary = true;
    }

    if (((uiDepth < pcCU->getDepth(uiAbsPartIdx)) && (uiDepth < (g_uiMaxCUDepth - g_uiAddCUDepth))) || bBoundary)
    {
        UInt uiQNumParts = (pcPic->getNumPartInCU() >> (uiDepth << 1)) >> 2;
        if ((g_uiMaxCUWidth >> uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
        {
            setdQPFlag(true);
        }
        for (UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx += uiQNumParts)
        {
            uiLPelX = pcCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiAbsPartIdx]];
            uiTPelY = pcCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiAbsPartIdx]];
            Bool bInSlice = pcCU->getSCUAddr() + uiAbsPartIdx + uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && pcCU->getSCUAddr() + uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
            if (bInSlice && (uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
            {
                // （2）如果当前CU会继续向下进行分割，那么递归调用xEncodeCU，直到到达最底层
                xEncodeCU(pcCU, uiAbsPartIdx, uiDepth + 1);
            }
        }
        return;
    }

    if ((g_uiMaxCUWidth >> uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
        setdQPFlag(true);
    }
    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
        // （3）调用encodeCUTransquantBypassFlag，对变换量化跳过标志进行编码（最终调用的是TEncSbac::codeCUTransquantBypassFlag）
        m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, uiAbsPartIdx);
    }
    if (!pcCU->getSlice()->isIntra())
    {
        // （4）如果是帧间，编码skip标志（最后调用的是TEncSbac::codeSkipFlag）
        m_pcEntropyCoder->encodeSkipFlag(pcCU, uiAbsPartIdx);
    }

    if (pcCU->isSkipped(uiAbsPartIdx))
    {
        // （5）如果当前的CU是帧间skip模式的，那么调用encodeMergeIndex（最后调用TEncSbac::codeMergeIndex），对merge模式选出的索引进行编码，然后结束编码，返回（因为它的MV等信息是预测出来的，而不是计算出来的，因此到此结束处理）
        m_pcEntropyCoder->encodeMergeIndex(pcCU, uiAbsPartIdx);
        finishCU(pcCU, uiAbsPartIdx, uiDepth);
        return;
    }
    // （6）调用encodePredMode，对预测的模式(帧间还是帧内)进行编码（最后调用TEncSbac::codePredMode）
    // 实际上不会编码任何东西 直接 return 出来了
    m_pcEntropyCoder->encodePredMode(pcCU, uiAbsPartIdx);

    // （7）调用encodePartSize，对分割的尺寸进行编码（最后调用TEncSbac::codePartSize）
    m_pcEntropyCoder->encodePartSize(pcCU, uiAbsPartIdx, uiDepth);

    if (pcCU->isIntra(uiAbsPartIdx) && pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N)
    {
        // （8）如果是帧内预测，并且划分的方式是SIZE_2Nx2N，那么调用encodeIPCMInfo对IPCM的信息进行编码（最后调用TEncSbac::codeIPCMInfo），如果确实使用了IPCM，那么结束编码，返回
        m_pcEntropyCoder->encodeIPCMInfo(pcCU, uiAbsPartIdx);

        if (pcCU->getIPCMFlag(uiAbsPartIdx))
        {
            // Encode slice finish
            finishCU(pcCU, uiAbsPartIdx, uiDepth);
            return;
        }
    }

    // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
    // （9）调用encodePredInfo，对预测的信息(35种模式)进行编码
    m_pcEntropyCoder->encodePredInfo(pcCU, uiAbsPartIdx);

    // Encode Coefficients
    Bool bCodeDQP = getdQPFlag();
    // （10）调用encodeCoeff，对残差系数进行编码（最后调用TEncSbac::codeCoeffNxN）
    m_pcEntropyCoder->encodeCoeff(pcCU, uiAbsPartIdx, uiDepth, pcCU->getWidth(uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx), bCodeDQP);
    setdQPFlag(bCodeDQP);

    // --- write terminating bit ---
    finishCU(pcCU, uiAbsPartIdx, uiDepth);
}

Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg)
{
    Int k, i, j, jj;
    Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

    for (k = 0; k < 64; k += 8)
    {
        diff[k + 0] = piOrg[0];
        diff[k + 1] = piOrg[1];
        diff[k + 2] = piOrg[2];
        diff[k + 3] = piOrg[3];
        diff[k + 4] = piOrg[4];
        diff[k + 5] = piOrg[5];
        diff[k + 6] = piOrg[6];
        diff[k + 7] = piOrg[7];

        piOrg += iStrideOrg;
    }

    //horizontal
    for (j = 0; j < 8; j++)
    {
        jj = j << 3;
        m2[j][0] = diff[jj] + diff[jj + 4];
        m2[j][1] = diff[jj + 1] + diff[jj + 5];
        m2[j][2] = diff[jj + 2] + diff[jj + 6];
        m2[j][3] = diff[jj + 3] + diff[jj + 7];
        m2[j][4] = diff[jj] - diff[jj + 4];
        m2[j][5] = diff[jj + 1] - diff[jj + 5];
        m2[j][6] = diff[jj + 2] - diff[jj + 6];
        m2[j][7] = diff[jj + 3] - diff[jj + 7];

        m1[j][0] = m2[j][0] + m2[j][2];
        m1[j][1] = m2[j][1] + m2[j][3];
        m1[j][2] = m2[j][0] - m2[j][2];
        m1[j][3] = m2[j][1] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][6];
        m1[j][5] = m2[j][5] + m2[j][7];
        m1[j][6] = m2[j][4] - m2[j][6];
        m1[j][7] = m2[j][5] - m2[j][7];

        m2[j][0] = m1[j][0] + m1[j][1];
        m2[j][1] = m1[j][0] - m1[j][1];
        m2[j][2] = m1[j][2] + m1[j][3];
        m2[j][3] = m1[j][2] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][5];
        m2[j][5] = m1[j][4] - m1[j][5];
        m2[j][6] = m1[j][6] + m1[j][7];
        m2[j][7] = m1[j][6] - m1[j][7];
    }

    //vertical
    for (i = 0; i < 8; i++)
    {
        m3[0][i] = m2[0][i] + m2[4][i];
        m3[1][i] = m2[1][i] + m2[5][i];
        m3[2][i] = m2[2][i] + m2[6][i];
        m3[3][i] = m2[3][i] + m2[7][i];
        m3[4][i] = m2[0][i] - m2[4][i];
        m3[5][i] = m2[1][i] - m2[5][i];
        m3[6][i] = m2[2][i] - m2[6][i];
        m3[7][i] = m2[3][i] - m2[7][i];

        m1[0][i] = m3[0][i] + m3[2][i];
        m1[1][i] = m3[1][i] + m3[3][i];
        m1[2][i] = m3[0][i] - m3[2][i];
        m1[3][i] = m3[1][i] - m3[3][i];
        m1[4][i] = m3[4][i] + m3[6][i];
        m1[5][i] = m3[5][i] + m3[7][i];
        m1[6][i] = m3[4][i] - m3[6][i];
        m1[7][i] = m3[5][i] - m3[7][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
    }

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            iSumHad += abs(m2[i][j]);
        }
    }
    iSumHad -= abs(m2[0][0]);
    iSumHad = (iSumHad + 2) >> 2;
    return (iSumHad);
}

Int TEncCu::updateLCUDataISlice(TComDataCU *pcCU, Int LCUIdx, Int width, Int height)
{
    Int xBl, yBl;
    const Int iBlkSize = 8;

    Pel *pOrgInit = pcCU->getPic()->getPicYuvOrg()->getLumaAddr(pcCU->getAddr(), 0);
    Int iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride();
    Pel *pOrg;

    Int iSumHad = 0;
    for (yBl = 0; (yBl + iBlkSize) <= height; yBl += iBlkSize)
    {
        for (xBl = 0; (xBl + iBlkSize) <= width; xBl += iBlkSize)
        {
            pOrg = pOrgInit + iStrideOrig * yBl + xBl;
            iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
        }
    }
    return (iSumHad);
}

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, Bool *earlyDetectionSkipMode)
{
    assert(rpcTempCU->getSlice()->getSliceType() != I_SLICE);
    TComMvField cMvFieldNeighbours[2 * MRG_MAX_NUM_CANDS]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;
    const Bool bTransquantBypassFlag = rpcTempCU->getCUTransquantBypass(0);

    for (UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui)
    {
        uhInterDirNeighbours[ui] = 0;
    }
    UChar uhDepth = rpcTempCU->getDepth(0);
    rpcTempCU->setPartSizeSubParts(SIZE_2Nx2N, 0, uhDepth); // interprets depth relative to LCU level
    rpcTempCU->getInterMergeCandidates(0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand);

    Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
    for (UInt ui = 0; ui < numValidMergeCand; ++ui)
    {
        mergeCandBuffer[ui] = 0;
    }

    Bool bestIsSkip = false;

    UInt iteration;
    if (rpcTempCU->isLosslessCoded(0))
    {
        iteration = 1;
    }
    else
    {
        iteration = 2;
    }

    for (UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual)
    {
        for (UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand)
        {
            if (!(uiNoResidual == 1 && mergeCandBuffer[uiMergeCand] == 1))
            {
                if (!(bestIsSkip && uiNoResidual == 0))
                {
                    // set MC parameters
                    rpcTempCU->setPredModeSubParts(MODE_INTER, 0, uhDepth); // interprets depth relative to LCU level
                    rpcTempCU->setCUTransquantBypassSubParts(bTransquantBypassFlag, 0, uhDepth);
                    rpcTempCU->setPartSizeSubParts(SIZE_2Nx2N, 0, uhDepth);                                                            // interprets depth relative to LCU level
                    rpcTempCU->setMergeFlagSubParts(true, 0, 0, uhDepth);                                                              // interprets depth relative to LCU level
                    rpcTempCU->setMergeIndexSubParts(uiMergeCand, 0, 0, uhDepth);                                                      // interprets depth relative to LCU level
                    rpcTempCU->setInterDirSubParts(uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth);                                  // interprets depth relative to LCU level
                    rpcTempCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField(cMvFieldNeighbours[0 + 2 * uiMergeCand], SIZE_2Nx2N, 0, 0); // interprets depth relative to rpcTempCU level
                    rpcTempCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField(cMvFieldNeighbours[1 + 2 * uiMergeCand], SIZE_2Nx2N, 0, 0); // interprets depth relative to rpcTempCU level

                    // do MC
                    m_pcPredSearch->motionCompensation(rpcTempCU, m_ppcPredYuvTemp[uhDepth]);
                    // estimate residual and encode everything
                    m_pcPredSearch->encodeResAndCalcRdInterCU(rpcTempCU,
                                                              m_ppcOrigYuv[uhDepth],
                                                              m_ppcPredYuvTemp[uhDepth],
                                                              m_ppcResiYuvTemp[uhDepth],
                                                              m_ppcResiYuvBest[uhDepth],
                                                              m_ppcRecoYuvTemp[uhDepth],
                                                              (uiNoResidual ? true : false));

                    if (uiNoResidual == 0 && rpcTempCU->getQtRootCbf(0) == 0)
                    {
                        // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
                        mergeCandBuffer[uiMergeCand] = 1;
                    }

                    rpcTempCU->setSkipFlagSubParts(rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth);
                    Int orgQP = rpcTempCU->getQP(0);
                    xCheckDQP(rpcTempCU);
                    xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
                    rpcTempCU->initEstData(uhDepth, orgQP, bTransquantBypassFlag);

                    if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
                    {
                        bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
                    }
                }
            }
        }

        if (uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
        {
            if (rpcBestCU->getQtRootCbf(0) == 0)
            {
                if (rpcBestCU->getMergeFlag(0))
                {
                    *earlyDetectionSkipMode = true;
                }
                else
                {
                    Int absoulte_MV = 0;
                    for (UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
                    {
                        if (rpcBestCU->getSlice()->getNumRefIdx(RefPicList(uiRefListIdx)) > 0)
                        {
                            TComCUMvField *pcCUMvField = rpcBestCU->getCUMvField(RefPicList(uiRefListIdx));
                            Int iHor = pcCUMvField->getMvd(0).getAbsHor();
                            Int iVer = pcCUMvField->getMvd(0).getAbsVer();
                            absoulte_MV += iHor + iVer;
                        }
                    }

                    if (absoulte_MV == 0)
                    {
                        *earlyDetectionSkipMode = true;
                    }
                }
            }
        }
    }
}

#if AMP_MRG
Void TEncCu::xCheckRDCostInter(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, PartSize ePartSize, Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, PartSize ePartSize)
#endif
{
    UChar uhDepth = rpcTempCU->getDepth(0);

    rpcTempCU->setDepthSubParts(uhDepth, 0);

    rpcTempCU->setSkipFlagSubParts(false, 0, uhDepth);

    rpcTempCU->setPartSizeSubParts(ePartSize, 0, uhDepth);
    rpcTempCU->setPredModeSubParts(MODE_INTER, 0, uhDepth);

#if AMP_MRG
    rpcTempCU->setMergeAMP(true);
    m_pcPredSearch->predInterSearch(rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], false, bUseMRG);
#else
    m_pcPredSearch->predInterSearch(rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth]);
#endif

#if AMP_MRG
    if (!rpcTempCU->getMergeAMP())
    {
        return;
    }
#endif

    m_pcPredSearch->encodeResAndCalcRdInterCU(rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false);
    rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion());

    xCheckDQP(rpcTempCU);
    xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth);
}

Void TEncCu::xCheckRDCostIntra(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, PartSize eSize)
{
    UInt uiDepth = rpcTempCU->getDepth(0);

    rpcTempCU->setSkipFlagSubParts(false, 0, uiDepth);

    rpcTempCU->setPartSizeSubParts(eSize, 0, uiDepth);
    rpcTempCU->setPredModeSubParts(MODE_INTRA, 0, uiDepth);

    Bool bSeparateLumaChroma = true; // choose estimation mode
    UInt uiPreCalcDistC = 0;
    // 不会进入该分支
    if (!bSeparateLumaChroma)
    {
        m_pcPredSearch->preestChromaPredMode(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth]);
    }
    // 亮度部分 帧内编码
    m_pcPredSearch->estIntraPredQT(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC, bSeparateLumaChroma);

    // 在 estIntraPredQT 里面就已经对 rpcTempCU 重建过了, 这里有啥必要?
    // m_ppcRecoYuvTemp[uiDepth]->copyToPicLuma(rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU());

    m_pcPredSearch->estIntraPredChromaQT(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], uiPreCalcDistC);

    m_pcEntropyCoder->resetBits();
    if (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
        m_pcEntropyCoder->encodeCUTransquantBypassFlag(rpcTempCU, 0, true);
    }

    // 利用 RD 过程中插入的记录数据计算 L 分块 L 部分的总代价
    // 搜索过程中没有加上 bypass flag, 这里加上
    if (eSize != SIZE_NxN)
    {
        rpcTempCU->getTotalCostnpLpart(0b0111) = rpcTempCU->dBestCostLpartY0111 + rpcTempCU->dBestCostLpartC0111 + m_pcEntropyCoder->getNumberOfWrittenBits(); // + PartSizeCost_B_0111;
        rpcTempCU->getTotalCostnpLpart(0b1011) = rpcTempCU->dBestCostLpartY1011 + rpcTempCU->dBestCostLpartC1011 + m_pcEntropyCoder->getNumberOfWrittenBits(); // + PartSizeCost_B_1011;
        rpcTempCU->getTotalCostnpLpart(0b1101) = rpcTempCU->dBestCostLpartY1101 + rpcTempCU->dBestCostLpartC1101 + m_pcEntropyCoder->getNumberOfWrittenBits(); // + PartSizeCost_B_1101;
        rpcTempCU->getTotalCostnpLpart(0b1110) = rpcTempCU->dBestCostLpartY1110 + rpcTempCU->dBestCostLpartC1110 + m_pcEntropyCoder->getNumberOfWrittenBits(); // + PartSizeCost_B_1110;
    }

    // 项目不会编码的标志
    // m_pcEntropyCoder->encodeSkipFlag(rpcTempCU, 0, true);
    // m_pcEntropyCoder->encodePredMode(rpcTempCU, 0, true);
    m_pcEntropyCoder->encodePartSize(rpcTempCU, 0, uiDepth, true);
    m_pcEntropyCoder->encodePredInfo(rpcTempCU, 0, true);
    // m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true);

    // Encode Coefficients
    // 编码系数(无损时编码残差)
    // 此处重新编码是因为前面编码是为了计算 RD 代价 (不特殊设置的话 RD 代价并不是编码体积, 直接拿体积做 RD 只发生在特殊的无损模式下)
    // 同时保持熵编码器连续性 因为此时熵编码器的状态是测试完亮度最后一个模式\色差最后一个模式的状态 并不是最佳模式的状态
    Bool bCodeDQP = getdQPFlag(); // 项目中永远为 false
    m_pcEntropyCoder->encodeCoeff(rpcTempCU, 0, uiDepth, rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), bCodeDQP);
    // setdQPFlag(bCodeDQP);

    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

    rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
    // rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion());

    // xCheckDQP(rpcTempCU);
    // xCheckBestMode 向下划分时, 比较对象是 DOUBLE_MAX, RD 必定更好, 存储上层的结果. 特殊的是 4与8 的比较也在此处实现
    xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);
}

/** Check R-D costs for a CU with PCM mode. 
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 * 
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU)
{
    UInt uiDepth = rpcTempCU->getDepth(0);

    rpcTempCU->setSkipFlagSubParts(false, 0, uiDepth);

    rpcTempCU->setIPCMFlag(0, true);
    rpcTempCU->setIPCMFlagSubParts(true, 0, rpcTempCU->getDepth(0));
    rpcTempCU->setPartSizeSubParts(SIZE_2Nx2N, 0, uiDepth);
    rpcTempCU->setPredModeSubParts(MODE_INTRA, 0, uiDepth);
    rpcTempCU->setTrIdxSubParts(0, 0, uiDepth);

    m_pcPredSearch->IPCMSearch(rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);

    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

    m_pcEntropyCoder->resetBits();
    if (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
        m_pcEntropyCoder->encodeCUTransquantBypassFlag(rpcTempCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag(rpcTempCU, 0, true);
    m_pcEntropyCoder->encodePredMode(rpcTempCU, 0, true);
    m_pcEntropyCoder->encodePartSize(rpcTempCU, 0, uiDepth, true);
    m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true);

    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

    rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
    rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost(rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion());

    xCheckDQP(rpcTempCU);
    xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth);
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckBestMode(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
{
    // 区分 保持大块部分/L0111/L1011/L1101/L1110 需要一个新标志, 根据频率分配bits 3 3 2 2 2

    // 向下计算时 Status=0, 向上回归时 Status=1
    Bool bStatus = rpcBestCU->getTotalCost() != MAX_DOUBLE;

    if (*rpcBestCU->getWidth() != 8)
    {
        if (bStatus == 0) // 表示在向下搜索的过程中
        {
            rpcTempCU->getTotalBits() += (1 + 3); // 模拟编码分块标志 0 和切块方式保持大块
        }
        else
        {
            rpcTempCU->getTotalBits() += (1);                  // 模拟编码分块标志 1
            rpcBestCU->getTotalCostnpLpart(0b0111) += (1 + 3); // 模拟编码分块标志 0 和切块方式(L)
            rpcBestCU->getTotalCostnpLpart(0b1011) += (1 + 2); // 模拟编码分块标志 0 和切块方式(L)
            rpcBestCU->getTotalCostnpLpart(0b1101) += (1 + 2); // 模拟编码分块标志 0 和切块方式(L)
            rpcBestCU->getTotalCostnpLpart(0b1110) += (1 + 2); // 模拟编码分块标志 0 和切块方式(L)
        }
    }                                                    // FIXIT: 向上回归的时候是不是重复计算了?
    else if (*rpcTempCU->getPartitionSize() == SIZE_NxN) // 8x8 往下分的情况, 和标准不同, 需要记录怎么切 四分还是L
    {
        rpcTempCU->getTotalBits() += (1);              // 模拟编码搜索过程中没有计算的SIZE_NXN partsize (如果 part size == nxn 肯定是正常四分, 因为我们将 3个4x4的L+1个4x4块 定义为 8x8 层 也就是使用了 partsize == 2nx2n 的编码体积)
        rpcBestCU->getTotalCostnpLpart(0b0111) += (3); // 模拟编码切块方式(L)
        rpcBestCU->getTotalCostnpLpart(0b1011) += (2); // 模拟编码切块方式(L)
        rpcBestCU->getTotalCostnpLpart(0b1101) += (2); // 模拟编码切块方式(L)
        rpcBestCU->getTotalCostnpLpart(0b1110) += (2); // 模拟编码切块方式(L)
    }
    else // 8x8 块不往下分的情况
    {
        rpcTempCU->getTotalBits() += 3; // 模拟编码切块方式保持大块
    }
    rpcTempCU->getTotalCost() = rpcTempCU->getTotalBits();

    // 新方法 Cost 计算: Best L + Temp 1/4
    if (bStatus == 1) // 向上回归时计算新方法的 Cost
    {
        rpcBestCU->m_dTotalCostnp0111 = rpcBestCU->getTotalCostnpLpart(0b0111) + rpcTempCU->dBestCostQuarPartLT;
        rpcBestCU->m_dTotalCostnp1011 = rpcBestCU->getTotalCostnpLpart(0b1011) + rpcTempCU->dBestCostQuarPartRT;
        rpcBestCU->m_dTotalCostnp1101 = rpcBestCU->getTotalCostnpLpart(0b1101) + rpcTempCU->dBestCostQuarPartLB;
        rpcBestCU->m_dTotalCostnp1110 = rpcBestCU->getTotalCostnpLpart(0b1110) + rpcTempCU->dBestCostQuarPartRB;
    }

    Int iMinPos;
    // if (bStatus == 1) // 模拟 HEVC 标准状况
    // {
    //     Double CostList[2] = {rpcBestCU->getTotalCost(),
    //                           rpcTempCU->getTotalCost()};
    //     iMinPos = min_element(CostList, CostList + 2) - CostList;
    //     dMin = CostList[iMinPos];
    // }
    if (bStatus == 1) // 向上回归时从所有 6 种情况里面找最小值
    {
        Double CostList[6] = {rpcBestCU->getTotalCost(),
                              rpcTempCU->getTotalCost(),
                              rpcBestCU->m_dTotalCostnp0111,
                              rpcBestCU->m_dTotalCostnp1011,
                              rpcBestCU->m_dTotalCostnp1101,
                              rpcBestCU->m_dTotalCostnp1110};
        iMinPos = min_element(CostList, CostList + 6) - CostList;
    }
    else
    {
        iMinPos = 1;
    }

    if (iMinPos != 0)
    {
        TComDataCU *pcCU = rpcBestCU;

        switch (iMinPos)
        {
        case 1:
            rpcBestCU = rpcTempCU;
            rpcTempCU = pcCU;
            break;
        case 2:
            MergeLnQuar(rpcBestCU, rpcTempCU, 0b0111);
            break;
        case 3:
            MergeLnQuar(rpcBestCU, rpcTempCU, 0b1011);
            break;
        case 4:
            MergeLnQuar(rpcBestCU, rpcTempCU, 0b1101);
            break;
        case 5:
            MergeLnQuar(rpcBestCU, rpcTempCU, 0b1110);
            break;
        }

        pcCU = NULL;
        // 其实没有意义 对于无损来说哪次的重建结果都一样
        m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
        m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];

        m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
    }
    else
    {
        // 填不切分时 CU 的 L 分块标记 // 需要吗?
    }

    // {
    //     // 如果 L 分块更好,要处理coeff dir cbf,还要记得把totalcost(bits)换成对应的totalcostnp
    //     // switch (iMinPos)
    //     // {
    //     // case 1:
    //     //     break;

    //     // }
    //     TComYuv *pcYuv;
    //     // Change Information data
    //     TComDataCU *pcCU = rpcBestCU;
    //     rpcBestCU = rpcTempCU;
    //     rpcTempCU = pcCU;
    //     rpcBestCU->getTotalCost() = dMin;
    //     rpcBestCU->getTotalBits() = dMin;

    //     // Change Prediction data
    //     // 此时的预测值地址存放的东西已经名不副实了, 是重建值. 而且已经不会再使用预测值了, 不用把 Temp 换过去也没事
    //     // pcYuv = m_ppcPredYuvBest[uiDepth];
    //     m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    //     // m_ppcPredYuvTemp[uiDepth] = pcYuv;

    //     // Change Reconstruction data
    //     // pcYuv = m_ppcRecoYuvBest[uiDepth];
    //     m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    //     // m_ppcRecoYuvTemp[uiDepth] = pcYuv;

    //     pcYuv = NULL;
    //     pcCU = NULL;

    //     // store temp best CI for next CU coding
    //     m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
    // }
}

// 如果 L 形分块的总 Cost 更小, 把对应的 L 形分块组合后的数据处理好存放到 rpcTempCU
// rpcBestCU: 上层大块数据
// rpcTempCU: 下层 4 分小块数据
// 函数内部修改 Temp 的数据, 函数后应有一步把 Best 指向 Temp
// 运行此函数时肯定处于向上回归的过程
Void TEncCu::MergeLnQuar(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt mask)
{
    UInt uiWidth = rpcBestCU->getWidth(0);

    // 处理亮度系数部分
    {
        UInt uiQuarSize = (uiWidth >> 1) * (uiWidth >> 1);
        TCoeff *pcCoeffTempY = rpcTempCU->getCoeffY();
        TCoeff *pcCoeffTempYQuarpart = pcCoeffTempY;
        TCoeff *pcCoeffBestYLpart;
        TCoeff *pcCoeffMergeY;
        switch (mask)
        {
        case 0b0111:
            pcCoeffBestYLpart = rpcBestCU->m_pcTrCoeffYnp0111;
            pcCoeffMergeY = pcCoeffBestYLpart;
            break;
        case 0b1011:
            pcCoeffBestYLpart = rpcBestCU->m_pcTrCoeffYnp1011;
            pcCoeffMergeY = pcCoeffBestYLpart + (uiWidth >> 1);
            pcCoeffTempYQuarpart += uiQuarSize;
            break;
        case 0b1101:
            pcCoeffBestYLpart = rpcBestCU->m_pcTrCoeffYnp1101;
            pcCoeffMergeY = pcCoeffBestYLpart + 2 * uiQuarSize;
            pcCoeffTempYQuarpart += 2 * uiQuarSize;
            break;
        case 0b1110:
            pcCoeffBestYLpart = rpcBestCU->m_pcTrCoeffYnp1110;
            pcCoeffMergeY = pcCoeffBestYLpart + 2 * uiQuarSize + (uiWidth >> 1);
            pcCoeffTempYQuarpart += 3 * uiQuarSize;
            break;
        }
        for (Int i = 0; i < (uiWidth >> 1); i++)
        {
            ::memcpy(pcCoeffMergeY, pcCoeffTempYQuarpart, sizeof(TCoeff) * (uiWidth >> 1));
            pcCoeffMergeY += uiWidth;
            pcCoeffTempYQuarpart += (uiWidth >> 1);
        }
        // ::memcpy(pcCoeffTempY, pcCoeffBestYLpart, sizeof(TCoeff) * uiWidth * uiWidth);
        ::memcpy(rpcBestCU->getCoeffY(), pcCoeffBestYLpart, sizeof(TCoeff) * uiWidth * uiWidth);
        pcCoeffTempY = NULL;
        pcCoeffTempYQuarpart = NULL;
        pcCoeffBestYLpart = NULL;
        pcCoeffMergeY = NULL;
    }
    // 处理色差系数部分
    {
        Bool b8flag = uiWidth == 8;
        UInt uiQuarSizeC = (uiWidth >> 2) * (uiWidth >> 2);
        TCoeff *pcCoeffTempCb = rpcTempCU->getCoeffCb();
        TCoeff *pcCoeffTempCbQuarpart = pcCoeffTempCb;
        TCoeff *pcCoeffBestCbLpart;
        TCoeff *pcCoeffMergeCb;
        TCoeff *pcCoeffTempCr = rpcTempCU->getCoeffCr();
        TCoeff *pcCoeffTempCrQuarpart = pcCoeffTempCr;
        TCoeff *pcCoeffBestCrLpart;
        TCoeff *pcCoeffMergeCr;
        switch (mask)
        {
        case 0b0111:
            pcCoeffBestCbLpart = rpcBestCU->m_pcTrCoeffCbnp0111;
            pcCoeffMergeCb = pcCoeffBestCbLpart;
            pcCoeffBestCrLpart = rpcBestCU->m_pcTrCoeffCrnp0111;
            pcCoeffMergeCr = pcCoeffBestCrLpart;
            break;
        case 0b1011:
            pcCoeffBestCbLpart = rpcBestCU->m_pcTrCoeffCbnp1011;
            pcCoeffMergeCb = pcCoeffBestCbLpart + (b8flag ? 0 : (uiWidth >> 2));
            pcCoeffTempCbQuarpart += (b8flag ? 0 : uiQuarSizeC);
            pcCoeffBestCrLpart = rpcBestCU->m_pcTrCoeffCrnp1011;
            pcCoeffMergeCr = pcCoeffBestCrLpart + (b8flag ? 0 : (uiWidth >> 2));
            pcCoeffTempCrQuarpart += (b8flag ? 0 : uiQuarSizeC);
            break;
        case 0b1101:
            pcCoeffBestCbLpart = rpcBestCU->m_pcTrCoeffCbnp1101;
            pcCoeffMergeCb = pcCoeffBestCbLpart + (b8flag ? 0 : 2 * uiQuarSizeC);
            pcCoeffTempCbQuarpart += (b8flag ? 0 : 2 * uiQuarSizeC);
            pcCoeffBestCrLpart = rpcBestCU->m_pcTrCoeffCrnp1101;
            pcCoeffMergeCr = pcCoeffBestCrLpart + (b8flag ? 0 : 2 * uiQuarSizeC);
            pcCoeffTempCrQuarpart += (b8flag ? 0 : 2 * uiQuarSizeC);
            break;
        case 0b1110:
            pcCoeffBestCbLpart = rpcBestCU->m_pcTrCoeffCbnp1110;
            pcCoeffMergeCb = pcCoeffBestCbLpart + (b8flag ? 0 : 2 * uiQuarSizeC + (uiWidth >> 2));
            pcCoeffTempCbQuarpart += (b8flag ? 0 : 3 * uiQuarSizeC);
            pcCoeffBestCrLpart = rpcBestCU->m_pcTrCoeffCrnp1110;
            pcCoeffMergeCr = pcCoeffBestCrLpart + (b8flag ? 0 : 2 * uiQuarSizeC + (uiWidth >> 2));
            pcCoeffTempCrQuarpart += (b8flag ? 0 : 3 * uiQuarSizeC);
            break;
        }
        for (Int i = 0; i < (b8flag ? 4 : (uiWidth >> 2)); i++)
        {
            ::memcpy(pcCoeffMergeCb, pcCoeffTempCbQuarpart, sizeof(TCoeff) * (b8flag ? 4 : (uiWidth >> 2)));
            pcCoeffMergeCb += (uiWidth >> 1);
            pcCoeffTempCbQuarpart += (b8flag ? 4 : (uiWidth >> 2));
            ::memcpy(pcCoeffMergeCr, pcCoeffTempCrQuarpart, sizeof(TCoeff) * (b8flag ? 4 : (uiWidth >> 2)));
            pcCoeffMergeCr += (uiWidth >> 1);
            pcCoeffTempCrQuarpart += (b8flag ? 4 : (uiWidth >> 2));
        }
        // ::memcpy(pcCoeffTempCb, pcCoeffBestCbLpart, sizeof(TCoeff) * (uiWidth >> 1) * (uiWidth >> 1));
        // ::memcpy(pcCoeffTempCr, pcCoeffBestCrLpart, sizeof(TCoeff) * (uiWidth >> 1) * (uiWidth >> 1));
        ::memcpy(rpcBestCU->getCoeffCb(), pcCoeffBestCbLpart, sizeof(TCoeff) * (uiWidth >> 1) * (uiWidth >> 1));
        ::memcpy(rpcBestCU->getCoeffCr(), pcCoeffBestCrLpart, sizeof(TCoeff) * (uiWidth >> 1) * (uiWidth >> 1));
        pcCoeffTempCb = NULL;
        pcCoeffTempCb = NULL;
        pcCoeffTempCbQuarpart = NULL;
        pcCoeffBestCbLpart = NULL;
        pcCoeffMergeCb = NULL;
        pcCoeffTempCr = NULL;
        pcCoeffTempCrQuarpart = NULL;
        pcCoeffBestCrLpart = NULL;
        pcCoeffMergeCr = NULL;
    }
    // 处理 Cost 记录部分
    {
        switch (mask)
        {
        case 0b0111:
            rpcBestCU->getTotalCost() = rpcBestCU->m_dTotalCostnp0111;
            rpcBestCU->getTotalBits() = rpcBestCU->m_dTotalCostnp0111;
            break;
        case 0b1011:
            rpcBestCU->getTotalCost() = rpcBestCU->m_dTotalCostnp1011;
            rpcBestCU->getTotalBits() = rpcBestCU->m_dTotalCostnp1011;
            break;
        case 0b1101:
            rpcBestCU->getTotalCost() = rpcBestCU->m_dTotalCostnp1101;
            rpcBestCU->getTotalBits() = rpcBestCU->m_dTotalCostnp1101;
            break;
        case 0b1110:
            rpcBestCU->getTotalCost() = rpcBestCU->m_dTotalCostnp1110;
            rpcBestCU->getTotalBits() = rpcBestCU->m_dTotalCostnp1110;
            break;
        }
    }
    // 处理预测方向部分
    {
        UChar *puhDirTempY = rpcTempCU->getLumaIntraDir();
        UChar *puhDirTempYQuarpart = puhDirTempY;
        UChar *puhDirBestYLpart;
        UChar *puhDirMergeY;
        UChar *puhDirTempC = rpcTempCU->getChromaIntraDir();
        UChar *puhDirTempCQuarpart = puhDirTempC;
        UChar *puhDirBestCLpart;
        UChar *puhDirMergeC;
        UInt uiQuarSizeDir = (uiWidth * uiWidth) >> 4;
        switch (mask)
        {
        case 0b0111:
            puhDirBestYLpart = rpcBestCU->m_puhLumaIntraDirnp0111;
            puhDirMergeY = puhDirBestYLpart;
            puhDirBestCLpart = rpcBestCU->m_puhChromaIntraDirnp0111;
            puhDirMergeC = puhDirBestCLpart;
            break;
        case 0b1011:
            puhDirBestYLpart = rpcBestCU->m_puhLumaIntraDirnp1011;
            puhDirMergeY = puhDirBestYLpart + 1 * uiQuarSizeDir;
            puhDirTempYQuarpart += 1 * uiQuarSizeDir;
            puhDirBestCLpart = rpcBestCU->m_puhChromaIntraDirnp1011;
            puhDirMergeC = puhDirBestCLpart + 1 * uiQuarSizeDir;
            puhDirTempCQuarpart += 1 * uiQuarSizeDir;
            break;
        case 0b1101:
            puhDirBestYLpart = rpcBestCU->m_puhLumaIntraDirnp1101;
            puhDirMergeY = puhDirBestYLpart + 2 * uiQuarSizeDir;
            puhDirTempYQuarpart += 2 * uiQuarSizeDir;
            puhDirBestCLpart = rpcBestCU->m_puhChromaIntraDirnp1101;
            puhDirMergeC = puhDirBestCLpart + 2 * uiQuarSizeDir;
            puhDirTempCQuarpart += 2 * uiQuarSizeDir;
            break;
        case 0b1110:
            puhDirBestYLpart = rpcBestCU->m_puhLumaIntraDirnp1110;
            puhDirMergeY = puhDirBestYLpart + 3 * uiQuarSizeDir;
            puhDirTempYQuarpart += 3 * uiQuarSizeDir;
            puhDirBestCLpart = rpcBestCU->m_puhChromaIntraDirnp1110;
            puhDirMergeC = puhDirBestCLpart + 3 * uiQuarSizeDir;
            puhDirTempCQuarpart += 3 * uiQuarSizeDir;
            break;
        }
        ::memcpy(puhDirMergeY, puhDirTempYQuarpart, sizeof(UChar) * uiQuarSizeDir);
        // ::memcpy(puhDirTempY, puhDirBestYLpart, sizeof(UChar) * uiQuarSizeDir * 4);
        ::memcpy(rpcBestCU->getLumaIntraDir(), puhDirBestYLpart, sizeof(UChar) * uiQuarSizeDir * 4);
        ::memcpy(puhDirMergeC, puhDirTempCQuarpart, sizeof(UChar) * uiQuarSizeDir);
        // ::memcpy(puhDirTempC, puhDirBestCLpart, sizeof(UChar) * uiQuarSizeDir * 4);
        ::memcpy(rpcBestCU->getChromaIntraDir(), puhDirBestCLpart, sizeof(UChar) * uiQuarSizeDir * 4);
        puhDirTempY = NULL;
        puhDirTempYQuarpart = NULL;
        puhDirBestYLpart = NULL;
        puhDirMergeY = NULL;
        puhDirTempC = NULL;
        puhDirTempCQuarpart = NULL;
        puhDirBestCLpart = NULL;
        puhDirMergeC = NULL;
    }
    // 处理 Cbf 部分
    // 注意新方法的 CBF 其实是不准确的, RD过程中 CBF 还是看整个块是否为0 并没有区分出 L 部分来看
    // 需要同步处理完 L 编码逻辑才能启用
    {
        // 想法: 自己重新算
        //     UInt uiQuarSizeCbf = ((uiWidth >> 2) * (uiWidth >> 2)) >> 2;
        //     UInt setcbfY;
        //     UInt setcbfU;
        //     UInt setcbfV;
        //     switch (mask)
        //     {
        //     case 0b0111:
        //         if (*(rpcTempCU->getCbf(TEXT_LUMA)) == 0 && *(rpcBestCU->getCbfnp(TEXT_LUMA, 0b0111)) == 0)
        //         {
        //             setcbfY = 0;
        //         }
        //         else
        //         {
        //             setcbfY = *(rpcTempCU->getCbf(TEXT_LUMA));
        //         }
        //     }
        //     // memset(rpcTempCU->getCbf(TEXT_LUMA))

        // 想法: 直接复制对应 Tempnp 结果
        //     ::memcpy(rpcTempCU->getCbf(TEXT_LUMA), rpcBestCU->getCbfnp(TEXT_LUMA, 0b0111), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_U), rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b0111), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_V), rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b0111), sizeof(UChar) * uiSizeCbf);
        //     break;
        // case 0b1011:
        //     ::memcpy(rpcTempCU->getCbf(TEXT_LUMA), rpcBestCU->getCbfnp(TEXT_LUMA, 0b1011), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_U), rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1011), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_V), rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1011), sizeof(UChar) * uiSizeCbf);
        //     break;
        // case 0b1101:
        //     ::memcpy(rpcTempCU->getCbf(TEXT_LUMA), rpcBestCU->getCbfnp(TEXT_LUMA, 0b1101), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_U), rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1101), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_V), rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1101), sizeof(UChar) * uiSizeCbf);
        //     break;
        // case 0b1110:
        //     ::memcpy(rpcTempCU->getCbf(TEXT_LUMA), rpcBestCU->getCbfnp(TEXT_LUMA, 0b1110), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_U), rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1110), sizeof(UChar) * uiSizeCbf);
        //     ::memcpy(rpcTempCU->getCbf(TEXT_CHROMA_V), rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1110), sizeof(UChar) * uiSizeCbf);
        //     break;

        // 想法: 同其他 保留 BEST L 和 TEMP QUAR
        // UChar *puhCbfTempY = rpcTempCU->getCbf(TEXT_LUMA);
        // UChar *puhCbfTempYQuarpart = puhCbfTempY;
        // UChar *puhCbfBestYLpart;
        // UChar *puhCbfMergeY;
        // UChar *puhCbfTempU = rpcTempCU->getCbf(TEXT_CHROMA_U);
        // UChar *puhCbfTempUQuarpart = puhCbfTempU;
        // UChar *puhCbfBestULpart;
        // UChar *puhCbfMergeU;
        // UChar *puhCbfTempV = rpcTempCU->getCbf(TEXT_CHROMA_V);
        // UChar *puhCbfTempVQuarpart = puhCbfTempV;
        // UChar *puhCbfBestVLpart;
        // UChar *puhCbfMergeV;
        // UInt uiQuarSizeCbf = (uiWidth * uiWidth) >> 6;
        // switch (mask)
        // {
        // case 0b0111:
        //     puhCbfBestYLpart = rpcBestCU->getCbfnp(TEXT_LUMA, 0b0111);
        //     puhCbfMergeY = puhCbfBestYLpart;
        //     puhCbfBestULpart = rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b0111);
        //     puhCbfMergeU = puhCbfBestULpart;
        //     puhCbfBestVLpart = rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b0111);
        //     puhCbfMergeV = puhCbfBestVLpart;
        //     break;
        // case 0b1011:
        //     puhCbfBestYLpart = rpcBestCU->getCbfnp(TEXT_LUMA, 0b1011);
        //     puhCbfMergeY = puhCbfBestYLpart + 1 * uiQuarSizeCbf;
        //     puhCbfTempYQuarpart += 1 * uiQuarSizeCbf;
        //     puhCbfBestULpart = rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1011);
        //     puhCbfMergeU = puhCbfBestULpart + 1 * uiQuarSizeCbf;
        //     puhCbfTempUQuarpart += 1 * uiQuarSizeCbf;
        //     puhCbfBestVLpart = rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1011);
        //     puhCbfMergeV = puhCbfBestVLpart + 1 * uiQuarSizeCbf;
        //     puhCbfTempVQuarpart += 1 * uiQuarSizeCbf;
        //     break;
        // case 0b1101:
        //     puhCbfBestYLpart = rpcBestCU->getCbfnp(TEXT_LUMA, 0b1101);
        //     puhCbfMergeY = puhCbfBestYLpart + 2 * uiQuarSizeCbf;
        //     puhCbfTempYQuarpart += 2 * uiQuarSizeCbf;
        //     puhCbfBestULpart = rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1101);
        //     puhCbfMergeU = puhCbfBestULpart + 2 * uiQuarSizeCbf;
        //     puhCbfTempUQuarpart += 2 * uiQuarSizeCbf;
        //     puhCbfBestVLpart = rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1101);
        //     puhCbfMergeV = puhCbfBestVLpart + 2 * uiQuarSizeCbf;
        //     puhCbfTempVQuarpart += 2 * uiQuarSizeCbf;
        //     break;
        // case 0b1110:
        //     puhCbfBestYLpart = rpcBestCU->getCbfnp(TEXT_LUMA, 0b1110);
        //     puhCbfMergeY = puhCbfBestYLpart + 3 * uiQuarSizeCbf;
        //     puhCbfTempYQuarpart += 3 * uiQuarSizeCbf;
        //     puhCbfBestULpart = rpcBestCU->getCbfnp(TEXT_CHROMA_U, 0b1110);
        //     puhCbfMergeU = puhCbfBestULpart + 3 * uiQuarSizeCbf;
        //     puhCbfTempUQuarpart += 3 * uiQuarSizeCbf;
        //     puhCbfBestVLpart = rpcBestCU->getCbfnp(TEXT_CHROMA_V, 0b1110);
        //     puhCbfMergeV = puhCbfBestVLpart + 3 * uiQuarSizeCbf;
        //     puhCbfTempVQuarpart += 3 * uiQuarSizeCbf;
        //     break;
        // }
        // ::memcpy(puhCbfMergeY, puhCbfTempYQuarpart, sizeof(UChar) * uiQuarSizeCbf);
        // // ::memcpy(puhCbfTempY, puhCbfBestYLpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // ::memcpy(rpcBestCU->getCbf(TEXT_LUMA), puhCbfBestYLpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // ::memcpy(puhCbfMergeU, puhCbfTempUQuarpart, sizeof(UChar) * uiQuarSizeCbf);
        // // ::memcpy(puhCbfTempU, puhCbfBestULpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // ::memcpy(rpcBestCU->getCbf(TEXT_CHROMA_U), puhCbfBestULpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // ::memcpy(puhCbfMergeV, puhCbfTempVQuarpart, sizeof(UChar) * uiQuarSizeCbf);
        // // ::memcpy(puhCbfTempV, puhCbfBestVLpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // ::memcpy(rpcBestCU->getCbf(TEXT_CHROMA_V), puhCbfBestVLpart, sizeof(UChar) * uiQuarSizeCbf * 4);
        // puhCbfTempY = NULL;
        // puhCbfTempYQuarpart = NULL;
        // puhCbfBestYLpart = NULL;
        // puhCbfMergeY = NULL;
        // puhCbfTempU = NULL;
        // puhCbfTempUQuarpart = NULL;
        // puhCbfBestULpart = NULL;
        // puhCbfMergeU = NULL;
        // puhCbfTempV = NULL;
        // puhCbfTempVQuarpart = NULL;
        // puhCbfBestVLpart = NULL;
        // puhCbfMergeV = NULL;
    }
    // 处理 PartSize 部分
    {
        // Char *pePartSizeBest = rpcBestCU->getPartitionSize();
        // UInt uiQuarSizePartSize = (uiWidth * uiWidth) >> 6;
        // switch (mask)
        // {
        // case 0b0111:
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 1), SIZE_B_0111, uiQuarSizePartSize * 3);
        //     break;
        // case 0b1011:
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 0), SIZE_B_1011, uiQuarSizePartSize * 1);
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 2), SIZE_B_1011, uiQuarSizePartSize * 2);
        // case 0b1101:
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 0), SIZE_B_1101, uiQuarSizePartSize * 2);
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 3), SIZE_B_1101, uiQuarSizePartSize * 1);
        // case 0b1110:
        //     memset(pePartSizeBest + (uiQuarSizePartSize * 0), SIZE_B_1110, uiQuarSizePartSize * 3);
        // }
        // pePartSizeBest = NULL;
    }
}

Void TEncCu::xCheckDQP(TComDataCU *pcCU)
{
    UInt uiDepth = pcCU->getDepth(0);

    if (pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth >> uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize())
    {
        if (pcCU->getCbf(0, TEXT_LUMA, 0) || pcCU->getCbf(0, TEXT_CHROMA_U, 0) || pcCU->getCbf(0, TEXT_CHROMA_V, 0))
        {
#if !RDO_WITHOUT_DQP_BITS
            m_pcEntropyCoder->resetBits();
            m_pcEntropyCoder->encodeQP(pcCU, 0, false);
            pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
            pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac *)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
            pcCU->getTotalCost() = m_pcRdCost->calcRdCost(pcCU->getTotalBits(), pcCU->getTotalDistortion());
#endif
        }
        else
        {
            pcCU->setQPSubParts(pcCU->getRefQP(0), 0, uiDepth); // set QP to default QP
        }
    }
}

Void TEncCu::xCopyAMVPInfo(AMVPInfo *pSrc, AMVPInfo *pDst)
{
    pDst->iN = pSrc->iN;
    for (Int i = 0; i < pSrc->iN; i++)
    {
        pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
    }
}
Void TEncCu::xCopyYuv2Pic(TComPic *rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU *pcCU, UInt uiLPelX, UInt uiTPelY)
{
    UInt uiRPelX = uiLPelX + (g_uiMaxCUWidth >> uiDepth) - 1;
    UInt uiBPelY = uiTPelY + (g_uiMaxCUHeight >> uiDepth) - 1;
    TComSlice *pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx &&
                       pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx + (pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1));
    Bool bSliceEnd = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx &&
                     pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx + (pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1));
    if (!bSliceEnd && !bSliceStart && (uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
    {
        UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
        UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
        UInt uiBlkWidth = rpcPic->getNumPartInWidth() >> (uiDepth);
        UInt uiPartIdxX = ((uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth()) % uiSrcBlkWidth) / uiBlkWidth;
        UInt uiPartIdxY = ((uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth()) % uiSrcBlkWidth) / uiBlkWidth;
        UInt uiPartIdx = uiPartIdxY * (uiSrcBlkWidth / uiBlkWidth) + uiPartIdxX;
        m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv(rpcPic->getPicYuvRec(), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
    }
    else
    {
        UInt uiQNumParts = (pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1)) >> 2;

        for (UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx += uiQNumParts)
        {
            UInt uiSubCULPelX = uiLPelX + (g_uiMaxCUWidth >> (uiDepth + 1)) * (uiPartUnitIdx & 1);
            UInt uiSubCUTPelY = uiTPelY + (g_uiMaxCUHeight >> (uiDepth + 1)) * (uiPartUnitIdx >> 1);

            Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx + uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() &&
                            rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr()) * pcCU->getPic()->getNumPartInCU() + uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
            if (bInSlice && (uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
            {
                xCopyYuv2Pic(rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth + 1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY); // Copy Yuv data to picture Yuv
            }
        }
    }
}

Void TEncCu::xCopyYuv2Tmp(UInt uiPartUnitIdx, UInt uiNextDepth)
{
    UInt uiCurrDepth = uiNextDepth - 1;
    m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv(m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx);
}

/** Function for filling the PCM buffer of a CU using its original sample array 
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer(TComDataCU *&pCU, TComYuv *pOrgYuv)
{

    UInt width = pCU->getWidth(0);
    UInt height = pCU->getHeight(0);

    Pel *pSrcY = pOrgYuv->getLumaAddr(0, width);
    Pel *pDstY = pCU->getPCMSampleY();
    UInt srcStride = pOrgYuv->getStride();

    for (Int y = 0; y < height; y++)
    {
        for (Int x = 0; x < width; x++)
        {
            pDstY[x] = pSrcY[x];
        }
        pDstY += width;
        pSrcY += srcStride;
    }

    Pel *pSrcCb = pOrgYuv->getCbAddr();
    Pel *pSrcCr = pOrgYuv->getCrAddr();
    ;

    Pel *pDstCb = pCU->getPCMSampleCb();
    Pel *pDstCr = pCU->getPCMSampleCr();
    ;

    UInt srcStrideC = pOrgYuv->getCStride();
    UInt heightC = height >> 1;
    UInt widthC = width >> 1;

    for (Int y = 0; y < heightC; y++)
    {
        for (Int x = 0; x < widthC; x++)
        {
            pDstCb[x] = pSrcCb[x];
            pDstCr[x] = pSrcCr[x];
        }
        pDstCb += widthC;
        pDstCr += widthC;
        pSrcCb += srcStrideC;
        pSrcCr += srcStrideC;
    }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff *rpcCoeff, Int *rpcArlCoeff, Int NumCoeffInCU, Double *cSum, UInt *numSamples)
{
    for (Int n = 0; n < NumCoeffInCU; n++)
    {
        Int u = abs(rpcCoeff[n]);
        Int absc = rpcArlCoeff[n];

        if (u != 0)
        {
            if (u < LEVEL_RANGE)
            {
                cSum[u] += (Double)absc;
                numSamples[u]++;
            }
            else
            {
                cSum[LEVEL_RANGE] += (Double)absc - (Double)(u << ARL_C_PRECISION);
                numSamples[LEVEL_RANGE]++;
            }
        }
    }

    return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU *rpcCU)
{
    Double cSum[LEVEL_RANGE + 1];     //: the sum of DCT coefficients corresponding to datatype and quantization output
    UInt numSamples[LEVEL_RANGE + 1]; //: the number of coefficients corresponding to datatype and quantization output

    TCoeff *pCoeffY = rpcCU->getCoeffY();
    Int *pArlCoeffY = rpcCU->getArlCoeffY();

    UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
    UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

    memset(cSum, 0, sizeof(Double) * (LEVEL_RANGE + 1));
    memset(numSamples, 0, sizeof(UInt) * (LEVEL_RANGE + 1));

    // Collect stats to cSum[][] and numSamples[][]
    for (Int i = 0; i < rpcCU->getTotalNumPart(); i++)
    {
        UInt uiTrIdx = rpcCU->getTransformIdx(i);

        if (rpcCU->getPredictionMode(i) == MODE_INTER)
            if (rpcCU->getCbf(i, TEXT_LUMA, uiTrIdx))
            {
                xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
            } //Note that only InterY is processed. QP rounding is based on InterY data only.

        pCoeffY += uiMinNumCoeffInCU;
        pArlCoeffY += uiMinNumCoeffInCU;
    }

    for (Int u = 1; u < LEVEL_RANGE; u++)
    {
        m_pcTrQuant->getSliceSumC()[u] += cSum[u];
        m_pcTrQuant->getSliceNSamples()[u] += numSamples[u];
    }
    m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[LEVEL_RANGE];
    m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[LEVEL_RANGE];
}
#endif
//! \}
