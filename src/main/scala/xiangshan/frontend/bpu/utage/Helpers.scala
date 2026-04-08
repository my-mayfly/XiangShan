// Copyright (c) 2024-2026 Beijing Institute of Open Source Chip (BOSC)
// Copyright (c) 2020-2026 Institute of Computing Technology, Chinese Academy of Sciences
// Copyright (c) 2020-2021 Peng Cheng Laboratory
//
// XiangShan is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          https://license.coscl.org.cn/MulanPSL2
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
//
// See the Mulan PSL v2 for more details.

package xiangshan.frontend.bpu.utage

import chisel3._
import chisel3.util._
import scala.math.min
import xiangshan.HasXSParameter
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.HalfAlignHelper
import xiangshan.frontend.bpu.MicroTageInfo
import xiangshan.frontend.bpu.PhrHelper
import xiangshan.frontend.bpu.history.phr.PhrAllFoldedHistories

trait Helpers extends HasMicroTageParameters with HalfAlignHelper with PhrHelper {
  private object TAGEHistoryType {
    val Short   = 0
    val Medium  = 1
    val Long    = 2
    val Unknown = 3
  }
  def getUnhashedIdx(pc: PrunedAddr): UInt = pc(VAddrBits - 1, instOffsetBits)

  def getUnhashedTag(pc: PrunedAddr, tableId: Int): UInt = {
    def concatBits(bits: Seq[Bool]): UInt = if (bits.isEmpty) 0.U(1.W) else bits.foldLeft(0.U(0.W))(Cat(_, _))
    val tagPC = pc(VAddrBits - 1, instOffsetBits)
    val xorPC = tableId match {
      case TAGEHistoryType.Short =>
        concatBits(PCTagXorBitsForShortHistory.map(tagPC(_)))
      case TAGEHistoryType.Medium =>
        concatBits(PCTagXorBitsForMediumHistory.map(tagPC(_)))
      case TAGEHistoryType.Long =>
        concatBits(PCTagXorBitsForLongHistory.map(tagPC(_)))
      case _ =>
        concatBits(PCTagXorBitsForVeryLongHistory.map(tagPC(_)))
    }
    xorPC
  }
  def connectPcTag(partPc: UInt, tableId: Int): UInt = {
    require(tableId >= 0 && tableId <= 3, s"tableId must be in [0,3], got $tableId")
    def concatBits(bits: Seq[Bool]): UInt = if (bits.isEmpty) 0.U(1.W) else bits.foldLeft(0.U(0.W))(Cat(_, _))
    val tagPC = partPc
    val connectPC = tableId match {
      case TAGEHistoryType.Short =>
        concatBits(PCTagConcatBitsForShortHistory.map(tagPC(_)))
      case TAGEHistoryType.Medium =>
        concatBits(PCTagConcatBitsForMediumHistory.map(tagPC(_)))
      case TAGEHistoryType.Long =>
        concatBits(PCTagConcatBitsForLongHistory.map(tagPC(_)))
      case _ =>
        concatBits(PCTagConcatBitsForVeryLongHistory.map(tagPC(_)))
    }
    connectPC
  }

  def computeHashIdx(
      pc:         PrunedAddr,
      pathHist:   UInt,
      tablesInfo: Seq[MicroTageInfo],
      tableId:    Int
  ): UInt = {
    val histLen     = tablesInfo(tableId).HistoryLength
    val numSets     = tablesInfo(tableId).NumSets
    val compLen     = min(histLen, log2Ceil(numSets))
    val idxFh       = computeFoldedHist(pathHist, compLen)(histLen)
    val unhashedIdx = getUnhashedIdx(pc)
    val idx         = (unhashedIdx ^ idxFh)(log2Ceil(numSets) - 1, 0)
    idx
  }
  def computeHashTag(
      pc:         PrunedAddr,
      pathHist:   UInt,
      tablesInfo: Seq[MicroTageInfo],
      tableId:    Int
  ): UInt = {
    val histLen       = tablesInfo(tableId).HistoryLength
    val tagLen        = tablesInfo(tableId).TagWidth
    val histBitsInTag = tablesInfo(tableId).HistBitsInTag
    val tagCompLen    = min(histLen, histBitsInTag)
    val tagFh         = computeFoldedHist(pathHist, tagCompLen)(histLen)
    val altTagFh      = computeFoldedHist(pathHist, tagCompLen - 1)(histLen)
    val unhashedTag   = getUnhashedTag(pc, tableId)
    val unhashedIdx   = getUnhashedIdx(pc)
    val lowTag        = (unhashedTag ^ tagFh ^ (altTagFh << 1))(histBitsInTag - 1, 0)
    val highTag       = connectPcTag(unhashedIdx, tableId)
    val tag           = Cat(highTag, lowTag)(tagLen - 1, 0)
    // Temporarily expand the bit width to be consistent for easier processing.
    // This may later be changed to use SRAM storage
    tag.pad(log2Ceil(MaxTagLen))
  }

  def getBankId(index: UInt, numBanks: Int): UInt = {
    val bankId = index(log2Ceil(numBanks) - 1, 0)
    bankId
  }
  def getBankInnerIndex(index: UInt, numBanks: Int, numSets: Int): UInt = {
    val bankInnerIndex = index(log2Ceil(numSets) - 1, log2Ceil(numBanks))
    bankInnerIndex
  }
}
