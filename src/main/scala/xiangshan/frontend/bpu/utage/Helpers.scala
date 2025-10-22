package xiangshan.frontend.bpu.utage

import chisel3._
import chisel3.util._
import xiangshan.HasXSParameter
import xiangshan.frontend.bpu.RotateHelper
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.phr.PhrAllFoldedHistories
import xiangshan.frontend.bpu.FoldedHistoryInfo

trait Helpers extends HasMicroTageParameters with RotateHelper {
  def getSetIndex(pc: PrunedAddr, hist: UInt, numSets: Int): UInt = {
    val setIdxWidth = log2Ceil(numSets)
    pc(setIdxWidth, instOffsetBits) ^ hist
  }
}