// Copyright (c) 2024-2025 Beijing Institute of Open Source Chip (BOSC)
// Copyright (c) 2020-2025 Institute of Computing Technology, Chinese Academy of Sciences
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
import org.chipsalliance.cde.config.Parameters
import scala.math.min
import utility.XSPerfAccumulate
import xiangshan.backend.datapath.DataConfig.VAddrBits
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.SaturateCounter
import xiangshan.frontend.bpu.phr.PhrAllFoldedHistories

class MicroTageTable(
    val numSets:       Int,
    val histLen:       Int,
    val tagLen:        Int,
    val histBitsInTag: Int,
    val numWays:       Int,
    val tableId:       Int
)(implicit p: Parameters) extends MicroTageModule with Helpers {
  class MicroTageTableIO extends MicroTageBundle {
    class MicroTageReq extends Bundle {
      val startPc:        PrunedAddr            = new PrunedAddr(VAddrBits)
      val foldedPathHist: PhrAllFoldedHistories = Input(new PhrAllFoldedHistories(AllFoldedHistoryInfo))
    }
    class MicroTageResp extends Bundle {
      val taken:       Bool      = Bool()
      val cfiPosition: UInt      = UInt(CfiPositionWidth.W)
      val useful:      Bool      = Bool()
      val tableMeta:   TableMeta = new TableMeta
    }
    class MicroTageUpdate extends Bundle {
      val startPc:                PrunedAddr            = new PrunedAddr(VAddrBits)
      val cfiPosition:            UInt                  = UInt(CfiPositionWidth.W)
      val alloc:                  Bool                  = Bool()
      val allocTaken:             Bool                  = Bool()
      val correct:                Bool                  = Bool()
      val foldedPathHistForTrain: PhrAllFoldedHistories = new PhrAllFoldedHistories(AllFoldedHistoryInfo)
      val writeWayIdx:            UInt                  = UInt(log2Ceil(numWays).W)
    }
    val req:         MicroTageReq           = Input(new MicroTageReq)
    val resp:        Valid[MicroTageResp]   = Output(Valid(new MicroTageResp))
    val update:      Valid[MicroTageUpdate] = Input(Valid(new MicroTageUpdate))
    val usefulReset: Bool                   = Input(Bool())
  }
  class MicroTageEntry() extends MicroTageBundle {
    val valid:       Bool            = Bool()
    val tag:         UInt            = UInt(tagLen.W)
    val takenCtr:    SaturateCounter = new SaturateCounter(TakenCtrWidth)
    val cfiPosition: UInt            = UInt(CfiPositionWidth.W)
    val useful:      SaturateCounter = new SaturateCounter(2)
  }
  val io              = IO(new MicroTageTableIO)
  private val entries = RegInit(VecInit(Seq.fill(numSets)(0.U.asTypeOf(Vec(numWays, new MicroTageEntry)))))
  private val usefulEntries =
    RegInit(VecInit(Seq.fill(numSets)(VecInit(Seq.fill(numWays)(0.U.asTypeOf(new SaturateCounter(2)))))))

  val idxFhInfo    = new FoldedHistoryInfo(histLen, min(log2Ceil(numSets), histLen))
  val tagFhInfo    = new FoldedHistoryInfo(histLen, min(histLen, histBitsInTag))
  val altTagFhInfo = new FoldedHistoryInfo(histLen, min(histLen, histBitsInTag - 1))

  def computeHash(startPc: UInt, allFh: PhrAllFoldedHistories, tableId: Int): (UInt, UInt) = {
    val unhashedIdx = getUnhashedIdx(startPc)
    val unhashedTag = getUnhashedTag(startPc)
    val idxFh       = allFh.getHistWithInfo(idxFhInfo).foldedHist
    val tagFh       = allFh.getHistWithInfo(tagFhInfo).foldedHist
    val altTagFh    = allFh.getHistWithInfo(altTagFhInfo).foldedHist
    val idx = if (idxFhInfo.FoldedLength < log2Ceil(numSets)) {
      (unhashedIdx ^ Cat(idxFh, idxFh))(log2Ceil(numSets) - 1, 0)
    } else {
      (unhashedIdx ^ idxFh)(log2Ceil(numSets) - 1, 0)
    }
    val lowTag  = (unhashedTag ^ tagFh ^ (altTagFh << 1))(histBitsInTag - 1, 0)
    val highTag = connectPcTag(unhashedIdx, tableId)
    val tag     = Cat(highTag, lowTag)(tagLen - 1, 0)
    (idx, tag)
  }

  // predict
  private val (s0_idx, s0_tag) = computeHash(io.req.startPc.toUInt, io.req.foldedPathHist, tableId)
  private val readEntries      = entries(s0_idx)
  private val readHits         = VecInit(readEntries.map(entry => (entry.tag === s0_tag) && entry.valid))
  private val readEntry        = Mux1H(PriorityEncoderOH(readHits), readEntries)
  private val readUsefuls      = usefulEntries(s0_idx).map(_.isPositive)

  io.resp.valid            := readHits.reduce(_ || _)
  io.resp.bits.taken       := readEntry.takenCtr.isPositive
  io.resp.bits.cfiPosition := readEntry.cfiPosition
  io.resp.bits.useful      := readUsefuls.reduce(_ && _)
  io.resp.bits.tableMeta   := 0.U.asTypeOf(new TableMeta)
  io.resp.bits.tableMeta.allocWayIdx := selectWriteWayIdx(
    numWays,
    VecInit(readEntries.map(_.valid)),
    VecInit(readUsefuls)
  )
  io.resp.bits.tableMeta.touchWayIdx := PriorityEncoder(readHits)

  // train
  private val (trainIdx, trainTag) =
    computeHash(io.update.bits.startPc.toUInt, io.update.bits.foldedPathHistForTrain, tableId)
  private val wayIdx      = io.update.bits.writeWayIdx
  private val oldCtr      = entries(trainIdx)(wayIdx).takenCtr
  private val oldUseful   = usefulEntries(trainIdx)(wayIdx)
  private val updateEntry = Wire(new MicroTageEntry)
  updateEntry.valid := true.B
  updateEntry.tag   := trainTag
  updateEntry.takenCtr.value := Mux(
    io.update.bits.alloc,
    oldCtr.getNeutral,
    // Mux(io.update.bits.allocTaken, oldCtr.getWeakPositive, oldCtr.getWeakNegative),
    oldCtr.getUpdate(io.update.bits.correct)
  )
  updateEntry.cfiPosition := io.update.bits.cfiPosition
  // updateEntry.useful         := Mux(io.update.bits.alloc || io.update.bits.correct, true.B, false.B)
  updateEntry.useful.value := Mux(
    io.update.bits.alloc,
    oldUseful.getNeutral,
    oldUseful.getUpdate(io.update.bits.correct)
  )

  when(io.update.valid) {
    entries(trainIdx)(wayIdx)       := updateEntry
    usefulEntries(trainIdx)(wayIdx) := updateEntry.useful
  }

  when(io.usefulReset) {
    usefulEntries.foreach(_.foreach(_.value := 0.U))
  }
  // Per-index access distribution
  for (i <- 0 until numSets) {
    XSPerfAccumulate(f"update_idx_access_$i", (trainIdx === i.U) && io.update.valid)
  }

  for (i <- 0 until numSets) {
    XSPerfAccumulate(
      f"pred_weak_taken_${i}",
      readHits(0) && (s0_idx === i.U) && entries(s0_idx)(0).takenCtr.value === (1 << (TakenCtrWidth - 1)).U
    )
    XSPerfAccumulate(
      f"pred_weak_notaken_${i}",
      readHits(0) && (s0_idx === i.U) && entries(s0_idx)(0).takenCtr.value === ((1 << (TakenCtrWidth - 1)) - 1).U
    )
    XSPerfAccumulate(
      f"pred_strong_taken_${i}",
      readHits(0) && (s0_idx === i.U) && entries(s0_idx)(0).takenCtr.value === ((1 << TakenCtrWidth) - 1).U
    )
    XSPerfAccumulate(
      f"pred_strong_notaken_${i}",
      readHits(0) && (s0_idx === i.U) && entries(s0_idx)(0).takenCtr.value === 0.U
    )
  }
  // private val predHist  =
  // private val testHistEntries  = RegInit(VecInit(Seq.fill(numSets)(0.U(32.W))))
  // when(io.update.valid) {
  //   testHistEntries(trainIdx) :=
  // }
}
