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
import freechips.rocketchip.util.SeqToAugmentedSeq
import org.chipsalliance.cde.config.Parameters
import scala.math.min
import utility.XSPerfAccumulate
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.BasePredictor
import xiangshan.frontend.bpu.BasePredictorIO
import xiangshan.frontend.bpu.BpuTrain
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.phr.PhrAllFoldedHistories

/**
 * This module is the implementation of the TAGE (TAgged GEometric history length predictor).
 */
class MicroTage(implicit p: Parameters) extends BasePredictor with HasMicroTageParameters with Helpers {
  class MicroTageIO(implicit p: Parameters) extends BasePredictorIO {
    val foldedPathHist:         PhrAllFoldedHistories = Input(new PhrAllFoldedHistories(AllFoldedHistoryInfo))
    val foldedPathHistForTrain: PhrAllFoldedHistories = Input(new PhrAllFoldedHistories(AllFoldedHistoryInfo))
    val prediction:             Valid[MicroTageMeta]  = Output(Valid(new MicroTageMeta))
  }
  val io: MicroTageIO = IO(new MicroTageIO)
  io.resetDone := true.B

  /* *** submodules *** */
  private val tables = TableInfos.zipWithIndex.map {
    case (info, i) =>
      val t = Module(new MicroTageTable(
        numSets = info.NumSets,
        histLen = info.HistoryLength,
        tagLen = info.TagWidth,
        histBitsInTag = info.HistBitsInTag,
        numWays = TableNumWays(i),
        tableId = i
      )).io
      t
  }
  private val tickCounter = RegInit(0.U((TickWidth + 1).W))
  // Predict
  tables.foreach { t =>
    t.req.startPc        := io.startVAddr
    t.req.foldedPathHist := io.foldedPathHist
    t.usefulReset        := tickCounter(TickWidth)
  }

  io.prediction.valid          := tables.map(_.resp.valid).reduce(_ || _)
  io.prediction.bits.hitMap    := tables.map(_.resp.valid)
  io.prediction.bits.takenMap  := tables.map(_.resp.bits.taken)
  io.prediction.bits.hit       := tables.map(_.resp.valid).reduce(_ || _)
  io.prediction.bits.usefulMap := VecInit(tables.map(_.resp.bits.useful)).asUInt
  io.prediction.bits.tableMeta := tables(0).resp.bits.tableMeta
  private val takenCases       = tables.reverse.map(t => t.resp.valid -> t.resp.bits.taken)
  private val cfiPositionCases = tables.reverse.map(t => t.resp.valid -> t.resp.bits.cfiPosition)
  io.prediction.bits.taken       := MuxCase(false.B, takenCases)
  io.prediction.bits.cfiPosition := MuxCase(0.U(CfiPositionWidth.W), cfiPositionCases)

  private val trainNext = RegNext(io.train, 0.U.asTypeOf(Valid(new BpuTrain)))
  private val trainData = trainNext.bits
  private val trainMeta = trainNext.bits.meta.utage
  // private val trainValid = trainNext.valid
  private val trainValid = Wire(Bool())

  // ------------ MicroTage is only concerned with conditional branches ---------- //
  private val misPred = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
      (((b.bits.cfiPosition < trainMeta.cfiPosition) && b.bits.taken) ||
        ((b.bits.cfiPosition === trainMeta.cfiPosition) && (b.bits.taken ^ trainMeta.taken)) ||
        (b.bits.cfiPosition > trainMeta.cfiPosition)) && trainMeta.hit
  ))
  private val hasTaken = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional && b.bits.taken
  ))
  private val missHitTaken            = hasTaken.reduce(_ || _) && !trainMeta.hit
  private val missHitTakenCfiPosition = Mux1H(PriorityEncoderOH(hasTaken), trainData.branches.map(_.bits.cfiPosition))
  private val hitMisPred              = misPred.reduce(_ || _)
  private val hitMisPredCfiPosition   = Mux1H(PriorityEncoderOH(misPred), trainData.branches.map(_.bits.cfiPosition))

  private val needAllocated    = hitMisPred || missHitTaken
  private val allocCfiPosition = Mux(missHitTaken, missHitTakenCfiPosition, hitMisPredCfiPosition)

  private val hasPredBr = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional && (b.bits.cfiPosition === trainMeta.cfiPosition)
  )).reduce(_ || _)
  private val hasPredBrCorrect = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
      (b.bits.cfiPosition === trainMeta.cfiPosition)
      && (trainMeta.taken === b.bits.taken)
  )).reduce(_ || _)

  private val providerMask = PriorityEncoderOH(trainMeta.hitMap.reverse).reverse

  private val hitMask            = trainMeta.hitMap.asUInt
  private val lowerFillMask      = Mux(hitMask === 0.U, 0.U, hitMask | (hitMask - 1.U))
  private val usefulMask         = trainMeta.usefulMap
  private val allocCandidateMask = ~(lowerFillMask | usefulMask)
  private val allocMask          = PriorityEncoderOH(allocCandidateMask)

  when(tickCounter(TickWidth)) {
    tickCounter := 0.U
  }.elsewhen((allocMask === 0.U) && needAllocated && trainValid) {
    tickCounter := tickCounter + 1.U
  }

// The training logic consists of two operations: updating entries and
// allocating new ones.
//
// Update behavior:
// - If train_position < table_position: entry remains unchanged.
// - If train_position === table_position: value is adjusted based on
//   prediction outcome (increment or decrement).
// - If train_position > table_position: entry remains unchanged.
//
// Allocation behavior (triggered only on misprediction):
// - A new entry is allocated to a higher-level table.
// - Target the lowest such table with an available slot (useful == 0).
// - If no slot is available, allocation fails.
//
// Update rule: To reduce noise, updates occur only when positions match.
//              The direction (inc/dec) is determined by the training result.
//
// Allocation rule: The selected entry replaces an available slot.
//                  If no free slot exists, allocation fails.
//                  Each failure is recorded; after 8 consecutive failures,
//                  all 'useful' counters are reset to 0.
  private val trainTableMeta = trainMeta.tableMeta
  tables.zipWithIndex.foreach { case (t, i) =>
    t.update.valid            := ((allocMask(i) && needAllocated) || (providerMask(i) && hasPredBr)) && trainValid
    t.update.bits.startPc     := trainData.startVAddr
    t.update.bits.cfiPosition := Mux(allocMask(i) && needAllocated, allocCfiPosition, trainMeta.cfiPosition)
    t.update.bits.alloc       := allocMask(i) && needAllocated
    t.update.bits.allocTaken  := (hitMisPred && !trainMeta.taken) || missHitTaken
    t.update.bits.correct     := providerMask(i) && hasPredBr && hasPredBrCorrect
    t.update.bits.foldedPathHistForTrain := io.foldedPathHistForTrain
    t.update.bits.writeWayIdx :=
      Mux(
        providerMask(i) && hasPredBr,
        trainTableMeta.touchWayIdx,
        trainTableMeta.allocWayIdx
      )
  }

  // ==========================================================================
  // === PERF === Performance Counters Section
  // ==========================================================================
  private val misPredEQ = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
      ((b.bits.cfiPosition === trainMeta.cfiPosition) && (b.bits.taken ^ trainMeta.taken)) && trainMeta.hit
  )).reduce(_ || _)
  private val misPredLT = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
      ((b.bits.cfiPosition < trainMeta.cfiPosition) && b.bits.taken) && trainMeta.hit
  )).reduce(_ || _)
  private val misPredGT = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
      ((b.bits.cfiPosition > trainMeta.cfiPosition) && b.bits.taken) && trainMeta.hit
  )).reduce(_ || _)

  private val trainHasBr = VecInit(trainData.branches.map(b =>
    b.valid && b.bits.attribute.isConditional
  )).reduce(_ || _)

  private val predictionValidNext = RegEnable(io.prediction.valid, io.stageCtrl.s0_fire)
  private val predictionHitNext   = RegEnable(io.prediction.bits.hit, io.stageCtrl.s0_fire)
  XSPerfAccumulate("microtage_pred_valid", io.stageCtrl.s1_fire)
  XSPerfAccumulate("microtage_pred_hit", io.stageCtrl.s1_fire && predictionValidNext && predictionHitNext)
  XSPerfAccumulate("microtage_pred_miss", io.stageCtrl.s1_fire && !predictionHitNext)

  // === Training feedback stage ===
  XSPerfAccumulate("microtage_train_valid", trainValid)
  XSPerfAccumulate("microtage_train_br_valid", trainValid && trainHasBr)
  XSPerfAccumulate("microtage_train_br_taken_valid", trainValid && trainHasBr && hasTaken.reduce(_ || _))

  // train hit and correct
  private val hitPredBrCorrect   = trainValid && trainMeta.hit && hasPredBrCorrect
  private val hitPredBrWrong     = trainValid && trainMeta.hit && hitMisPred
  private val trainMiss          = trainValid && !trainMeta.hit
  private val trainMissBrCorrect = trainValid && !missHitTaken
  private val trainMissBrWrong   = trainValid && missHitTaken
  private val trainUnseenPredBr  = trainValid && !hasPredBr && trainHasBr
  XSPerfAccumulate("microtage_train_hit_predBr_correct", hitPredBrCorrect)
  XSPerfAccumulate("microtage_train_hit_predBr_wrong", hitPredBrWrong)

  XSPerfAccumulate("microtage_train_hit_EQ_predBr_wrong", trainValid && misPredEQ)
  XSPerfAccumulate("microtage_train_hit_LT_predBr_wrong", trainValid && misPredLT)
  XSPerfAccumulate("microtage_train_hit_GT_predBr_wrong", trainValid && misPredGT)

  def findMSBIndex(h: UInt): UInt = PriorityEncoder(Reverse(h))
  for (i <- 0 until NumTables) {
    XSPerfAccumulate(
      s"microtage_train_hit_EQ_predBr_mutil_hit_${i + 1}",
      trainValid && hasPredBr && (PopCount(trainMeta.hitMap) === (i + 1).U)
    )
    XSPerfAccumulate(
      s"microtage_train_hit_EQ_predBr_wrong_mutil_hit_${i + 1}",
      trainValid && misPredEQ && (PopCount(trainMeta.hitMap) === (i + 1).U)
    )

    for (j <- 0 until NumTables) {
      XSPerfAccumulate(
        s"microtage_train_hit_EQ_predBr_wrong_mutil_hit_${i + 1}_use_table${2 - j}",
        trainValid && misPredEQ && (findMSBIndex(trainMeta.hitMap.asUInt) === j.U) &&
          (PopCount(trainMeta.hitMap) === (i + 1).U)
      )
    }

    for (j <- 0 until NumTables) {
      XSPerfAccumulate(
        s"microtage_train_hit_EQ_predBr_mutil_hit_${i + 1}_use_table${2 - j}",
        trainValid && hasPredBr && (findMSBIndex(trainMeta.hitMap.asUInt) === j.U) &&
          (PopCount(trainMeta.hitMap) === (i + 1).U)
      )
    }

    XSPerfAccumulate(
      s"microtage_train_hit_EQ_predBr_wrong_mutil_hit_3_table${i}_correct",
      trainValid && misPredEQ &&
        (trainMeta.takenMap(i) =/= trainMeta.taken) && (PopCount(trainMeta.hitMap) === 3.U)
    )
  }

  // XSPerfAccumulate(
  //   s"microtage_alloc_table_0_way0",
  //   trainValid && allocMask(0) && needAllocated && (tables(0).update.bits.writeWayIdx === 0.U)
  // )
  // XSPerfAccumulate(
  //   s"microtage_alloc_table_0_way1",
  //   trainValid && allocMask(0) && needAllocated && (tables(0).update.bits.writeWayIdx === 1.U)
  // )

  XSPerfAccumulate("microtage_train_miss", trainMiss)
  XSPerfAccumulate("microtage_train_miss_Brcorrect", trainMissBrCorrect)
  XSPerfAccumulate("microtage_train_miss_Brwrong", trainMissBrWrong)
  XSPerfAccumulate("microtage_train_unseenPredBr", trainUnseenPredBr)

  // === Allocation / update events ===
  XSPerfAccumulate("microtage_need_alloc", trainValid && needAllocated)
  XSPerfAccumulate("microtage_alloc_triggered", trainValid && (allocMask.orR && needAllocated))
  XSPerfAccumulate("microtage_tick_reset", tickCounter(TickWidth))

  // === Debugging auxiliary statistics ===
  // Predictions with feedback, regardless of hit or miss
  XSPerfAccumulate("microtage_train_with_feedback", trainValid && hasPredBr)

  // Missed but trained (indicating the prediction structure might be saturated)
  XSPerfAccumulate("microtage_miss_trained", trainValid && trainMiss && needAllocated)

  // Per-table allocation occurrences (which table handles replacement)
  for (i <- 0 until NumTables) {
    XSPerfAccumulate(s"microtage_alloc_table_$i", trainValid && allocMask(i) && needAllocated)
  }

  // Multi-hit detection: more than one table valid simultaneously
  XSPerfAccumulate("microtage_table_multi_hit", io.stageCtrl.s0_fire && (PopCount(tables.map(_.resp.valid)) > 1.U))

  // No valid table response (potential predictor invalid path)
  XSPerfAccumulate("microtage_table_resp_invalid", io.stageCtrl.s0_fire && !tables.map(_.resp.valid).reduce(_ || _))

  // === PHR Test ===
  private val testIdxFhInfos = TableInfos.zipWithIndex.map {
    case (info, i) =>
      val t = new FoldedHistoryInfo(info.HistoryLength, min(log2Ceil(info.NumSets), info.HistoryLength))
      t
  }
  private val testTagFhInfos = TableInfos.zipWithIndex.map {
    case (info, i) =>
      val t = new FoldedHistoryInfo(info.HistoryLength, min(info.HistoryLength, info.HistBitsInTag))
      t
  }
  private val testAltTagFhInfos = TableInfos.zipWithIndex.map {
    case (info, i) =>
      val t = new FoldedHistoryInfo(info.HistoryLength, min(info.HistoryLength, info.HistBitsInTag - 1))
      t
  }

  def computeHash(startPc: UInt, allFh: PhrAllFoldedHistories, tableId: Int): (UInt, UInt) = {
    val unhashedIdx = getUnhashedIdx(startPc)
    val unhashedTag = getUnhashedTag(startPc)
    val idxFh       = allFh.getHistWithInfo(testIdxFhInfos(tableId)).foldedHist
    val tagFh       = allFh.getHistWithInfo(testTagFhInfos(tableId)).foldedHist
    val altTagFh    = allFh.getHistWithInfo(testAltTagFhInfos(tableId)).foldedHist
    val idx = if (testIdxFhInfos(tableId).FoldedLength < log2Ceil(TableInfos(tableId).NumSets)) {
      (unhashedIdx ^ Cat(idxFh, idxFh))(log2Ceil(TableInfos(tableId).NumSets) - 1, 0)
    } else {
      (unhashedIdx ^ idxFh)(log2Ceil(TableInfos(tableId).NumSets) - 1, 0)
    }
    val lowTag  = (unhashedTag ^ tagFh ^ (altTagFh << 1))(TableInfos(tableId).HistBitsInTag - 1, 0)
    val highTag = connectPcTag(unhashedIdx, tableId)
    val tag     = Cat(highTag, lowTag)(TableInfos(tableId).TagWidth - 1, 0)
    (idx, tag)
  }

  // private val (s0_idx, s0_tag) = computeHash(io.startVAddr.toUInt, io.foldedPathHist, 0)
  private val (s0_idx_table0, s0_tag_table0) = computeHash(io.startVAddr.toUInt, io.foldedPathHist, 0)
  private val (s0_idx_table1, s0_tag_table1) = computeHash(io.startVAddr.toUInt, io.foldedPathHist, 1)
  private val (s0_idx_table2, s0_tag_table2) = computeHash(io.startVAddr.toUInt, io.foldedPathHist, 2)
  io.prediction.bits.testPredIdx0 := s0_idx_table0
  io.prediction.bits.testPredTag0 := s0_tag_table0
  io.prediction.bits.testPredIdx1 := s0_idx_table1
  io.prediction.bits.testPredTag1 := s0_tag_table1
  io.prediction.bits.testPredIdx2 := s0_idx_table2
  io.prediction.bits.testPredTag2 := s0_tag_table2

  io.prediction.bits.testPredStartAddr := io.startVAddr.toUInt
  // io.prediction.bits.testFoldedPathHist := io.foldedPathHist

  private val (trainIdx0, trainTag0) = computeHash(trainData.startVAddr.toUInt, io.foldedPathHistForTrain, 0)
  private val (trainIdx1, trainTag1) = computeHash(trainData.startVAddr.toUInt, io.foldedPathHistForTrain, 1)
  private val (trainIdx2, trainTag2) = computeHash(trainData.startVAddr.toUInt, io.foldedPathHistForTrain, 2)

  XSPerfAccumulate("train_idx_hit", trainValid && (trainMeta.testPredIdx0 === trainIdx0))
  XSPerfAccumulate("train_tag_hit", trainValid && (trainMeta.testPredTag0 === trainTag0))
  XSPerfAccumulate("train_idx_miss", trainValid && (trainMeta.testPredIdx0 =/= trainIdx0))
  XSPerfAccumulate("train_tag_miss", trainValid && (trainMeta.testPredTag0 =/= trainTag0))
  // trainValid  := trainNext.valid && (trainMeta.testPredIdx === trainIdx) && (trainMeta.testPredTag === trainTag)
  trainValid := trainNext.valid && (trainMeta.testPredIdx0 === trainIdx0) && (trainMeta.testPredTag0 === trainTag0) &&
    (trainMeta.testPredIdx1 === trainIdx1) && (trainMeta.testPredTag1 === trainTag1) &&
    (trainMeta.testPredIdx2 === trainIdx2) && (trainMeta.testPredTag2 === trainTag2)
  XSPerfAccumulate(
    "train_idx_tag_hit",
    trainNext.valid && (trainMeta.testPredIdx0 === trainIdx0) &&
      (trainMeta.testPredTag0 === trainTag0) && (trainMeta.testPredStartAddr === trainNext.bits.startVAddr.toUInt)
  )
  XSPerfAccumulate(
    "train_idx_tag_miss",
    trainNext.valid && ((trainMeta.testPredIdx0 =/= trainIdx0) ||
      (trainMeta.testPredTag0 =/= trainTag0)) && (trainMeta.testPredStartAddr === trainNext.bits.startVAddr.toUInt)
  )
}
