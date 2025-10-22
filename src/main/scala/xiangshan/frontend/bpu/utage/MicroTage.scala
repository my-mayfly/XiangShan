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
import xiangshan.frontend.bpu.BasePredictor
import xiangshan.frontend.bpu.BasePredictorIO
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.phr.PhrAllFoldedHistories

/**
 * This module is the implementation of the TAGE (TAgged GEometric history length predictor).
 */
class MicroTage(implicit p: Parameters) extends BasePredictor with HasMicroTageParameters with Helpers {
  class MicroTageIO(implicit p: Parameters) extends BasePredictorIO {
    val foldedPathHist:     PhrAllFoldedHistories = Input(new PhrAllFoldedHistories(AllFoldedHistoryInfo))
    val foldedPathHistForTrain: PhrAllFoldedHistories = Input(new PhrAllFoldedHistories(AllFoldedHistoryInfo))
    val prediction: Valid[MicroTageMeta] = Output(Valid(new MicroTageMeta))
  }
  val io: MicroTageIO = IO(new MicroTageIO)

  /* *** submodules *** */
  private val tables = TableInfos.zipWithIndex.map {
    case (info, i) =>
      val t = Module(new MicroTageTable(info.NumSets, info.HistoryLength, TagWidth)).io
      t
  }
  private val tickCounter = RegInit(0.U((TickWidth+1).W))
  // Predict
  tables.foreach { t =>
    t.req.startPc  := io.startVAddr
    t.req.foldedPathHist := io.foldedPathHist
    t.usefulReset        := tickCounter(TickWidth)
  }

  io.prediction.valid         := tables.map(_.resp.valid).reduce(_ || _)
  io.prediction.bits.hitMap   := tables.map(_.resp.valid)
  io.prediction.bits.takenMap := tables.map(_.resp.bits.taken)
  io.prediction.bits.hit      := tables.map(_.resp.valid).reduce(_ || _)
  private val takenCases = tables.reverse.map { t => t.resp.valid -> t.resp.bits.taken}
  private val cfiPositionCases = tables.reverse.map { t => t.resp.valid -> t.resp.bits.cfiPosition}
  io.prediction.bits.taken    := MuxCase(false.B, takenCases)
  io.prediction.bits.cfiPosition  := MuxCase(0.U(CfiPositionWidth.W), cfiPositionCases)


  private val train     = io.train.bits
  private val trainMeta = io.train.bits.meta.utage

  private val misPred = VecInit(train.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
    (((b.bits.cfiPosition < trainMeta.cfiPosition) && b.bits.taken) ||
      ((b.bits.cfiPosition === trainMeta.cfiPosition) && (b.bits.taken ^ trainMeta.taken)) ||
      (b.bits.cfiPosition > trainMeta.cfiPosition)) && trainMeta.hit
  ))
  private val hasTaken = VecInit(train.branches.map(b =>
    b.valid && b.bits.attribute.isConditional && b.bits.taken
  )).reduce(_ || _)
  private val missHit = hasTaken && !trainMeta.hit
  private val missHitCfiPosition  = Mux1H(PriorityEncoderOH(hasTaken), train.branches.map(_.bits.cfiPosition))
  private val hitMisPred = misPred.reduce(_ || _)
  private val hitMisPredCfiPosition = Mux1H(PriorityEncoderOH(misPred), train.branches.map(_.bits.cfiPosition))

  private val needAllocated = hitMisPred || missHit
  private val allocCfiPosition = Mux(missHit, missHitCfiPosition, hitMisPredCfiPosition)

  private val hasPredBr = VecInit(train.branches.map(b =>
    b.valid && b.bits.attribute.isConditional && (b.bits.cfiPosition === trainMeta.cfiPosition)
  )).reduce(_ || _)
  private val hasPredCorrect = VecInit(train.branches.map(b =>
    b.valid && b.bits.attribute.isConditional &&
     (b.bits.cfiPosition === trainMeta.cfiPosition)
      && (trainMeta.taken === b.bits.taken)
  )).reduce(_ || _)

  private val providerMask = PriorityEncoderOH(trainMeta.hitMap.reverse).reverse

  private val hitMask = trainMeta.hitMap.asUInt
  private val lowerFillMask  = Mux(hitMask === 0.U, 0.U, hitMask | (hitMask - 1.U))
  private val usefulMask     = trainMeta.usefulMap
  private val allocCandidateMask = ~(lowerFillMask & usefulMask)
  private val allocMask      = PriorityEncoderOH(allocCandidateMask)

  when(tickCounter(TickWidth)) {
    tickCounter := 0.U
  }.elsewhen((allocMask === 0.U) && needAllocated && io.train.valid) {
    tickCounter := tickCounter + 1.U
  }

  // 更新逻辑分为替换项和更新项。
  // 对于更新项：如果train_position < table_position, 当前项的值保持不变。
  // 对于更新项：如果train_position === table_position，当前项的值，按照预测要求进行增减。
  // 对于更新项：如果train_position > table_position， 当前项的值，保持当前项不变。

  // 更新项的逻辑：由训练时的对应项结果决定 trainCfiPosition === tableCfiPosition。
  // 分配项的逻辑：由选出的项直接进行替换


  // 更新逻辑：当train_position <= table_position, 当前项的
  // Update
  tables.zipWithIndex.foreach { case(t, i) =>
    t.update.valid        := ((allocMask(i) && needAllocated) || (providerMask(i) && hasPredBr)) && io.train.valid
    t.update.bits.startPc := io.train.bits.startVAddr
    t.update.bits.alloc   := allocMask(i) && needAllocated
    t.update.bits.correct := providerMask(i) && hasPredBr && hasPredCorrect
    t.update.bits.foldedPathHistForTrain := io.foldedPathHistForTrain
  }

  // ==========================================================================
  // === PERF === Performance Counters Section
  // ==========================================================================
  
  // === Prediction structure level ===

}