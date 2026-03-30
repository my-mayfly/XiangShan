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

package xiangshan.frontend.bpu.abtb

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utility.XSPerfAccumulate
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.BasePredictor
import xiangshan.frontend.bpu.BasePredictorIO
import xiangshan.frontend.bpu.HasFastTrainIO
import xiangshan.frontend.bpu.Prediction

/**
 * This module is the implementation of the ahead BTB (Branch Target Buffer).
 */
class AheadBtb(implicit p: Parameters) extends BasePredictor with Helpers {
  class AheadBtbIO(implicit p: Parameters) extends BasePredictorIO with HasFastTrainIO {
    val redirectValid:      Bool                   = Input(Bool())
    val redirectPrevPartPc: UInt                   = Input(UInt(4.W))
    val redirectSimpleHist: UInt                   = Input(UInt(4.W))
    val overrideValid:      Bool                   = Input(Bool())
    val overridePrevPartPc: UInt                   = Input(UInt(4.W))
    val overrideSimpleHist: UInt                   = Input(UInt(4.W))
    val brPrediction:       Vec[Valid[Prediction]] = Output(Vec(NumAheadBtbPredictionEntries, Valid(new Prediction)))
    val jumpPrediction:     Valid[Prediction]      = Output(Valid(new Prediction))
    val abtbResult:    Vec[Valid[AheadBtbResult]] = Output(Vec(NumAheadBtbPredictionEntries, Valid(new AheadBtbResult)))
    val abtbResultPos: Vec[UInt]                  = Output(Vec(NumAheadBtbPredictionEntries, UInt(CfiPositionWidth.W)))
    val abtbPos:       Vec[UInt]                  = Output(Vec(NumAheadBtbPredictionEntries, UInt(CfiPositionWidth.W)))
    val meta:          AheadBtbMeta               = Output(new AheadBtbMeta)
    val debug_startPc: PrunedAddr                 = Output(PrunedAddr(VAddrBits))
  }
  val io: AheadBtbIO = IO(new AheadBtbIO)

  println(f"AheadBtb:")
  println(f"  Size(set, way, bank): $NumSets * $BrNumWays * $NumBanks = $BrNumEntries")
  println(f"  Address fields:")
  addrFields.show(indent = 4)

  private val brBanks     = Seq.tabulate(NumBanks)(i => Module(new AheadBtbBank(BrNumWays, i)))
  private val brReplacers = Seq.fill(NumBanks)(Module(new AheadBtbReplacer(BrNumWays)))

  private val jumpBanks     = Seq.tabulate(NumBanks)(i => Module(new AheadBtbBank(JumpNumWays, i)))
  private val jumpReplacers = Seq.fill(NumBanks)(Module(new AheadBtbReplacer(JumpNumWays)))

  private val resetDone = RegInit(false.B)
  when(brBanks.map(_.io.readReq.ready).reduce(_ && _) && jumpBanks.map(_.io.readReq.ready).reduce(_ && _)) {
    resetDone := true.B
  }
  io.resetDone := resetDone

  io.trainReady := true.B

  private val takenCounter = RegInit(
    VecInit.fill(NumBanks)(
      VecInit.fill(NumSets)(
        VecInit.fill(BrNumWays)(TakenCounter.Zero)
      )
    )
  )

  private val s0_fire = Wire(Bool())
  private val s1_fire = Wire(Bool())
  private val s2_fire = Wire(Bool())

  private val s1_ready = Wire(Bool())
  private val s2_ready = Wire(Bool())

  private val s1_flush = Wire(Bool())
  private val s2_flush = Wire(Bool())

  private val s1_valid = RegInit(false.B)
  private val s2_valid = RegInit(false.B)

  private val predictReqValid    = io.stageCtrl.s0_fire
  private val predictionSent     = io.stageCtrl.s1_fire
  private val redirectValid      = io.redirectValid
  private val redirectPrevPartPc = io.redirectPrevPartPc
  private val redirectSimpleHist = io.redirectSimpleHist
  private val overrideValid      = io.overrideValid
  private val overridePrevPartPc = io.overridePrevPartPc
  private val overrideSimpleHist = io.overrideSimpleHist

  s0_fire := io.enable && predictReqValid
  s1_fire := io.enable && s1_valid && s2_ready && predictReqValid
  s2_fire := io.enable && s2_valid && predictionSent

  s1_ready := s1_fire || !s1_valid
  s2_ready := s2_fire || !s2_valid || overrideValid || redirectValid

  s2_flush := redirectValid
  s1_flush := s2_flush

  when(s0_fire)(s1_valid := true.B)
    .elsewhen(s1_flush)(s1_valid := false.B)
    .elsewhen(s1_fire)(s1_valid := false.B)

  when(s1_fire)(s2_valid := true.B)
    .elsewhen(s2_flush)(s2_valid := false.B)
    .elsewhen(s2_fire)(s2_valid := false.B)

  /* --------------------------------------------------------------------------------------------------------------
     predict pipeline stage 0
     - get set index and bank index
     - send read request to selected bank
     -------------------------------------------------------------------------------------------------------------- */

  private val s0_previousStartPc = io.startPc
  private val s0_pprevPartPc     = RegEnable(s0_previousStartPc(7, 4), s0_fire)
  private val s0_realPPrevPartPc = MuxCase(
    s0_pprevPartPc,
    Seq(
      redirectValid -> redirectPrevPartPc,
      overrideValid -> overridePrevPartPc
    )
  )
  private val s0_simpleHist = RegEnable(s0_realPPrevPartPc ^ s0_previousStartPc(5, 2), s0_fire)
  private val s0_realSimpleHist = MuxCase(
    s0_simpleHist,
    Seq(
      redirectValid -> redirectSimpleHist,
      overrideValid -> overrideSimpleHist
    )
  )
  private val s0_hashIndex = Cat(0.U(4.W), s0_realSimpleHist(3, 0)) ^ s0_previousStartPc(8, 1)
  // private val s0_hashIndex = s0_previousStartPc(8, 1)
  private val s0_setIdx = s0_hashIndex(log2Ceil(NumSets * NumBanks) - 1, log2Ceil(NumBanks))
  // getSetIndex(s0_hashIndex)
  private val s0_bankIdx  = s0_hashIndex(log2Ceil(NumBanks) - 1, 0) // getBankIndex(s0_hashIndex)
  private val s0_bankMask = UIntToOH(s0_bankIdx)

  brBanks.zipWithIndex.foreach { case (b, i) =>
    b.io.readReq.valid       := predictReqValid && s0_bankMask(i)
    b.io.readReq.bits.setIdx := s0_setIdx
  }

  jumpBanks.zipWithIndex.foreach { case (b, i) =>
    b.io.readReq.valid       := predictReqValid && s0_bankMask(i)
    b.io.readReq.bits.setIdx := s0_setIdx
  }

  /* --------------------------------------------------------------------------------------------------------------
     predict pipeline stage 1
     - get the latest start pc for compare tag in s2
     - get entries from bank
     -------------------------------------------------------------------------------------------------------------- */

  private val s1_startPc = io.startPc

  private val s1_setIdx   = RegEnable(s0_setIdx, s0_fire)
  private val s1_bankIdx  = RegEnable(s0_bankIdx, s0_fire)
  private val s1_bankMask = RegEnable(s0_bankMask, s0_fire)

  private val s1_brEntries   = Mux1H(s1_bankMask, brBanks.map(_.io.readResp.entries))
  private val s1_jumpEntries = Mux1H(s1_bankMask, jumpBanks.map(_.io.readResp.entries))
  private val s1_ctrVec      = takenCounter(s1_bankIdx)(s1_setIdx)
  private val s1_ctrResult   = VecInit(s1_ctrVec.map(_.isPositive))
  private val s1_strongBias  = VecInit(s1_ctrVec.map(_.isSaturate))

  /* --------------------------------------------------------------------------------------------------------------
     predict pipeline stage 2 / 3
     - get taken counter result
     - compare tag and get taken mask
     - compare positions and find first taken entry
     - get target from found entry
     - output prediction
     - stage 3 is only for fast prediction when override is valid
     -------------------------------------------------------------------------------------------------------------- */

  private val s3_setIdx      = RegInit(0.U.asTypeOf(s1_setIdx))
  private val s3_bankIdx     = RegInit(0.U.asTypeOf(s1_bankIdx))
  private val s3_bankMask    = RegInit(0.U.asTypeOf(s1_bankMask))
  private val s3_brEntries   = RegInit(0.U.asTypeOf(s1_brEntries))
  private val s3_jumpEntries = RegInit(0.U.asTypeOf(s1_jumpEntries))
  private val s3_startPc     = RegInit(0.U.asTypeOf(s1_startPc))
  private val s3_ctrResult   = RegInit(VecInit.fill(BrNumWays)(false.B))
  private val s3_strongBias  = RegInit(VecInit.fill(BrNumWays)(false.B))
  private val s3_prevPartPc  = RegInit(0.U(4.W))
  private val s3_simpleHist  = RegInit(0.U(4.W))

  private val s1_realBrEntries   = Mux(overrideValid, s3_brEntries, s1_brEntries)
  private val s1_realJumpEntries = Mux(overrideValid, s3_jumpEntries, s1_jumpEntries)
  private val s1_tag             = getTag(s1_startPc)
  private val s1_realBrHitMask   = VecInit(s1_realBrEntries.map(entry => entry.valid && entry.tag === s1_tag))
  private val s1_realJumpHitMask = VecInit(s1_realJumpEntries.map(entry => entry.valid && entry.tag === s1_tag))
  private val s2_setIdx          = RegEnable(Mux(overrideValid, s3_setIdx, s1_setIdx), s1_fire)
  private val s2_bankIdx         = RegEnable(Mux(overrideValid, s3_bankIdx, s1_bankIdx), s1_fire)
  private val s2_bankMask        = RegEnable(Mux(overrideValid, s3_bankMask, s1_bankMask), s1_fire)
  private val s2_ctrResult       = RegEnable(Mux(overrideValid, s3_ctrResult, s1_ctrResult), s1_fire)
  private val s2_strongBias      = RegEnable(Mux(overrideValid, s3_strongBias, s1_strongBias), s1_fire)
  private val s2_brEntries       = RegEnable(s1_realBrEntries, s1_fire)
  private val s2_jumpEntries     = RegEnable(s1_realJumpEntries, s1_fire)
  private val s2_startPc         = RegEnable(s1_startPc, s1_fire)
  private val s2_brHitMask       = RegEnable(s1_realBrHitMask, s1_fire)
  private val s2_jumpHitMask     = RegEnable(s1_realJumpHitMask, s1_fire)
  private val s2_prevPartPc      = RegEnable(s1_startPc(7, 4), s1_fire)
  private val s2_simpleHist =
    RegEnable(Mux(overrideValid, s3_prevPartPc ^ s1_startPc(5, 2), s2_prevPartPc ^ s1_startPc(5, 2)), s1_fire)

  when(s2_fire) {
    s3_setIdx      := s2_setIdx
    s3_bankIdx     := s2_bankIdx
    s3_bankMask    := s2_bankMask
    s3_brEntries   := s2_brEntries
    s3_jumpEntries := s2_jumpEntries
    s3_startPc     := s2_startPc
    s3_ctrResult   := s2_ctrResult
    s3_strongBias  := s2_strongBias
    s3_prevPartPc  := s2_prevPartPc
    s3_simpleHist  := s2_simpleHist
  }

  private val s2_hit = s2_brHitMask.reduce(_ || _) || s2_jumpHitMask.reduce(_ || _)

  // When detect multi-hit, we need to invalidate one entry.
  private val (s2_brMultiHit, s2_brMultiHitWayIdx) = detectMultiHit(s2_brHitMask, s2_brEntries.map(_.position))
  private val jumpEntry                            = PriorityMux(s2_jumpHitMask, s2_jumpEntries)

  io.brPrediction.zipWithIndex.foreach { case (pred, i) =>
    pred.valid            := s2_valid && s2_brHitMask(i)
    pred.bits.taken       := s2_ctrResult(i)
    pred.bits.cfiPosition := s2_brEntries(i).position
    pred.bits.attribute   := s2_brEntries(i).attribute
    pred.bits.target      := getFullTarget(s2_startPc, s2_brEntries(i).targetLowerBits, s2_brEntries(i).targetCarry)
  }
  io.jumpPrediction.valid            := s2_valid && s2_jumpHitMask.reduce(_ || _)
  io.jumpPrediction.bits.taken       := true.B
  io.jumpPrediction.bits.cfiPosition := jumpEntry.position
  io.jumpPrediction.bits.attribute   := jumpEntry.attribute
  io.jumpPrediction.bits.target      := getFullTarget(s2_startPc, jumpEntry.targetLowerBits, jumpEntry.targetCarry)

  io.abtbResult.zipWithIndex.foreach { case (pred, i) =>
    pred.valid             := s2_valid && s2_brHitMask(i)
    pred.bits.taken        := s2_ctrResult(i)
    pred.bits.cfiPosition  := s2_brEntries(i).position
    pred.bits.attribute    := s2_brEntries(i).attribute
    pred.bits.isStrongBias := s2_strongBias(i)
  }
  io.abtbResultPos.zipWithIndex.map { case (pos, i) =>
    // Aggregates scattered position bits from wide entries for matrix comparison.
    pos := RegEnable(s1_realBrEntries(i).position, s1_fire)
  }
  io.abtbPos.zipWithIndex.map { case (pos, i) =>
    // Direct routing to MicroTage for pipelined comparison; potentially timing beneficial.
    pos := s1_realBrEntries(i).position
  }

  io.meta.valid      := s2_valid
  io.meta.setIdx     := s2_setIdx
  io.meta.bankMask   := s2_bankMask
  io.meta.prevPartPc := s2_prevPartPc
  io.meta.simpleHist := s2_simpleHist
  io.meta.brEntries.zipWithIndex.foreach { case (e, i) =>
    e.hit             := s2_brHitMask(i)
    e.attribute       := s2_brEntries(i).attribute
    e.position        := s2_brEntries(i).position
    e.targetLowerBits := s2_brEntries(i).targetLowerBits
  }
  io.meta.jumpEntries.zipWithIndex.foreach { case (e, i) =>
    e.hit             := s2_jumpHitMask(i)
    e.attribute       := s2_jumpEntries(i).attribute
    e.position        := s2_jumpEntries(i).position
    e.targetLowerBits := s2_jumpEntries(i).targetLowerBits
  }

  // used for check abtb output
  io.debug_startPc := s2_startPc

  /* --------------------------------------------------------------------------------------------------------------
     train pipeline stage 0
     - receive train request
     -------------------------------------------------------------------------------------------------------------- */

  private val t0_train = io.fastTrain.get.bits

  private val t0_fire = io.enable && io.fastTrain.get.valid && t0_train.finalPrediction.taken && t0_train.abtbMeta.valid

  /* --------------------------------------------------------------------------------------------------------------
     train pipeline stage 1
     - update taken counter
     - write a new entry or modify an existing entry if needed
     -------------------------------------------------------------------------------------------------------------- */

  private val t1_fire  = RegNext(t0_fire, init = false.B)
  private val t1_train = RegEnable(t0_train, t0_fire)

  private val t1_meta = t1_train.abtbMeta

  private val t1_setIdx   = t1_meta.setIdx
  private val t1_setMask  = UIntToOH(t1_setIdx)
  private val t1_bankMask = t1_meta.bankMask

  // use taken branch of s3 prediction to train abtb
  private val t1_trainTaken           = t1_train.finalPrediction.taken
  private val t1_trainPosition        = t1_train.finalPrediction.cfiPosition
  private val t1_trainAttribute       = t1_train.finalPrediction.attribute
  private val t1_trainTarget          = t1_train.finalPrediction.target
  private val t1_trainTargetLowerBits = getTargetLowerBits(t1_trainTarget)

  private val t1_condMask           = t1_meta.brEntries.map(e => e.hit && e.attribute.isConditional)
  private val t1_positionBeforeMask = t1_meta.brEntries.map(_.position < t1_trainPosition)
  private val t1_positionEqualMask  = t1_meta.brEntries.map(_.position === t1_trainPosition)

  takenCounter.zip(brBanks).zipWithIndex.foreach { case ((ctrsPerBank, bank), bankIdx) =>
    ctrsPerBank.zipWithIndex.foreach { case (ctrsPerSet, setIdx) =>
      val updateThisSet = t1_fire && t1_bankMask(bankIdx) && t1_setMask(setIdx)
      ctrsPerSet.zipWithIndex.foreach { case (ctr, wayIdx) =>
        val isCond    = t1_condMask(wayIdx)
        val posBefore = t1_positionBeforeMask(wayIdx)
        val posEqual  = t1_positionEqualMask(wayIdx)

        val needReset = bank.io.writeResp.valid && bank.io.writeResp.bits.needResetCtr &&
          setIdx.U === bank.io.writeResp.bits.setIdx && wayIdx.U === bank.io.writeResp.bits.wayIdx
        val needDecrease = updateThisSet && isCond && (!t1_trainTaken || t1_trainTaken && posBefore)
        val needIncrease = updateThisSet && isCond && t1_trainTaken && posEqual

        when(needReset)(ctr.resetWeakPositive())
          .elsewhen(needDecrease)(ctr.selfDecrease())
          .elsewhen(needIncrease)(ctr.selfIncrease())
      }
    }
  }

  // if the taken branch is not hit, we need write a new entry
  private val t1_brHitMask = t1_meta.brEntries.map { e =>
    e.hit && e.position === t1_trainPosition && e.attribute === t1_trainAttribute
  }
  private val t1_jumpHitMask = t1_meta.jumpEntries.map { e =>
    e.hit && e.position === t1_trainPosition && e.attribute === t1_trainAttribute
  }
  private val t1_brHit               = t1_brHitMask.reduce(_ || _)
  private val t1_brNeedWriteNewEntry = !t1_brHit && t1_trainAttribute.isConditional

  private val t1_jumpHit               = t1_jumpHitMask.reduce(_ || _)
  private val t1_jumpNeedWriteNewEntry = !t1_jumpHit && !t1_trainAttribute.isConditional

  // If the target of indirect branch is wrong, we need correct it.
  // Since the entry only stores the lower bits of the target, we only need to check the lower bits.
  private val t1_brHitMaskOH         = PriorityEncoderOH(t1_brHitMask)
  private val t1_brPredictInfo       = Mux1H(t1_brHitMaskOH, t1_meta.brEntries)
  private val t1_brTargetDiff        = t1_brPredictInfo.targetLowerBits =/= t1_trainTargetLowerBits
  private val t1_brNeedCorrectTarget = t1_brHit && t1_brTargetDiff

  private val t1_jumpHitMaskOH         = PriorityEncoderOH(t1_jumpHitMask)
  private val t1_jumpPredictInfo       = Mux1H(t1_jumpHitMaskOH, t1_meta.jumpEntries)
  private val t1_jumpTargetDiff        = t1_jumpPredictInfo.targetLowerBits =/= t1_trainTargetLowerBits
  private val t1_jumpNeedCorrectTarget = t1_jumpHit && t1_jumpTargetDiff

  // TODO: if the attribute of the taken branch is wrong, we need replace it or invalidate it

  private val t1_writeEntry = Wire(new AheadBtbEntry)
  t1_writeEntry.valid           := true.B
  t1_writeEntry.tag             := getTag(t1_train.startPc)
  t1_writeEntry.position        := t1_trainPosition
  t1_writeEntry.attribute       := t1_trainAttribute
  t1_writeEntry.targetLowerBits := t1_trainTargetLowerBits
  t1_writeEntry.targetCarry.foreach(_ := getTargetCarry(t1_train.startPc, t1_trainTarget)) // if (EnableTargetFix)

  brReplacers.foreach(_.io.replaceSetIdx := t1_setIdx)
  private val brVictimWayIdx = brReplacers.map(_.io.victimWayIdx)

  jumpReplacers.foreach(_.io.replaceSetIdx := t1_setIdx)
  private val jumpVictimWayIdx = jumpReplacers.map(_.io.victimWayIdx)

  brBanks.zipWithIndex.foreach { case (b, i) =>
    when(t1_fire && t1_brNeedWriteNewEntry && t1_bankMask(i)) {
      b.io.writeReq.valid             := true.B
      b.io.writeReq.bits.needResetCtr := true.B
      b.io.writeReq.bits.setIdx       := t1_setIdx
      b.io.writeReq.bits.wayIdx       := brVictimWayIdx(i)
      b.io.writeReq.bits.entry        := t1_writeEntry
    }.elsewhen(t1_fire && t1_brNeedCorrectTarget && t1_bankMask(i)) {
      b.io.writeReq.valid             := true.B
      b.io.writeReq.bits.needResetCtr := false.B
      b.io.writeReq.bits.setIdx       := t1_setIdx
      b.io.writeReq.bits.wayIdx       := OHToUInt(t1_brHitMaskOH)
      b.io.writeReq.bits.entry        := t1_writeEntry
    }.elsewhen(s2_valid && s2_brMultiHit && s2_bankMask(i)) {
      b.io.writeReq.valid             := true.B
      b.io.writeReq.bits.needResetCtr := true.B
      b.io.writeReq.bits.setIdx       := s2_setIdx
      b.io.writeReq.bits.wayIdx       := s2_brMultiHitWayIdx
      b.io.writeReq.bits.entry        := 0.U.asTypeOf(new AheadBtbEntry)
    }.otherwise {
      b.io.writeReq.valid := false.B
      b.io.writeReq.bits  := 0.U.asTypeOf(new BankWriteReq(BrNumWays))
    }
  }

  jumpBanks.zipWithIndex.foreach { case (b, i) =>
    when(t1_fire && t1_jumpNeedWriteNewEntry && t1_bankMask(i)) {
      b.io.writeReq.valid             := true.B
      b.io.writeReq.bits.needResetCtr := true.B
      b.io.writeReq.bits.setIdx       := t1_setIdx
      b.io.writeReq.bits.wayIdx       := jumpVictimWayIdx(i)
      b.io.writeReq.bits.entry        := t1_writeEntry
    }.elsewhen(t1_fire && t1_jumpNeedCorrectTarget && t1_bankMask(i)) {
      b.io.writeReq.valid             := true.B
      b.io.writeReq.bits.needResetCtr := false.B
      b.io.writeReq.bits.setIdx       := t1_setIdx
      b.io.writeReq.bits.wayIdx       := OHToUInt(t1_jumpHitMaskOH)
      b.io.writeReq.bits.entry        := t1_writeEntry
    }.otherwise {
      b.io.writeReq.valid := false.B
      b.io.writeReq.bits  := 0.U.asTypeOf(new BankWriteReq(JumpNumWays))
    }
  }

  brReplacers.zipWithIndex.foreach { case (r, i) =>
    r.io.readValid   := t1_fire && t1_bankMask(i)
    r.io.readSetIdx  := t1_setIdx
    r.io.readWayMask := t1_brHitMaskOH
  }

  jumpReplacers.zipWithIndex.foreach { case (r, i) =>
    r.io.readValid   := t1_fire && t1_bankMask(i)
    r.io.readSetIdx  := t1_setIdx
    r.io.readWayMask := t1_jumpHitMaskOH
  }

  brReplacers.zip(brBanks).foreach { case (r, b) =>
    r.io.writeValid  := b.io.writeResp.valid
    r.io.writeSetIdx := b.io.writeResp.bits.setIdx
    r.io.writeWayIdx := b.io.writeResp.bits.wayIdx
  }

  jumpReplacers.zip(jumpBanks).foreach { case (r, b) =>
    r.io.writeValid  := b.io.writeResp.valid
    r.io.writeSetIdx := b.io.writeResp.bits.setIdx
    r.io.writeWayIdx := b.io.writeResp.bits.wayIdx
  }

  /* --------------------------------------------------------------------------------------------------------------
     performance counter
     -------------------------------------------------------------------------------------------------------------- */

  private val s2_hitMask = s2_brHitMask ++ s2_jumpHitMask
  XSPerfAccumulate("predict_req_num", predictReqValid)
  XSPerfAccumulate("predict_num", s2_fire)
  XSPerfAccumulate("predict_hit", s2_fire && s2_hit)
  XSPerfAccumulate("predict_miss", s2_fire && !s2_hit)
  XSPerfAccumulate("predict_hit_entry_num", Mux(s2_fire, PopCount(s2_hitMask), 0.U))
  XSPerfAccumulate("predict_brMulti_hit", s2_fire && s2_brMultiHit)

  XSPerfAccumulate("train_req_num", io.fastTrain.get.valid)
  XSPerfAccumulate("train_num", t1_fire)
  XSPerfAccumulate("train_actual_taken", t1_fire && t1_trainTaken)
  XSPerfAccumulate("train_actual_not_taken", t1_fire && !t1_trainTaken)

  XSPerfAccumulate(
    "total_br_write",
    t1_fire && (t1_brNeedWriteNewEntry || t1_brNeedCorrectTarget) || s2_valid && s2_brMultiHit
  )
  XSPerfAccumulate("train_br_write_new_entry", t1_fire && t1_brNeedWriteNewEntry)
  XSPerfAccumulate("train_br_correct_target", t1_fire && t1_brNeedCorrectTarget)
  XSPerfAccumulate(
    "train_write_conflict",
    t1_fire && (t1_brNeedWriteNewEntry || t1_brNeedCorrectTarget) && s2_valid && s2_brMultiHit
  )
}
