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
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.SaturateCounter
import xiangshan.frontend.bpu.CompareMatrix
import xiangshan.frontend.bpu.history.phr.PhrAllFoldedHistories
import yunsuan.vector.alu.VIntFixpTable.table

class BypassShadowBuffer(
    val numSets:  Int,
    val numWay:   Int,
    val numEntry: Int = 16,  // 固定为16，保持最近16项
    val tableId:  Int,
    val NumBanks: Int = 4
)(implicit p: Parameters) extends MicroTageModule with Helpers {
  class BypassBufferIO extends MicroTageBundle {
    class Req extends MicroTageBundle {
      val readIndex: UInt = UInt(log2Ceil(MaxNumSets).W)
    }
    class Resp extends MicroTageBundle {
      val hit:         Vec[Bool]           = Vec(numWay, Bool())
      val readEntries: Vec[MicroTageEntry] = Vec(numWay, new MicroTageEntry)
    }
    class WriteReq extends MicroTageBundle {
      val writeIndex: UInt           = UInt(log2Ceil(MaxNumSets).W)
      val writeData:  Vec[MicroTageEntry] = Vec(numWay, new MicroTageEntry)
      val forceWrite: Bool           = Bool()
      val wMask:      UInt           = UInt(numWay.W)
    }
    val req:          Req             = Input(new Req)
    val resp:         Resp            = Output(new Resp)
    val train:        MicroTageTrain  = new MicroTageTrain(numWay, numSets)
    val tryWrite:     Valid[WriteReq] = Output(Valid(new WriteReq))
    val writeSuccess: Bool            = Input(Bool())
    val usefulReset:  Bool            = Input(Bool())
  }
  val io = IO(new BypassBufferIO)
  class BufferEntry extends Bundle {
    val valid:     Bool           = Bool()
    val entryData: Vec[Valid[MicroTageEntry]] = Vec(numWay, Valid(new MicroTageEntry))
    val index:     UInt           = UInt(log2Ceil(MaxNumSets).W)
  }

  class ReplaceItem extends Bundle {
    val valid:     Bool           = Bool()
    val age:       UInt           = UInt(log2Ceil(numEntry).W)
    val dirty:     Bool           = Bool()
  }

  private val entries       = RegInit(VecInit(Seq.fill(numEntry)(0.U.asTypeOf(new BufferEntry))))
  private val statusEntries = RegInit(VecInit(Seq.fill(numEntry)(0.U.asTypeOf(new ReplaceItem))))
  private val enqPtr = RegInit(0.U(log2Ceil(numEntry).W)) // Next available position
  private val enqMask = RegInit(0.U(numEntry.W))
  private val deqPtr = RegInit(0.U(log2Ceil(numEntry).W)) // Position ready for write-back

  // Banked useful registers
  private val usefulEntries = RegInit(VecInit.tabulate(NumBanks) { bankIdx =>
    VecInit(Seq.fill(numSets / NumBanks)(
      VecInit(Seq.fill(numWay)(0.U.asTypeOf(new SaturateCounter(UsefulWidth))))
    ))
  })

  // Prediction logic
  private val entryDataVec   = VecInit(entries.map(e => e.entryData))
  private val a0_entryHitOH  = Wire(Vec(numEntry, Bool()))
  a0_entryHitOH  := entries.map(e => (e.index === io.req.readIndex) && e.valid)

  private val a1_entryHitOH  = RegNext(a0_entryHitOH, 0.U.asTypeOf(Vec(numEntry, Bool())))
  private val a1_bufferEntry = Mux1H(a1_entryHitOH, entryDataVec)
  private val a1_hasHit      = a1_entryHitOH.reduce(_ || _)
  private val a1_microTageHitVec = a1_bufferEntry.map(e => e.valid && a1_hasHit)
  private val a1_microTageEntryVec = a1_bufferEntry.map(e => e.bits)
  io.resp.hit := a1_microTageHitVec
  io.resp.readEntries := a1_microTageEntryVec

  // Training logic - stage 0
  private val t0_trainIndex    = io.train.t0_trainIndex
  private val t0_entryHitOH    = Wire(Vec(numEntry, Bool()))
  t0_entryHitOH  := entries.map(e => (e.index === t0_trainIndex) && e.valid)
  private val t0_hasHit  = t0_entryHitOH.reduce(_ || _)
  private val t0_bufferEntry = Mux1H(t0_entryHitOH, entryDataVec)
  private val t0_microTageHitVec = VecInit(t0_bufferEntry.map(e => e.valid && t0_hasHit))
  private val t0_microTageEntryVec = VecInit(t0_bufferEntry.map(e => e.bits))
  // Access useful registers for t0 stage
  private val t0_bankIdx         = getBankId(t0_trainIndex, NumBanks)
  private val t0_bankOffset      = getBankInnerIndex(t0_trainIndex, NumBanks, numSets)
  private val t0_trainReadUseful = usefulEntries(t0_bankIdx)(t0_bankOffset)
  private val t0_hitBufferId     = OHToUInt(t0_entryHitOH)
  private val t0_trainReadEntries = t0_microTageEntryVec

  for (way <- 0 until numWay) {
    val entry  = t0_trainReadEntries(way)
    val useful = t0_trainReadUseful(way)
    io.train.t0_read(way).canGetPosition := t0_microTageHitVec(way)
    io.train.t0_read(way).cfiPosition    := entry.cfiPosition
    io.train.t0_read(way).useful         := useful.value
  }

  private val t1_trainIndex       = RegNext(t0_trainIndex)
  private val t1_hasHit           = RegNext(t0_hasHit, false.B)
  private val t1_microTageHitVec  = RegNext(t0_microTageHitVec)
  private val t1_hitBufferId      = RegNext(t0_hitBufferId)
  private val t1_trainReadEntries = RegNext(t0_trainReadEntries)
  private val t1_bufferWriteId    = Mux(t1_hasHit, t1_hitBufferId, enqPtr)

  // Access useful registers for t1 stage
  private val t1_bankIdx         = getBankId(t1_trainIndex, NumBanks)
  private val t1_bankOffset      = getBankInnerIndex(t1_trainIndex, NumBanks, numSets)
  private val t1_trainReadUseful = usefulEntries(t1_bankIdx)(t1_bankOffset)

  private val writeBufferValid      = WireDefault(VecInit(Seq.fill(numWay)(false.B)))
  private val newMicroTageEntryVec  = WireDefault(VecInit(Seq.fill(numWay)(0.U.asTypeOf(new MicroTageEntry))))
  for (way <- 0 until numWay) {
    val oldEntry       = t1_trainReadEntries(way)
    val oldTakenCtr    = oldEntry.takenCtr
    val updateTakenCtr = io.train.t1_update(way).bits.updateTakenCtr

    // Update logic: either allocation or update
    val doAlloc  = io.train.t1_alloc.valid && io.train.t1_alloc.bits.wayMask(way)
    val doUpdate = io.train.t1_update(way).valid && io.train.t1_update(way).bits.updateValid
    writeBufferValid(way) := doAlloc || doUpdate

    // New entry values
    newMicroTageEntryVec(way).valid := true.B
    newMicroTageEntryVec(way).tag   := io.train.t1_tag
    newMicroTageEntryVec(way).cfiPosition :=
      Mux(doAlloc, io.train.t1_alloc.bits.cfiPosition, io.train.t1_update(way).bits.updateCfiPosition)
    newMicroTageEntryVec(way).takenCtr := Mux(
      doAlloc,
      Mux(io.train.t1_alloc.bits.taken, TakenCounter.WeakPositive, TakenCounter.WeakNegative),
      // updateTakenCtr.getUpdate(io.train.t1_update(way).bits.updateTaken)
      Mux(
        t1_microTageHitVec(way) && (oldEntry.tag === io.train.t1_tag),
        oldTakenCtr.getUpdate(io.train.t1_update(way).bits.updateTaken),
        updateTakenCtr.getUpdate(io.train.t1_update(way).bits.updateTaken)
      )
    )
  }

  for (way <- 0 until numWay) {
    val doAlloc   = io.train.t1_alloc.valid && io.train.t1_alloc.bits.wayMask(way)
    val oldUseful = t1_trainReadUseful(way)
    val newUseful = Mux(
      doAlloc,
      // UsefulCounter.WeakPositive,
      if(tableId < NumTables/2) UsefulCounter.WeakNegative else UsefulCounter.WeakPositive,
      oldUseful.getUpdate(io.train.t1_update(way).bits.needUseful)
    )
    when(doAlloc || (io.train.t1_update(way).valid && io.train.t1_update(way).bits.usefulValid)) {
      t1_trainReadUseful(way) := newUseful
    }
  }

  // Useful counter reset logic
  when(io.usefulReset) {
    for (bankIdx <- 0 until NumBanks) {
      for (setIdx <- 0 until numSets / NumBanks) {
        for (wayIdx <- 0 until numWay) {
          val entry = usefulEntries(bankIdx)(setIdx)(wayIdx)
          if (tableId < NumTables/2) {
            usefulEntries(bankIdx)(setIdx)(wayIdx).value :=
              Mux(entry.value === 0.U, 0.U, entry.value - 1.U)
          } else {
            usefulEntries(bankIdx)(setIdx)(wayIdx).value := entry.value >> 1.U
          }
          // usefulEntries(bankIdx)(setIdx)(wayIdx).value := entry.value >> 1.U
        }
      }
    }
  }

  private val newBufferEntry = Wire(new BufferEntry)
  newBufferEntry.valid       := true.B
  newBufferEntry.index       := t1_trainIndex
  for(i <- 0 until numWay) {
    newBufferEntry.entryData(i).valid := writeBufferValid(i) || entries(t1_bufferWriteId).entryData(i).valid
    newBufferEntry.entryData(i).bits := Mux(writeBufferValid(i), newMicroTageEntryVec(i), entries(t1_bufferWriteId).entryData(i).bits)
  }

  private val t1_hasWrite = writeBufferValid.reduce(_ || _)
  when(t1_hasWrite) {
    entries(t1_bufferWriteId) := newBufferEntry
  }

  private val t1_ageVec = Wire(Vec(numEntry, UInt(log2Ceil(numEntry).W)))
  t1_ageVec := statusEntries.map(e => e.age)
  t1_ageVec(enqPtr) := (numEntry - 1).U

  // 更新规则：
  // 1. 被访问的项：timestamp = 15（最年轻）
  // 2. 未被访问的项：每个周期 timestamp = timestamp - 1（逐渐变老）
  // 3. 选择替换时：找timestamp最小的（最老）
  private val t1_compareMatrix   = CompareMatrix(t1_ageVec)
  private val t1_invalidEntryVec = VecInit(statusEntries.map(e => e.valid === false.B)).asUInt & ~enqMask
  private val t1_cleanEntryVec   = VecInit(statusEntries.map(e => e.valid && !e.dirty)).asUInt & ~enqMask
  private val t1_dirtyEntryVec   = VecInit(statusEntries.map(e => e.valid && e.dirty)).asUInt | enqMask

  private val t1_invalidEntryOH  = t1_compareMatrix.getLeastElementOH(VecInit(t1_invalidEntryVec.asBools))
  private val t1_cleanEntryOH    = t1_compareMatrix.getLeastElementOH(VecInit(t1_cleanEntryVec.asBools))
  private val t1_dirtyEntryOH    = t1_compareMatrix.getLeastElementOH(VecInit(t1_dirtyEntryVec.asBools))
  private val t1_invalidId = OHToUInt(t1_invalidEntryOH)
  private val t1_cleanId = OHToUInt(t1_cleanEntryOH)
  private val t1_dirtyId = OHToUInt(t1_dirtyEntryOH)
  private val t1_hasInValid = t1_invalidEntryVec.orR
  private val t1_hasInClean = t1_cleanEntryVec.orR
  private val t1_hasInDirty = t1_dirtyEntryVec.orR

  when(io.writeSuccess || !(statusEntries(deqPtr).dirty)) {
    deqPtr := t1_dirtyId
  }

  private val nexEnqPtr = Mux(t1_hasInValid, t1_invalidId, Mux(t1_hasInClean, t1_cleanId, t1_dirtyId))
  when(t1_hasWrite && (!t1_hasHit || (enqPtr === t1_bufferWriteId))) {
    enqPtr := nexEnqPtr
    enqMask := UIntToOH(nexEnqPtr)
  }

  when(io.writeSuccess || t1_hasWrite) {
    for (i <- 0 until numEntry) {
      statusEntries(i).valid := Mux(i.U === t1_bufferWriteId && t1_hasWrite, true.B, statusEntries(i).valid)
      statusEntries(i).dirty := Mux(
        i.U === t1_bufferWriteId && t1_hasWrite,
        true.B,
        Mux(i.U === deqPtr && io.writeSuccess, false.B, statusEntries(i).dirty)
      )
      statusEntries(i).age   :=
        Mux(
          t1_hasWrite,
          Mux(i.U === t1_bufferWriteId, (numEntry - 1).U, Mux(statusEntries(i).age === 0.U, 0.U, statusEntries(i).age - 1.U)),
          statusEntries(i).age
        )
    }
  }

  // Write-back control logic
  private val bufferDirtyVec = VecInit(statusEntries.map{e => e.dirty && e.valid})
  private val bufferCounter = RegNext(PopCount(bufferDirtyVec), 0.U(log2Ceil(numEntry).W))
  private val forceWrite    = bufferCounter >= (numEntry - numWay).U

  io.tryWrite.valid           := statusEntries(deqPtr).valid && statusEntries(deqPtr).dirty
  io.tryWrite.bits.writeIndex := entries(deqPtr).index
  io.tryWrite.bits.writeData  := entries(deqPtr).entryData.map(_.bits)
  io.tryWrite.bits.forceWrite := forceWrite && statusEntries(deqPtr).valid && statusEntries(deqPtr).dirty
  io.tryWrite.bits.wMask      := VecInit(entries(deqPtr).entryData.map(_.valid)).asUInt
}