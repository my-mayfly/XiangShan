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

package xiangshan.frontend.bpu.abtb

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import scala.math.min
import utility.CircularQueuePtr
import utility.HasCircularQueuePtrHelper
import utility.ParallelPriorityMux
import utility.XSPerfAccumulate
import xiangshan.frontend.PrunedAddr
import xiangshan.frontend.bpu.CompareMatrix
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.SaturateCounter
import xiangshan.frontend.bpu.history.phr.PhrAllFoldedHistories

/**
 * Bypass Shadow Buffer - Write buffer and bypass cache
 *
 * Design goals:
 * 1. Cache recent TAGE table updates to avoid frequent SRAM write-backs (write coalescing)
 * 2. Provide RAW bypass (Read-After-Write) to ensure predictions see the latest updates
 * 3. Reduce write port pressure on TAGE tables, saving area and power
 *
 * Operation principles:
 * - Circular queue structure: new writes are enqueued, forced write-back to SRAM when nearly full
 * - When buffer is not full, attempt to write-back the oldest entry every cycle
 * - Entries already written back are not cleared, but marked as allocatable
 * - Priority mask ensures the newest copy is read when multiple copies of the same address exist
 * - Banked useful registers reduce fanout during reset
 */
class AheadBuffer(
    val numSets:  Int,
    val numWay:   Int,
    val numEntry: Int = 4
)(implicit p: Parameters) extends AheadBtbModule with HasCircularQueuePtrHelper with Helpers {
  class AheadBufferIO extends AheadBtbBundle {
    class Req extends AheadBtbBundle {
      val readIndex: UInt = UInt(log2Ceil(numSets).W)
    }
    class Resp extends AheadBtbBundle {
      val hit:         Bool               = Bool()
      val readEntries: Vec[AheadBtbEntry] = Vec(numWay, new AheadBtbEntry)
    }
    class WriteReq extends AheadBtbBundle {
      val writeIndex: UInt               = UInt(log2Ceil(numSets).W)
      val writeData:  Vec[AheadBtbEntry] = Vec(numWay, new AheadBtbEntry)
      val forceWrite: Bool               = Bool()
      val wMask:      UInt               = UInt(numWay.W)
    }
    val req:          Req                    = Input(new Req)
    val resp:         Resp                   = Output(new Resp)
    val train:        Valid[AheadBufferData] = Input(Valid(new AheadBufferData))
    val tryWrite:     Valid[WriteReq]        = Output(Valid(new WriteReq))
    val writeSuccess: Bool                   = Input(Bool())
  }
  val io = IO(new AheadBufferIO)
  class BufferEntry extends Bundle {
    val valid:     Bool               = Bool()
    val entryData: Vec[AheadBtbEntry] = Vec(numWay, new AheadBtbEntry)
    val index:     UInt               = UInt(log2Ceil(numSets).W)
    val wayMask:   UInt               = UInt(numWay.W)
  }

  class ReplaceItem extends Bundle {
    val valid: Bool = Bool()
    val dirty: Bool = Bool()
  }
  class BufferPtr(implicit p: Parameters) extends CircularQueuePtr[BufferPtr](numEntry) {}

  private val entries       = RegInit(VecInit(Seq.fill(numEntry)(0.U.asTypeOf(new BufferEntry))))
  private val statusEntries = RegInit(VecInit(Seq.fill(numEntry)(0.U.asTypeOf(new ReplaceItem))))
  private val enqPtr        = RegInit(0.U.asTypeOf(new BufferPtr)) // Next available position
  private val deqPtr        = RegInit(0.U.asTypeOf(new BufferPtr)) // Position ready for write-back
  private val priorityMask  = RegInit(0.U(numEntry.W))

  // Prediction logic
  private val entryDataVec = VecInit(entries.map(e => e.entryData))
  private val a0_entryHit  = Wire(Vec(numEntry, Bool()))
  a0_entryHit := entries.map(e => (e.index === io.req.readIndex) && e.valid)

  private val a0_chosenFirst = (a0_entryHit.asUInt & priorityMask).orR
  private val a1_chosenFirst = RegNext(a0_chosenFirst, false.B)
  private val a1_firstHit    = RegInit(VecInit(Seq.fill(numEntry)(false.B)))
  private val a1_entryHit    = RegInit(VecInit(Seq.fill(numEntry)(false.B)))
  a1_firstHit := (a0_entryHit.asUInt & priorityMask).asBools
  a1_entryHit := a0_entryHit
  private val a1_firstEntry  = ParallelPriorityMux(a1_firstHit.reverse, entryDataVec.reverse)
  private val a1_secondEntry = ParallelPriorityMux(a1_entryHit.reverse, entryDataVec.reverse)
  private val a1_bufferEntry = Mux(a1_chosenFirst, a1_firstEntry, a1_secondEntry)
  private val a1_hasHit      = a1_entryHit.reduce(_ || _)
  io.resp.hit         := a1_hasHit
  io.resp.readEntries := a1_bufferEntry

  // Training logic - stage 0
  private val t0_fire       = io.train.valid
  private val t0_trainIndex = io.train.bits.trainIndex
  private val t0_trainData  = io.train.bits.trainData
  private val t0_wayMask    = io.train.bits.wayMask
  private val t0_entryHit   = Wire(Vec(numEntry, Bool()))
  t0_entryHit := entries.map(e => (e.index === t0_trainIndex) && e.valid)

  private val t0_firstHit    = Wire(Vec(numEntry, Bool()))
  private val t0_chosenFirst = (t0_entryHit.asUInt & priorityMask).orR
  t0_firstHit := (t0_entryHit.asUInt & priorityMask).asBools
  private val t0_firstEntry  = ParallelPriorityMux(t0_firstHit.reverse, entryDataVec.reverse)
  private val t0_secondEntry = ParallelPriorityMux(t0_entryHit.reverse, entryDataVec.reverse)

  private val t0_hasHit      = t0_entryHit.reduce(_ || _)
  private val t0_bufferEntry = Mux(t0_chosenFirst, t0_firstEntry, t0_secondEntry)

  private val t0_cleanId =
    Mux(t0_chosenFirst, ~PriorityEncoder(t0_firstHit.reverse), ~PriorityEncoder(t0_entryHit.reverse))

  private val t0_hasWrite    = io.train.valid
  private val newBufferEntry = Wire(new BufferEntry)
  newBufferEntry.valid     := true.B
  newBufferEntry.index     := t0_trainIndex
  newBufferEntry.entryData := t0_trainData
  newBufferEntry.wayMask   := Mux(!t0_hasHit, t0_wayMask, Fill(NumWays, 1.U))

  when(t0_hasWrite) {
    entries(enqPtr.value) := newBufferEntry
    enqPtr                := enqPtr + 1.U
    priorityMask          := Fill(numEntry, 1.U(1.W)) >> ~enqPtr.value
  }

  private val isEmpty = deqPtr === enqPtr
  when(io.writeSuccess || (!statusEntries(deqPtr.value).dirty && !isEmpty)) {
    deqPtr := deqPtr + 1.U
  }

  when(io.writeSuccess) {
    statusEntries(deqPtr.value).dirty := false.B
  }

  when(t0_hasWrite) {
    statusEntries(enqPtr.value).valid := true.B
    statusEntries(enqPtr.value).dirty := true.B
  }

  when(t0_hasWrite && t0_hasHit) {
    statusEntries(t0_cleanId).dirty := false.B
  }

  private val forceWrite = RegInit(false.B)
  // Early writeback to distribute SRAM write operations and avoid write pressure concentration
  // that exacerbates read/write conflicts on individual entries.
  forceWrite                  := distanceBetween(enqPtr, deqPtr) > (numEntry - 2).U
  io.tryWrite.valid           := statusEntries(deqPtr.value).valid && statusEntries(deqPtr.value).dirty
  io.tryWrite.bits.writeIndex := entries(deqPtr.value).index
  io.tryWrite.bits.writeData  := entries(deqPtr.value).entryData
  io.tryWrite.bits.forceWrite := forceWrite && statusEntries(deqPtr.value).valid && statusEntries(deqPtr.value).dirty
  io.tryWrite.bits.wMask      := entries(deqPtr.value).wayMask

  // ==========================================================================
  // Buffer Performance Diagnostic Counters
  // ==========================================================================
  // 1. Replacement statistics
  XSPerfAccumulate("buffer_write_total", t0_hasWrite)
  XSPerfAccumulate("buffer_train_hit_total", t0_hasWrite && t0_hasHit)

  // 2. Bypass and error statistics
  private val multihit = (PopCount(t0_entryHit) > 1.U) && t0_fire
  XSPerfAccumulate("error_multihit", multihit)
}
