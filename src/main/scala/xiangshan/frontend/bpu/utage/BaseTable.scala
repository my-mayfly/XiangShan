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
import utility.UIntToMask
import freechips.rocketchip.util.SeqToAugmentedSeq
import xiangshan.frontend.bpu.BranchInfo

class MicroBaseTable(
    val numSets:        Int
)(implicit p: Parameters) extends MicroTageModule with Helpers {
  class MicroBaseTableIO extends MicroTageBundle {
    class MicroBaseReq extends Bundle {
      val startPc:      PrunedAddr           = new PrunedAddr(VAddrBits)
    }
    class MicroBaseResp extends Bundle {
      val taken:        Bool    = Bool()
      val cfiPosition:  UInt    = UInt(CfiPositionWidth.W)
    }
    class MicroBaseUpdate extends Bundle {
      val startPc:    PrunedAddr             = new PrunedAddr(VAddrBits)
      val branches:   Vec[Valid[BranchInfo]] = Vec(ResolveEntryBranchNumber, Valid(new BranchInfo))
    }
    val req:    MicroBaseReq    = Input(new MicroBaseReq)
    val resp:   MicroBaseResp   = Output(new MicroBaseResp)
    val update: Valid[MicroBaseUpdate]  = Input(Valid(new MicroBaseUpdate))
  }
  class MicroBaseEntry() extends MicroTageBundle {
    val valid:       Bool    = Bool()
    val cfiPosition: UInt    = UInt(CfiPositionWidth.W)
    val counter:     SaturateCounter = new SaturateCounter(2)
  }
  val io        = IO(new MicroBaseTableIO)
  private val entries   = RegInit(VecInit(Seq.fill(numSets)(0.U.asTypeOf(Vec(32, new MicroBaseEntry)))))
  private val s0_idx    = (io.req.startPc.toUInt)(log2Ceil(numSets) + 6 - 1, 6)
  private val takenMask = UIntToMask((io.req.startPc.toUInt)(5,1), 32)
  private val readEntryVec  = entries(s0_idx)
  private val takenVec      = readEntryVec.map(entry => entry.valid && entry.counter.isPositive).asUInt
  private val realTakenVec  = (~takenMask) & takenVec
  private val readEntry = Mux1H(PriorityEncoderOH(realTakenVec), readEntryVec)
  private val hitTaken  = realTakenVec.orR

  io.resp.taken       := false.B // hitTaken
  io.resp.cfiPosition := 0.U(CfiPositionWidth.W) // readEntry.cfiPosition

  // update
  private val trainIdx = (io.update.bits.startPc.toUInt)(log2Ceil(numSets) + 6 - 1, 6)
  private val oldCounters = entries(trainIdx).map(_.counter)
  private val t1_branches = io.update.bits.branches
  private val t1_needTrainMask   = UIntToMask((io.update.bits.startPc.toUInt)(5,1), 32)
  private val t1_needTrainValid  = Wire(Vec(32, Bool()))
  private val t1_newEntry = Wire(Vec(32, new MicroBaseEntry))
  t1_needTrainValid.zip(t1_newEntry).zipWithIndex.foreach { case ((needUpdate, newEntry), position) =>
    val hitMask = t1_branches.map { branch =>
      branch.valid && branch.bits.attribute.isConditional && position.U === branch.bits.cfiPosition
    }
    val taken = Mux1H(hitMask, t1_branches.map(_.bits.taken))
    needUpdate    := hitMask.reduce(_ || _)
    newEntry.valid       := true.B
    newEntry.cfiPosition := position.U
    newEntry.counter.value     := oldCounters(position).getUpdate(taken)
  }
  for(i <- 0 until 32){
    when(t1_needTrainValid(i) && io.update.valid){
      entries(trainIdx)(i)  := t1_newEntry(i)
    }
  }
}