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
import utility.sram.SplittedSRAMTemplate
import xiangshan.frontend.bpu.WriteBuffer

/**
  * This module stores the ahead BTB entries.
  */
class AheadBtbBank(bankId: Int)(implicit p: Parameters) extends AheadBtbModule {
  class BankIO(implicit p: Parameters) extends AheadBtbBundle {
    val readReq:  DecoupledIO[BankReadReq] = Flipped(Decoupled(new BankReadReq))
    val readResp: BankReadResp             = Output(new BankReadResp)
    val writeReq: Valid[BankWriteReq]      = Flipped(Valid(new BankWriteReq))
  }
  val io: BankIO = IO(new BankIO)

  private val sram = Module(new SplittedSRAMTemplate(
    new AheadBtbEntry,
    set = NumSets,
    way = NumWays,
    waySplit = NumWays / 2,
    dataSplit = 1,
    shouldReset = true,
    singlePort = true,
    withClockGate = true,
    holdRead = true,
    hasMbist = hasMbist,
    hasSramCtl = hasSramCtl,
    suffix = Option("bpu_abtb")
  ))
  /* --------------------------------------------------------------------------------------------------------------
     read
     -------------------------------------------------------------------------------------------------------------- */

  sram.io.r.apply(
    valid = io.readReq.valid,
    setIdx = io.readReq.bits.setIdx
  )
  io.readReq.ready := sram.io.r.req.ready

  io.readResp.entries := sram.io.r.resp.data

  private val writeValid    = io.writeReq.valid
  private val writeEntryVec = io.writeReq.bits.entryData
  private val writeSetIdx   = io.writeReq.bits.setIdx
  private val writeWayMask  = io.writeReq.bits.wayMask

  sram.io.w.apply(
    valid = writeValid,
    data = writeEntryVec,
    setIdx = writeSetIdx,
    waymask = writeWayMask
  )
}
