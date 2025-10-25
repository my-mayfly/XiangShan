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
import xiangshan.frontend.bpu.FoldedHistoryInfo
import xiangshan.frontend.bpu.SaturateCounter
import xiangshan.frontend.bpu.phr.PhrAllFoldedHistories
class TableMeta(implicit p: Parameters) extends MicroTageBundle {
  val touchWayIdx: UInt = UInt(log2Ceil(MaxNumWays).W)
  val allocWayIdx: UInt = UInt(log2Ceil(MaxNumWays).W)
}

class MicroTageMeta(implicit p: Parameters) extends MicroTageBundle {
  val hitMap:    Vec[Bool] = Vec(NumTables, Bool())
  val usefulMap: UInt      = UInt(NumTables.W)
  // val usefulMap:   Vec[Bool] = Vec(NumTables, Bool())
  val takenMap:    Vec[Bool] = Vec(NumTables, Bool())
  val hit:         Bool      = Bool()
  val taken:       Bool      = Bool()
  val cfiPosition: UInt      = UInt(CfiPositionWidth.W)
  val tableMeta:   TableMeta = new TableMeta
  // val writeWay:    UInt      = UInt(log2Ceil(MaxNumWays).W)

  val testPredIdx0:      UInt = UInt(TestPredIdx0Width.W)
  val testPredTag0:      UInt = UInt(TestPredTag0Width.W)
  val testPredIdx1:      UInt = UInt(TestPredIdx1Width.W)
  val testPredTag1:      UInt = UInt(TestPredTag1Width.W)
  val testPredIdx2:      UInt = UInt(TestPredIdx2Width.W)
  val testPredTag2:      UInt = UInt(TestPredTag2Width.W)
  val testPredStartAddr: UInt = UInt(VAddrBits.W)
//  val testFoldedPathHist: PhrAllFoldedHistories = new PhrAllFoldedHistories(AllFoldedHistoryInfo)

  // val pred_s0_idx: UInt      = UInt(7.W)
  // val pred_s0_Tag: UInt      = UInt(4.W)
  // val pred_s0_startAddr: UInt = UInt(VAddrBits.W)
}
