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

import chisel3.util._
import xiangshan.frontend.bpu.HasBpuParameters
import xiangshan.frontend.bpu.MicroTageInfo
import xiangshan.frontend.bpu.TageTableInfo

case class MicroTageParameters(
    TableInfos: Seq[MicroTageInfo] = Seq(
      new MicroTageInfo(128, 6, 6, 15),
      new MicroTageInfo(64, 16, 8, 18),
      new MicroTageInfo(64, 32, 16, 24)
    ),
    MaxNumWays:    Int = 1,
    TakenCtrWidth: Int = 3,
    TagWidth:      Int = 13,
    NumTables:     Int = 3,
    TickWidth:     Int = 3
) {
  require(MaxNumWays == 1 || MaxNumWays == 2, "MaxNumWays must be either 1 or 2")
}

trait HasMicroTageParameters extends HasBpuParameters {
  val utageParameters: MicroTageParameters = MicroTageParameters()
  def TableInfos:      Seq[MicroTageInfo]  = utageParameters.TableInfos
  def MaxNumWays:      Int                 = utageParameters.MaxNumWays
  def TakenCtrWidth:   Int                 = utageParameters.TakenCtrWidth
  def TagWidth:        Int                 = utageParameters.TagWidth
  def NumTables:       Int                 = utageParameters.NumTables
  def TickWidth:       Int                 = utageParameters.TickWidth

  def TableNumWays:      Seq[Int] = Seq(2, 1, 1)
  def TestPredIdx0Width: Int      = log2Ceil(TableInfos(0).NumSets)
  def TestPredTag0Width: Int      = TableInfos(0).TagWidth
  def TestPredIdx1Width: Int      = log2Ceil(TableInfos(1).NumSets)
  def TestPredTag1Width: Int      = TableInfos(1).TagWidth
  def TestPredIdx2Width: Int      = log2Ceil(TableInfos(2).NumSets)
  def TestPredTag2Width: Int      = TableInfos(2).TagWidth
  // abtb can only be fast-trained, we don't have continous predict block on resolve
  def EnableFastTrain: Boolean = false

  def PCTagHashBitsForShortHistory:  Seq[Int] = Seq(15, 13, 11, 9, 8, 7, 5, 3, 1)
  def PCTagHashBitsForMediumHistory: Seq[Int] = Seq(16, 15, 13, 11, 10, 8, 6, 4, 2, 1)
  def PCTagHashXorPairsForLongHistory: Seq[(Int, Int)] = Seq(
    (17, 3),
    (16, 1),
    (15, 11),
    (13, 9),
    (12, 8),
    (10, 6),
    (7, 4),
    (5, 2)
  )
  def PCTagHashBitsDefault: Seq[Int] = Seq(1, 0)
}

// sealed trait TAGEHistoryType
// object TAGEHistoryType {
//   case object Short   extends TAGEHistoryType
//   case object Medium  extends TAGEHistoryType
//   case object Long    extends TAGEHistoryType
//   case object Unknown extends TAGEHistoryType
// }
