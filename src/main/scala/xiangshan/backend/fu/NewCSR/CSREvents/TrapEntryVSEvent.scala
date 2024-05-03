package xiangshan.backend.fu.NewCSR.CSREvents

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import utility.{SignExt, ZeroExt}
import xiangshan.{ExceptionNO, HasXSParameter}
import xiangshan.ExceptionNO._
import xiangshan.backend.fu.NewCSR.CSRBundles.{CauseBundle, OneFieldBundle, PrivState}
import xiangshan.backend.fu.NewCSR.CSRConfig.{VaddrMaxWidth, XLEN}
import xiangshan.backend.fu.NewCSR.CSRDefines.SatpMode
import xiangshan.backend.fu.NewCSR._
import xiangshan.backend.fu.util.CSRConst


class TrapEntryVSEventOutput extends Bundle with EventUpdatePrivStateOutput with EventOutputBase  {

  val vsstatus = ValidIO((new SstatusBundle ).addInEvent(_.SPP, _.SPIE, _.SIE))
  val vsepc    = ValidIO((new Epc           ).addInEvent(_.ALL))
  val vscause  = ValidIO((new CauseBundle   ).addInEvent(_.Interrupt, _.ExceptionCode))
  val vstval   = ValidIO((new OneFieldBundle).addInEvent(_.ALL))

  def getBundleByName(name: String): Valid[CSRBundle] = {
    name match {
      case "vsstatus" => this.vsstatus
      case "vsepc"    => this.vsepc
      case "vscause"  => this.vscause
      case "vstval"   => this.vstval
    }
  }
}

class TrapEntryVSEventInput(implicit val p: Parameters) extends Bundle with HasXSParameter {
  val vsstatus = Input(new SstatusBundle)
  val trapPc = Input(UInt(VaddrMaxWidth.W))
  val privState = Input(new PrivState)
  val isInterrupt = Input(Bool())
  val trapVec = Input(UInt(64.W))
  val isCrossPageIPF = Input(Bool())
  val memExceptionVAddr = Input(UInt(VAddrBits.W))
  val memExceptionGPAddr = Input(UInt(GPAddrBits.W))
  // always current privilege
  val iMode = Input(new PrivState())
  // take MRPV into consideration
  val dMode = Input(new PrivState())
  val vsatp = Input(new SatpBundle)
}

class TrapEntryVSEventModule(implicit val p: Parameters) extends Module with CSREventBase {
  val in = IO(new TrapEntryVSEventInput)
  val out = IO(new TrapEntryVSEventOutput)

  when (valid) {
    assert(in.privState.isVirtual, "The mode must be VU or VS when entry VS mode")
  }

  private val current = in

  private val trapPC = Wire(UInt(XLEN.W))
  private val trapMemVA = SignExt(in.memExceptionVAddr, XLEN)
  private val trapMemGPA = SignExt(in.memExceptionGPAddr, XLEN)
  private val ivmVS = !current.iMode.isModeVS && current.vsatp.MODE =/= SatpMode.Bare
  // When enable virtual memory, the higher bit should fill with the msb of address of Sv39/Sv48/Sv57
  trapPC := Mux(ivmVS, SignExt(in.trapPc, XLEN), ZeroExt(in.trapPc, XLEN))

  private val isInterrupt = in.isInterrupt
  private val isException = !in.isInterrupt
  private val fetchIsVirt = current.iMode.isVirtual
  private val memIsVirt   = current.dMode.isVirtual

  // Todo: support more interrupt and exception
  private val exceptionNO = ExceptionNO.priorities.foldRight(0.U)((i: Int, sum: UInt) => Mux(in.trapVec(i), i.U, sum))
  private val interruptNO = CSRConst.IntPriority.foldRight(0.U)((i: Int, sum: UInt) => Mux(in.trapVec(i), i.U, sum))

  when (valid && in.isInterrupt) {
    import InterruptNO._
    assert(Seq(SEI, STI, SSI).map(_.U === interruptNO).reduce(_ || _), "The VS mode can only handle SEI, STI, SSI")
  }

  private val highPrioTrapNO = Mux(isInterrupt, interruptNO, exceptionNO)

  private val isFetchExcp    = isException && Seq(/*EX_IAM, */ EX_IAF, EX_IPF).map(_.U === highPrioTrapNO).reduce(_ || _)
  private val isMemExcp      = isException && Seq(EX_LAM, EX_LAF, EX_SAM, EX_SAF, EX_LPF, EX_SPF).map(_.U === highPrioTrapNO).reduce(_ || _)
  private val isBpExcp       = isException && EX_BP.U === highPrioTrapNO
  private val fetchCrossPage = in.isCrossPageIPF

  // Software breakpoint exceptions are permitted to write either 0 or the pc to xtval
  // We fill pc here
  private val tvalFillPc       = isFetchExcp && !fetchCrossPage || isBpExcp
  private val tvalFillPcPlus2  = isFetchExcp && fetchCrossPage
  private val tvalFillMemVaddr = isMemExcp
  private val tvalFillGVA      =
    (isFetchExcp || isBpExcp) && fetchIsVirt ||
    isMemExcp && memIsVirt

  private val tval = Mux1H(Seq(
    (tvalFillPc                     ) -> trapPC,
    (tvalFillPcPlus2                ) -> (trapPC + 2.U),
    (tvalFillMemVaddr && !memIsVirt ) -> trapMemVA,
    (tvalFillMemVaddr &&  memIsVirt ) -> trapMemVA,
  ))

  out := DontCare

  out.privState.valid := valid

  out.vsstatus .valid := valid
  out.vsepc    .valid := valid
  out.vscause  .valid := valid
  out.vstval   .valid := valid

  out.privState.bits             := PrivState.ModeVS
  // vsstatus
  out.vsstatus.bits.SPP          := current.privState.PRVM.asUInt(0, 0) // SPP is not PrivMode enum type, so asUInt and shrink the width
  out.vsstatus.bits.SPIE         := current.vsstatus.SIE
  out.vsstatus.bits.SIE          := 0.U
  // SPVP is not PrivMode enum type, so asUInt and shrink the width
  out.vsepc.bits.ALL             := trapPC(trapPC.getWidth - 1, 1)
  out.vscause.bits.Interrupt     := in.isInterrupt
  out.vscause.bits.ExceptionCode := highPrioTrapNO
  out.vstval.bits.ALL            := tval

  dontTouch(tvalFillGVA)
}

trait TrapEntryVSEventSinkBundle { self: CSRModule[_] =>
  val trapToVS = IO(Flipped(new TrapEntryVSEventOutput))

  private val updateBundle: ValidIO[CSRBundle] = trapToVS.getBundleByName(self.modName.toLowerCase())

  (reg.asInstanceOf[CSRBundle].getFields zip updateBundle.bits.getFields).foreach { case (sink, source) =>
    if (updateBundle.bits.eventFields.contains(source)) {
      when(updateBundle.valid) {
        sink := source
      }
    }
  }
}
