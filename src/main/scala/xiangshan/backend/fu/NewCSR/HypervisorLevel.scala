package xiangshan.backend.fu.NewCSR

import chisel3._
import xiangshan.backend.fu.NewCSR.CSRDefines.{
  CSRRWField => RW,
  CSRROField => RO,
  CSRWLRLField => WLRL,
  CSRWARLField => WARL,
  _
}
import xiangshan.backend.fu.NewCSR.CSRFunc._
import xiangshan.backend.fu.NewCSR.CSRConfig._

import scala.collection.immutable.SeqMap

trait HypervisorLevel { self: NewCSR =>

  val hstatus = Module(new HstatusModule).setAddr(0x600)

  val hedeleg = Module(new CSRModule("Hedeleg", new HedelegBundle)).setAddr(0x602)

  val hideleg = Module(new CSRModule("Hideleg", new HidelegBundle)).setAddr(0x603)

  val hie = Module(new CSRModule("Hie", new HieBundle) with HypervisorBundle {
    val writeFromVsie = IO(Flipped(new VsieWriteHie))
    val wAliasVsie = IO(Input(new CSRAddrWriteBundle(new Vsie)))
    val wVsieIn = WireInit(wAliasVsie)
    wVsieIn.wen := wAliasVsie.wen && hideleg.VSSI
    Flipped

  }).setAddr(0x604)

  val htimedelta = Module(new CSRModule("Htimedelta", new CSRBundle {
    val VALUE = RW(63, 0)
  })).setAddr(0x605)

  val hcounteren = Module(new CSRModule("Hcounteren", new CSRBundle {
    val CY = RW(0)
    val TM = RW(1)
    val IR = RW(2)
    val HPM = RW(31, 3)
  })).setAddr(0x606)

  val hgeie = Module(new CSRModule("Hgeie", new HgeieBundle)).setAddr(0x607)

  val hvien = Module(new CSRModule("Hvien", new CSRBundle {
    val ien = RW(63, 13)
    // bits 12:0 read only 0
  })).setAddr(0x608)

  val hvictl = Module(new CSRModule("Hvictl", new CSRBundle {
    // Virtual Trap Interrupt control
    val VTI    = RW  (30)
    // WARL in AIA spec.
    // RW, since we support max width of IID
    val IID    = RW  (15 + HIIDWidth, 16)
    // determines the interrupt’s presumed default priority order relative to a (virtual) supervisor external interrupt (SEI), major identity 9
    // 0 = interrupt has higher default priority than an SEI
    // 1 = interrupt has lower default priority than an SEI
    // When hvictl.IID = 9, DPR is ignored.
    // Todo: sort the interrupt specified by hvictl with DPR
    val DPR    = RW  (9)
    val IPRIOM = RW  (8)
    val IPRIO  = RW  ( 7,  0)
  })).setAddr(0x609)

  val henvcfg = Module(new CSRModule("Henvcfg", new CSRBundle {
    val FIOM  = RW(0)     // Fence of I/O implies Memory
    val CBIE  = RW(5, 4)  // Zicbom Enable
    val CBCFE = RW(6)     // Zicbom Enable
    val CBZE  = RW(7)     // Zicboz Enable
    val PBMTE = RW(62)    // Svpbmt Enable
    val STCE  = RW(63)    // Sstc Enable
  })).setAddr(0x60A)

  val htval = Module(new CSRModule("Htval", new CSRBundle {
    val ALL = RW(63, 0)
  })).setAddr(0x643)

  val hip = Module(new CSRModule("Hip", new HipBundle) with HypervisorBundle {
    rdata.VSSIP := hvip.VSSIP
    rdata.VSTIP := hvip.VSTIP.asUInt.asBool | vsi.tip
    rdata.VSEIP := hvip.VSEIP.asUInt.asBool | vsi.eip | hgeip.ip.asUInt(hstatus.VGEIN.asUInt)
    rdata.SGEIP := (hgeip.ip.asUInt | hgeie.ie.asUInt).orR
  }).setAddr(0x644)

  val hvip = Module(new CSRModule("Hvip", new CSRBundle {
    val VSSIP = RW( 2)
    val VSTIP = RW( 6)
    val VSEIP = RW(10)
  })).setAddr(0x645)

  val hviprio1 = Module(new CSRModule("Hviprio1", new CSRBundle {
    val PrioSSI = RW(15,  8)
    val PrioSTI = RW(31, 24)
    val PrioCOI = RW(47, 40)
    val Prio14  = RW(55, 48)
    val Prio15  = RW(63, 56)
  })).setAddr(0x646)

  val hviprio2 = Module(new CSRModule("Hviprio2", new CSRBundle {
    val Prio16  = RW( 7,  0)
    val Prio17  = RW(15,  8)
    val Prio18  = RW(23, 16)
    val Prio19  = RW(31, 24)
    val Prio20  = RW(39, 32)
    val Prio21  = RW(47, 40)
    val Prio22  = RW(55, 48)
    val Prio23  = RW(63, 56)
  })).setAddr(0x647)

  val htinst = Module(new CSRModule("Htinst", new CSRBundle {
    val ALL = RO(63, 0)
  })).setAddr(0x64A)

  val hgatp = Module(new CSRModule("Hgatp", new CSRBundle {
    val MODE = HgatpMode(63, 60, wNoFilter)
    // WARL in privileged spec.
    // RW, since we support max width of VMID
    val VMID = RW(44 - 1 + VMIDLEN, 44)
    val PPN = RW(43, 0)
  })).setAddr(0x680)

  val hgeip = Module(new CSRModule("Hgeip", new HgeipBundle)).setAddr(0xE12)

  val hypervisorCSRMods: Seq[CSRModule[_]] = Seq(
    hstatus,
    hedeleg,
    hideleg,
    hie,
    htimedelta,
    hcounteren,
    hgeie,
    hvien,
    hvictl,
    henvcfg,
    htval,
    hip,
    hvip,
    hviprio1,
    hviprio2,
    htinst,
    hgatp,
    hgeip,
  )

  hypervisorCSRMods.foreach {
    case mod: HypervisorBundle =>
      mod.hstatus := hstatus.rdata
      mod.hvip := hvip.rdata
      mod.hideleg := hideleg.rdata
      mod.hedeleg := hedeleg.rdata
      mod.hgeip := hgeip.rdata
      mod.hgeie := hgeie.rdata
      mod.hip := hip.rdata
      mod.hie := hie.rdata
    case _ =>
  }

  val hypervisorCSRMap: SeqMap[Int, (CSRAddrWriteBundle[_], Data)] = SeqMap.from(
    hypervisorCSRMods.map(csr => (csr.addr -> (csr.w -> csr.rdata.asInstanceOf[CSRBundle].asUInt))).iterator
  )
}

class HstatusBundle extends CSRBundle {

  val VSBE  = RO(5).withReset(0.U)
  val GVA   = RW(6)
  val SPV   = RW(7)
  val SPVP  = RW(8)
  val HU    = RW(9)
  val VGEIN = HstatusVgeinField(17, 12, wNoFilter, rNoFilter)
  val VTVM  = RW(20)
  val VTM   = RW(21)
  val VTSR  = RW(22)
  val VSXL  = XLENField(33, 32).withReset(XLENField.XLEN64)

}

object HstatusVgeinField extends CSREnum with CSRWLRLApply {
  override def isLegal(enum: CSREnumType): Bool = enum.asUInt <= GEILEN.U
}

class HstatusModule extends CSRModule("Hstatus", new HstatusBundle)

class HvipBundle extends CSRBundle {
  val VSSIP = RW(2)
  val VSTIP = RW(6)
  val VSEIP = RW(10)
}

class HieBundle extends CSRBundle {
  val VSSIE = RW( 2)
  val VSTIE = RW( 6)
  val VSEIE = RW(10)
  val SGEIE = RW(12)
}

class HipBundle extends CSRBundle {
  val VSSIP = RW( 2) // alias of hvip.VSSIP
  val VSTIP = RO( 6) // hvip.VSTIP |　PLIC.VSTIP
  val VSEIP = RO(10) // hvip.VSEIP | hgeip(hstatus.VGEIN) | PLIC.VSEIP
  val SGEIP = RO(12) // |(hgeip & hegie)
}

class HgeieBundle extends CSRBundle {
  val ie = RW(GEILEN, 1)
  // bit 0 is read only 0
}

class HgeipBundle extends CSRBundle {
  val ip = RW(GEILEN, 1)
  // bit 0 is read only 0
}

class HedelegBundle extends ExceptionBundle {
  // default RW
  this.EX_HSCALL.setRO()
  this.EX_VSCALL.setRO()
  this.EX_MCALL .setRO()
  this.EX_IGPF  .setRO()
  this.EX_LGPF  .setRO()
  this.EX_VI    .setRO()
  this.EX_SGPF  .setRO()
}

class HidelegBundle extends InterruptBundle {
  // default RW
  this.SSI .setRO()
  this.MSI .setRO()
  this.STI .setRO()
  this.MTI .setRO()
  this.SEI .setRO()
  this.MEI .setRO()
  this.SGEI.setRO()
}

trait HypervisorBundle { self: CSRModule[_] =>
  val hstatus = IO(Input(new HstatusBundle))
  val hvip    = IO(Input(new HvipBundle))
  val hideleg = IO(Input(new HidelegBundle))
  val hedeleg = IO(Input(new HedelegBundle))
  val hgeip   = IO(Input(new HgeipBundle))
  val hgeie   = IO(Input(new HgeieBundle))
  val hip     = IO(Input(new HipBundle))
  val hie     = IO(Input(new HieBundle))
}
