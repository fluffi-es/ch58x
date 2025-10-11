use crate::{
    interrupt::CoreInterrupt,
    pfic::PficExt,
    raw::usb::{int_st::UisToken, Uep0Ctrl, Uep0Dma, Uep0TLen},
    Pfic, Sys, Usb,
};
use core::{future::poll_fn, marker::PhantomData, task::Poll};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointIn, EndpointInfo,
    EndpointOut, EndpointType, Event, Unsupported,
};

#[derive(Copy, Clone)]
struct EndpointData {
    typ: EndpointType,
    out: bool,
    in_: bool,
}

pub struct Driver<'a> {
    usb: Usb,

    eps: [EndpointData; 8],

    buf: &'a mut [u8],
    buf_offset: usize,
}

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP_WAKERS: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];

impl<'a> Driver<'a> {
    pub fn new(usb: Usb, pfic: &Pfic, buf: &'a mut [u8]) -> Self {
        pfic.enable(CoreInterrupt::USB, None);

        Self {
            usb,
            eps: [EndpointData {
                typ: EndpointType::Control,
                out: false,
                in_: false,
            }; 8],
            buf,
            buf_offset: 0,
        }
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<D>, EndpointAllocError> {
        let (index, data) = if let Some(addr) = ep_addr {
            let requested_index = addr.index();
            if requested_index >= 8 {
                return Err(EndpointAllocError);
            }
            if requested_index == 0 && ep_type != EndpointType::Control {
                return Err(EndpointAllocError);
            }
            if match D::dir() {
                Direction::Out => self.eps[requested_index].out,
                Direction::In => self.eps[requested_index].in_,
            } {
                return Err(EndpointAllocError);
            }
            (requested_index, &mut self.eps[requested_index])
        } else {
            self.eps
                .iter_mut()
                .enumerate()
                .find(|(i, ep)| {
                    if *i == 0 && ep_type != EndpointType::Control {
                        false
                    } else {
                        !ep.out && !ep.in_
                            || match D::dir() {
                                Direction::Out => !ep.out,
                                Direction::In => !ep.in_,
                            } && ep.typ == ep_type
                    }
                })
                .unwrap()
        };

        data.typ = ep_type;
        match D::dir() {
            Direction::Out => data.out = true,
            Direction::In => data.in_ = true,
        };

        Ok(Endpoint {
            _dir: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }
}

impl<'a> embassy_usb_driver::Driver<'a> for Driver<'a> {
    type EndpointOut = Endpoint<Out>;
    type EndpointIn = Endpoint<In>;
    type ControlPipe = ControlPipe;
    type Bus = Bus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        for (i, ep) in self.eps.iter().enumerate() {
            if ep.out && ep.in_ {
                let buf = unsafe { self.buf.as_mut_ptr().offset(self.buf_offset as _) };
                self.usb
                    .uep_dma(i)
                    .write(|w| unsafe { w.uep0_dma().bits(buf as u16) });
                self.buf_offset += 128;
            } else if ep.out || ep.in_ {
                let buf = unsafe { self.buf.as_mut_ptr().offset(self.buf_offset as _) };
                self.usb
                    .uep_dma(i)
                    .write(|w| unsafe { w.uep0_dma().bits(buf as u16) });
                self.buf_offset += 64;
            } else if i == 4 {
                // ep0
                let buf = unsafe { self.buf.as_mut_ptr().offset(self.buf_offset as _) };
                self.usb
                    .uep_dma(i)
                    .write(|w| unsafe { w.uep0_dma().bits(buf as u16) });
            }
            if i == 4 {
                // ep0
                self.buf_offset += 64;
            }
        }

        let out = self
            .alloc_endpoint(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();
        let in_ = self
            .alloc_endpoint(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();
        (
            Self::Bus {
                usb: self.usb,
                inited: false,
            },
            Self::ControlPipe { out, in_ },
        )
    }
}

pub struct Bus {
    usb: Usb,
    inited: bool,
}

impl Bus {
    fn reset(&mut self) {
        self.usb.dev_ad().reset();
        self.usb
            .uep_ctrl(0)
            .write(|w| w.uep_r_res().ack().uep_t_res().nak());
        for ep_addr in 1..8 {
            self.usb
                .uep_ctrl(ep_addr)
                .write(|w| w.uep_r_res().nak().uep_t_res().nak());
        }
    }
}

impl embassy_usb_driver::Bus for Bus {
    async fn enable(&mut self) {}

    async fn disable(&mut self) {}

    async fn poll(&mut self) -> Event {
        if !self.inited {
            self.usb.ctrl().write(|w| w);
            self.usb.dev_ad().reset();
            self.usb.int_fg().write(|w| unsafe { w.bits(0xFF) });

            unsafe { Sys::steal() }
                .pin_analog_ie()
                .modify(|_, w| w.pin_usb_ie().set_bit().pin_usb_dp_pu().set_bit());
            self.usb
                .udev_ctrl()
                .write(|w| w.ud_port_en().set_bit().ud_pd_dis().set_bit());

            self.usb.ctrl().write(|w| {
                w.uc_dev_pu_en()
                    .set_bit()
                    .uc_int_busy()
                    .set_bit()
                    .uc_dma_en()
                    .set_bit()
            });
            self.usb.int_en().write(|w| {
                w.uie_bus_rst()
                    .set_bit()
                    .uie_transfer()
                    .set_bit()
                    .uie_suspend()
                    .set_bit()
            });

            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            let intfg = self.usb.int_fg().read();
            if intfg.uif_bus_rst().bit() {
                self.reset();

                // mark as handled and re-enable interrupt
                self.usb.int_fg().write(|w| w.uif_bus_rst().set_bit());
                self.usb.int_en().modify(|_, w| w.uie_bus_rst().set_bit());

                Poll::Ready(Event::Reset)
            } else if intfg.uif_suspend().bit() {
                let misst = self.usb.mis_st().read();

                // mark as handled and re-enable interrupt
                self.usb.int_fg().write(|w| w.uif_suspend().set_bit());
                self.usb.int_en().modify(|_, w| w.uie_suspend().set_bit());

                if misst.ums_suspend().bit() {
                    Poll::Ready(Event::Suspend)
                } else {
                    Poll::Ready(Event::Resume)
                }
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let usb = unsafe { Usb::steal() };
        match (ep_addr.index(), ep_addr.direction()) {
            (4, Direction::In) => usb.uep4_1_mod().modify(|_, w| w.uep4_tx_en().bit(enabled)),
            (4, Direction::Out) => usb.uep4_1_mod().modify(|_, w| w.uep4_rx_en().bit(enabled)),
            (1, Direction::In) => usb.uep4_1_mod().modify(|_, w| w.uep1_tx_en().bit(enabled)),
            (1, Direction::Out) => usb.uep4_1_mod().modify(|_, w| w.uep1_rx_en().bit(enabled)),
            (2, Direction::In) => usb.uep2_3_mod().modify(|_, w| w.uep2_tx_en().bit(enabled)),
            (2, Direction::Out) => usb.uep2_3_mod().modify(|_, w| w.uep2_rx_en().bit(enabled)),
            (3, Direction::In) => usb.uep2_3_mod().modify(|_, w| w.uep3_tx_en().bit(enabled)),
            (3, Direction::Out) => usb.uep2_3_mod().modify(|_, w| w.uep3_rx_en().bit(enabled)),
            (5, Direction::In) => usb.uep567_mod().modify(|_, w| w.uep5_tx_en().bit(enabled)),
            (5, Direction::Out) => usb.uep567_mod().modify(|_, w| w.uep5_rx_en().bit(enabled)),
            (6, Direction::In) => usb.uep567_mod().modify(|_, w| w.uep6_tx_en().bit(enabled)),
            (6, Direction::Out) => usb.uep567_mod().modify(|_, w| w.uep6_rx_en().bit(enabled)),
            (7, Direction::In) => usb.uep567_mod().modify(|_, w| w.uep7_tx_en().bit(enabled)),
            (7, Direction::Out) => usb.uep567_mod().modify(|_, w| w.uep7_rx_en().bit(enabled)),
            _ => unreachable!(),
        };

        EP_WAKERS[ep_addr.index()].wake();
    }

    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
        unimplemented!()
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        unimplemented!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
}

pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

pub struct Endpoint<D: Dir> {
    _dir: PhantomData<D>,
    info: EndpointInfo,
}

impl<D: Dir> embassy_usb_driver::Endpoint for Endpoint<D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let usb = unsafe { Usb::steal() };

        poll_fn(|cx| {
            EP_WAKERS[self.info.addr.index()].register(cx.waker());
            if usb.uep_en(self.info.addr) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl embassy_usb_driver::EndpointOut for Endpoint<Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let usb = unsafe { Usb::steal() };
        usb.uep_ctrl(self.info.addr.index())
            .modify(|r, w| w.uep_r_res().ack().uep_r_tog().bit(!r.uep_r_tog().bit()));

        poll_fn(|cx| {
            EP_WAKERS[self.info.addr.index()].register(cx.waker());

            let intfg = usb.int_fg().read();
            let intst = usb.int_st().read();
            if intfg.uif_transfer().bit() && intst.uis_endp() == self.info.addr.index() as u8 {
                let res = if intst.uis_token().is_out() {
                    let len = usb.rx_len().read().bits() as usize;
                    buf[..len as usize].copy_from_slice(unsafe {
                        core::slice::from_raw_parts(usb.uep_dma_ptr(self.info.addr), len as usize)
                    });
                    Poll::Ready(Ok(len))
                } else {
                    Poll::Ready(Err(EndpointError::Disabled))
                };

                usb.uep_ctrl(self.info.addr.index())
                    .modify(|_, w| w.uep_r_res().nak());

                // mark as handled and re-enable interrupt
                usb.int_fg().write(|w| w.uif_transfer().set_bit());
                usb.int_en().modify(|_, w| w.uie_transfer().set_bit());

                res
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl embassy_usb_driver::EndpointIn for Endpoint<In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let usb = unsafe { Usb::steal() };

        unsafe { core::slice::from_raw_parts_mut(usb.uep_dma_ptr(self.info.addr), buf.len()) }
            .copy_from_slice(buf);
        usb.uep_t_len(self.info.addr.index())
            .write(|w| unsafe { w.uep0_t_len().bits(buf.len() as u8) });
        usb.uep_ctrl(self.info.addr.index())
            .modify(|r, w| w.uep_t_res().ack().uep_t_tog().bit(!r.uep_t_tog().bit()));

        poll_fn(|cx| {
            EP_WAKERS[self.info.addr.index()].register(cx.waker());

            let intfg = usb.int_fg().read();
            let intst = usb.int_st().read();
            if intfg.uif_transfer().bit() && intst.uis_endp() == self.info.addr.index() as u8 {
                let res = if intst.uis_token().is_in() {
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Ready(Err(EndpointError::Disabled))
                };

                usb.uep_ctrl(self.info.addr.index())
                    .modify(|_, w| w.uep_t_res().nak());

                // mark as handled and re-enable interrupt
                usb.int_fg().write(|w| w.uif_transfer().set_bit());
                usb.int_en().modify(|_, w| w.uie_transfer().set_bit());

                res
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

pub struct ControlPipe {
    out: Endpoint<Out>,
    in_: Endpoint<In>,
}

impl embassy_usb_driver::ControlPipe for ControlPipe {
    fn max_packet_size(&self) -> usize {
        64
    }

    async fn setup(&mut self) -> [u8; 8] {
        let usb = unsafe { Usb::steal() };

        poll_fn(|cx| {
            EP_WAKERS[0].register(cx.waker());

            let intfg = usb.int_fg().read();
            let intst = usb.int_st().read();
            if intfg.uif_transfer().bit() && intst.uis_token().is_setup() {
                let mut data = [0; 8];
                data.copy_from_slice(unsafe {
                    core::slice::from_raw_parts(
                        usb.uep_dma_ptr(EndpointAddress::from_parts(0, Direction::Out)),
                        8,
                    )
                });

                usb.uep_ctrl(0)
                    .modify(|_, w| w.uep_r_res().nak().uep_t_res().nak());

                // mark as handled and re-enable interrupt
                usb.int_fg().write(|w| w.uif_transfer().set_bit());
                usb.int_en().modify(|_, w| w.uie_transfer().set_bit());

                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        if first {
            unsafe { Usb::steal() }
                .uep_ctrl(0)
                .modify(|_, w| w.uep_r_tog().clear_bit());
        }
        self.out.read(buf).await
    }

    async fn data_in(&mut self, buf: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if first {
            unsafe { Usb::steal() }
                .uep_ctrl(0)
                .modify(|_, w| w.uep_t_tog().clear_bit());
        }
        self.in_.write(buf).await?;
        if last {
            unsafe { Usb::steal() }
                .uep_ctrl(0)
                .modify(|_, w| w.uep_r_tog().clear_bit());
            self.out.read(&mut []).await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        unsafe { Usb::steal() }
            .uep_ctrl(0)
            .modify(|_, w| w.uep_t_tog().clear_bit());
        _ = self.in_.write(&[]).await;
    }

    async fn reject(&mut self) {
        unsafe { Usb::steal() }
            .uep_ctrl(0)
            .modify(|_, w| w.uep_r_res().stall().uep_t_res().stall());
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        unsafe { Usb::steal() }
            .dev_ad()
            .write(|w| unsafe { w.addr().bits(addr) });
    }
}

trait UsbExt {
    fn uep_dma(&self, ep_addr: usize) -> &Uep0Dma;

    fn uep_dma_ptr(&self, ep_addr: EndpointAddress) -> *mut u8;

    fn uep_t_len(&self, ep_addr: usize) -> &Uep0TLen;

    fn uep_ctrl(&self, ep_addr: usize) -> &Uep0Ctrl;

    fn uep_en(&self, ep_addr: EndpointAddress) -> bool;
}

impl UsbExt for Usb {
    fn uep_dma(&self, ep_addr: usize) -> &Uep0Dma {
        match ep_addr {
            0 => self.uep0_dma(),
            1 => unsafe { &*(self.uep1_dma().as_ptr() as *mut Uep0Dma) },
            2 => unsafe { &*(self.uep2_dma().as_ptr() as *mut Uep0Dma) },
            3 => unsafe { &*(self.uep3_dma().as_ptr() as *mut Uep0Dma) },
            4 => self.uep0_dma(),
            5 => unsafe { &*(self.uep5_dma().as_ptr() as *mut Uep0Dma) },
            6 => unsafe { &*(self.uep6_dma().as_ptr() as *mut Uep0Dma) },
            7 => unsafe { &*(self.uep7_dma().as_ptr() as *mut Uep0Dma) },
            _ => panic!(),
        }
    }

    fn uep_dma_ptr(&self, ep_addr: EndpointAddress) -> *mut u8 {
        if !self.uep_en(ep_addr) {
            return core::ptr::null_mut();
        }

        // ep0 has a shared buf and ep4 uses the same address as ep0
        (if ep_addr.is_out()
            || !self.uep_en(EndpointAddress::from_parts(ep_addr.index(), Direction::Out)) // && is_in
            || ep_addr.index() == 0
        {
            0x20000000
        } else {
            0x20000040
        } + if ep_addr.index() == 4 { 0x40 } else { 0x00 }
            + self.uep_dma(ep_addr.index()).read().bits() as usize) as _
    }

    fn uep_t_len(&self, ep_addr: usize) -> &Uep0TLen {
        match ep_addr {
            0 => self.uep0_t_len(),
            1 => unsafe { &*(self.uep1_t_len().as_ptr() as *mut Uep0TLen) },
            2 => unsafe { &*(self.uep2_t_len().as_ptr() as *mut Uep0TLen) },
            3 => unsafe { &*(self.uep3_t_len().as_ptr() as *mut Uep0TLen) },
            4 => unsafe { &*(self.uep4_t_len().as_ptr() as *mut Uep0TLen) },
            5 => unsafe { &*(self.uep5_t_len().as_ptr() as *mut Uep0TLen) },
            6 => unsafe { &*(self.uep6_t_len().as_ptr() as *mut Uep0TLen) },
            7 => unsafe { &*(self.uep7_t_len().as_ptr() as *mut Uep0TLen) },
            _ => panic!(),
        }
    }

    fn uep_ctrl(&self, ep_addr: usize) -> &Uep0Ctrl {
        match ep_addr {
            0 => self.uep0_ctrl(),
            1 => unsafe { &*(self.uep1_ctrl().as_ptr() as *mut Uep0Ctrl) },
            2 => unsafe { &*(self.uep2_ctrl().as_ptr() as *mut Uep0Ctrl) },
            3 => unsafe { &*(self.uep3_ctrl().as_ptr() as *mut Uep0Ctrl) },
            4 => unsafe { &*(self.uep4_ctrl().as_ptr() as *mut Uep0Ctrl) },
            5 => unsafe { &*(self.uep5_ctrl().as_ptr() as *mut Uep0Ctrl) },
            6 => unsafe { &*(self.uep6_ctrl().as_ptr() as *mut Uep0Ctrl) },
            7 => unsafe { &*(self.uep7_ctrl().as_ptr() as *mut Uep0Ctrl) },
            _ => panic!(),
        }
    }

    fn uep_en(&self, ep_addr: EndpointAddress) -> bool {
        match (ep_addr.index(), ep_addr.direction()) {
            (0, _) => true,
            (4, Direction::In) => self.uep4_1_mod().read().uep4_tx_en().bit(),
            (4, Direction::Out) => self.uep4_1_mod().read().uep4_rx_en().bit(),
            (1, Direction::In) => self.uep4_1_mod().read().uep1_tx_en().bit(),
            (1, Direction::Out) => self.uep4_1_mod().read().uep1_rx_en().bit(),
            (2, Direction::In) => self.uep2_3_mod().read().uep2_tx_en().bit(),
            (2, Direction::Out) => self.uep2_3_mod().read().uep2_rx_en().bit(),
            (3, Direction::In) => self.uep2_3_mod().read().uep3_tx_en().bit(),
            (3, Direction::Out) => self.uep2_3_mod().read().uep3_rx_en().bit(),
            (5, Direction::In) => self.uep567_mod().read().uep5_tx_en().bit(),
            (5, Direction::Out) => self.uep567_mod().read().uep5_rx_en().bit(),
            (6, Direction::In) => self.uep567_mod().read().uep6_tx_en().bit(),
            (6, Direction::Out) => self.uep567_mod().read().uep6_rx_en().bit(),
            (7, Direction::In) => self.uep567_mod().read().uep7_tx_en().bit(),
            (7, Direction::Out) => self.uep567_mod().read().uep7_rx_en().bit(),
            _ => panic!(),
        }
    }
}

#[riscv_rt::core_interrupt(CoreInterrupt::USB)]
fn usb() {
    let usb = unsafe { Usb::steal() };

    let intfg = usb.int_fg().read();
    if intfg.uif_bus_rst().bit() {
        // will be handled later
        usb.int_en().modify(|_, w| w.uie_bus_rst().clear_bit());
        BUS_WAKER.wake();
    }
    if intfg.uif_transfer().bit() {
        let intst = usb.int_st().read();
        match intst.uis_token().variant() {
            Some(UisToken::Out) => {
                if intst.uis_tog_ok().bit() {
                    // will be handled later
                    usb.int_en().modify(|_, w| w.uie_transfer().clear_bit());
                    EP_WAKERS[intst.uis_endp().bits() as usize].wake();
                } else {
                    // no handling required
                    usb.int_fg().write(|w| w.uif_transfer().set_bit());
                }
            }
            Some(UisToken::In) => {
                // will be handled later
                usb.int_en().modify(|_, w| w.uie_transfer().clear_bit());
                EP_WAKERS[intst.uis_endp().bits() as usize].wake();
            }
            Some(UisToken::Setup) => {
                // will be handled later
                usb.int_en().modify(|_, w| w.uie_transfer().clear_bit());
                EP_WAKERS[0].wake();
            }
            _ => todo!(),
        }
    }
    if intfg.uif_suspend().bit() {
        // will be handled later
        usb.int_en().modify(|_, w| w.uie_suspend().clear_bit());
        BUS_WAKER.wake();
    }
    if intfg.uif_hst_sof().bit() {
        // no handling required
        usb.int_fg().write(|w| w.uif_hst_sof().set_bit());
    }
}

#[riscv_rt::core_interrupt(CoreInterrupt::USB2)]
fn usb2() {}
