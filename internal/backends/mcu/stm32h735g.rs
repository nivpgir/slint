// Copyright © SixtyFPS GmbH <info@slint-ui.com>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-commercial

extern crate alloc;

use super::TargetPixel;
pub use cortex_m_rt::entry;
use embedded_display_controller::{DisplayController, DisplayControllerLayer};
use embedded_graphics::prelude::RgbColor;
use hal::delay::Delay;
use hal::gpio::Speed::High;
use hal::ltdc::LtdcLayer1;
use hal::pac;
use hal::prelude::*;
use hal::rcc::rec::OctospiClkSelGetter;
use stm32h7xx_hal as hal;

use defmt_rtt as _; // global logger

#[cfg(feature = "panic-probe")]
use panic_probe as _;

#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    panic!("Out of memory {:?}", layout);
}
use alloc_cortex_m::CortexMHeap;

use crate::{Devices, PhysicalLength, PhysicalRect, PhysicalSize};

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

const DISPLAY_WIDTH: usize = 480;
const DISPLAY_HEIGHT: usize = 272;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub fn init() {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) }

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(400.MHz())
        // numbers adapted from Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_ospi.c
        // MX_OSPI_ClockConfig
        .pll2_p_ck(400.MHz() / 5)
        .pll2_q_ck(400.MHz() / 2)
        .pll2_r_ck(400.MHz() / 2)
        // numbers adapted from Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_lcd.c
        // MX_LTDC_ClockConfig
        .pll3_p_ck(800.MHz() / 2)
        .pll3_q_ck(800.MHz() / 2)
        .pll3_r_ck(800.MHz() / 83)
        .freeze(pwrcfg, &dp.SYSCFG);

    assert_eq!(ccdr.clocks.hclk(), 200.MHz::<1, 1>());
    // Octospi from HCLK at 200MHz
    assert_eq!(
        ccdr.peripheral.OCTOSPI2.get_kernel_clk_mux(),
        hal::rcc::rec::OctospiClkSel::RCC_HCLK3
    );
    assert_eq!(
        ccdr.peripheral.OCTOSPI1.get_kernel_clk_mux(),
        hal::rcc::rec::OctospiClkSel::RCC_HCLK3
    );

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);

    // setup OCTOSPI HyperRAM
    let _tracweswo = gpiob.pb3.into_alternate::<0>();
    let _ncs = gpiog.pg12.into_alternate::<3>().speed(High).internal_pull_up(true);
    let _dqs = gpiof.pf12.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _clk = gpiof.pf4.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io0 = gpiof.pf0.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io1 = gpiof.pf1.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io2 = gpiof.pf2.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io3 = gpiof.pf3.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io4 = gpiog.pg0.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io5 = gpiog.pg1.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io6 = gpiog.pg10.into_alternate::<3>().speed(High).internal_pull_up(true);
    let _io7 = gpiog.pg11.into_alternate::<9>().speed(High).internal_pull_up(true);

    let hyperram_size = 16 * 1024 * 1024; // 16 MByte
    let config = hal::xspi::HyperbusConfig::new(80.MHz())
        .device_size_bytes(21) // 16 Mbyte
        .refresh_interval(4.micros())
        .read_write_recovery(4) // 50ns
        .access_initial_latency(6);

    let hyperram =
        dp.OCTOSPI2.octospi_hyperbus_unchecked(config, &ccdr.clocks, ccdr.peripheral.OCTOSPI2);
    let hyperram_ptr: *mut u32 = hyperram.init();

    let _ncs = gpiog.pg6.into_alternate::<10>().speed(High).internal_pull_up(true);
    let _clk = gpiof.pf10.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _dqs = gpiob.pb2.into_alternate::<10>().speed(High).internal_pull_up(true);
    let _io0 = gpiod.pd11.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io1 = gpiod.pd12.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io2 = gpioe.pe2.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io3 = gpiod.pd13.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io4 = gpiod.pd4.into_alternate::<10>().speed(High).internal_pull_up(true);
    let _io5 = gpiod.pd5.into_alternate::<10>().speed(High).internal_pull_up(true);
    let _io6 = gpiog.pg9.into_alternate::<9>().speed(High).internal_pull_up(true);
    let _io7 = gpiod.pd7.into_alternate::<10>().speed(High).internal_pull_up(true);

    use stm32h7xx_hal::xspi::*;
    use OctospiWord as XW;

    let mut octospi =
        dp.OCTOSPI1.octospi_unchecked(12.MHz(), &ccdr.clocks, ccdr.peripheral.OCTOSPI1);

    /*octospi.configure_mode(OctospiMode::OneBit).unwrap();

    // RDID Read Identification. Abuses address as instruction phase, but that
    // works in SPI mode.
    let mut read: [u8; 3] = [0; 3];
    octospi.read(0x9F, &mut read).unwrap();
    i_slint_core::debug_log!("Read with instruction 0x9F : {:x?}", read);*/

    // Switch Macronix MX25LM51245GXDI00 to DTR OPI
    octospi.configure_mode(OctospiMode::EightBit).unwrap();

    const MX25LM51245G_OCTA_READ_DTR_CMD: u16 = 0xEE11;
    const MX25LM51245G_OCTA_PAGE_PROG_CMD: u16 = 0x12ED;
    const MX25LM51245G_OCTA_WRITE_ENABLE_CMD: u16 = 0x06F9;
    const MX25LM51245G_OCTA_READ_STATUS_REG_CMD: u16 = 0x05FA;
    const MX25LM51245G_AUTOPOLLING_INTERVAL_TIME: u16 = 0x10;
    const MX25LM51245G_OCTA_READ_CFG_REG2_CMD: u16 = 0x718E;
    const MX25LM51245G_OCTA_WRITE_CFG_REG2_CMD: u16 = 0x728D;
    const MX25LM51245G_CR2_REG1_ADDR: u32 = 0x00000000;
    const MX25LM51245G_CR2_REG3_ADDR: u32 = 0x00000300;
    const MX25LM51245G_CR2_DC_6_CYCLES: u8 = 0x7;
    const MX25LM51245G_CR2_DOPI: u8 = 0x2;
    const MX25LM51245G_WRITE_REG_MAX_TIME: u8 = 40;
    #[allow(non_snake_case)]
    fn MX25LM51245G_WriteEnable(octospi: &mut hal::xspi::Octospi<pac::OCTOSPI1>) {
        octospi
            .write_extended(XW::U16(MX25LM51245G_OCTA_WRITE_ENABLE_CMD), XW::None, XW::None, &[])
            .unwrap();

        octospi
            .write_extended(XW::U16(MX25LM51245G_OCTA_READ_STATUS_REG_CMD), XW::None, XW::None, &[])
            .unwrap();

        HAL_OSPI_AutoPolling(octospi);
    }
    #[allow(non_snake_case)]
    fn HAL_OSPI_AutoPolling(octospi: &mut hal::xspi::Octospi<pac::OCTOSPI1>) {
        unsafe {
            let regs = octospi.inner_mut();
            let ar = regs.ar.read().address().bits();
            let ir = regs.ir.read().instruction().bits();
            regs.psmar.write(|w| w.match_().bits(2));
            regs.psmkr.write(|w| w.mask().bits(2));
            regs.pir.write(|w| w.interval().bits(MX25LM51245G_AUTOPOLLING_INTERVAL_TIME));
            regs.cr.modify(|_, w| w.pmm().clear_bit().apms().set_bit().fmode().bits(2));
            if regs.ccr.read().admode().bits() != 0 {
                regs.ar.write(|w| w.address().bits(ar));
            } else {
                regs.ir.write(|w| w.instruction().bits(ir));
            }
            while !regs.sr.read().smf().bit() {}
            regs.fcr.write(|w| w.csmf().set_bit());
            while regs.sr.read().busy().bit() {}
        }
    }

    // BSP_OSPI_NOR_ConfigFlash - OSPI_NOR_EnterDOPIMode
    MX25LM51245G_WriteEnable(&mut octospi);
    octospi
        .write_extended(
            XW::U16(MX25LM51245G_OCTA_WRITE_CFG_REG2_CMD),
            XW::U32(MX25LM51245G_CR2_REG3_ADDR),
            XW::None,
            &[MX25LM51245G_CR2_DC_6_CYCLES],
        )
        .unwrap();

    MX25LM51245G_WriteEnable(&mut octospi);
    octospi
        .write_extended(
            XW::U16(MX25LM51245G_OCTA_WRITE_CFG_REG2_CMD),
            XW::U32(MX25LM51245G_CR2_REG1_ADDR),
            XW::None,
            &[MX25LM51245G_CR2_DOPI],
        )
        .unwrap();
    delay.delay_ms(MX25LM51245G_WRITE_REG_MAX_TIME);

    // HAL_OSPI_Init
    unsafe {
        let regs = octospi.inner_mut();
        regs.dcr1.modify(|_, w| {
            w.mtyp()
                .bits(1)
                .devsize()
                .bits(24) // 512M
                .csht()
                .bits(2 - 1)
                .frck()
                .clear_bit()
                .dlybyp()
                .clear_bit()
                .ckmode()
                .clear_bit()
        });
        regs.dcr2.modify(|_, w| w.wrapsize().bits(0));
        regs.dcr3.modify(|_, w| w.csbound().bits(0).maxtran().bits(0));
        regs.dcr4.write(|w| w.refresh().bits(0));
        regs.cr.modify(|_, w| w.fthres().bits(4 - 1));
        while regs.sr.read().busy().bit() {}
        regs.dcr2.modify(|_, w| w.prescaler().bits(2 - 1));
        regs.cr.modify(|_, w| w.dqm().clear_bit());
        regs.tcr.modify(|_, w| w.sshift().clear_bit().dhqc().set_bit());
        regs.cr.modify(|_, w| w.en().set_bit());
    }

    // MX25LM51245G_AutoPollingMemReady
    octospi
        .write_extended(XW::U16(MX25LM51245G_OCTA_READ_STATUS_REG_CMD), XW::None, XW::None, &[])
        .unwrap();
    HAL_OSPI_AutoPolling(&mut octospi);

    let mut read: [u8; 1] = [0];
    octospi
        .read_extended(
            XW::U16(MX25LM51245G_OCTA_READ_CFG_REG2_CMD),
            XW::U32(MX25LM51245G_CR2_REG1_ADDR),
            XW::None,
            5,
            &mut read,
        )
        .unwrap();

    assert_eq!(read[0], MX25LM51245G_CR2_DOPI);
    /*
    // Set WREN bit
    octospi.write_extended(XW::U8(0x06), XW::None, XW::None, &[]).unwrap();
    // Write Configuration Register 2
    octospi.write_extended(XW::U8(0x72), XW::U32(0), XW::None, &[1]).unwrap();
    // Change bus mode
    octospi.configure_mode(OctospiMode::EightBit).unwrap();

    // RDID Read Identification
    let mut read: [u8; 3] = [0; 3];
    octospi.read_extended(XW::U16(0x9F60), XW::U32(0), XW::None, 4, &mut read).unwrap();
    i_slint_core::debug_log!("Read with instruction 0x9F60 : {:x?}", read);

    {
        // Taken from MX25LM51245G_EnableDTRMemoryMappedMode of the STM32Cube

        octospi
            .write_extended(XW::U16(MX25LM51245G_OCTA_READ_DTR_CMD), XW::None, XW::None, &[])
            .unwrap();

        octospi
            .write_extended(XW::U16(MX25LM51245G_OCTA_PAGE_PROG_CMD), XW::None, XW::None, &[])
            .unwrap();

        // HAL_OSPI_MemoryMapped

        // octospi.inner_mut().ltpr.write(|w| w.timeout())
        //    unsafe { octospi.inner_mut().cr.modify(|_, w| w.fmode().bits(3).tcen().clear_bit()) };
    }*/

    /*

    octospi.configure_mode(OctospiMode::EightBit).unwrap();

    {
        let regs = octospi.inner_mut();

        regs.cr.write(|w| w.en().clear_bit());

        // Try to configure registers as per MX25LM51245G_EnableDTRMemoryMappedMode

        regs.ir.write(|w| unsafe { w.instruction().bits(0xEE11) }); // MX25LM51245G_OCTA_READ_DTR_CMD

        regs.ccr.write(|w| unsafe {
            w.imode()
                .bits(4)
                .idtr()
                .bit(true)
                .isize()
                .bits(1)
                .admode()
                .bits(4)
                .addtr()
                .bit(true)
                .dmode()
                .bits(4)
                .ddtr()
                .bit(true)
                .dqse()
                .bit(true)
        });

        regs.tcr.write(|w| unsafe { w.dcyc().bits(6) });

        regs.cr.write(|w| unsafe { w.fmode().bits(3) });

        regs.cr.modify(|_, w| w.en().set_bit());
    }*/

    let nor_flash_ptr = 0x90000000 as *mut u8;
    let flash_region = unsafe {
        core::slice::from_raw_parts_mut(
            nor_flash_ptr,
            16 * 1024 * 1024 / core::mem::size_of::<u8>(), // SIZE IS WRONG
        )
    };
    i_slint_core::debug_log!("reading from address {:#?}", nor_flash_ptr);
    i_slint_core::debug_log!("{}", flash_region[0]);
    i_slint_core::debug_log!("{}", flash_region[1]);
    i_slint_core::debug_log!("{}", flash_region[2]);

    /*
    let mut led_red = gpioc.pc2.into_push_pull_output();
    led_red.set_low(); // low mean "on"
    let mut led_green = gpioc.pc3.into_push_pull_output();
    led_green.set_low();
    */

    cp.SCB.enable_dcache(&mut cp.CPUID);

    #[link_section = ".frame_buffer"]
    static mut FB1: [TargetPixel; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
        [TargetPixel::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT];
    #[link_section = ".frame_buffer"]
    static mut FB2: [TargetPixel; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
        [TargetPixel::BLACK; DISPLAY_WIDTH * DISPLAY_HEIGHT];
    // SAFETY the init function is only called once (as enforced by Peripherals::take)
    let (fb1, fb2) = unsafe { (&mut FB1, &mut FB2) };

    assert!((hyperram_ptr as usize..hyperram_ptr as usize + hyperram_size)
        .contains(&(fb1.as_ptr() as usize)));
    assert!((hyperram_ptr as usize..hyperram_ptr as usize + hyperram_size)
        .contains(&(fb2.as_ptr() as usize)));

    // setup LTDC  (LTDC_MspInit)
    let _p = gpioa.pa3.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioa.pa4.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioa.pa6.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiob.pb0.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiob.pb1.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiob.pb8.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiob.pb9.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioc.pc6.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioc.pc7.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiod.pd0.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiod.pd3.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiod.pd6.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioe.pe0.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioe.pe1.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioe.pe11.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioe.pe12.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioe.pe15.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiog.pg7.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpiog.pg14.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph3.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph8.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph9.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph10.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph11.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph15.into_alternate::<14>().speed(High).internal_pull_up(true);
    let _p = gpioa.pa8.into_alternate::<13>().speed(High).internal_pull_up(true);
    let _p = gpioh.ph4.into_alternate::<9>().speed(High).internal_pull_up(true);

    let mut lcd_disp_en = gpioe.pe13.into_push_pull_output();
    let mut lcd_disp_ctrl = gpiod.pd10.into_push_pull_output();
    let mut lcd_bl_ctrl = gpiog.pg15.into_push_pull_output();

    delay.delay_ms(40u8);
    // End LTDC_MspInit

    let mut ltdc = hal::ltdc::Ltdc::new(dp.LTDC, ccdr.peripheral.LTDC, &ccdr.clocks);

    const RK043FN48H_HSYNC: u16 = 41; /* Horizontal synchronization */
    const RK043FN48H_HBP: u16 = 13; /* Horizontal back porch      */
    const RK043FN48H_HFP: u16 = 32; /* Horizontal front porch     */
    const RK043FN48H_VSYNC: u16 = 10; /* Vertical synchronization   */
    const RK043FN48H_VBP: u16 = 2; /* Vertical back porch        */
    const RK043FN48H_VFP: u16 = 2; /* Vertical front porch       */

    ltdc.init(embedded_display_controller::DisplayConfiguration {
        active_width: DISPLAY_WIDTH as _,
        active_height: DISPLAY_HEIGHT as _,
        h_back_porch: RK043FN48H_HBP - 11, // -11 from MX_LTDC_Init
        h_front_porch: RK043FN48H_HFP,
        v_back_porch: RK043FN48H_VBP,
        v_front_porch: RK043FN48H_VFP,
        h_sync: RK043FN48H_HSYNC,
        v_sync: RK043FN48H_VSYNC,
        h_sync_pol: false,
        v_sync_pol: false,
        not_data_enable_pol: false,
        pixel_clock_pol: false,
    });
    let mut layer = ltdc.split();

    // Safety: the frame buffer has the right size
    unsafe {
        layer.enable(fb1.as_ptr() as *const u8, embedded_display_controller::PixelFormat::RGB565);
    }

    lcd_disp_en.set_low();
    lcd_disp_ctrl.set_high();
    lcd_bl_ctrl.set_high();

    // Init Timer
    let mut timer = dp.TIM2.tick_timer(10000.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);
    timer.listen(hal::timer::Event::TimeOut);

    // Init RNG
    let rng = dp.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);

    // Init Touch screen
    let scl = gpiof.pf14.into_alternate::<4>().set_open_drain().speed(High).internal_pull_up(true);
    let sda = gpiof.pf15.into_alternate::<4>().set_open_drain().speed(High).internal_pull_up(true);
    let mut touch_i2c = dp.I2C4.i2c((scl, sda), 100u32.kHz(), ccdr.peripheral.I2C4, &ccdr.clocks);

    {
        let mut ft5336 = touch_device(&mut delay, &mut touch_i2c);
        ft5336.init(&mut touch_i2c);
    }

    crate::init_with_display(StmDevices {
        work_fb: fb2,
        displayed_fb: fb1,
        line_buffer: alloc::vec![TargetPixel::default(); DISPLAY_WIDTH],
        layer,
        timer,
        rng,
        delay,
        touch_i2c,
        system_control_block: cp.SCB,
        last_touch: Default::default(),
        prev_dirty: Default::default(),
    });
}

struct StmDevices {
    /// The frame buffer which is not currently shown and on which we can operate
    work_fb: &'static mut [TargetPixel],
    /// The frame buffer which is currently displayed
    displayed_fb: &'static mut [TargetPixel],
    /// A buffer that holds the line in the "fast" ram
    line_buffer: alloc::vec::Vec<TargetPixel>,
    layer: LtdcLayer1,
    timer: hal::timer::Timer<pac::TIM2>,
    rng: hal::rng::Rng,
    delay: Delay,
    touch_i2c: TouchI2C,
    last_touch: Option<i_slint_core::graphics::Point>,
    system_control_block: hal::device::SCB,

    /// When using double frame buffer, this is the part still dirty in the other buffer
    prev_dirty: PhysicalRect,
}

impl Devices for StmDevices {
    fn screen_size(&self) -> PhysicalSize {
        PhysicalSize::new(DISPLAY_WIDTH as _, DISPLAY_HEIGHT as _)
    }

    fn prepare_frame(&mut self, dirty_region: PhysicalRect) -> PhysicalRect {
        dirty_region.union(&core::mem::replace(&mut self.prev_dirty, dirty_region))
    }

    fn render_line(
        &mut self,
        line: PhysicalLength,
        dirty_region: crate::renderer::DirtyRegion,
        fill_buffer: &mut dyn FnMut(&mut [TargetPixel]),
    ) {
        let line = line.get() as usize;
        fill_buffer(&mut self.line_buffer);
        while self.layer.is_swap_pending() {}
        let region = dirty_region.cast::<usize>();
        self.work_fb[line * DISPLAY_WIDTH + region.min_x()..line * DISPLAY_WIDTH + region.max_x()]
            .copy_from_slice(&self.line_buffer[region.min_x()..region.max_x()]);

        // We don't render directly to the frame buffer because the frame buffer is in a slower RAM
        //fill_buffer(&mut self.work_fb[line * DISPLAY_WIDTH..(line + 1) * DISPLAY_WIDTH])
    }

    fn flush_frame(&mut self) {
        self.system_control_block.clean_dcache_by_slice(self.work_fb);
        // Safety: the frame buffer has the right size
        unsafe { self.layer.swap_framebuffer(self.work_fb.as_ptr() as *const u8) };
        // Swap the buffer pointer so we will work now on the second buffer
        core::mem::swap::<&mut [_]>(&mut self.work_fb, &mut self.displayed_fb);
    }

    fn debug(&mut self, text: &str) {
        i_slint_core::debug_log!("Debug: {}", text);
    }

    fn read_touch_event(&mut self) -> Option<i_slint_core::input::MouseEvent> {
        let mut ft5336 = touch_device(&mut self.delay, &mut self.touch_i2c);
        let touch = ft5336.detect_touch(&mut self.touch_i2c).unwrap();
        let button = i_slint_core::items::PointerEventButton::left;

        if touch > 0 {
            let state = ft5336.get_touch(&mut self.touch_i2c, 1).unwrap();
            let pos = i_slint_core::graphics::Point::new(state.y as _, state.x as _);
            Some(match self.last_touch.replace(pos) {
                Some(_) => i_slint_core::input::MouseEvent::MouseMoved { pos },
                None => i_slint_core::input::MouseEvent::MousePressed { pos, button },
            })
        } else {
            self.last_touch
                .take()
                .map(|pos| i_slint_core::input::MouseEvent::MouseReleased { pos, button })
        }
    }

    fn time(&self) -> core::time::Duration {
        // FIXME! the timer can overflow
        let val = self.timer.counter() / 10;
        core::time::Duration::from_millis(val.into())
    }

    fn random_seed(&mut self) -> u64 {
        ((self.rng.value().unwrap_or_default() as u64) << 32)
            | (self.rng.value().unwrap_or_default() as u64)
    }
}

type TouchI2C = hal::i2c::I2c<pac::I2C4>;
fn touch_device<'a>(
    delay: &'a mut Delay,
    touch_i2c: &mut TouchI2C,
) -> ft5336::Ft5336<'a, TouchI2C> {
    ft5336::Ft5336::new(touch_i2c, 0x70 >> 1, delay).unwrap()
}
