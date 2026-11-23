// SPDX-License-Identifier: GPL-3.0-or-later
//! UART/DMA adapter boundary. Traits keep framing and control independent of
//! the HAL; the mock implementation is used by host tests.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UartError {
    /// No completed DMA buffer yet.
    WouldBlock,
    /// Transfer error reported by the DMA/UART peripheral.
    Dma,
    /// Bytes arrived faster than a free slot could be provided.
    Overrun,
}

/// A completed DMA receive buffer, copied into `out`.
pub trait DoubleBufferRx {
    /// `Ok(None)` means no frame buffer is ready yet.
    fn take(&mut self, out: &mut [u8]) -> Result<Option<usize>, UartError>;
}

/// Zero-copy counterpart to [`DoubleBufferRx`]. Borrow the published DMA slot
/// instead of copying it into a caller-owned scratch buffer: a caller that
/// feeds a frame assembler from this slice performs one data move, not two.
pub trait ZeroCopyRx {
    /// Borrow the next published DMA chunk. `None` means nothing is ready.
    /// The borrow stays valid until the next `take_borrowed` on this receiver.
    fn take_borrowed(&mut self) -> Result<Option<&[u8]>, UartError>;
}

/// Request transmitter. Hardware implementations await the DMA write; the
/// wire bytes are always owned by the caller (a static buffer).
pub trait FrameTx {
    fn send(&mut self, frame: &[u8]) -> Result<(), UartError>;
}

/// Two static RX slots. While the control task validates the front slot, DMA
/// fills the back one; `swap` publishes the back slot atomically from the
/// control task's point of view.
#[derive(Clone, Copy, Debug)]
pub struct DmaBuffers<const N: usize> {
    pub slots: [[u8; N]; 2],
    pub lens: [Option<usize>; 2],
    pub active: usize,
}

impl<const N: usize> DmaBuffers<N> {
    pub const fn new() -> Self {
        Self {
            slots: [[0; N]; 2],
            lens: [None, None],
            active: 0,
        }
    }

    pub const fn back_index(&self) -> usize {
        1 - self.active
    }

    pub fn mark_ready(&mut self, slot: usize, len: usize) {
        assert!(slot < 2 && len <= N, "invalid DMA slot or length");
        self.lens[slot] = Some(len);
    }

    pub fn take_ready(&mut self, slot: usize) -> Option<usize> {
        assert!(slot < 2, "invalid DMA slot");
        self.lens[slot].take()
    }

    pub fn swap(&mut self) {
        self.active = self.back_index();
    }
}

impl<const N: usize> Default for DmaBuffers<N> {
    fn default() -> Self {
        Self::new()
    }
}
