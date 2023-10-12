  define p-dma-sts
    mon read32 0x40020000
  end

  document p-dma-sts
    Print N32L406 DMA Status Register
  end

target extended-remote :3333
file build/output.elf
load
b main

# as suggested from https://github.com/pyocd/pyOCD/issues/960#issuecomment-695844639
set mem inaccessible-by-default off

source PyCortexMDebug/scripts/gdb.py
svd_load svd/N32L406_with_core.svd

svd
