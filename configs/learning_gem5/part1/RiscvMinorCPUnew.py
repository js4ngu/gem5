import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")
import os

from caches import *
from common import SimpleOpts

# Default binary path setting for RISC-V
thispath = os.path.dirname(os.path.realpath(__file__))
default_binary = os.path.join(
    thispath,
    # "/home/gem5/tests/riscv-elf/cmp_scalar_rope.riscv"
    "/home/gem5/tests/riscv-elf/cmp_simd_rope_vfxV2-128.riscv",
    # "/home/gem5/tests/riscv-elf/cmp_simd_rvv_standard_rope.riscv"
)

# Add binary option
SimpleOpts.add_option("binary", nargs="?", default=default_binary)
args = SimpleOpts.parse_args()

# Create the system
system = System()

# Set the clock frequency
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

# Set up the system
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("512MB")]

# Create a RISC-V MinorCPU
system.cpu = RiscvMinorCPU()

# Create an L1 instruction and data cache
system.cpu.icache = L1ICache(args)
system.cpu.dcache = L1DCache(args)

# Connect the instruction and data caches to the CPU
system.cpu.icache_port = system.cpu.icache.cpu_side
system.cpu.dcache_port = system.cpu.dcache.cpu_side

# Create a memory bus for L2 connections
system.l2bus = L2XBar()

# Connect the L1 caches to the L2 bus
system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports

# Create an L2 cache and connect it to the L2 bus
system.l2cache = L2Cache(args)
system.l2cache.cpu_side = system.l2bus.mem_side_ports

# Create a memory bus
system.membus = SystemXBar()

# Connect the L2 cache to the memory bus
system.l2cache.mem_side = system.membus.cpu_side_ports

# Create the interrupt controller for the CPU and connect to the membus
system.cpu.createInterruptController()

# Create a DDR3 memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# Connect the system up to the membus
system.system_port = system.membus.cpu_side_ports

# Set the workload
system.workload = SEWorkload.init_compatible(args.binary)

# Create a process for the RISC-V binary
process = Process()
process.cmd = [args.binary]
system.cpu.workload = process
system.cpu.createThreads()

# Set up the root SimObject and start the simulation
root = Root(full_system=False, system=system)
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
