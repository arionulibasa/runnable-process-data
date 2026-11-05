# STM32F446RE + Dual LAN9250 Ring Process Data

A generic, multi-slave, no-heap, no-DC bench example using `../soem-lib`.
Uses SPI3 for the primary controller and SPI1 for the secondary controller.
The polling-capable, port-labelled frame trace is shared with the slaveinfo library.
No changes to the original C SOEM or the slave firmware are required by this example.

## UART Console

USART2 TX uses PA2 at 115200 baud, 8 data bits, no parity, one stop bit.
This firmware is for the modified PCB where PA2 has been physically disconnected
from the SPI1 LAN9250 IRQ net. Verify isolation with power off before flashing.

Connect PA2 to the USB-UART adapter RX and connect common ground. Use a 3.3 V
logic-level adapter; its TX is not needed for this transmit-only console.
PC12 is exclusively the SPI3 (primary) LAN9250 chip-select, not UART5 TX.
PA9 is unused by this program. The temporary `uart-pa9` feature and the
UART-configuration build guard have been removed; a normal build now works.

The SPI1 (secondary) controller uses a no-IRQ marker instead of reserving another GPIO.
The current LAN9250 driver polls SPI registers/FIFOs and does not read its IRQ
parameter, so both redundant interfaces continue operating without SPI1 IRQ.

## Discovery And Mapping

The program accepts 1 through 8 physical slaves, including different identities
and PDO layouts. Slave numbers follow discovery order, starting at 1; slot 0 is
the library's aggregate record. There is no fixed livingup identity or PDO contract.
`src/contract.rs` and `tools/inspect_esi.py` remain as legacy livingup reference
utilities, but the executable does not call them.

The mapper reads each slave's advertised mapping. Default `PdoSource::Auto` uses
CoE when available and falls back to SII when the library finds no usable CoE
mapping. Communication/protocol errors are not blindly ignored. `--features sii`
forces SII for the entire group and requires valid EEPROM PDO categories; the
previous livingup EEPROM without those categories is not suitable for this mode.
This is not universal device support: slaves requiring proprietary startup SDOs,
DC, unsupported mailbox services, or unsupported mappings need additional work.

Group 0 is mapped once, using byte-aligned sequential layout:

```text
[all slaves' outputs][all slaves' inputs]
```

The IOmap has 1024 bytes of capacity, not 1024 bytes per slave. Startup prints
each slave's actual byte/bit lengths and offsets. Oversized maps, invalid ranges,
or an entirely empty group stop the program; input-only and output-only slaves
are supported. Expected WKC comes from the mapper, not a fixed value or slave count.

`MAX_PHYSICAL_SLAVES` and `IO_MAP_BYTES` are in `src/main.rs`. The current IOmap
fits one datagram segment, so two frame buffers cover either LRW or separate
LRD/LWR. Increasing these limits requires checking RAM, frame-pool capacity,
state-poll latency, and cycle timing. Supported capacity does not guarantee that
every map fits the chosen 20 ms cycle on the two 4 MHz SPI interfaces.

## Safety And Modes

Use an isolated bench with actuators disconnected or independently disabled.
Zero outputs, SAFE_OP, and a successful INIT request are not hardware safety
guarantees. The slave application and hardware must implement suitable fail-safe
and watchdog behavior. Watchdog settings are not changed by this program.

- Default: stay in SAFE_OP and exchange PDOs, with every mapped output byte zero.
- `enable-op`: after a valid whole-group PDO exchange, broadcast OP and keep
  cycling while checking each slave independently. Use only after confirming
  that zero outputs and entering OP are appropriate for every connected device.
- No output patterns, SDO downloads, PDO remapping commands, or EEPROM writes.

Bad cyclic WKC, incomplete frames, AL errors, unexpected states, OP timeout, and
three consecutive cycle overruns stop the test. A fault saves the trace, attempts
broadcast INIT, and prints state diagnostics for all discovered records. There
is no automatic application restart or slave-state recovery. Frame-level
redundant recovery is attempted within each receive budget, before a failure
reaches this monitor. Even one unrecovered cycle stops this bench test. A panic cannot perform that cleanup.

## Board Connections

| Signal | Primary LAN9250 | Secondary LAN9250 |
| --- | --- | --- |
| Bus | SPI3 | SPI1 |
| SCLK | PB3 (AF6) | PA5 |
| MISO | PB4 (AF6) | PA6 |
| MOSI | PB5 (AF6) | PA7 |
| CS | PC12 | PC4 |
| RESET | PC11 | PC5 |
| IRQ | PB7 | Disconnected from MCU (no GPIO) |

The primary IRQ pin PB7 remains an input. Both controllers are polled over SPI;
no EXTI handler is used and PA2 is never configured as an IRQ input.
Both chip selects start high. Each controller gets its own reset sequence and
a distinct MAC (02:00:00:00:00:01 and 02:00:00:00:00:02).
TIM2 supplies shared reset/SPI delays and the monotonic clock without a heap.

UART is USART2 TX on PA2, 115200 8N1. ST-Link uses SWDIO PA13 and SWCLK PA14,
ground and target-voltage reference.
Do not use JTAG or SWO: PB3/PB4 are assigned to SPI3.
HSI 16 MHz, both SPIs 4 MHz mode 0, TIM2 1 MHz, soft-float Cortex-M4 build.
Both PHYs are forced to 100 Mbps full duplex.

Connect the physical ring as a slave chain with its ends at the two master NICs:

```text
SPI3 Primary -> Slave 1 IN -> Slave 1 OUT -> ... -> Last slave OUT -> SPI1 Secondary
```

When upgrading from v3 (SPI1 primary), swap only the two Ethernet cables at the
master so SPI3 connects to the first slave's IN port. Leave the inter-slave cables,
SPI wiring, reset wiring, PB7 IRQ input and PA2 UART wiring unchanged.
Trace labels and MAC roles follow the new assignment: Primary/src=0x0101 is SPI3,
Secondary/src=0x0404 is SPI1.

Do not connect this as a loop through an ordinary Ethernet switch. Startup checks
both controller IDs and requires both master PHY links up within ten seconds.
Both links up is only a cabling/PHY check, not proof that every slave is reachable.

## Timing And Raw Input Output

- PDO exchange period: 20,000 us, for the entire group, not per slave.
- PDO receive budget: 8,000 us shared across the cycle, including recovery;
  SPI submission takes additional time.
- CoE mailbox frame receive budget: 8,000 us, capped by the remaining overall
  SDO budget of 700,000 us. This allows full mailbox frames more SPI read time
  than the previous 2,000 us cap. It does not retry writes or ignore timeouts.
- AL-state register timeout: 4,000 us per transfer. Library discovery/EEPROM
  operations retain their own existing timeout settings.
- Route observation limit: 1,500 us, capped by the library at half each receive
  budget. These are initial bench values, not measured timing guarantees.
- One slave's AL state is polled every 100 ms, rotating through the slaves.
  With eight slaves, each is polled approximately every 800 ms. OP must be
  confirmed independently for each slave within two seconds of the request.
- One UART record every 500 ms, rotating through slaves. With two slaves, each
  appears approximately once per second. UART transmission runs in cycle idle time.

Example format (illustrative, not captured hardware output):

```text
n=26 slave=1 AL=0004/0000 WKC=4/4 lastPDO=Some(IntactRing) I[0..14/14]=[...] gap=20006us work=5901us missed=0 logdrop=0
n=51 slave=2 AL=0004/0000 WKC=4/4 lastPDO=Some(PartialViaSecondary) I[0..6/6]=[...] gap=20006us work=7901us missed=0 logdrop=0
```

`lastPDO` is the route of the LAST completed PDO datagram, captured before
AL-state reads can replace it. With split LRD/LWR it does not summarize both
frames. `IntactRing` means crossed real/probe returns were observed;
`SecondaryOnly` means the real response arrived at the secondary without a
stored crossed probe, which can also happen with an intact ring and a delayed
probe. Neither label alone proves overall ring health. `PrimaryOnly` selects
the real primary return, `OriginalViaSecondary` resends the original request
via secondary, and `PartialViaSecondary` forwards the partially processed
primary return via secondary. Always check the full-group WKC and all AL states.

`WKC` is for the whole group; `AL` is that slave's most recently polled state.
`I[start..end/total]` uses byte offsets relative to that slave's inputs, with an
exclusive end. Inputs longer than 16 bytes rotate through chunks on successive
turns. Chunks come from different cycles, not a frozen whole-input snapshot.
An output-only slave prints `I[0..0/0]=[]`. Partial-byte PDOs are displayed as raw
bytes; only the advertised bit count is meaningful. No signedness, scaling,
object names, or engineering units are inferred.

`gap` and `work` are maximum observed cycle-start interval and work duration;
`missed` counts skipped starts, and `logdrop` counts unqueued UART records.
Failed or incomplete PDO cycles are not printed as fresh inputs. WKC confirms
mapped EtherCAT accesses, not physical wiring or application callback correctness.
These diagnostics are not hard real-time guarantees; the polling SPI driver
can block on hardware faults. Adjust timing only with the slave requirements known.

## Ring Bench Test

With PA2 isolated from LAN9250 IRQ and the UART adapter connected:

1. Power the slaves, connect both ends of the chain, and start with the default
   SAFE_OP behavior (no `enable-op` feature). Keep actuators disconnected.
2. Confirm both ID_REV lines, successful discovery of the expected slave count,
   the mapped layouts, and repeated valid WKC with AL=0004/0000.
3. Disconnect only one EtherCAT cable at a time while leaving every slave powered.
   Test an inter-slave cable, the primary end, and the secondary end separately.
4. Check WKC/AL and the PDO route. An unrecovered frame, timeout, or AL error
   stops the program and prints the last 32 events from both NICs in one sequence.
   Records show Primary/Secondary and the source marker (`src=0x0101` for
   real traffic, `src=0x0404` for diagnostic BRD).
5. Restore the cable. If the test stopped, restore the complete ring and reset
   the master before the next trial. Reconnection behavior must also be measured.

A transition may lose a cycle even if later traffic would recover; this program
deliberately does not hide bad cycles or automatically reconfigure slaves.
The library still has its documented delayed-frame/index-reuse limitations.

## Build And Flash

These commands target the modified PA2-console board. The default build remains
SAFE_OP with zero outputs. Use the new ELF path below, not an older PA9/PC12 image.
The startup banner must say `dual-LAN9250 ring v4, SPI3 primary` and print
`Mailbox frame receive budget=8000 us; overall SDO budget=700000 us`.

```sh
cd /home/Usurper_01/programs/soem-stm32f4-processdata
cargo build --release --locked --target-dir target/ring-pa2
probe-rs download --chip STM32F446RE --protocol swd --verify target/ring-pa2/thumbv7em-none-eabi/release/soem-stm32f4-processdata
probe-rs reset --chip STM32F446RE --protocol swd
```

Select the master's ST-Link with `--probe` if multiple probes are connected.
This replaces the master application, not the slaves. Start the slave firmware,
connect the EtherCAT chain, open UART, then reset the master. Rebuilding alone
does not update the board. No flashing is performed automatically.

Only after output behavior is verified for every slave:

```sh
cargo build --release --locked --features enable-op --target-dir target/ring-pa2-op
```

Keep OP builds separate so they do not overwrite the default SAFE_OP ELF.
For forced SII mapping, use `--features sii --target-dir target/ring-pa2-sii`.

## Verification

```sh
cargo test --lib --target x86_64-unknown-linux-gnu --locked
cargo test --lib --manifest-path ../soem-stm32f4-slaveinfo/Cargo.toml --target x86_64-unknown-linux-gnu --locked
cargo test --manifest-path ../soem-lib/Cargo.toml --target x86_64-unknown-linux-gnu --locked
```

Host tests cover mixed-layout byte regions, one-direction and empty-slave ranges,
invalid bounds, chunk rotation, configured group WKC, independent OP confirmation,
timeouts, scheduling, reset sequencing, bounded ring telemetry, and polling
trace forwarding without logging empty polls. Legacy contract tests remain separate.
ARM builds and host tests do not replace multi-slave ring testing. The PA2-console
firmware is compile-verified, not hardware-tested or automatically flashed.

Startup follows original SOEM `samples/simple_ng/simple_ng.c`: discover, map,
confirm SAFE_OP, exchange PDOs, and optionally request and confirm OP while
continuing exchanges. DC and background recovery are deliberately omitted.
