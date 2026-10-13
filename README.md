# STM32F446RE + LAN9250 Process Data

A generic, multi-slave, no-heap, no-DC bench example using `../soem-lib`.
The board setup and frame trace are shared with the working slaveinfo example.
No changes to the original C SOEM or the slave firmware are required by this example.

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
every map fits the chosen 10 ms cycle on the 4 MHz SPI interface.

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
is no automatic restart or recovery. A panic cannot perform that cleanup.

## Board Connections

SPI1 PA5 SCLK, PA6 MISO, PA7 MOSI; LAN9250 CS PC4, RESET PC5, INT PA2 (polled).
UART5 TX PC12, 115200 8N1; use a 3.3 V UART adapter and common ground.
ST-Link: SWDIO PA13, SWCLK PA14, ground and target-voltage reference.
HSI 16 MHz, SPI 4 MHz mode 0, TIM2 1 MHz, soft-float Cortex-M4 build.
The PHY is forced to 100 Mbps full duplex as in the tested slaveinfo program.

## Timing And Raw Input Output

- PDO exchange period: 10,000 us, for the entire group, not per slave.
- PDO receive budget: 2,000 us; SPI submission takes additional time.
- State-register timeout: 2,000 us per transfer.
- One slave's AL state is polled every 100 ms, rotating through the slaves.
  With eight slaves, each is polled approximately every 800 ms. OP must be
  confirmed independently for each slave within two seconds of the request.
- One UART record every 500 ms, rotating through slaves. With two slaves, each
  appears approximately once per second. UART transmission runs in cycle idle time.

Example format (illustrative, not captured hardware output):

```text
n=51 slave=1 AL=0004/0000 WKC=4/4 I[0..14/14]=[...] gap=10006us work=3901us missed=0 logdrop=0
n=101 slave=2 AL=0004/0000 WKC=4/4 I[0..6/6]=[...] gap=10006us work=3901us missed=0 logdrop=0
```

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

## Build And Flash

```sh
cd /home/Usurper_01/programs/soem-stm32f4-processdata
cargo build --release --locked
probe-rs download --chip STM32F446RE --protocol swd --verify target/thumbv7em-none-eabi/release/soem-stm32f4-processdata
probe-rs reset --chip STM32F446RE --protocol swd
```

Select the master's ST-Link with `--probe` if multiple probes are connected.
This replaces the master application, not the slaves. Start the slave firmware,
connect the EtherCAT chain, open UART, then reset the master. Rebuilding alone
does not update the board. No flashing is performed automatically.

Only after output behavior is verified for every slave:

```sh
cargo build --release --locked --features enable-op --target-dir target/op
```

Keep OP builds separate so they do not overwrite the default SAFE_OP ELF.
For forced SII mapping, use `--features sii --target-dir target/sii`.

## Verification

```sh
cargo test --lib --target x86_64-unknown-linux-gnu --locked
cargo test --manifest-path ../soem-lib/Cargo.toml --target x86_64-unknown-linux-gnu --locked
```

Host tests cover mixed-layout byte regions, one-direction and empty-slave ranges,
invalid bounds, chunk rotation, configured group WKC, independent OP confirmation,
timeouts, scheduling, and bounded telemetry. Legacy contract tests remain separate.
ARM builds and host tests do not replace multi-slave board testing.

Startup follows original SOEM `samples/simple_ng/simple_ng.c`: discover, map,
confirm SAFE_OP, exchange PDOs, and optionally request and confirm OP while
continuing exchanges. DC and background recovery are deliberately omitted.
