# CS452 K3 Assignment Report

## Group Members

| Name              | Waterloo ID | Student Number |
|-------------------|-------------|----------------|
| Tongkun Zhang     | t365zhan    | 20764299       |
| Zixuan Cheng      | z24cheng    | 21002294       |

---

## Repository Information

- **Repository URL:** https://git.uwaterloo.ca/tongkun/choochoo
- **Commit SHA:** `344b78dfacda9993522cea09daf5389d318a890d`
- **Commit Date:** [View Commit](https://git.uwaterloo.ca/tongkun/choochoo/-/tree/344b78dfacda9993522cea09daf5389d318a890d)

---

## Kernel Structure Description

Our K3 kernel extends the previous K2 milestone with **pre-emptive interrupt handling, an event system, a clock server, and an idle task with CPU-usage instrumentation**.

### GIC and Interrupt Sub-system
---
The interrupt subsystem is implemented through a two-layer architecture: a low-level GIC driver and a high-level interrupt management layer.

#### GIC Controller
**Initialization Process** (`gic_init()`):
1. **Reset State**: Disable all interrupts, clear pending/active states, and reset priorities
2. **Interrupt Routing**: Route all Shared Peripheral Interrupts (SPIs) to CPU 0
3. **Security Configuration**: Configure all interrupts as Group 1 (non-secure) for EL0/EL1 access
4. **Priority Setup**: Set default priority `0x80` for all interrupt sources
5. **Controller Enable**: Enable both distributor and CPU interface with Group 1 forwarding

**Dynamic Handler Registration**:
- **Handler Table**: Static array `irq_handlers[1024]` for dispatching by IRQ number

#### Architecture-Independent Interrupt Layer
**Interrupt Handling Flow**:
1. **Hardware Exception**: CPU vectors to `irq_el0_handler` in exception table
2. **Context Save**: Assembly code saves complete EL0 context (registers, PC, SP, SPSR) to kernel stack
3. **IRQ Identification**: `gic_get_interrupt()` reads `GICC_IAR` to identify active interrupt
4. **Handler Dispatch**: Look up registered handler in `irq_handlers[]` and invoke with IRQ number and data
5. **EOI Signal**: `gic_end_interrupt()` writes to `GICC_EOIR` to signal completion
6. **Context Switch**: `sched_schedule()`

#### Interrupt Masking Strategy

**EL1 (Kernel) Interrupt Masking**:
- **Boot Initialization**: `msr daifset, #2` in `boot.S` masks IRQs during kernel initialization

**EL0 (User) Interrupt Enablement**:
- **SPSR Manipulation**: When switching to user mode, the kernel clears bit 7 (IRQ mask) in SPSR_EL1
- **Context Restoration**: `restore_el0_context` macro with `el0_irq_enabled=1` parameter:
  ```assembly
  .if \el0_irq_enabled
      bic x19, x19, #(1 << 7)  // Clear IRQ mask bit in SPSR
  .endif
  msr spsr_el1, x19
  ```

### Event Mechanism & AwaitEvent Syscall
---

   • Tasks block on an integer event id; ISR producers call `event_unblock_waiting_tasks` to wake them.
   • Provides a clean separation between hardware interrupts and user-level servers.

#### Event System Design

The event system provides an asynchronous communication mechanism between interrupt service routines (ISRs) and user-space tasks, enabling clean separation of hardware handling from application logic.

**Event Flow**:
1. **Blocking**: User tasks call `AwaitEvent(event_id)` syscall to block on a specific event
   - Task state changes to `TASK_BLOCKED_EVENT`
   - Task's `event_id` field set to the awaited event
   - Task removed from ready queue and added to blocked queue

2. **Event Generation**: Hardware interrupts trigger ISRs which call `event_unblock_waiting_tasks(event_id, event_data)`
   - Validates the event ID against `EVENT_MAX`
   - Calls scheduler to wake all tasks waiting on that specific event

3. **Unblocking**: Scheduler scans blocked tasks for matching `event_id`
   - Matching tasks transition from `TASK_BLOCKED_EVENT` to `TASK_READY`
   - Tasks return from `AwaitEvent()` with the provided event data
   - Normal scheduling resumes

### Clock Server & Notifier
---

The clock server is a user-level service that provides precise timing functionality to other tasks via three APIs: `Time`, `Delay`, and `DelayUntil`, as defined in `clock.h`. It operates in tandem with a notifier task that listens for timer tick events and drives the advancement of system time.

Internally, the clock server maintains an O(log n) ordered intrusive linked list of at-most 64 delayed tasks. Each entry in this list contains a task ID and a target wake-up tick. A fixed-size pool of 64 `delayed_task_t` structures is managed with a circular free list, ensuring no dynamic memory allocation is required at runtime. When a task requests a delay, it is inserted into the delay list in ascending order of wake-up time. This design enables efficient expiration checks and preserves deterministic behavior for equal-tick delays.

The notifier is created at priority 1 and uses `AwaitEvent(EVENT_TIMER_TICK)` to block until a 10ms timer interrupt occurs. Upon receiving an event, the notifier sends a `CLOCK_TICK_NOTIFY` message to the clock server. This increments the internal tick counter and checks the delay list for any expired tasks. Tasks whose delay has elapsed are replied to, unblocked, and returned to the free list.

The clock server also responds to `CLOCK_TIME` requests by returning the current tick count, and to `CLOCK_DELAY_UNTIL` or `CLOCK_DELAY` by calculating the appropriate wake-up tick and inserting the task accordingly. Error handling is performed for negative delays and when the task pool is exhausted.

To preserve responsiveness and fairness, `Receive()` is used to dequeue client requests in arrival order, and the notifier is prioritized above all clients to ensure tick advancement is not starved by user-level task load.

Overall, this implementation provides robust and predictable timer services in constant memory, with O(log n) insertion and O(#expired) wake up, and is a foundational component for scheduling and time-sensitive operations in the kernel environment.


### Idle Task & CPU Usage Stats
---
 When no other tasks are ready to run, the idle task executes the ARM `wfi` (Wait For Interrupt) instruction, putting the CPU core into a low-power state until the next interrupt arrives. It also monitors CPU utilization using the kernel's idle time tracking and reports the usage every 100ms.

#### Idle Time Calculation Algorithm

The idle time calculation uses a sliding window approach with microsecond precision:

**Data Structure** (`idle_stats_t`):
- `last_idle_start_time`: Timestamp when the current idle period began
- `idle_time_in_window`: Accumulated idle time within the current measurement window
- `measurement_window_us`: Size of measurement window (1,000,000 μs = 1 second)
- `last_measurement_time`: When the current measurement window started
- `idle_percentage`: Calculated idle percentage for the completed window

**Accounting Process**:
1. **Idle Entry** (`idle_start_accounting()`): When the scheduler selects the idle task, we record the current timestamp as `last_idle_start_time` and set `is_idle_running = true`.

2. **Idle Exit** (`idle_stop_accounting()`): When any other task becomes ready, we:
   - Calculate `idle_duration = current_time - last_idle_start_time`
   - Add this duration to `idle_time_in_window`
   - Set `is_idle_running = false`
   - Trigger percentage calculation

3. **Percentage Calculation** (`idle_update_percentage()`):
   - Check if measurement window (1 second) has elapsed
   - If yes: `idle_percentage = (idle_time_in_window * 100) / elapsed_time`
   - Reset window: `idle_time_in_window = 0` and update `last_measurement_time`

**CPU Usage Reporting**:
The idle task reports CPU usage as `100 - idle_percentage` every 100ms at the top of the console output.

### Core Data Structures and Algorithms
---

| Sub-system | Structure / Algo | Reasoning |
|------------|------------------|-----------|
| IRQ table | `irq_entry_t irq_handlers[1024]` | Static array indexed by IRQ number for O(1) dispatch. Memory cost << stack switch time. |
| Event wait list | Kernel blocked queue filtered by `task->event_id` | Re-uses intrusive dlist already present for other block reasons ⇒ no extra heap allocations. |
| Delayed tasks | Sorted doubly linked list + free-list pool (64 slots) | O(#expired) wake-up, O(n) insertion but small n ≤ 64; no dynamic malloc in userland. |
| Idle stats | `idle_stats_t` ring-buffer | Constant time accounting; integer math only. |

### System Parameters & Limits (unchanged unless noted)
---

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Delayed tasks per clock server | 64 | Upper-bounded by `MAX_TASKS`, avoids dynamic allocation. |
| Timer tick period | 10 ms | Meets assignment requirement. |

---

## Expected Output

```
CPU Usage: 1 % [window: 1000 ms] [count: 492]
...
[     0.000][0][INFO][gic_init:17] Initializing GIC
[     0.000][0][INFO][gic_init:64] GIC initialized
[     0.000][0][INFO][interrupt_init:13] Interrupt subsystem initialized
[     0.000][0][INFO][time_setup_timer_tick:43] Timer C1 interrupt configured for 10ms intervals (IRQ 97)
...
Clock client 6: Starting
Clock client 7: Starting
Clock client 8: Starting
Clock client 9: Starting
Init done
Clock client 6: Delayed for 10 ticks (1/20)
Clock client 6: Delayed for 10 ticks (2/20)
Clock client 7: Delayed for 23 ticks (1/9)
Clock client 6: Delayed for 10 ticks (3/20)
Clock client 8: Delayed for 33 ticks (1/6)
Clock client 6: Delayed for 10 ticks (4/20)
Clock client 7: Delayed for 23 ticks (2/9)
Clock client 6: Delayed for 10 ticks (5/20)
Clock client 6: Delayed for 10 ticks (6/20)
Clock client 8: Delayed for 33 ticks (2/6)
Clock client 6: Delayed for 10 ticks (7/20)
Clock client 7: Delayed for 23 ticks (3/9)
Clock client 9: Delayed for 71 ticks (1/3)
Clock client 6: Delayed for 10 ticks (8/20)
Clock client 6: Delayed for 10 ticks (9/20)
Clock client 7: Delayed for 23 ticks (4/9)
Clock client 6: Delayed for 10 ticks (10/20)
Clock client 8: Delayed for 33 ticks (3/6)
Clock client 6: Delayed for 10 ticks (11/20)
Clock client 7: Delayed for 23 ticks (5/9)
Clock client 6: Delayed for 10 ticks (12/20)
Clock client 6: Delayed for 10 ticks (13/20)
Clock client 8: Delayed for 33 ticks (4/6)
Clock client 7: Delayed for 23 ticks (6/9)
Clock client 6: Delayed for 10 ticks (14/20)
Clock client 9: Delayed for 71 ticks (2/3)
Clock client 6: Delayed for 10 ticks (15/20)
Clock client 6: Delayed for 10 ticks (16/20)
Clock client 7: Delayed for 23 ticks (7/9)
Clock client 8: Delayed for 33 ticks (5/6)
Clock client 6: Delayed for 10 ticks (17/20)
Clock client 6: Delayed for 10 ticks (18/20)
Clock client 7: Delayed for 23 ticks (8/9)
Clock client 6: Delayed for 10 ticks (19/20)
Clock client 8: Delayed for 33 ticks (6/6)
Clock client 6: Delayed for 10 ticks (20/20)
Clock client 7: Delayed for 23 ticks (9/9)
Clock client 9: Delayed for 71 ticks (3/3)
```
### First User Task Output Description

This output demonstrates the correct behavior of the clock server and its interaction with multiple client tasks, validating the functionality of the system’s priority queue and timer interrupt mechanism.

Each clock client receives its delay parameters from the first user task and begins periodic delays accordingly. Clients with shorter delay intervals and higher priorities (lower numeric values) appear more frequently and earlier in the output, confirming that the system respects both timing and task priority. For example, Clock Client 6 (priority 3, 10-tick interval) is serviced most frequently, with its 20 delay iterations appearing consistently in the output. In contrast, lower-priority clients with longer intervals, such as Client 9 (priority 6, 71-tick interval), appear less frequently and later.

This scheduling pattern reflects correct implementation of the delay queue: delayed tasks are enqueued based on their wake-up tick and dequeued in order of earliest wake time, regardless of task creation order. The consistent reappearance of each client at the correct tick intervals confirms that timer interrupts are firing at 10ms intervals as configured, and that delayed tasks are being correctly unblocked and rescheduled.

In summary, this output validates that:
- Timer interrupts are occurring at the expected frequency.
- The clock server unblocks tasks at the correct wake-up time.
- The priority queue correctly preserves delay ordering and task scheduling behavior, ensuring predictable and fair task execution.



## Known Issues

### Intermittent image corruption on `linux.student.cs.uwaterloo.ca`

- **Symptom**: A freshly-built `kernel.img` sometimes changes hash a few seconds after `objcopy` finishes, even though `stat` shows no further modifications.
- **Environment**
This happens on the `linux.student.cs.uwaterloo.ca` server within the user's home directory which is mounted on `cephfs`.

```bash
  $ df -hT .
  Filesystem                            Type  Size  Used Avail Use% Mounted on
  10.1.244.35,10.1.244.38,10.1.244.41:/ ceph  272T  101T  171T  38% /u6
```

- **Reproduction command**

```bash
make
/u/cs452/public/xdev/bin/aarch64-none-elf-objcopy \
    -O binary \
    /u6/t365zhan/cs452/choochoo/build/kernel.elf \
    /u6/t365zhan/cs452/choochoo/build/kernel.img \
&& sync \
&& sleep 1 \
&& touch build/kernel.img \
&& sha256sum build/kernel.img
```
	1.	objcopy … — converts the ELF to a raw binary image.
	2.	sync     — asks the kernel to flush pending writes.
	3.	sleep 1  — gives CephFS time to finish outstanding I/O.
	4.	touch    — bumps the mtime
	5.	sha256sum— records the hash immediately after the build.
Running
```bash
watch -n 1 'sha256sum build/kernel.img && stat build/kernel.img'
```
shows that the hash flips a few seconds later while the inode metadata stays the same.

#### What we've verified

1. No competing writers
    - `objcopy` exits cleanly, and `lsof` confirm no other process has `kernel.img` open or trying to write to it.
2. Local filesystems unaffected
    - Building in `/tmp` (local ext4) produces a stable image:
```bash
$ df -hT /tmp
Filesystem  Type Size Used Avail Use% Mounted on
/dev/vda2   ext4 985G 176G  759G 19% /
```

3. Immediate copy is reliable
    - If the image is copied right after creation, the backup remains intact while the original may later become corrupted:
```makefile
all:
	rm -rf build
	mkdir  build
	cd build && cmake -G Ninja -DCMAKE_BUILD_TYPE=Release \
	                   -DMMU=on -DCMAKE_CROSS_COMPILER_PATH=/u/cs452/public/xdev/bin ..
	cd build && ninja
	cp build/kernel.img build/kernel.img.bak   # stays good
```

#### Possible cause
- We think the behaviour might be caused by CephFS client write-back caching. While we are writing this report, the file system seems to be in a consistent state.

#### Work-arounds:
- Perform the build on a local disk like `/tmp`.
- Use the copied image `kernel.img.bak` instead of the original `kernel.img`.

---
