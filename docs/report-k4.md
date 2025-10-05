# CS452 K4 Assignment Report

## K4 at a Glance

| Area | Key change in K4 | Benefit |
|------|------------------|---------|
| **I/O** | Busy-wait → **interrupt-driven I/O servers** | Zero CPU spin, true concurrency |
| **Marklin control** | Dedicated servers (topology, command, conductor, trains) | Clear separation of concerns |

---

## IO Server Architecture
```
User Tasks
   │  (Send/Receive)
┌──▼──────────────────────────┐
│          I/O Server         │◄─────┐
│  – Console (Buffered)       │      │
│  – Marklin                  │      │
└──┬──────────────────────────┘      │
   │                                 │
┌──▼───────────────┐                 │
│  Notifier Tasks  │─────────────────┘
│  – RX Notifier   │
│  – TX Notifier   │
└──┬───────────────┘
  AwaitEvent
┌──▼───────────────┐
│  Kernel Events   │
└──────────────────┘
```
---

##  I/O Interface

### Servers & Channels
| Server / Task | Purpose |
|---------------|---------|
| **I/O Server** | Central dispatcher |
| **RX Notifier** | AwaitEvent `EVENT_UART_RX` → notify server |
| **TX Notifier** | AwaitEvent `EVENT_UART_TX` → notify server |

| Channel | Buffering | Input | Output |
|---------|-----------|-------|--------|
| **Console (UART0)** | 10 KB circular | Blocking `Getc` / non-blocking `TryGetc` | Buffered `Putc` / bulk `Putn` |
| **Marklin (UART3)** | None (timing critical) | Same as console | Direct `Putc`; senders queue if unable to send |

### Public API
| Call | Behaviour |
|------|-----------|
| `Getc(chan)` | Block until a character is available |
| `TryGetc(chan)` | Return immediately or `IO_NO_DATA` |
| `Putc(chan, c)` | Output one char (may block on UART3) |
| `Putn(str, len)` | Bulk write (console only) |

### Design Highlights
* **Interrupt-driven** RX/TX eliminates busy-wait loops.
* **Client pool** of 64 `io_client_t`; no runtime allocation.
* Optional **busy-wait debug path**: compile-time flag `ENABLE_BUSY_WAIT_DEBUG` (default **OFF**) swaps the interrupt based io for simple polling routines, useful when debugging
---

## Marklin Control Stack
```
Controller Server
├── Topology Server    (track graphs A/B)
├── Command Server     (rate-limited UART3 writes)
├── Conductor Server   (signals, switches, reservations)
├── Train Tasks        (one per active train)
└── Message Queue      (publish/subscribe events)
```
| Component | Key duties |
|-----------|------------|
| **Controller** | Spawn trains, global state, reset/init |
| **Command** | Serialize Marklin commands and send to IO Server|
| **Topology** | Authoritative graph|
| **Conductor** | Sensor attribution, switch state, future collision-avoidance & path-finding |
| **Train task** | Speed/direction, localisation |

---

## Kernel Algorithms & Data Structures

| Subsystem | Technique | Rationale |
|-----------|-----------|-----------|
| **Scheduler** | Priority bitmap + 32 FIFO queues | O(1) dispatch, strict fairness |
| **IPC** | Per-task intrusive sender queue | Keeps order, no extra alloc |
| **Name Server** | linear probing | O(n) lookup, malloc-free |
| **Event Sys** | Global blocked list filtered by ID | One list ⇒ low memory |
| **Delay Queue** | Sorted intrusive list + freelist | O(log n) insert, deterministic |
| **IRQ Table** | Static array indexed by IRQ # | O(1) handler lookup |

---

## Critical System Parameters

| Parameter | Value | Reason |
|-----------|-------|--------|
| Task stack size | **2 MB** | Deep Marklin call chains |
| Max tasks | 64 | Covers train-control workload |
| Priorities | 32 | Fine-grained scheduling |
| Console TX buffer | 10 KB | Handles log bursts |

---

## Kernel Design Summary

### 1. Task and Scheduling System

**Scheduling**

- Priority-based *pre-emptive* scheduler with **32 priority levels** (0 = highest).
- **O(1) ready-task selection** implemented by a *priority bitmap* plus 32 FIFO ready queues.
- *Strict pre-emption* guarantees that a high-priority task is never starved by lower levels.
- *FIFO within each priority* preserves arrival order and provides fairness amongst tasks of equal importance.

**Task Control Block (task_t)**

| Field | Purpose |
|-------|---------|
| `tid`, `parent_tid` | Identification & parentage |
| `state` | READY / RUNNING / BLOCKED / ZOMBIE |
| `priority` | 0-31 |
| `context` | 304-byte ARM64 register save area |
| `stack_base`, `stack_top`, `stack_size` | 2 MB user stack |
| `sender_queue` | Intrusive list head for pending senders |
| `ready_node`, `blocked_node` | List nodes for ready / wait lists |
| `event_id` | What event the task is blocked on (if any) |

**System limits**

- **Max tasks**: 64 (0-63) – enough for the train workload while keeping the bitmap in a single 64-bit word.
- **Stack per task**: **2 MB** – deep enough for typical call chains.

### 2. Context Switching

On each syscall (`svc #0`) or interrupt:
1. User registers are pushed to the *kernel stack* then copied to the task's `context` field.
2. The scheduler selects the next READY task via the bitmap.
3. Selected task's context is restored and execution resumes via `eret` back to EL0.

### 3. Inter-Process Communication – Send / Receive / Reply (SRR)

- Synchronized IPC – the sender sleeps until a reply arrives.
- Each task owns an **intrusive sender queue**; order of arrival is preserved naturally by the list.
- Messages are copied directly between user buffers
- A non-blocking `receive` is implemented to enable certain tasks to check for messages without blocking.

### 4. Name Server

A dedicated task with *linear probing*. This provides O(n) `RegisterAs` / `WhoIs`.

---

## Event System and Interrupts

### 5. Interrupt Handling

- Hardware routed through **GIC**; the kernel programs priority mask and enables UART0, UART3 and system timer interrupts.
- A **1024-entry ISR table** maps IRQ number → handler. The dispatcher calls the registered C function which typically: clears hardware, unblocks waiting tasks, and nests back to scheduler.

### 6. AwaitEvent

`AwaitEvent(id)` simply inserts the calling task into the *global blocked list* and sets its `event_id`. When the matching ISR fires, all tasks with that ID are spliced back onto the ready queue.

---

## System Services

### 7. Clock Server & Notifier

- The *notifier* blocks in `AwaitEvent(EVENT_TIMER_TICK)` (10 ms period).
- On every tick it time-stamps and notifies the *clock server*.
- The server maintains a **sorted intrusive delay queue** (binary insertion) offering `Time`, `Delay`, and `DelayUntil` APIs.

### 8. I/O Servers & Notifiers

Covered earlier, the console and Marklin channels are fully *interrupt-driven*. The TX/RX notifiers wait on `EVENT_UART_*` and communicate with a central I/O server. Zero busy-wait ensures maximum CPU for control logic.

### 9. Idle Task

The permanent lowest-priority task executes `wfi` in a loop and records elapsed cycles. CPU utilisation is reported to the console every 100 ms.

---

For a deeper dive into earlier milestones, please refer to:

* `docs/report-k1.md` – Scheduling & Context Switch
* `docs/report-k2.md` – IPC & Name Server
* `docs/report-k3.md` – Clock & Interrupt handling
