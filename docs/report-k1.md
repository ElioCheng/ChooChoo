# CS452 Assignment K1 Report

## Group Members

| Name              | Waterloo ID | Student Number |
|-------------------|-------------|----------------|
| Tongkun Zhang     | t365zhan    | 20764299       |
| Zixuan Cheng      | z24cheng    | 21002294       |

## Repository Information

- **Repository:** https://git.uwaterloo.ca/tongkun/choochoo
- **Commit SHA:** `59ea9c1142b6b7d7532b89c3b4260a07efbc846d`
- **Commit URL:** [View Commit CHANGEME](https://git.uwaterloo.ca/tongkun/choochoo/-/tree/59ea9c1142b6b7d7532b89c3b4260a07efbc846d)

---

## Kernel Structure Description

### Overview

For this assignment, we implemented the foundational components of our kernel, including the distinction between user mode and kernel mode, context switching, and scheduling. Our kernel supports multiple user programs with different priorities and uses system calls to manage interactions between user programs and the kernel.

Each time we transition from user mode to kernel mode (for example, through a system call), the scheduler is invoked. The scheduler selects the next user program to run based on its priority, choosing the highest-priority task from the ready queue. Once a task is selected, the kernel performs a context switch and resumes execution in user mode with the selected program. After the user program finishes running, it exits, and the scheduler is invoked again to schedule the next task.

### Algorithms and Data Structures

#### Scheduler
For the scheduler, our current choice of algorithm is purely based on priority. Once we enter the scheduler, we look at the ready queue, which has 32 lists, each list representing a priority. We also have a flag for each priority, indicating whether the list for the current priority is empty or not. The scheduler basically finds the list with the highest priority that's not empty, and takes a task from that list to schedule it.

There are also some important details for the scheduler:

- **32 priority levels** (0 = highest priority, 31 = lowest priority)
- **FIFO scheduling within each priority level** - tasks of the same priority are scheduled in round-robin fashion
- **Strict preemption** - higher priority tasks always run before lower priority tasks
- **O(1) scheduling complexity** using a priority bitmap for fast highest-priority lookup

**Implementation Details:**
- The scheduler maintains an array of 32 FIFO ready queues, one per priority level
- A 32-bit priority bitmap tracks which priority levels have ready tasks (bit manipulation for O(1) lookup)
- The `sched_find_highest_priority()` function uses `ffs_u32()` (find first set bit) for efficient priority discovery
- Tasks are enqueued at the tail and dequeued from the head of their priority queue

#### Task Layout
A task is the unit of scheduling in our implementation. For each task, we first have the current task's PID and the parent's PID. Then we also record the state of the task and its priority. Then we have the context, which we use to store the necessary registers during context switching. We also have two stack pointers: stack_base and stack_top. Stack_base doesn't change and is a fixed address, whereas stack_top grows as we use more space in the stack. We also have a stack_size field remembering the size of the stack. Moving forward, we have a couple of fields related to executing the user program and putting it into different queues. We first have entry_point, a pointer to the first line of the user program, and we also have single nodes for ready_queue, blocked_queue, and sender_queue, for us to be able to conveniently put the current task into these queues.

#### Stack Management

**Per-Task Stack Allocation:**
- Each task receives its own dedicated stack space
- Stack size is configurable per task (default: 4KB)

**Kernel Stack:**
- Shared kernel stack for all tasks during kernel mode execution
- Reset to `_start` when returning to polling loop

#### Context Switching Mechanism

Our context switching implementation uses **ARM64 exception handling** with the following components:

**Context Structure (304 bytes):**
```c
struct arch_regs {
    u64 x[32];      // General purpose registers x0-x31 with x[31] as padding (256 bytes)
    u64 sp;         // Stack pointer
    u64 pc;         // Program counter
    u64 spsr;       // Saved program status register
    u64 elr;        // Exception link register
    u64 tpidr;      // Thread pointer register
    u8 padding[8];  // Alignment padding
};
```

**Context Switch Process:**
1. **Exception Entry** - User task makes syscall (SVC instruction), triggering EL0→EL1 transition
2. **Context Save** - Assembly macros in `entry.S` save all registers to kernel stack
3. **Kernel Processing** - Syscall handler copies context to current task's TCB
4. **Scheduling Decision** - Scheduler enqueues the current task to the ready queue and picks the next highest-priority ready task
5. **Context Restore** - `switch_to_user_mode()` restores new task's context
6. **Exception Return** - `eret` instruction transitions EL1→EL0 with new task

### System Parameters

#### Task Configuration
- **Maximum concurrent tasks:** 64
  - *Rationale:* Sufficient for current requirements while maintaining efficient data structures

#### Memory Management
- **Default stack size per task:** 4KB
  - *Rationale:* Adequate for typical user programs while conserving memory

#### Task Identification
- **Task ID range:** 0-63 (matching maximum task count)
- **Priority levels:** 0-31 (0 = highest priority)

The numbers we chose are flexible and sufficient for now. If we need more, we can change the numbers easily.

---

## Unimplemented Features and Known Bugs

### Unimplemented Features

#### Polling Loop Integration
We retained the polling loop from Assignment A0 to maintain UI and timer functionality, but this is not an ideal solution. Currently, the polling loop is only triggered in two specific scenarios:

1. **During `syscall_yield()`**
2. **When the scheduler has no ready tasks**

```c
// In sched_yield() and sched_schedule() (sched.c)
asm volatile("ldr x2, =_start\n"
             "mov sp, x2\n"
             "b polling_loop");
```

**Possible Improvements to remove kernel-level polling:**
1. **Interrupt-Driven Approach:** Replace polling with timer interrupts to trigger periodic kernel services
2. **User-Space Services:** Convert UI functionality into dedicated user-space tasks with appropriate priorities

### Known Bugs

#### 1. Kernel Stack Space Leak During Context Switching

**Description:** When handling EL0 exceptions, the `el0_entry` macro allocates 304 bytes on the kernel stack to temporarily store the user task's context. If the scheduler returns to the same task, this space is properly reclaimed. However, when the scheduler picks a different task and performs a context switch via `switch_to_user_mode()`, the allocated stack space is never reclaimed.

**Current Mitigation:** This bug does not cause immediate system failure because:
- The kernel stack pointer is reset to `_start` every time we return to the polling loop
- The polling loop acts as a "stack reset" mechanism, preventing kernel stack overflow
- Each syscall/context switch leaks exactly 304 bytes, but this is cleared on the next polling loop iteration

**Proposed Fix:** Modify the context switching mechanism to properly reclaim the allocated stack space before switching to the new task, or restructure the exception handling to avoid temporary stack allocation.


## Program Output and Behavior

### Sample Output Description
**Output Format:** Each log entry follows the format `[timestamp][cpu][level][location] message`
- **Timestamp:** Boot time in seconds with millisecond precision (e.g., `0.004` = 4ms after boot)
- **CPU:** CPU core ID
- **Level:** Log level
- **Location:** Function name and line number where the log was generated
- **Message:** Log message content, with task info `[t:tid p:priority]` for syscalls

```
[     0.004][0][INFO][kmain:62] Creating test user task
[     0.006][0][INFO][syscall_mytid:90] [t:1 p:10] syscall_mytid: 1
[     0.006][0][INFO][syscall_klog:115] [t:1 p:10] Hello from task 1! Starting first user task with priority 1
[     0.007][0][INFO][syscall_create:84] [t:1 p:10] syscall_create: priority = 1, function = 0x00400180 -> 2
[     0.007][0][INFO][syscall_create:84] [t:2 p:1] syscall_create: priority = 2, function = 0x00400240 -> 3
[     0.007][0][INFO][syscall_klog:115] [t:2 p:1] Created: 3 (priority: 2)
[     0.008][0][INFO][syscall_create:84] [t:2 p:1] syscall_create: priority = 2, function = 0x00400240 -> 4
[     0.008][0][INFO][syscall_klog:115] [t:2 p:1] Created: 4 (priority: 2)
[     0.008][0][INFO][syscall_create:84] [t:2 p:1] syscall_create: priority = 0, function = 0x00400240 -> 5
[     0.008][0][INFO][syscall_mytid:90] [t:5 p:0] syscall_mytid: 5
[     0.008][0][INFO][syscall_myparenttid:96] [t:5 p:0] syscall_myparenttid: 2
[     0.009][0][INFO][syscall_klog:115] [t:5 p:0] Task 5 (1): My parent is 2, about to yield
[     0.011][0][INFO][syscall_klog:115] [t:5 p:0] Task 5 (2): My parent is 2, about to exit
[     0.011][0][INFO][syscall_klog:115] [t:2 p:1] Created: 5 (priority: 0)
[     0.011][0][INFO][syscall_create:84] [t:2 p:1] syscall_create: priority = 0, function = 0x00400240 -> 5
[     0.012][0][INFO][syscall_mytid:90] [t:5 p:0] syscall_mytid: 5
[     0.012][0][INFO][syscall_myparenttid:96] [t:5 p:0] syscall_myparenttid: 2
[     0.012][0][INFO][syscall_klog:115] [t:5 p:0] Task 5 (1): My parent is 2, about to yield
[     0.014][0][INFO][syscall_klog:115] [t:5 p:0] Task 5 (2): My parent is 2, about to exit
[     0.014][0][INFO][syscall_klog:115] [t:2 p:1] Created: 5 (priority: 0)
[     0.014][0][INFO][syscall_klog:115] [t:2 p:1] FirstUserTask: exiting
[     0.015][0][INFO][syscall_mytid:90] [t:3 p:2] syscall_mytid: 3
[     0.015][0][INFO][syscall_mytid:90] [t:4 p:2] syscall_mytid: 4
[     0.015][0][INFO][syscall_myparenttid:96] [t:3 p:2] syscall_myparenttid: 2
[     0.015][0][INFO][syscall_myparenttid:96] [t:4 p:2] syscall_myparenttid: 2
[     0.015][0][INFO][syscall_klog:115] [t:3 p:2] Task 3 (1): My parent is 2, about to yield
[     0.015][0][INFO][syscall_klog:115] [t:4 p:2] Task 4 (1): My parent is 2, about to yield
[     0.019][0][INFO][syscall_klog:115] [t:3 p:2] Task 3 (2): My parent is 2, about to exit
[     0.020][0][INFO][syscall_klog:115] [t:4 p:2] Task 4 (2): My parent is 2, about to exit
[     0.020][0][INFO][syscall_klog:115] [t:1 p:10] Created child task 2
```

So for the first user task, task 1 with a low priority would first get created, and it gets scheduled first and starts running.

The only thing that task 1 does is create task 2, with priority 1, and after that task 1 ends and task 2 gets scheduled and starts running.

The first thing task 2 does is create task 3, with priority 2. Just a quick side note: every time we create a syscall, we put the currently executing task back to the end of the ready queue, and reschedule the tasks based on their priority. But in this case, since the newly created task has lower priority than the task that's currently executing (task 2), after rescheduling, it would still be task 2 running.

Then, task 2 creates task 4, also with priority 2, and the same thing happens: the scheduler gets invoked again but it would still be task 2 running.

After that, task 2 creates task 5, with priority 0, and the scheduler gets invoked again. Because task 5 has higher priority, it gets scheduled and starts running. Task 2, on the other hand, gets put back to the end of the priority 1 ready queue.

So we have task 5 running now, and task 5 calls yield. By calling yield, we put task 5 into the ready queue for priority 0, and reschedule. But because we don't have another task with priority 0, task 5 is still the task with the highest priority, and it would still be the task running after yielding. After yielding and printing the parent ID, task 5 exits, and the tasks still in the ready queue are task 2, 3, and 4.

So we reschedule, and go back to task 2, and it creates task 5 with priority 0 again, as the previous task 5 exited and its TID is available again, and the same thing happens. We will again go back to the situation where we have task 2, 3, and 4 in the ready queue, and it would be task 2 running again.

Then, as task 2 finishes everything it's going to do, it exits, and we're left with task 3 and 4 in the ready queue as we reschedule.

Task 3 will be the first to run, as it got created first and never left the queue, so it will be the first to be scheduled. It calls yield. As it calls yield, it gets put to the end of the ready queue for priority 2, and now its position in the ready queue would be behind task 4. So as we reschedule, task 4 would be the one to run this time.

Then task 4 runs, and it yields, and gets put to the end of the ready queue. Then we reschedule again, and task 3 gets to run again.

Task 3 doesn't have anything else to do, so it exits. That invokes the scheduler, and we reschedule. We look at all the ready queues, and see that only task 4 hasn't finished running yet, so we schedule task 4, and task 4 also finishes running.
