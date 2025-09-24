# CS652 Assignment 0 Report

## Commit Information
- `e4ea001e4bc4d83dd28dbd76e064bf4aad696064`
- [Gitlab](https://git.uwaterloo.ca/tongkun/choochoo/-/tree/e4ea001e4bc4d83dd28dbd76e064bf4aad696064)

## How to run the program
- See `README-A0` for instructions

## Program Structure

The kernel consists of several key components:

### 1. Kernel Initialization (`kernel.c`)
- Serves as the main entry point for the system
- Initializes hardware components
- Sets up subsystems: timers, logging, marklin train control, user interface
- Implements the main processing loop that handles UART communication, timer processing, and UI updates

### 2. Timer Subsystem (`timer/timer.c`)
- Provides time-based event scheduling functionality
- Processes expired timers and executes their callbacks

### 3. Time Management (`time/time.c`)
- Provides system-wide time tracking functionality
- Manages access to the hardware system timer (1MHz frequency)
- Implements boot time tracking to support relative time measurements
- Provides handy macros for time conversion, time formatting, and time sleep, etc.

### 4. Marklin Train Control (`marklin.c`)
- Manages communication with the Marklin model train hardware

### 4. Work Queue System (`workqueue.c`)
- Implements a deferred work execution mechanism
- Allows scheduling of work items to be executed at a later time

### 5. User Interface (`tui.c`)
- Implements a text-based user interface for interacting with the system
- Allows partial screen updates to reduce the amount of data sent to the UART

### 6. Double Linked List (`dlist.h`)
- Implements a doubly-linked list data structure

### 7. UART (`uart.c`)
- Manages serial communication with external devices
- Implements buffered UART transmission to handle high throughput
- Implements buffer status monitoring and visualization via GPIO pins

### 8. Debug Infrastructures
- (`symbol.c`) Implements a symbol table for backtrace
- (`klog.c`) Implements a logging system
- (`boot_test.c`) Runs a couple of small tests to verify the system is working when booting up in a debug build
- (`panic.c`) Implements a panic handler that prints the context of the panic (`exception.c`)

## Algorithms and Data Structures

### 1. Timer Management
- **Data Structure**: Sorted doubly-linked list (implemented via `dlist.h`)
- **Algorithm**:
  - Timers are stored in a list sorted by expiration time
  - The timer processing function checks for expired timers and executes their callbacks

This approach was chosen for its efficiency in both insertion and processing. Since timers are sorted by expiration time, the system only needs to process timers from the head of the list until it finds a timer that hasn't expired yet.

### 2. Work Queue
- **Data Structure**: Doubly-linked list for queued work items (implemented via `dlist.h`)
- **Algorithm**:
  - Work items are added to the tail of the queue
  - A timer periodically triggers processing of work items
  - Work items are removed from the queue and executed in FIFO order

This design allows for efficient scheduling of deferred work and helps manage tasks that need to be executed after specific delays, for example, sending commands to the Marklin model train hardware.

### 3. Marklin Command Dispatching
- **Data Structure**: Fixed-size pool of command work structures
- **Algorithm**:
  - Commands are wrapped in work items and scheduled on a dedicated work queue
  - The work queue processes commands with appropriate timing constraints

This approach ensures commands are sent to the train system with proper timing constraints to prevent communication issues.

## Unimplemented Aspects and Known Bugs

- The TUI is not showing properly after toggling it on and off
- The sensor timing measurements are not implemented yet

## How do you know that your clock does not miss updates or lose time?

The timer system is designed to ensure accurate timekeeping through several mechanisms:

1. The main loop in `kernel.c` calls `timer_process()` on every iteration, ensuring timers are checked frequently.

2. Timers are stored in a sorted list, with the earliest expiring timer at the head. This means the system only needs to check the head of the list to know if any timers have expired.

3. When processing timers, the system:
   - Compares the current time with each timer's expiration time
   - Executes callbacks for expired timers
   - For periodic timers, reschedules them with a new expiration time
   - Maintains the sorted order of the timer list

These mechanisms work together to ensure that even if a timer processing cycle is delayed, timers will still execute correctly in the next cycle without losing track of time.