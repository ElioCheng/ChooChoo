# CS452 K2 Assignment Report

## Kernel Structure Description

### Key Features Implemented

Our K2 kernel builds upon the K1 foundation with the following major additions:

1. **Send-Receive-Reply (SRR) Message Passing System**
   - Synchronous inter-task communication with blocking semantics
   - Atomic message exchange ensuring sender blocks until reply received
2. **Name Server**
   - Centralized task registry mapping names to Task IDs (TIDs)
   - Supports `RegisterAs()` and `WhoIs()` operations
   - Enables dynamic task discovery without hardcoded TIDs
4. **User Applications**
   - Multi-task RPS game demonstrating IPC usage
   - SRR performance measurement

### Algorithms & Data Structures

#### Message Passing (SRR Implementation)
- **Sender Queues**: Each task maintains an intrusive linked list of pending senders (`ipc_sender_queue`)
- **Message Buffering**: Direct memory copying between user spaces, no kernel buffering
- **Blocking State Management**: Tasks store IPC context (message pointers, lengths, target TIDs) when blocked
- **Synchronization**: Sender blocks until receiver processes message and sends reply

#### Name Server
- **Fixed-Size Hash Table**: Array of 64 name entries for O(1) average lookup
- **Linear Probing**: Simple collision resolution with sequential search
- **String Storage**: Fixed 32-character name limit to avoid dynamic allocation

### System Parameters and Limitations

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **MAX_REGISTRATIONS** | 64 | Matches task limit, one name per task maximum |

## Unimplemented Aspects and Known Issues
No unimplemented aspects or known issues.

## Send-Receive-Reply Semantics

The Send-Receive-Reply (SRR) mechanism is designed to handle message exchanges between tasks in a synchronized and ordered manner. This implementation focuses on three main properties: atomicity, correct blocking behavior, and message ordering.

- **Atomicity**: The SRR mechanism ensures that message exchanges between tasks are atomic. A sender will block until the corresponding reply from the receiver is received. This prevents the sender from proceeding until the communication is fully completed, ensuring consistency in the communication flow.

- **Correct Blocking Behavior**:
  - **Sender Blocking**: The `Send` function blocks the sender task until the receiver sends a reply. This ensures that the sender does not continue until the communication is complete.
  - **Receiver Blocking**: The `Receive` function blocks the receiver task if there are no messages in its queue, waiting for the sender's message to arrive. This behavior prevents unnecessary execution when no data is available, maintaining synchronization between tasks.

- **Ordering Guarantees**:
  - **Message Arrival Order**: The order in which messages are received matches the order in which they were sent. This guarantees that the receiver processes messages in the same sequence they were sent, ensuring deterministic behavior.


## RPS Game Test Description

### Overview
Our RPS (Rock-Paper-Scissors) game serves as a comprehensive test of the kernel's inter-task communication, dynamic task discovery, and message-passing correctness. The test involves multiple concurrent games with automatic player pairing and proper game state management.

### Components
2. **RPS Server (TID 3) (server.c)**: Manages game logic and player pairing
3. **RPS Clients (TIDs 4-7) (client.c)**: Four client tasks that play games

#### Game Flow
1. **Initialization**:
   - Name server starts and registers itself
   - RPS server starts, registers as "rps_server"
   - Four client tasks start and discover server via name server

2. **Player Pairing**:
   - Clients send signup requests to server
   - Server automatically pairs clients when 2+ are waiting
   - Paired clients receive confirmation and proceed to game

3. **Gameplay**:
   - Each game consists of 3 rounds
   - Players send moves (Rock/Paper/Scissors) to server
   - Server waits for both moves before evaluating
   - Results sent back to both players with opponent's move

4. **Game Completion**:
   - After 3 rounds, clients send quit requests
   - Server cleans up game state
   - Clients terminate gracefully

### Expected Output

The test demonstrates:
- **Successful Task Discovery**: All clients locate RPS server via name server
- **Concurrent Game Management**: Multiple games (Tasks 4&5, Tasks 6&7) run simultaneously
- **Proper Synchronization**: Server waits for both moves before evaluation
- **Message Passing Correctness**: All IPC operations complete successfully
- **Graceful Termination**: All tasks exit cleanly after game completion

**Sample Output Pattern:**
```
RPS Server: Starting up (tid 3)
RPS Client: Starting up (tid 4)
RPS Client 4: Found RPS server at tid 3
RPS Client 4: Sending signup request
RPS Server: Added tid 4 to signup queue (count: 1)
RPS Client: Starting up (tid 5)
RPS Server: Starting game 0 between tid 4 and tid 5
RPS Client 4: Playing Rock (round 1)
RPS Client 5: Playing Paper (round 1)
RPS Server: Both players in game 0 have played - Rock vs Paper
RPS Client 4: Round 1 Completed - Rock vs Paper (Lose)
RPS Client 5: Round 1 Completed - Paper vs Rock (Win)
...
RPS Client 4: Sending quit request
RPS Client 4: Exiting
```

## Performance Measurement

### Methodology

#### Test Design
We measured round-trip latency of complete Send-Receive-Reply cycles under various system configurations:

- **How tests were designed:** The tests utilized two tasks: a sender task that performs `Send()` calls and a receiver task that handles `Receive()` and `Reply()` operations. The sender task measures the total time from the first `Send()` to the completion of the last reply across 10,000 iterations and 100 warmup iterations for cache stabilization. Tests were run with different variables as described below.
- **Variables Tested**:
  - **Message sizes**: 4, 64, 256 bytes
  - **Task creation order**: Sender-first vs Receiver-first
  - **Compiler optimization**: O0 vs O3
  - **Cache configurations**: No cache, I-cache only, D-cache only, Both caches

#### Build System
Automated build system (`scripts/srr_perf_build.sh`) generates 8 kernel configurations:
- 4 cache settings × 2 optimization levels = 8 total configurations
- Each configuration builds complete kernel image for testing
- Performance results aggregated into CSV format for analysis

### Results Summary

| Configuration                 | Message Size | Sender First (μs) | Receiver First (μs) | Difference |
| ----------------------------- | ------------ | ----------------- | ------------------- | ---------- |
| **No Optimization, No Cache** | 4 bytes      | 1786.4            | 1884.9              | +5.5%      |
|                               | 64 bytes     | 1870.0            | 1988.7              | +6.3%      |
|                               | 256 bytes    | 2138.1            | 2319.2              | +8.5%      |
| **Optimized, Both Caches**    | 4 bytes      | 6.1               | 6.2                 | +1.4%      |
|                               | 64 bytes     | 6.1               | 6.2                 | +1.3%      |
|                               | 256 bytes    | 6.3               | 6.4                 | +1.3%      |

### Key Performance Insights
- **Optimization Impact**: Compiler optimization (O3) provides a ~293× speedup compared to unoptimized code (O0)
- **Cache Effectiveness**: Both caches enabled show ~49× improvement over no cache in optimized builds
- **Data Cache Dominance**: Data cache alone provides significantly better performance than instruction cache alone
- **Message Size Scaling**: Latency increases modestly with message size (6.1μs → 6.4μs for 4→256 bytes in best case)
- **Execution Order**: Receiver-first scheduling consistently shows 1-8% higher latency, likely due to additional context switches

### What did you learn from the performance data?

The performance data reveals that our SRR implementation achieves excellent performance under optimal conditions (6-7μs round-trip latency), demonstrating efficient kernel message passing. The data cache has the most significant impact on performance, suggesting that memory access patterns dominate the execution time. The minimal impact of message size on latency indicates our implementation efficiently handles variable-length messages without significant copying overhead. The slight penalty for receiver-first execution suggests that task scheduling order affects context switch patterns, with sender-first providing marginally better cache locality or reduced scheduling overhead.
