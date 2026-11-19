# Speed Testing

## CPP

All tests run with max compiler optimizations.

**Telemetry Updates**

```
telemetry speed, 200000 iterations, 10% warmup, seed 1

batches of 1, 1 update each
  per-batch       22.9 ns  ~126.5x faster
  per-update      22.9 ns  ~126.5x faster

batches of 12, 1 update each
  per-batch      151.9 ns  ~29.0x faster
  per-update      12.7 ns  ~28.9x faster

batches of 12, 20 updates each
  per-batch      178.0 ns  ~30.4x faster
  per-update      14.8 ns  ~30.5x faster

random batches 1-26, random updates 1-20
  per-batch      185.1 ns  ~28.7x faster
  per-update      14.3 ns  ~28.5x faster
```

**Logging**

```
log write speed, 500000 messages after 30000 warmup

single character
  file:        384.9 ns/msg  ~39.3x faster
  console:     665.3 ns/msg  ~22.8x faster (line buffered)

long string
  file:       1015.2 ns/msg  ~15.5x faster
  console:    2845.9 ns/msg  ~5.7x faster (line buffered)

one decimal
  file:        471.4 ns/msg  ~33.9x faster
  console:     827.8 ns/msg  ~18.9x faster (line buffered)

long string with values
  file:       2650.5 ns/msg  ~8.1x faster
  console:    5205.5 ns/msg  ~4.2x faster (line buffered)
```

**Data Monitor File Writing**

```
candump, 1 item per call
  per-batch       86.5 ns  ~30.5x faster
  per-update      86.5 ns  ~30.5x faster

candump, 32 items per call
  per-batch     2086.5 ns  ~2.2x faster
  per-update      65.2 ns  ~2.2x faster

candump, random 1-32 items per call
  per-batch     1237.1 ns  ~3.3x faster
  per-update      68.7 ns  ~3.3x faster

batches of 1, 1 update each
  per-batch      113.5 ns  ~27.0x faster
  per-update     113.5 ns  ~27.0x faster

batches of 12, 1 update each
  per-batch      270.6 ns  ~12.4x faster
  per-update      22.6 ns  ~12.4x faster

batches of 12, 20 updates each
  per-batch     1948.0 ns  ~3.5x faster
  per-update     162.3 ns  ~3.5x faster

random batches 1-26, random updates 1-20
  per-batch     1186.0 ns  ~5.2x faster
  per-update      91.1 ns  ~5.2x faster
```

**Full-System Updates (No-SDK, Paced max=350/s)**

```
full motor update, barrier, 20 motors, 773 updates in 2.001 s
  updates/s      386.3  ~1.33x faster
  latency us  min    147.4  mean    588.3  p95    702.1  p99    808.6  max   1121.4
full motor update, paced, 20 motors, 700 updates in 2.000 s, dropped 0
  updates/s      349.9  equal
  latency us  min    131.6  mean    575.4  p95    705.0  p99    829.9  max   1184.1
```

## Python

**Telemetry Updates**

```
telemetry speed, 40000 iterations, 10% warmup, seed 1

batches of 1, 1 update each
  per-batch     2897.7 ns
  per-update    2897.7 ns

batches of 12, 1 update each
  per-batch     4407.6 ns
  per-update     367.3 ns

batches of 12, 20 updates each
  per-batch     5409.6 ns
  per-update     450.8 ns

random batches 1-26, random updates 1-20
  per-batch     5317.3 ns
  per-update     408.0 ns
```

**Logging**

```
log write speed, 100000 messages after 10000 warmup

single character
  file:      15124.2 ns/msg
  console:   15182.3 ns/msg (line buffered)

long string
  file:      15747.5 ns/msg
  console:   16169.3 ns/msg (line buffered)

one decimal
  file:      15968.9 ns/msg
  console:   15638.2 ns/msg (line buffered)

long string with values
  file:      21437.2 ns/msg
  console:   21654.3 ns/msg (line buffered)
```

**Data Monitor File Writing**

```
candump, 1 item per call
  per-batch     2637.2 ns
  per-update    2637.2 ns

candump, 32 items per call
  per-batch     4591.6 ns
  per-update     143.5 ns

candump, random 1-32 items per call
  per-batch     4051.6 ns
  per-update     225.1 ns

batches of 1, 1 update each
  per-batch     3070.0 ns
  per-update    3070.0 ns

batches of 12, 1 update each
  per-batch     3368.1 ns
  per-update     280.7 ns

batches of 12, 20 updates each
  per-batch     6840.8 ns
  per-update     570.1 ns

random batches 1-26, random updates 1-20
  per-batch     6175.9 ns
  per-update     474.5 ns
```

**Full-System Updates (No-SDK, Paced max=350/s)**

```
full motor update, barrier, 20 motors, 577 updates in 2.000 s
  updates/s      288.5
  latency us  min    432.5  mean   2522.5  p95   3131.6  p99   3367.4  max   3617.0
full motor update, paced, 20 motors, 700 updates in 2.000 s, dropped 0
  updates/s      350.0
  latency us  min    152.6  mean   1862.6  p95   3265.2  p99   3712.4  max   4366.4
```


## Teensy TCP vs. UDP

UDP: 

```
teensy echo, window 1, 19678 messages in 2.000 s
  messages/s    9838.7
  latency us  min     62.7  mean    100.1  p95    130.3  p99    221.9  max    449.3
```

TCP:

```
teensy echo, window 32, 151766 messages in 2.000 s
  messages/s   75869.7
  latency us  min    107.2  mean    394.0  p95    441.9  p99    461.6  max   1096.2
```
