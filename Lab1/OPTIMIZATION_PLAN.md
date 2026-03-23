# Lab1 Routing - Continuous Optimization Framework

## Current Version Analysis

### V1 (Original) 
- **Testcase1 Result**: 29 tracks, cost ≈ 2455
- **Search**: Random seeds + 2 phases
- **Time**: ~60s

### V1.1 (Current)
- **Key Changes**: 
  - More generous seed count  (20000 seeds for tracks <= 26)
  - Start from track 23 (closer to baseline 30)
  - Phase 2 explores ±1 adjacent tracks
- **Expected Improvement**: 5-10% better on coupling metrics

---

## Critical Optimization Targets

### 1. **Track Count Optimization** (HIGHEST IMPACT)
The scoring function shows:
- Baseline = 30 tracks (65% score)
- > 30 tracks = 0 points
- Every track UNDER 30 = significant score boost

**Strategy**: Aggressive exploration from 20-28 tracks with 50,000+ seeds each

```
Track 20: HIGH priority (up to +35 points if successful)
Track 21-24: MEDIUM-HIGH priority
Track 25-28: MEDIUM priority  
Track 29+: Fill remaining time
```

### 2. **Coupling Length Penalty** (SECOND HIGHEST)
Formula: Penalty = δ × (coupling_length / net_span)

**Aggressive Tactics**:
- Dynamically reorder pins to minimize parallel segments
- Post-routing **track swapping** between adjacent nets
- **Vertical spacing**: Insert vias earlier to break coupling

### 3. **Via Cost vs Spillover Trade-off**
- Via cost = 5
- Spillover penalty = α × (spillover_width)
- Break tracks apart BEFORE spillover happens

---

## Next Generation Strategies

### V4: **Post-Routing Local Search**
```
for each pair of adjacent tracks:
  if swapping nets reduces coupling:
    perform swap
    recalculate cost
    keep if better
repeat until no improvement
```

### V5: **Predictive Track Pre-assignment**
```
// Before random search:
1. Build constraint graph (which nets conflict)
2. Identify "must-separate" net pairs
3. Pre-assign tracks to independent net groups
4. Only randomize within groups
```

### V6: **Coupling-Driven Seed Generation**
instead of random seeds:
```
// Seed = hash of pin/constraint positions
seed = hash(top_pins[0..50] + constraints)
...with variation: seed XOR random_offset
```

### V7: **Dynamic Track Allocation**
```
for each seed:
  if (tracks <= 28) seed_iterations = 100,000
  else if (tracks <= 30) seed_iterations = 50,000
  else seed_iterations = 10,000
```

---

## Benchmark Progression

| Version | TC1 Tracks | TC1 Cost | TC1 Time | Improvement |
|---------|-----------|---------|---------|------------|
| V1      | 29        | 2455    | 60s     | baseline   |  
| V1.1    | ?         | ?       | ?       | TBD        |
| V4      | 28-29     | <2300   | 280s    | +5-10%     |
| V5      | 27-28     | <2100   | 280s    | +15-20%    |

---

## Submission Checklist

```
[ ] Makefile produces executable "Lab1"
[ ] Runtime < 300 seconds
[ ] Output format matches spec exactly
[ ] Tested on both testcase1.txt and testcase2.txt
[ ] Track count <= 30 (testcase1) and <= 60 (testcase2)
[ ] No compilation warnings/errors with gcc 8.5.0
```

## Time Management

```
Phase 1 (65% = 182s):  
  - Track 20-24: 20,000 seeds each = 100,000 total
  - Track 25-30: 10,000 seeds each = 60,000 total  
  - Track 31-35: 3,000 seeds each = 15,000 total
  
Phase 2 (35% = 98s):
  - Best track ± 1: 50,000 seeds each
  - Prioritize < 30 tracks hard
```
