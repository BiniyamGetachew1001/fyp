# 3.11 Controller Design

The autonomous person-following behaviour is realised by a **Decoupled
Finite-State Visual-Servoing (DFS-VS) controller** [1]–[4], [7], [8]. Rather than mapping the
visual error onto a single continuous velocity vector, the controller resolves
each control cycle into exactly one of five mutually-exclusive motion
primitives. This decoupling of *rotation* from *translation* is a deliberate
response to two physical constraints of the platform:

1. **Forward-only differential drive** [5]. Each motor is wired to the L298N driver
   through a single direction pin, so neither wheel can reverse. True in-place
   counter-rotation is therefore impossible; the tightest achievable turn is a
   *pivot* about a stopped wheel. A controller that simultaneously commands
   forward and angular velocity (as in a classical coupled scheme) would force
   the robot to *arc* toward the target, and the target's angular position grows
   faster than a bounded differential can correct as range decreases — causing
   the subject to slide out of frame.
2. **Split "Thinker–Doer" architecture.** All perception and control logic runs
   on the PC; the embedded board acts purely as a motor executor. The control
   law must therefore emit *discrete per-wheel actuation commands* that survive
   transmission over a best-effort wireless link, instead of continuous
   floating-point velocities.

Accordingly, the controller first **rotates until the target is centred**, then
**advances along a straight heading**, re-evaluating the decision every control
cycle. The complete signal chain is:

> camera frame → person detector → tracking-error extraction → EMA filtering →
> finite-state arbitration → differential-drive duty mapping → low-level motor
> conditioning.

---

## 3.11.1 Vision-Based Tracking Error Formulation

The ESP32-CAM transmits an MJPEG video stream at a frame resolution of
**320 × 240 px (QVGA)**. On the host, a deep-learning detector
(YOLO [11], [12]; *person* class only, confidence threshold $c_{\min} = 0.40$) returns the
target's axis-aligned bounding box $(x_1, y_1, x_2, y_2)$ for the most prominent
person in view.

Two scalar quantities are extracted from this box: the **horizontal centroid**,
which drives heading, and the **normalised bounding-box area**, which serves as
a monocular proxy for range.

**Horizontal centroid (normalised):**

$$
\hat{x}(t) = \frac{1}{W}\cdot\frac{x_1(t) + x_2(t)}{2}, \qquad \hat{x}\in[0,1]
$$

where $W = 320\,\text{px}$ is the frame width. Normalising by $W$ makes the
controller **resolution-independent**: the same gains apply unchanged if the
stream resolution is altered.

**Yaw (heading) error:** defined relative to the optical centre and shifted onto
the symmetric interval $[-1, +1]$ so that gains carry a consistent meaning:

$$
e(t) = 2\big(\hat{x}(t) - x_{\text{ref}} - \delta_{\text{cal}}\big)
$$

| Symbol | Meaning | Value |
|--------|---------|-------|
| $x_{\text{ref}}$ | Reference centroid (exact frame centre) | $0.5$ (≡ 160 px) |
| $\delta_{\text{cal}}$ | Calibration offset for camera mounting misalignment | $0.0$ |

Sign convention: $e > 0$ ⇒ target lies to the **right** of centre; $e < 0$ ⇒
target lies to the **left**.

**Range proxy (normalised bounding-box area):**

$$
A(t) = \frac{\big(x_2(t)-x_1(t)\big)\big(y_2(t)-y_1(t)\big)}{W \cdot H},
\qquad A\in[0,1]
$$

with $H = 240\,\text{px}$. A larger $A$ implies the subject is closer. This
replaces the linear distance error $e_{\text{linear}}$ of a depth-based scheme:
because the platform carries a single monocular camera, **apparent size** is
used as the distance cue, and approach is regulated by comparison against a
threshold rather than against a metric set-point.

---

## 3.11.2 Exponential Moving Average (EMA) Error Filtering

Raw detector centroids exhibit frame-to-frame jitter (sub-pixel box wobble,
partial occlusion, motion blur). Feeding this directly into the arbitration
logic would cause chattering between states. The yaw error is therefore smoothed
by a first-order **Exponential Moving Average** (equivalently a first-order IIR
low-pass filter) [9], [10]:

$$
\bar{e}(t) = \alpha\, e(t) + (1-\alpha)\,\bar{e}(t-1)
$$

| Symbol | Meaning | Value |
|--------|---------|-------|
| $\alpha$ | EMA smoothing factor (per-frame weight of the new sample) | $0.25$ |

**Seeding.** On the first frame after target (re)acquisition the filter is
initialised to the measured value, $\bar{e}(t_0) = e(t_0)$, rather than ramped
from zero. This eliminates the large transient (a "startup kick") that a
zero-initialised filter would otherwise inject on the first motion command.

The smoothed error $\bar{e}(t)$ is the quantity consumed by all downstream
steering decisions; the unfiltered $e(t)$ and area $A(t)$ are used only where an
immediate reaction is required (edge guard, close-range stop).

---

## 3.11.3 Decoupled Finite-State Control Architecture

At each cycle the controller occupies one of five states. The active state fully
determines the actuation command, guaranteeing that rotation and translation are
never commanded simultaneously.

| State | Entry condition | Robot action |
|-------|-----------------|--------------|
| **SEARCH** | No detection for $N_{\text{lost}}$ consecutive frames | Motors stopped |
| **WARMUP** | Target just (re)acquired | Hold still for $N_{\text{warm}}$ frames while EMA settles |
| **TURN** | Target off-centre ($\lvert\bar{e}\rvert$ above band) | Pivot in place toward target |
| **ADVANCE** | Target centred ($\lvert\bar{e}\rvert$ within band) | Creep straight forward |
| **STOPCLS** | Range proxy $A \ge A_{\text{close}}$ | Motors stopped (too close) |

| Symbol | Meaning | Value |
|--------|---------|-------|
| $N_{\text{lost}}$ | Consecutive missed frames before declaring the target lost | $3$ |
| $N_{\text{warm}}$ | Settling frames held still after acquisition | $2$ |
| $A_{\text{close}}$ | Area fraction at which the target is "too close" | $0.40$ |

**Transition robustness.** A single dropped detection does not reset the
controller; only after $N_{\text{lost}}$ successive misses is the full state
(EMA, alignment flag, warm-up counter) cleared and SEARCH entered. This prevents
intermittent detection from repeatedly re-triggering the warm-up hold.

---

## 3.11.4 Yaw Control — Pivot Primitive (Hysteresis-Gated Bang-Bang)

Because the drive cannot reverse, heading correction is performed by a
**pivot**: one wheel is driven while the other is held stopped, rotating the
chassis about the stationary wheel. The decision to pivot versus advance is
governed by a **dual-threshold hysteresis band** on $\lvert\bar{e}\rvert$, which
prevents the controller from oscillating between TURN and ADVANCE at the
boundary:

$$
\text{aligned} =
\begin{cases}
\text{true}  & \text{if } \lvert\bar{e}\rvert < \tau_{\text{in}} \\[4pt]
\text{false} & \text{if } \lvert\bar{e}\rvert > \tau_{\text{out}} \\[4pt]
\text{unchanged} & \text{otherwise (within the band)}
\end{cases}
$$

| Symbol | Meaning | Value |
|--------|---------|-------|
| $\tau_{\text{in}}$ | Enter ADVANCE (become "aligned") below this | $0.09$ (≈ ±14 px) |
| $\tau_{\text{out}}$ | Re-enter TURN (lose alignment) above this | $0.16$ (≈ ±26 px) |

When *not* aligned, the pivot direction is set by the sign of the smoothed
error. The single driven wheel runs at a fixed pivot duty $P_{\text{turn}}$:

$$
(u_L, u_R) =
\begin{cases}
(P_{\text{turn}},\; 0) & \text{if } \bar{e} > 0 \quad(\text{target right} \Rightarrow \text{yaw right})\\[4pt]
(0,\; P_{\text{turn}}) & \text{if } \bar{e} < 0 \quad(\text{target left} \Rightarrow \text{yaw left})
\end{cases}
$$

| Symbol | Meaning | Value |
|--------|---------|-------|
| $P_{\text{turn}}$ | Commanded duty of the driven wheel during a pivot (of 255) | $100$ |

This is a **bang-bang** (relay) angular law rather than a proportional one [6], [13]:
centring is achieved not by scaling the pivot speed with error, but by *ceasing*
to pivot once $\lvert\bar{e}\rvert$ falls inside the hysteresis band. The fixed,
modest pivot duty combined with the EMA filter yields smooth convergence without
the gain-tuning sensitivity of a proportional turn at low speed (where motor
dead-zone non-linearity dominates).

**Edge guard.** If the *unfiltered* centroid approaches a frame border —
$\hat{x} < m$ or $\hat{x} > 1-m$ with margin $m = 0.09$ — the target is about to
leave the field of view. The controller then forces a pivot back toward centre
using the instantaneous (not smoothed) error sign, bypassing the hysteresis so
it reacts before EMA lag can lose the target.

---

## 3.11.5 Linear Control — Advance Primitive with Proportional Heading Trim

When aligned, the robot creeps forward along its current heading. Both wheels
run at a common base duty $P_{\text{adv}}$, with a small, hard-limited
**proportional heading trim** $\tau$ superimposed to hold the centre without
turning the straight advance into an arc:

$$
\tau(t) = \operatorname{clip}\!\big(K_p\,\bar{e}(t),\; -\tau_{\max},\; +\tau_{\max}\big)
$$

$$
u_L = P_{\text{adv}} + \tau(t), \qquad u_R = P_{\text{adv}} - \tau(t)
$$

| Symbol | Meaning | Value |
|--------|---------|-------|
| $P_{\text{adv}}$ | Base forward duty of both wheels while advancing (of 255) | $90$ |
| $K_p$ | Proportional gain of the heading trim | $16.0$ |
| $\tau_{\max}$ | Saturation limit of the trim term | $12$ |

The trim is the only **proportional** element of the heading control, and it is
intentionally capped at a small fraction of the base duty so that ADVANCE
remains essentially straight-line motion — large heading errors are handled by
the dedicated TURN state, not by skewing the advance.

---

## 3.11.6 Distance Regulation and Safety Behaviours

Approach distance is regulated by a **threshold (bang-bang) policy** on the area
proxy rather than a continuous distance loop:

$$
\text{STOPCLS active} \iff A(t) \ge A_{\text{close}} = 0.40
$$

While ADVANCE keeps the robot closing at a constant creep, crossing the area
threshold immediately halts both wheels and clears the controller state,
preventing collision. Combined with the SEARCH (target lost → stop), WARMUP
(settle → hold), and edge-guard behaviours, this yields four independent safety
conditions that all resolve to a *zero-velocity* command, biasing the system
toward stopping whenever the perceptual input is uncertain.

---

## 3.11.7 Differential-Drive Actuation Mapping

The state-dependent wheel duties $(u_L, u_R)$ are summarised below. Each is an
8-bit PWM command in $[0, 255]$ transmitted to the executor.

| State | $u_L$ (left) | $u_R$ (right) |
|-------|--------------|---------------|
| SEARCH / STOPCLS / WARMUP | $0$ | $0$ |
| TURN, $\bar{e} > 0$ | $P_{\text{turn}}$ | $0$ |
| TURN, $\bar{e} < 0$ | $0$ | $P_{\text{turn}}$ |
| ADVANCE | $P_{\text{adv}} + \tau$ | $P_{\text{adv}} - \tau$ |

**Chirality correction.** Because the camera image is horizontally mirrored
relative to the robot's forward view, the left/right wheel assignment is swapped
at the output stage so that a target appearing on one side produces a turn
toward its true physical side. The swap is applied to the final duty pair (and
the direction label), ensuring both the pivot and the advance trim are corrected
consistently.

---

## 3.11.8 Low-Level Motor Conditioning (Executor)

The commanded duties are not applied to the motors directly. The embedded
executor conditions them at a fixed actuation tick $T_{\text{tick}}$ to respect
the electromechanical limits of the TT gear-motor / L298N combination:

**1. Minimum-speed (kinetic floor) clamp.** Any non-zero target below the floor
is raised to it, since the motors merely buzz below the floor:

$$
u' = \begin{cases} \max(u,\, U_{\min}) & u > 0 \\ 0 & u = 0 \end{cases}
$$

**2. Static-friction kickstart.** When a wheel transitions from rest
($u_{\text{prev}} = 0$) to a moving command, a brief high-duty pulse breaks
static friction before settling to the (low) cruise duty:

$$
u(t) = U_{\text{kick}} \quad \text{for } t \in [t_0,\, t_0 + t_{\text{kick}}],
\quad\text{then resume } u'
$$

**3. Slew-rate limiting.** To prevent abrupt ("violent") acceleration, the
applied duty may only *rise* by a bounded step per tick; deceleration is
instantaneous (braking is always permitted):

$$
u_{\text{applied}}(t) = \min\!\big(u_{\text{applied}}(t{-}1) + \Delta U_{\max},\; u'(t)\big)
\quad\text{when increasing}
$$

**4. Saturation.** Final clamp to the safe operating range $[0,\, U_{\max}]$.

**5. Communication failsafe.** If no command is received within
$T_{\text{fail}}$, the executor stops the motors, guarding against a host crash
or wireless dropout.

| Symbol | Meaning | Value |
|--------|---------|-------|
| $U_{\min}$ | Kinetic minimum-speed floor | $70$ |
| $U_{\max}$ | Maximum applied duty | $180$ |
| $U_{\text{kick}}$ | Kickstart pulse amplitude | $160$ |
| $t_{\text{kick}}$ | Kickstart pulse duration | $60\,\text{ms}$ |
| $\Delta U_{\max}$ | Maximum duty rise per tick (slew limit) | $18$ |
| $T_{\text{tick}}$ | Executor actuation period | $25\,\text{ms}$ |
| $T_{\text{fail}}$ | Failsafe stop timeout | $250\,\text{ms}$ |

---

## 3.11.9 Discrete Control Timing

The perception–control loop on the host executes at a target rate of
$f_c = 30\,\text{Hz}$, giving a nominal control update step:

$$
\Delta t_c = \frac{1}{f_c} \approx 0.033\,\text{s}
$$

The host emits a fresh command each cycle, while the executor runs its own
faster actuation tick ($T_{\text{tick}} = 25\,\text{ms}$) so that slew limiting
and kickstart remain smooth and deterministic, independent of network jitter in
the command stream.

---

## 3.11.10 Control Parameter Summary

| Group | Parameter | Symbol | Value |
|-------|-----------|--------|-------|
| Vision | Frame width / height | $W,H$ | $320,240\,\text{px}$ |
| Vision | Detection confidence threshold | $c_{\min}$ | $0.40$ |
| Vision | Reference centroid | $x_{\text{ref}}$ | $0.5$ |
| Vision | Calibration offset | $\delta_{\text{cal}}$ | $0.0$ |
| Filter | EMA smoothing factor | $\alpha$ | $0.25$ |
| Arbitration | Lost-frame debounce | $N_{\text{lost}}$ | $3$ |
| Arbitration | Warm-up frames | $N_{\text{warm}}$ | $2$ |
| Arbitration | Close-range area threshold | $A_{\text{close}}$ | $0.40$ |
| Arbitration | Edge-guard margin | $m$ | $0.09$ |
| Yaw | Alignment enter / exit band | $\tau_{\text{in}},\tau_{\text{out}}$ | $0.09 / 0.16$ |
| Yaw | Pivot duty | $P_{\text{turn}}$ | $100$ |
| Linear | Advance base duty | $P_{\text{adv}}$ | $90$ |
| Linear | Heading-trim gain | $K_p$ | $16.0$ |
| Linear | Heading-trim limit | $\tau_{\max}$ | $12$ |
| Actuation | Min / max duty | $U_{\min},U_{\max}$ | $70 / 180$ |
| Actuation | Kickstart amplitude / duration | $U_{\text{kick}},t_{\text{kick}}$ | $160 / 60\,\text{ms}$ |
| Actuation | Slew limit | $\Delta U_{\max}$ | $18$ |
| Timing | Control rate / actuation tick | $f_c,T_{\text{tick}}$ | $30\,\text{Hz} / 25\,\text{ms}$ |
| Timing | Failsafe timeout | $T_{\text{fail}}$ | $250\,\text{ms}$ |

---

## 3.12 Controller Response Characterisation

The controller is evaluated in two complementary regimes. The **open-loop**
tests characterise the *plant* — how the robot moves in response to a fixed
actuation command with the visual feedback path disconnected. The
**closed-loop** tests characterise the *complete servoing system* — how the full
perception → control → actuation loop regulates the tracking error in real time.

> **Note on data.** Values labelled *design-predicted* are derived analytically
> from the controller parameters of §3.11 and hold regardless of any particular
> run. Cells marked “—” are **empirical** and must be filled with the
> measurements taken on the physical robot; they are left blank here so that no
> unverified figure is reported.

### 3.12.1 Open-Loop Response (Plant Characterisation)

**Objective.** Quantify the static and dynamic mapping from commanded PWM duty
to physical motion, and verify the low-level conditioning (§3.11.8).

**Method.** The visual feedback path is disabled and fixed commands are issued
directly to the executor. Motion is measured with a marked floor grid and timed
video (or wheel-encoder counts where available). Each command is repeated five
times and averaged.

**OL-1 — Yaw (pivot) step.** A single pivot command (one wheel at
$P_{\text{turn}}$, the other at $0$) is held for a fixed on-time; the heading
change $\Delta\theta$ is measured.

**OL-2 — Linear (advance) step.** Both wheels are driven at $P_{\text{adv}}$ for
a fixed duration; the forward displacement $\Delta s$ and lateral drift are
measured.

**OL-3 — Actuation primitives.** The kickstart breakaway and slew ramp are
verified by commanding a step from rest and observing the start-up transient.

**Design-predicted behaviour.**

- **Dead-zone / floor.** Commanded duties below $U_{\min}=70$ produce no
  sustained motion; the floor clamp lifts any non-zero command to at least this
  value, so the effective input is $u' \in \{0\}\cup[70,180]$.
- **Static-friction breakaway.** From rest, a cruise duty of $P_{\text{adv}}=90$
  or $P_{\text{turn}}=100$ alone is insufficient to start the motors reliably;
  the $U_{\text{kick}}=160$ pulse for $t_{\text{kick}}=60\,\text{ms}$ guarantees
  break-away before settling to the cruise duty.
- **Bounded acceleration.** With slew limit $\Delta U_{\max}=18$ per
  $T_{\text{tick}}=25\,\text{ms}$ tick, the duty ramps from floor to a typical
  cruise in
  $\big\lceil (90-70)/18 \big\rceil = 2$ ticks $\approx 50\,\text{ms}$, giving a
  smooth, repeatable start rather than a step.
- **Forward-only constraint.** No reverse motion is possible; the pivot is the
  only rotational primitive.

**OL results (to be completed from measurement).**

| Test | Command | Duration | Measured output | Derived rate |
|------|---------|----------|-----------------|--------------|
| OL-1 Yaw | $u_L=100,\,u_R=0$ | 1.0 s | $\Delta\theta = $ — | $\omega_{ol}=$ — (°/s) |
| OL-1 Yaw | $u_L=100,\,u_R=0$ | 0.5 s | $\Delta\theta = $ — | — |
| OL-2 Linear | $u_L=u_R=90$ | 1.0 s | $\Delta s = $ — | $v_{ol}=$ — (mm/s) |
| OL-2 Linear | $u_L=u_R=120$ | 1.0 s | $\Delta s = $ — | — |
| OL-3 Breakaway | rest → 90 (kick on) | — | started? Y/N | — |
| OL-3 Breakaway | rest → 90 (kick off) | — | started? Y/N | — |

*Suggested figures:* (i) commanded duty vs. measured wheel speed (static gain
curve showing the dead-zone and floor); (ii) start-up duty-vs-time trace showing
the kickstart pulse and the slew ramp.

### 3.12.2 Closed-Loop Response (Visual Servoing)

**Objective.** Characterise how the complete system regulates the tracking error
$\bar{e}(t)$ and the range proxy $A(t)$ under real detection input.

**Method.** A human target is positioned at a known initial horizontal offset
and range. The controller is enabled and the smoothed error $\bar{e}(t)$, the
active state, and the commanded duties are logged each control cycle
($f_c=30\,\text{Hz}$). The following scenarios are run:

- **CL-1 — Yaw step (centring).** Target placed at a large offset (near a frame
  edge); measure the convergence of $\bar{e}(t)$ into the aligned band.
- **CL-2 — Approach.** Centred target at range; measure the growth of $A(t)$ to
  $A_{\text{close}}$ and the final stopping distance.
- **CL-3 — Moving-target tracking.** Target walks left↔right at moderate speed;
  measure the tracking-error envelope and lag.
- **CL-4 — Loss and recovery.** Target leaves the field of view; verify the
  SEARCH stop after $N_{\text{lost}}$ frames and clean re-acquisition (warm-up,
  EMA re-seed) on return.

**Design-predicted behaviour.**

- **Steady-state heading error (non-zero by design).** Convergence terminates
  when $\lvert\bar{e}\rvert$ enters the alignment band, so the residual centring
  error is bounded — not driven to zero — by the hysteresis enter threshold:
  $$
  \lvert e_{ss}\rvert \le \tau_{\text{in}} = 0.09
  \;\;\Longleftrightarrow\;\;
  \lvert x_c - x_{\text{ref}}\rvert \le 0.045\,W \approx \pm 14\ \text{px}.
  $$
  This is the deliberate trade-off of a hysteresis-gated bang-bang law: chatter
  immunity in exchange for a small dead-band.
- **No sustained limit cycle.** The hysteresis gap
  $\tau_{\text{out}}-\tau_{\text{in}} = 0.16-0.09 = 0.07$ together with EMA
  smoothing prevents oscillation between TURN and ADVANCE at the boundary; the
  system settles into ADVANCE/HOLD rather than dithering.
- **Filter lag.** The first-order EMA ($\alpha=0.25$ at $\Delta t_c\approx
  0.033\,\text{s}$) has an effective time constant
  $$
  \tau_{\text{ema}} = \frac{-\Delta t_c}{\ln(1-\alpha)}
  = \frac{-0.033}{\ln 0.75} \approx 0.115\ \text{s}
  \quad(\approx 4\text{ frames}),
  $$
  which sets the dominant lag of the heading response and is the main contributor
  to tracking error against a moving target.
- **Acquisition transient suppression.** EMA seeding plus the $N_{\text{warm}}=2$
  frame hold ($\approx 0.07\,\text{s}$) eliminate the first-command overshoot,
  so CL-1 should show no initial wrong-direction lurch.
- **Approach safety.** Forward motion ceases the cycle $A(t)$ first reaches
  $0.40$; the stopping distance therefore scales with the subject's apparent
  size and is bounded above by one control cycle of creep plus the slew-down.

**CL results (to be completed from measurement).**

| Metric | Symbol | Design-predicted | Measured |
|--------|--------|------------------|----------|
| Steady-state heading error | $e_{ss}$ | $\le \pm 0.09$ ($\approx\pm14$ px) | — |
| Settling time into band (CL-1) | $t_s$ | — | — |
| Overshoot (CL-1) | $M_p$ | $\approx 0$ (warm-up + seeding) | — |
| Limit-cycle amplitude | — | none (hysteresis) | — |
| EMA lag | $\tau_{\text{ema}}$ | $\approx 0.115$ s | — |
| Moving-target tracking error (RMS) | $e_{rms}$ | — | — |
| Stopping distance (CL-2) | $d_{stop}$ | — | — |
| Re-acquisition time (CL-4) | $t_{re}$ | — | — |

*Suggested figures:* (i) $\bar{e}(t)$ vs. time for CL-1 with the
$\pm\tau_{\text{in}}/\pm\tau_{\text{out}}$ bands overlaid, annotated with the
active state; (ii) $A(t)$ vs. time for CL-2 with the $A_{\text{close}}$ line;
(iii) target position vs. robot heading for CL-3 showing the tracking lag.

### 3.12.3 Open-Loop vs. Closed-Loop Comparison

| Aspect | Open-loop | Closed-loop |
|--------|-----------|-------------|
| Visual feedback | Disconnected | Active |
| Input | Fixed duty command | Live tracking error $\bar{e}(t)$, area $A(t)$ |
| What it characterises | Plant gain, dead-zone, kickstart, slew | Regulation: settling, steady-state error, robustness |
| Heading error | Accumulates without correction (drifts) | Bounded to the hysteresis band |
| Disturbance rejection | None | Continuous re-centring + edge guard + failsafe |
| Primary use | Calibrating $U_{\min}$, $U_{\text{kick}}$, duty→speed map | Validating tracking performance and safety |

The contrast confirms the role of feedback: the open-loop plant exhibits an
unregulated dead-zone and heading drift, whereas closing the visual loop bounds
the steady-state error to the designed $\pm\tau_{\text{in}}$ band and adds
disturbance rejection (re-centring), loss handling, and collision safety.

---

## References

[1] S. Hutchinson, G. D. Hager, and P. I. Corke, "A tutorial on visual servo
control," *IEEE Transactions on Robotics and Automation*, vol. 12, no. 5,
pp. 651–670, Oct. 1996.

[2] F. Chaumette and S. Hutchinson, "Visual servo control. Part I: Basic
approaches," *IEEE Robotics & Automation Magazine*, vol. 13, no. 4, pp. 82–90,
Dec. 2006.

[3] F. Chaumette and S. Hutchinson, "Visual servo control. Part II: Advanced
approaches," *IEEE Robotics & Automation Magazine*, vol. 14, no. 1, pp. 109–118,
Mar. 2007.

[4] P. Corke, *Robotics, Vision and Control: Fundamental Algorithms in MATLAB*,
2nd ed. Cham, Switzerland: Springer, 2017.

[5] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to
Autonomous Mobile Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011.

[6] K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for
Scientists and Engineers*. Princeton, NJ, USA: Princeton University Press, 2008.

[7] R. A. Brooks, "A robust layered control system for a mobile robot," *IEEE
Journal on Robotics and Automation*, vol. 2, no. 1, pp. 14–23, Mar. 1986.

[8] R. C. Arkin, *Behavior-Based Robotics*. Cambridge, MA, USA: MIT Press, 1998.

[9] J. S. Hunter, "The exponentially weighted moving average," *Journal of
Quality Technology*, vol. 18, no. 4, pp. 203–210, 1986.

[10] A. V. Oppenheim and R. W. Schafer, *Discrete-Time Signal Processing*,
3rd ed. Upper Saddle River, NJ, USA: Prentice Hall, 2010.

[11] J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, "You Only Look Once:
Unified, real-time object detection," in *Proc. IEEE Conf. Computer Vision and
Pattern Recognition (CVPR)*, Las Vegas, NV, USA, 2016, pp. 779–788.

[12] G. Jocher, A. Chaurasia, and J. Qiu, "Ultralytics YOLO," 2023. [Online].
Available: https://github.com/ultralytics/ultralytics

[13] Ya. Z. Tsypkin, *Relay Control Systems*. Cambridge, U.K.: Cambridge
University Press, 1984.
