# 6D Homogeneous Formation Controller — Codex Engineering Spec

## 0. Goal

Implement a **new theory-consistent 6D controller core** without replacing the existing `6d_artstein_disc` implementation.

Final theory conventions are frozen as follows:

- formation position offset is fixed in **map frame**;
- follower yaw tracks leader yaw;
- robot translational velocity is measured/represented in **body frame**;
- map velocity is `R(theta) * v_body`;
- homogeneous controller output is an equivalent **map-frame formation force + yaw moment**;
- theoretical model keeps `m` and `I`;
- nominal 6D error model is three double-integrator channels.

Do not reintroduce the old Leader-frame position-error matrix `A_L(omega_l, v_l)` into this new core.

---

## 1. State and frames

Robot state:

```text
x_i = [px_map, py_map, theta, vx_body, vy_body, omega]
```

Rotation:

$$
R(\theta)=
\begin{bmatrix}
\cos\theta&-\sin\theta\\
\sin\theta&\cos\theta
\end{bmatrix}
$$

$$
J=
\begin{bmatrix}
0&-1\\
1&0
\end{bmatrix}
$$

Map velocity:

$$
v_i^m=R(\theta_i)v_i^b.
$$

Equivalent body-input model:

$$
m\dot v_i^b=F_i^b,
\qquad
I\dot\omega_i=M_i.
$$

Hence:

$$
\dot v_i^m
=
R_i
\left(
F_i^b/m+\omega_iJv_i^b
\right).
$$

---

## 2. Formation definition

Use a fixed map-frame offset:

$$
d_p^m=[d_x,d_y]^T=\text{const}.
$$

Do NOT rotate `d_p` by leader yaw.

Position error:

$$
e_p=p_f^m-p_l^m-d_p^m.
$$

Yaw error:

$$
e_\theta=\operatorname{wrap}(\theta_f-\theta_l-d_\theta).
$$

Default:

```text
d_theta = 0
```

Velocity error:

$$
e_v=R_fv_f^b-R_lv_l^b.
$$

Yaw-rate error:

$$
e_\omega=\omega_f-\omega_l.
$$

6D error state order MUST be:

```text
e = [ex, ey, e_theta, evx, evy, e_omega]^T
```

---

## 3. Exact nominal error dynamics

Because `d_p_map` is constant:

$$
\dot e_p=e_v.
$$

Define equivalent map-frame relative formation force:

$$
F^m=
R_fF_f^b-R_lF_l^b
+mR_f\omega_fJv_f^b
-mR_l\omega_lJv_l^b.
$$

Then:

$$
\dot e_v=F^m/m.
$$

Define relative yaw moment:

$$
M_z=M_f-M_l.
$$

Then:

$$
\dot e_\omega=M_z/I.
$$

Input:

```text
u = [Fx_map_equiv, Fy_map_equiv, Mz_equiv]^T
```

---

## 4. State-space model

$$
\dot e=Ae+Bu
$$

with

$$
A=
\begin{bmatrix}
0&I_3\\
0&0
\end{bmatrix}
$$

and

$$
B=
\begin{bmatrix}
0\\D
\end{bmatrix},
\qquad
D=\operatorname{diag}(1/m,1/m,1/I).
$$

Expanded:

```text
ex_dot       = evx
ey_dot       = evy
e_theta_dot  = e_omega

evx_dot      = Fx / m
evy_dot      = Fy / m
e_omega_dot  = Mz / I
```

---

## 5. Homogeneous upgrade

Controllability:

$$
rank[B,AB]=6.
$$

Use:

$$
G_0=
\begin{bmatrix}
-I_3&0\\
0&0
\end{bmatrix},
\qquad
Y_0=0,
\qquad
K_0=0.
$$

Verify numerically:

```text
G0 * B == 0
A * G0 - G0 * A == A
```

Select:

```text
-1 < mu < 0
```

and

$$
G_d=I_6+\mu G_0
=
\operatorname{diag}(1-\mu,1-\mu,1-\mu,1,1,1).
$$

Verify:

```text
A*Gd - Gd*A == mu*A
Gd*B == B
```

---

## 6. Linear gain

A simple valid structure:

$$
K_{\rm lin}
=
[-D^{-1}k_pI_3,\ -D^{-1}k_vI_3]
$$

with:

```text
kp > 0
kv > 0
```

Check:

```text
max(real(eig(A + B*Klin))) < 0
```

---

## 7. P/Lyapunov conditions

Find `P = P^T > 0` and `rho > 0` satisfying:

$$
PG_d+G_d^TP>0
$$

$$
P(A+BK_{\rm lin})+(A+BK_{\rm lin})^TP<0
$$

and preferably:

$$
P(A+BK_{\rm lin})
+(A+BK_{\rm lin})^TP
+\rho(PG_d+G_d^TP)
\le0.
$$

Do not hard-code a proof claim unless these are checked for the actual parameter set.

---

## 8. Canonical homogeneous norm

Let:

$$
d(s)=\exp(sG_d).
$$

For `e != 0`, `r = ||e||_d` is the positive solution of:

$$
[d(-\ln r)e]^TP[d(-\ln r)e]=1.
$$

Required invariant:

```text
norm_d(d(s)*e) == exp(s) * norm_d(e)
```

within numerical tolerance.

Implement robust scalar root solving for `r`.

---

## 9. Theoretical homogeneous controller

Since `K0 = 0`:

$$
u_h(e)
=
\|e\|_d^{1+\mu}
K_{\rm lin}
d(-\ln\|e\|_d)e.
$$

Output:

```text
u_h = [Fx, Fy, Mz]
```

This is the **theoretical controller path**.

Do not silently replace it with the old clipped/shifted engineering HPC formula.

If a regularized controller is added, implement it as a separate function/path and label it explicitly as non-theorem-equivalent.

---

## 10. Vector-field homogeneity test

Numerically verify random `e`, `s`:

$$
u_h(d(s)e)
=
e^{(1+\mu)s}u_h(e)
$$

and:

$$
f(d(s)e)
=
e^{\mu s}d(s)f(e)
$$

where:

$$
f(e)=Ae+Bu_h(e).
$$

---

## 11. Follower total input and leader feedforward

Define each robot's map-frame equivalent translational input:

$$
U_i^m=
R_iF_i^b+mR_i\omega_iJv_i^b.
$$

Then:

$$
m\dot e_v=U_f^m-U_l^m.
$$

Preferred virtual-leader implementation:

$$
U_{f,cmd}^m=U_l^m+[F_x,F_y]^T.
$$

Yaw:

$$
M_{f,cmd}=M_l+M_z.
$$

If leader feedforward is unavailable, the omitted leader term is a matched disturbance; do not claim exact nominal finite-time tracking in that case.

---

## 12. Convert map equivalent input to body acceleration

From:

$$
U_f^m
=
R_fF_f^b
+mR_f\omega_fJv_f^b
$$

derive:

$$
F_f^b=
R_f^TU_f^m-m\omega_fJv_f^b.
$$

Hence:

$$
\dot v_f^b
=
\frac1mR_f^TU_f^m
-\omega_fJv_f^b.
$$

Yaw:

$$
\dot\omega_f=M_{f,cmd}/I.
$$

---

## 13. Generate `cmd_vel`

Initial implementation may use one-step Euler integration:

```text
v_cmd_body =
    v_body_measured
    + dt * v_body_dot_cmd

omega_cmd =
    omega_measured
    + dt * omega_dot_cmd
```

Publish:

```cpp
cmd.linear.x  = vx_cmd_body;
cmd.linear.y  = vy_cmd_body;
cmd.angular.z = omega_cmd;
```

Apply limits only OUTSIDE the theoretical controller core.

Keep:

```text
theory controller
    -> equivalent force/moment
    -> inverse input transform
    -> velocity reference
    -> saturation / actuator layer
    -> cmd_vel
```

as distinct layers.

---

## 14. Required tests

### Test A — frame conversion

Verify:

```text
R(-theta_l) * R(theta_f)
```

and direct map/body transforms are consistent.

### Test B — map-fixed offset

Rotate leader yaw while holding leader position fixed.

Expected:

```text
desired follower position DOES NOT rotate around leader
```

Yaw target may rotate.

### Test C — 4D reduction

Set:

```text
e_theta = 0
e_omega = 0
Mz = 0
```

Verify translational subsystem exactly matches the existing 4D double-integrator structure.

### Test D — matrix identities

Verify:

```text
rank([B, A*B]) == 6
G0*B == 0
A*G0 - G0*A == A
A*Gd - Gd*A == mu*A
Gd*B == B
```

### Test E — stability matrices

Verify eigenvalues / matrix definiteness for the actual parameters.

### Test F — homogeneous scaling

Verify controller and closed-loop vector-field scaling laws numerically.

### Test G — zero-error equilibrium

Construct:

```text
ep = 0
e_theta = 0
ev = 0
e_omega = 0
```

with leader yaw/velocity values in the allowed scenario.

With correct leader feedforward, the error derivative should remain zero within numerical tolerance.

---

## 15. Engineering boundaries

Do NOT claim the nominal finite-time theorem directly for:

- clipped controller;
- acceleration / velocity saturation;
- wheel-speed saturation;
- Euler integration to `cmd_vel`;
- actuator first-order lag;
- communication delay;
- Artstein layer;
- HOCBF-active control.

These are later engineering extensions.

---

## 16. Recommended implementation order

```text
Phase 0
  new 6D error calculation
  + map-fixed dp
  + exact A/B/G0/Gd tests

Phase 1
  theoretical homogeneous controller
  + numerical plant test

Phase 2
  inverse input transform
  + cmd_vel generation
  + Gazebo

Phase 3
  regularization / limits

Phase 4
  Artstein predictor

Phase 5
  HOCBF / switching
```

Do not combine all phases in the first patch.

---

## 17. Non-negotiable conventions

Do not change these without explicitly revisiting the theory:

```text
dp is map-frame constant
position error is map-frame
velocity error is map-frame
body velocity is converted by R(theta)
yaw follows leader separately
Fx/Fy in homogeneous error system are map-frame equivalent correction forces
m and I remain in B
old Leader-frame A_L is not used in this controller
theoretical u_h and regularized implementation are separate
```
