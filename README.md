# Cart-pole MATLAB teaching laboratories (base MATLAB)

Three standalone `.m` files. Place all files in a MATLAB folder, set that folder as Current Folder, and run the desired function from the Command Window. No Simulink, Control System Toolbox, App Designer, or additional user files are required.

## 1. `CartPole2DOF_DynamicsLab.m`

Run `CartPole2DOF_DynamicsLab`.

- Keeps the two-column PD and LQR swing-up comparison and the angle, force, and energy plots, including phase-specific colors and switching indicators.
- Adds **COM/L** and **IG/(mL^2)** so the rod center-of-mass position and transverse mass moment of inertia can vary independently. The original uniform-rod defaults are 0.5 and 1/12.
- **Linear overlay:** turn **SWING-UP ON** off and set the initial angle near upright (for example 5 deg) to compare the nonlinear angular response with the upright-linearized model; the latter is gray dashed. The linear overlay is deliberately skipped during swing-up.
- **LENGTH SWEEP:** simulates nominal 0.7L, 1.0L, 1.3L lengths with *fixed nominal controller gains* and plots angle and force for PD and LQR. A small near-upright initial angle is used when the current swing-up initial angle is too large.
- **OBSERVER:** after a near-upright simulation with swing-up disabled, plots noisy cart and pendulum position measurements versus true values, plus estimated and true velocities. Uses a continuous-time observer gain derived from a dual Riccati equation. The observer is an illustrative offline estimate and does not replace the state used for feedback in this demo.
- **EXPORT DATA:** saves all simulated time histories and parameters to a `.mat` file.
- The GUI may be taller than a small laptop screen to preserve the existing chart sizes.

## 2. `CartPole3DOF_GUI.m`

Run `CartPole3DOF_GUI`.

- Mechanical DOF: x, alpha (X tilt), beta (Y tilt). Full state: `[x,alpha,beta,xdot,alphadot,betadot]`.
- **One actuator:** horizontal Fx. In the upright linearization the transverse beta mode is unstable and not controllable. The pole placement and LQR gains are intentionally designed only for the controllable planar subsystem. The full 3-DOF plant remains nonlinear during the simulation.
- Defaults: X tilt 5 deg and Y tilt 1 deg. The Y tilt grows and the integration stops safely once a tilt approaches 73.3 deg, rather than entering the coordinate singularity near beta = 90 deg. Set Y tilt to zero to see the controllable planar balancing special case.
- Two synchronized animated 3D MATLAB axes, `view(42,24)`, `camproj(...,'perspective')`, equal physical axis scaling, grid/ground plane, X-rail, cart body/wheels, 3D rod motion, burgundy tip trail, and yellow input force arrow.
- Matching angle plots for X and Y tilt; one force plot for Fx.
- **AUTO-DESIGN PD** prints the gains. Real or conjugate complex pole entries are supported.
- **EXPORT** saves complete simulation records as a `.mat` file.

## 3. `CartPole4DOF_GUI.m`

Run `CartPole4DOF_GUI`.

- Mechanical DOF: x, y, alpha, beta. Full state: `[x,y,alpha,beta,xdot,ydot,alphadot,betadot]`.
- **Two actuators:** Fx and Fy. The eight-state upright linearization is controllable.
- The pole-placement column applies the same four desired pole locations to each locally decoupled X/alpha and Y/beta channel. The second column solves the full **8-state, 2-input** continuous-time LQR problem using the Hamiltonian stable invariant subspace. Both columns integrate the full nonlinear model with coupling.
- The 3D cart moves across a true XY ground stage rather than a 1D rail. Yellow and purple arrows indicate Fx and Fy. Two angle curves and two input-force curves appear below the synchronized perspective animations.
- Independent X/Y force limits and X/Y disturbance pulses, and an adjustable LQR R weight for each actuator.
- For physically useful default initial angles, use X tilt 5 deg and Y tilt 3 deg.

## Shared conventions

- One rigid rod, **no double pendulum**.
- Uniform-rod reference parameters: `M=1 kg`, `m=0.2 kg`, `L=0.5 m`, `g=9.81 m/s^2`, `lc/L=0.5`, `IG/(m L^2)=1/12`.
- Preferred trajectory trail duration 0.5 s, preferred PD third and fourth poles `-0.5+4i` and `-0.5-4i`.
- Colors: blue for PD, teal for LQR, burgundy for tip trails and swing-up, purple for transverse tilt / Fy, amber for input arrows.
- Tilt angles are absolute directions relative to upright. The unit rod direction is `n=[sin(alpha)*cos(beta),sin(beta),cos(alpha)*cos(beta)]`.
- For spatial systems, rotational kinetic energy uses `J*(cos(beta)^2*alphadot^2+betadot^2)/2`, based on an axisymmetric rigid rod without an independent axial-spin DOF.
- Nonlinear dynamics use `M(q)*qdd + h(q,qd) = [forces;zeros(2,1)]` and are integrated with `ode45`.
- The spatial experiments are **local balancing**, not swing-up. Angle safety events stop integration before the spatial angle-coordinate singularity.
- GUI run-time behavior, layout, and MATLAB-version compatibility could not be tested in MATLAB in this environment. Equations, upright pole-placement designs, and LQR behavior were checked independently in Python/SciPy, including the 3-DOF uncontrollable mode and the 4-DOF stabilizing responses.
