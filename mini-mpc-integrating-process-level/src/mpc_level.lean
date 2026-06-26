/-
  mpc_level.lean — Formal Verification of MPC for Integrating Level Control
  Lean 4 formalization of MPC theorems for integrating processes.
-/

/-! ## Core Definitions (L1) -/

structure IntegratingProcess where
  gain : Float
  timeConstant : Float
  samplingTime : Float
  deriving Repr

structure MPCTuning where
  predictionHorizon : Nat
  controlHorizon : Nat
  moveSuppression : Float
  outputWeight : Float
  deriving Repr

structure SurgeTank where
  crossArea : Float
  maxLevel : Float
  level : Float
  valvePosition : Float
  deriving Repr

/-! ## Integrating Process Dynamics (L2) -/

def pureIntegratorStep (x : Float) (K : Float) (Ts : Float) (u : Float) : Float :=
  x + K * Ts * u

theorem pure_integrator_unforced_drift (x0 K Ts : Float) :
    pureIntegratorStep (pureIntegratorStep (pureIntegratorStep x0 K Ts 0.0) K Ts 0.0) K Ts 0.0 = x0 := by
  simp [pureIntegratorStep]

theorem pure_integrator_response_zero_input (x K Ts : Float) :
    pureIntegratorStep x K Ts 0.0 = x := by
  simp [pureIntegratorStep]

def integratorWithLag (x1 x2 K tau Ts u : Float) : Float × Float :=
  let alpha := Float.exp (-Ts / tau)
  let x1' := x1 + tau * (1.0 - alpha) * x2 + K * (Ts - tau * (1.0 - alpha)) * u
  let x2' := alpha * x2 + K * (1.0 - alpha) * u
  (x1', x2')

theorem lag_dynamics_stable (K tau Ts : Float) (h : tau > 0.0) (hTs : Ts > 0.0) :
    Float.exp (-Ts / tau) < 1.0 := by
  have hpos : Ts / tau > 0.0 := div_pos hTs h
  have hexp : Float.exp (-(Ts / tau)) < Float.exp 0.0 := by
    apply Float.exp_lt_exp.mpr
    linarith
  simpa using hexp

/-! ## Mass Balance (Tank Dynamics) (L4) -/

def tankMassBalance (h : Float) (Fin Fout A Ts : Float) : Float :=
  h + Ts * (Fin - Fout) / A

theorem mass_balance_at_steady_state (h Fin A Ts : Float) (hA : A > 0.0) :
    tankMassBalance h Fin Fin A Ts = h := by
  simp [tankMassBalance]
  ring

theorem mass_balance_strictly_increasing (h Fin Fout A Ts : Float)
    (hA : A > 0.0) (hTs : Ts > 0.0) (hFlow : Fin > Fout) :
    tankMassBalance h Fin Fout A Ts > h := by
  dsimp [tankMassBalance]
  have hpos : Ts * (Fin - Fout) / A > 0.0 := by
    apply div_pos
    · nlinarith
    · exact hA
  nlinarith

/-! ## Quadratic Program Optimality (L5) -/

structure QPSolution where
  variables : List Float
  objective : Float
  status : Nat  -- 0=optimal, 1=infeasible
  deriving Repr

def computeQPObjective (H : Float → Float → Float) (c : Float → Float)
    (x : List Float) (n : Nat) : Float :=
  let n' := x.length
  let quad : Float := Id.run do
    let mut sum := 0.0
    for i in [:n'] do
      for j in [:n'] do
        sum := sum + 0.5 * x.get! i * H (Float.ofNat i) (Float.ofNat j) * x.get! j
    pure sum
  let lin : Float := Id.run do
    let mut sum := 0.0
    for i in [:n'] do
      sum := sum + c (Float.ofNat i) * x.get! i
    pure sum
  quad + lin

theorem optimal_solution_has_nonneg_objective
    (H : Float → Float → Float) (c : Float → Float) (x : List Float)
    (hSPD : ∀ i, H i i ≥ 0.0) (hZero : ∀ i j, j ≠ i → H i j = 0.0) :
    computeQPObjective H c x 2 ≥ 0.0 := by
  unfold computeQPObjective
  -- For diagonal SPD H with positive diagonal, the quadratic term is ≥ 0
  have hquad : ∀ (vals : List Float), 0.5 * (vals.get? 0).getD 0.0 * H 0.0 0.0 * (vals.get? 0).getD 0.0 +
    0.5 * (vals.get? 1).getD 0.0 * H 1.0 1.0 * (vals.get? 1).getD 0.0 ≥ 0.0 := by
    intro vs
    have h1 : 0.5 * (vs.get? 0).getD 0.0 * H 0.0 0.0 * (vs.get? 0).getD 0.0 ≥ 0.0 := by
      have hsq : (vs.get? 0).getD 0.0 * (vs.get? 0).getD 0.0 ≥ 0.0 := by
        nlinarith
      nlinarith [hSPD 0.0]
    have h2 : 0.5 * (vs.get? 1).getD 0.0 * H 1.0 1.0 * (vs.get? 1).getD 0.0 ≥ 0.0 := by
      nlinarith [hSPD 1.0]
    nlinarith
  have hlin : ∀ (vals : List Float),
    c 0.0 * (vals.get? 0).getD 0.0 + c 1.0 * (vals.get? 1).getD 0.0 ≥ - (c 0.0).abs := by
    intro vs; nlinarith
  nlinarith

/-! ## Level Constraint Feasibility (L4) -/

def levelWithinBounds (level lo hi : Float) : Prop :=
  lo ≤ level ∧ level ≤ hi

theorem tight_bounds_imply_lo_le_hi (level lo hi : Float)
    (h : levelWithinBounds level lo hi) : lo ≤ hi := by
  rcases h with ⟨hlo, hhi⟩
  exact le_trans hlo hhi

theorem level_bounds_transitive (level lo1 hi1 lo2 hi2 : Float)
    (h1 : levelWithinBounds level lo1 hi1)
    (h2 : lo2 ≤ lo1) (h3 : hi1 ≤ hi2) :
    levelWithinBounds level lo2 hi2 := by
  rcases h1 with ⟨hlo1, hhi1⟩
  constructor
  · exact le_trans h2 hlo1
  · exact le_trans hhi1 h3

/-! ## Surge Tank Filter Factor (L6) -/

def filterFactor (omega : Float) (tauRes : Float) : Float :=
  1.0 / (1.0 + omega * tauRes)

theorem filter_factor_bounded (omega tauRes : Float)
    (hω : omega ≥ 0.0) (hτ : tauRes ≥ 0.0) :
    0.0 ≤ filterFactor omega tauRes ∧ filterFactor omega tauRes ≤ 1.0 := by
  constructor
  · have hdenom : 1.0 + omega * tauRes ≥ 1.0 := by nlinarith
    have hpos : 0.0 ≤ 1.0 / (1.0 + omega * tauRes) :=
      div_nonneg (by norm_num) hdenom
    exact hpos
  · have hdenom : 1.0 + omega * tauRes ≥ 1.0 := by nlinarith
    have hle : filterFactor omega tauRes ≤ 1.0 := by
      dsimp [filterFactor]
      have h_div : 1.0 / (1.0 + omega * tauRes) ≤ 1.0 / 1.0 :=
        one_div_le_one_div (by nlinarith) (by nlinarith) (by
          have hpos : 1.0 + omega * tauRes ≥ 1.0 := by nlinarith
          exact hpos)
      simpa using h_div
    exact hle

/-! ## Receding Horizon Property (L2) -/

def applyFirstMove (moves : List Float) : Float :=
  match moves with
  | [] => 0.0
  | (m :: _) => m

theorem first_move_changes_plan (moves : List Float) (h : moves ≠ []) :
    applyFirstMove moves = moves.head (by
      intro hnil; exact h hnil) := by
  cases moves
  · contradiction
  · simp [applyFirstMove]
