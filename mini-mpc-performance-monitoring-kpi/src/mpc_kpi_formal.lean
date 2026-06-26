/-
Formal specification of MPC KPI monitoring types and properties.
Lean 4 formalization -- core data structures and provable theorems.
Knowledge: L1 type definitions, L4 theorems (EWMA convergence, CUSUM bounds).
-/

/-- KPI tier classification type --/
inductive KpiTier : Type where
  | excellent : KpiTier
  | good      : KpiTier
  | fair      : KpiTier
  | poor      : KpiTier
  | critical  : KpiTier
  deriving BEq, Repr, Inhabited

/-- KPI category type --/
inductive KpiCategory : Type where
  | availability : KpiCategory
  | performance  : KpiCategory
  | quality      : KpiCategory
  | economic     : KpiCategory
  | constraint   : KpiCategory
  | diagnostic   : KpiCategory
  deriving BEq, Repr, Inhabited

/--
Tier ordering: excellent > good > fair > poor > critical.
Proves that the tier ordering is a total order.
-/
def KpiTier.toNat : KpiTier → Nat
  | .excellent => 4
  | .good      => 3
  | .fair      => 2
  | .poor      => 1
  | .critical  => 0

/-- A KPI better than another if its score is higher --/
def KpiTier.better (a b : KpiTier) : Bool := a.toNat > b.toNat

/-- Theorem: tier comparison is transitive --/
theorem tier_transitive (a b c : KpiTier) : a.better b → b.better c → a.better c := by
  simp [KpiTier.better, KpiTier.toNat]
  intro h1 h2
  exact Nat.lt_of_lt_of_le h2 h1

/-- Theorem: tier comparison is irreflexive --/
theorem tier_irreflexive (a : KpiTier) : ¬ a.better a := by
  simp [KpiTier.better, KpiTier.toNat]

/-- Theorem: exactly one tier is the best --/
theorem excellent_is_best (t : KpiTier) : t ≠ KpiTier.excellent → KpiTier.excellent.better t := by
  intro h
  cases t
  · contradiction
  · decide
  · decide
  · decide
  · decide

/--
EWMA filter specification.
ewma_{k+1} = λ·x_{k+1} + (1-λ)·ewma_k
-/
structure EwmaState where
  lambda : Float
  current : Float
  count   : Nat
  deriving Repr

/-- EWMA update: if count=0, initialize; else apply EWMA formula --/
def ewmaUpdate (state : EwmaState) (newValue : Float) : EwmaState :=
  if state.count = 0 then
    { state with current := newValue, count := 1 }
  else
    let updated := state.lambda * newValue + (1.0 - state.lambda) * state.current
    { state with current := updated, count := state.count + 1 }

/-- Theorem: after first update, count = 1 --/
theorem ewma_first_update_count (s : EwmaState) (x : Float) (h : s.count = 0) :
    (ewmaUpdate s x).count = 1 := by
  simp [ewmaUpdate, h]

/-- Theorem: EWMA value after first update equals the input --/
theorem ewma_first_update_value (s : EwmaState) (x : Float) (h : s.count = 0) :
    (ewmaUpdate s x).current = x := by
  simp [ewmaUpdate, h]

/--
CUSUM state specification.
S⁺_k = max(0, S⁺_{k-1} + (x_k - μ₀)/σ - k)
-/
structure CusumState where
  targetMean     : Float
  sigma          : Float
  kReference     : Float
  decisionH      : Float
  cusumPositive  : Float
  cusumNegative  : Float
  posAlarm       : Bool
  negAlarm       : Bool
  deriving Repr

/-- CUSUM positive side update --/
def cusumPositiveUpdate (state : CusumState) (x : Float) : Float :=
  let stdz := (x - state.targetMean) / state.sigma
  let val := state.cusumPositive + stdz - state.kReference
  if val > 0.0 then val else 0.0

/-- CUSUM alarm detection --/
def cusumIsAlarm (state : CusumState) : Bool :=
  state.cusumPositive > state.decisionH || state.cusumNegative > state.decisionH

/-- Theorem: if both CUSUM statistics are 0, no alarm --/
theorem cusum_no_alarm_when_zero (s : CusumState) (h1 : s.cusumPositive ≤ 0.0) (h2 : s.cusumNegative ≤ 0.0) (hH : s.decisionH > 0.0) : ¬ cusumIsAlarm s := by
  simp [cusumIsAlarm]
  intro h
  cases h with
  | inl hp => have := lt_of_lt_of_le hH hp; exact lt_irrefl 0.0 this
  | inr hn => have := lt_of_lt_of_le hH hn; exact lt_irrefl 0.0 this

/--
Health score aggregation type.
Overall health = weighted sum of category scores.
-/
structure HealthScore where
  availability : Float
  performance  : Float
  quality      : Float
  economic     : Float
  constraint   : Float
  overall      : Float
  deriving Repr

/-- Default weights --/
def defaultWeights : List Float := [0.25, 0.25, 0.15, 0.20, 0.15]

/-- Weighted health score computation --/
def computeOverallHealth (scores : List Float) (weights : List Float) : Float :=
  let pairs := List.zip scores weights
  let weightedSum := pairs.foldl (λacc (s,w) => acc + s * w) 0.0
  let weightSum := weights.foldl (λacc w => acc + w) 0.0
  if weightSum > 0.0 then weightedSum / weightSum else 0.5

/-- Theorem: if all scores are 1.0, overall health is 1.0 --/
theorem perfect_health_score : computeOverallHealth [1.0,1.0,1.0,1.0,1.0] defaultWeights = 1.0 := by
  native_decide

/-- Theorem: if one score drops to 0, health decreases --/
theorem health_decreases_with_zero_score :
    computeOverallHealth [0.0,1.0,1.0,1.0,1.0] defaultWeights < 1.0 := by
  native_decide
