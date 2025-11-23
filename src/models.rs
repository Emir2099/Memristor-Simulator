use std::fmt;

// Defines the physical behavior of a memristor.
pub trait MemristorPhysics: Send + Sync {
    /// Returns the resistance (Ohms) for a given state `s`.
    fn resistance(&self, s: f64) -> f64;

    /// Returns the time derivative ds/dt given current state, voltage, and current.
    /// Note: Some models depend on Voltage (VTEAM), others on Current (HP).
    /// We pass both so the model can choose.
    fn derivative(&self, s: f64, v: f64, i: f64) -> f64;

    /// Defines the valid state range (usually 0.0 to 1.0).
    fn clip_state(&self, s: f64) -> f64 {
        s.clamp(0.0, 1.0)
    }

    /// Clone into boxed trait object
    fn box_clone(&self) -> Box<dyn MemristorPhysics>;
}

impl Clone for Box<dyn MemristorPhysics> {
    fn clone(&self) -> Box<dyn MemristorPhysics> { self.box_clone() }
}

// --- VTEAM implementation ---
#[derive(Clone)]
pub struct VTEAM {
    pub r_on: f64,
    pub r_off: f64,
    pub v_off: f64, // positive threshold
    pub v_on: f64,  // negative threshold (usually < 0)
    pub k_off: f64,
    pub k_on: f64,
    pub alpha_off: f64,
    pub alpha_on: f64,
}

impl VTEAM {
    pub fn new(r_on: f64, r_off: f64, v_on: f64, v_off: f64) -> Self {
        VTEAM { r_on, r_off, v_off, v_on, k_off: 1e-3, k_on: 1e-3, alpha_off: 1.0, alpha_on: 1.0 }
    }
}

impl MemristorPhysics for VTEAM {
    fn resistance(&self, s: f64) -> f64 {
        let s = s.clamp(0.0, 1.0);
        // linear mapping between r_on (s=1) and r_off (s=0)
        self.r_on * s + self.r_off * (1.0 - s)
    }

    fn derivative(&self, _s: f64, v: f64, _i: f64) -> f64 {
        // VTEAM uses voltage thresholds. v_on typically negative, v_off positive.
        if v > self.v_off {
            let ratio = v / self.v_off - 1.0;
            if ratio <= 0.0 { 0.0 } else { self.k_off * ratio.powf(self.alpha_off) }
        } else if v < self.v_on {
            // v_on is negative; (v / v_on - 1.0) likewise handled
            let ratio = v / self.v_on - 1.0;
            if ratio <= 0.0 { 0.0 } else { self.k_on * ratio.powf(self.alpha_on) }
        } else {
            0.0
        }
    }

    fn box_clone(&self) -> Box<dyn MemristorPhysics> { Box::new(self.clone()) }
}

impl fmt::Debug for VTEAM {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "VTEAM(r_on={}, r_off={}, v_on={}, v_off={})", self.r_on, self.r_off, self.v_on, self.v_off)
    }
}

// --- Yakopcic implementation (simplified) ---
#[derive(Clone)]
pub struct Yakopcic {
    pub r_on: f64,
    pub r_off: f64,
    pub a1: f64,
    pub a2: f64,
    pub b: f64,
}

impl Yakopcic {
    pub fn new(r_on: f64, r_off: f64) -> Self {
        Yakopcic { r_on, r_off, a1: 1e-3, a2: 1e-3, b: 1.0 }
    }
}

impl MemristorPhysics for Yakopcic {
    fn resistance(&self, s: f64) -> f64 {
        let s = s.clamp(0.0, 1.0);
        // simplified mapping for stability
        self.r_on * (-s).exp() + self.r_off * (1.0 - s)
    }

    fn derivative(&self, s: f64, v: f64, _i: f64) -> f64 {
        // sinh-based dynamics
        let base_change = if v >= 0.0 { self.a1 * (self.b * v).sinh() } else { self.a2 * (self.b * v).sinh() };
        let window = 1.0 - (2.0 * s - 1.0).powi(2);
        base_change * window
    }

    fn box_clone(&self) -> Box<dyn MemristorPhysics> { Box::new(self.clone()) }
}

impl fmt::Debug for Yakopcic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Yakopcic(r_on={}, r_off={})", self.r_on, self.r_off)
    }
}
