use crate::Memristor;
use nalgebra::{DMatrix, DVector, linalg::LU};
use std::collections::HashMap;
use std::fmt;

#[derive(Clone, Debug)]
pub enum VoltageKind {
    DC(f64),
    Sine { amp: f64, freq: f64, phase: f64 },
}

impl VoltageKind {
    pub fn value_at(&self, t: f64) -> f64 {
        match self {
            VoltageKind::DC(v) => *v,
            VoltageKind::Sine { amp, freq, phase } => amp * (2.0 * std::f64::consts::PI * freq * t + phase).sin(),
        }
    }
}

#[derive(Clone, Debug)]
pub enum Component {
    Resistor { id: String, n1: usize, n2: usize, r: f64 },
    Memristor { id: String, n1: usize, n2: usize, mem: Memristor },
    Capacitor { id: String, n1: usize, n2: usize, c: f64 },
    CurrentSource { id: String, n_plus: usize, n_minus: usize, i: f64 },
    VSource { id: String, n_plus: usize, n_minus: usize, kind: VoltageKind },
}

impl fmt::Display for Component {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Component::Resistor { id, n1, n2, .. } => write!(f, "R {} {} {}\n", id, n1, n2),
            Component::Memristor { id, n1, n2, mem } => write!(f, "M {} {} {} state={:.3}\n", id, n1, n2, mem.state()),
            Component::Capacitor { id, n1, n2, .. } => write!(f, "C {} {} {}\n", id, n1, n2),
            Component::CurrentSource { id, n_plus, n_minus, .. } => write!(f, "I {} {} {}\n", id, n_plus, n_minus),
            Component::VSource { id, n_plus, n_minus, kind } => write!(f, "V {} {} {} {:?}\n", id, n_plus, n_minus, kind),
        }
    }
}

pub struct Netlist {
    pub comps: Vec<Component>,
    pub max_node: usize,
    // per-capacitor previous voltage (comp index -> v_prev)
    pub cap_vprev: HashMap<usize, f64>,
}

impl Netlist {
    pub fn new() -> Self {
        Netlist { comps: Vec::new(), max_node: 0, cap_vprev: HashMap::new() }
    }

    pub fn add(&mut self, c: Component) {
        let ci = self.comps.len();
        match &c {
            Component::Resistor { n1, n2, .. } | Component::Memristor { n1, n2, .. } | Component::Capacitor { n1, n2, .. } | Component::CurrentSource { n_plus: n1, n_minus: n2, .. } => {
                self.max_node = self.max_node.max(*n1).max(*n2);
            }
            Component::VSource { n_plus, n_minus, .. } => {
                self.max_node = self.max_node.max(*n_plus).max(*n_minus);
            }
        }
        // initialize capacitor previous voltage entry if needed
        if let Component::Capacitor { .. } = &c {
            self.cap_vprev.insert(ci, 0.0);
        }
        self.comps.push(c);
    }
}

// The project now uses `nalgebra` for linear solves; the old solver is kept commented
// for reference.

pub struct MnaResult {
    pub node_voltages: Vec<f64>, // indexed 0..max_node, node 0 is ground
    pub vs_currents: Vec<f64>,
}

impl Netlist {
    // assemble and solve MNA at time t. dt is provided for memristor state updates later.
    pub fn solve(&mut self, t: f64, dt: f64) -> Option<MnaResult> {
        let n_nodes = self.max_node + 1; // include node 0
        let n_var_nodes = if n_nodes > 1 { n_nodes - 1 } else { 0 }; // exclude ground
        let n_vs = self.comps.iter().filter(|c| matches!(c, Component::VSource { .. })).count();
        let n = n_var_nodes + n_vs; // total unknowns

        // We'll perform a Newton-Raphson loop on node voltages. For each NR iteration we
        // linearize memristors by treating their internal state as fixed for that iterate
        // (we solve the memristor state implicitly per candidate voltage via a small inner loop),
        // then stamp the instantaneous conductance g = 1 / R(s_guess) and solve the linear MNA.
        // Repeat until voltage corrections are small.

        if n == 0 {
            return Some(MnaResult { node_voltages: vec![0.0; n_nodes], vs_currents: Vec::new() });
        }

        // Inner solver: for a given voltage drop vdrop and previous state s_old, solve for s_new via fixed-point
        let solve_state_given_voltage = |m: &Memristor, vdrop: f64, s_old: f64, dt_local: f64| -> f64 {
            let mut s = s_old;
            for _iter in 0..12 {
                let r = m.resistance_for_state(s);
                let current = if r <= 0.0 { 0.0 } else { vdrop / r };
                let s_next = m.predict_state(s, vdrop, current, dt_local);
                if (s_next - s).abs() < 1e-8 { s = s_next; break; }
                s = s_next;
            }
            s
        };

        // Initial guess: solve linear problem using current mem resistances (explicit-like) to get starting voltages
        // Build system A0 * V0 = b0
        // Use pre-allocated vectors to reduce allocations
        let mut flat0 = Vec::with_capacity(n * n);
        let mut v_guess = vec![0.0f64; n_var_nodes];
        {
            // Pre-allocate matrix as flat array for better cache performance
            let mut a0 = vec![0.0; n * n];
            let mut b0 = vec![0.0; n];
            let mut vs_map = Vec::new();
            for (ci, comp) in self.comps.iter().enumerate() {
                match comp {
                    Component::Resistor { n1, n2, r, .. } => {
                        let g = 1.0 / *r;
                        if *n1 != 0 { let i = *n1 - 1; a0[i * n + i] += g; }
                        if *n2 != 0 { let j = *n2 - 1; a0[j * n + j] += g; }
                        if *n1 != 0 && *n2 != 0 { 
                            let i = *n1 - 1; 
                            let j = *n2 - 1; 
                            a0[i * n + j] -= g; 
                            a0[j * n + i] -= g; 
                        }
                    }
                    Component::Memristor { n1, n2, mem, .. } => {
                        let r_now = mem.resistance();
                        let g = if r_now <= 0.0 { 0.0 } else { 1.0 / r_now };
                        if *n1 != 0 { let i = *n1 - 1; a0[i * n + i] += g; }
                        if *n2 != 0 { let j = *n2 - 1; a0[j * n + j] += g; }
                        if *n1 != 0 && *n2 != 0 { 
                            let i = *n1 - 1; 
                            let j = *n2 - 1; 
                            a0[i * n + j] -= g; 
                            a0[j * n + i] -= g; 
                        }
                    }
                    Component::Capacitor { id: _, n1, n2, c } => {
                        let g = *c / dt;
                        let vprev = *self.cap_vprev.get(&ci).unwrap_or(&0.0);
                        if *n1 != 0 { 
                            let i = *n1 - 1; 
                            a0[i * n + i] += g; 
                            if *n2 != 0 { 
                                let j = *n2 - 1; 
                                a0[i * n + j] -= g; 
                            } 
                            b0[i] += g * vprev; 
                        }
                        if *n2 != 0 { 
                            let j = *n2 - 1; 
                            a0[j * n + j] += g; 
                            if *n1 != 0 { 
                                let i = *n1 - 1; 
                                a0[j * n + i] -= g; 
                            } 
                            b0[j] -= g * vprev; 
                        }
                    }
                    Component::CurrentSource { id: _, n_plus, n_minus, i } => {
                        if *n_plus != 0 { b0[*n_plus - 1] -= *i; }
                        if *n_minus != 0 { b0[*n_minus - 1] += *i; }
                    }
                    Component::VSource { .. } => { vs_map.push(ci); }
                }
            }
            for (k, &ci) in vs_map.iter().enumerate() {
                let vs_col = n_var_nodes + k;
                if let Component::VSource { n_plus, n_minus, kind, .. } = &self.comps[ci] {
                    if *n_plus != 0 { 
                        let i = *n_plus - 1; 
                        a0[i * n + vs_col] += 1.0; 
                        a0[vs_col * n + i] += 1.0; 
                    }
                    if *n_minus != 0 { 
                        let j = *n_minus - 1; 
                        a0[j * n + vs_col] -= 1.0; 
                        a0[vs_col * n + j] -= 1.0; 
                    }
                    b0[vs_col] = kind.value_at(t);
                }
            }
            // Already flat, just copy
            flat0.extend_from_slice(&a0);
            let a_mat0 = DMatrix::from_row_slice(n, n, &flat0);
            let b_vec0 = DVector::from_row_slice(&b0);
            let lu0 = LU::new(a_mat0);
            if let Some(sol0) = lu0.solve(&b_vec0) {
                for ni in 1..n_nodes { v_guess[ni - 1] = sol0[ni - 1]; }
            }
        }

        let mut v_curr = v_guess.clone();
        let mut vs_currents = vec![0.0f64; n_vs];

        // Pre-allocate matrix storage for NR iterations (reuse across iterations)
        let mut a_flat = vec![0.0; n * n];
        let mut b = vec![0.0; n];
        
        // Newton-Raphson on voltages
        for _nr_iter in 0..12 {
            // Clear matrix and RHS for this iteration
            a_flat.fill(0.0);
            b.fill(0.0);
            let mut vs_map = Vec::new();

            // For memristor state guesses we will compute provisional s for the current v_curr
            let mut mem_s_guess: HashMap<usize, f64> = HashMap::new();
            for (ci, comp) in self.comps.iter().enumerate() {
                if let Component::Memristor { n1: _, n2: _, mem, .. } = comp {
                    mem_s_guess.insert(ci, mem.state());
                }
            }

            // Update each memristor's s_guess given v_curr at its terminals (inner fixed-point)
            for (ci, comp) in self.comps.iter().enumerate() {
                if let Component::Memristor { n1, n2, mem, .. } = comp {
                    let v1 = if *n1 == 0 { 0.0 } else { v_curr[*n1 - 1] };
                    let v2 = if *n2 == 0 { 0.0 } else { v_curr[*n2 - 1] };
                    let vdrop = v1 - v2;
                    let s_old = mem.state();
                    let s_new = solve_state_given_voltage(mem, vdrop, s_old, dt);
                    mem_s_guess.insert(ci, s_new);
                }
            }

            // stamp components using mem_s_guess for memristors
            for (ci, comp) in self.comps.iter().enumerate() {
                match comp {
                    Component::Resistor { n1, n2, r, .. } => {
                        let g = 1.0 / *r;
                        if *n1 != 0 { let i = *n1 - 1; a_flat[i * n + i] += g; }
                        if *n2 != 0 { let j = *n2 - 1; a_flat[j * n + j] += g; }
                        if *n1 != 0 && *n2 != 0 { 
                            let i = *n1 - 1; 
                            let j = *n2 - 1; 
                            a_flat[i * n + j] -= g; 
                            a_flat[j * n + i] -= g; 
                        }
                    }
                    Component::Memristor { n1, n2, mem, .. } => {
                        let s_guess = *mem_s_guess.get(&ci).unwrap_or(&mem.state());
                        let r_now = mem.resistance_for_state(s_guess);
                        let g = if r_now <= 0.0 { 0.0 } else { 1.0 / r_now };
                        if *n1 != 0 { let i = *n1 - 1; a_flat[i * n + i] += g; }
                        if *n2 != 0 { let j = *n2 - 1; a_flat[j * n + j] += g; }
                        if *n1 != 0 && *n2 != 0 { 
                            let i = *n1 - 1; 
                            let j = *n2 - 1; 
                            a_flat[i * n + j] -= g; 
                            a_flat[j * n + i] -= g; 
                        }
                    }
                    Component::Capacitor { id: _, n1, n2, c } => {
                        let g = *c / dt;
                        let vprev = *self.cap_vprev.get(&ci).unwrap_or(&0.0);
                        if *n1 != 0 { 
                            let i = *n1 - 1; 
                            a_flat[i * n + i] += g; 
                            if *n2 != 0 { 
                                let j = *n2 - 1; 
                                a_flat[i * n + j] -= g; 
                            } 
                            b[i] += g * vprev; 
                        }
                        if *n2 != 0 { 
                            let j = *n2 - 1; 
                            a_flat[j * n + j] += g; 
                            if *n1 != 0 { 
                                let i = *n1 - 1; 
                                a_flat[j * n + i] -= g; 
                            } 
                            b[j] -= g * vprev; 
                        }
                    }
                    Component::CurrentSource { id: _, n_plus, n_minus, i } => {
                        if *n_plus != 0 { b[*n_plus - 1] -= *i; }
                        if *n_minus != 0 { b[*n_minus - 1] += *i; }
                    }
                    Component::VSource { .. } => { vs_map.push(ci); }
                }
            }

            for (k, &ci) in vs_map.iter().enumerate() {
                let vs_col = n_var_nodes + k;
                if let Component::VSource { n_plus, n_minus, kind, .. } = &self.comps[ci] {
                    if *n_plus != 0 { 
                        let i = *n_plus - 1; 
                        a_flat[i * n + vs_col] += 1.0; 
                        a_flat[vs_col * n + i] += 1.0; 
                    }
                    if *n_minus != 0 { 
                        let j = *n_minus - 1; 
                        a_flat[j * n + vs_col] -= 1.0; 
                        a_flat[vs_col * n + j] -= 1.0; 
                    }
                    b[vs_col] = kind.value_at(t);
                }
            }

            // Create DMatrix directly from flat array
            let a_mat = DMatrix::from_row_slice(n, n, &a_flat);
            let b_vec = DVector::from_row_slice(&b);
            let lu = LU::new(a_mat);
            let sol_vec_opt = lu.solve(&b_vec);
            let sol_vec = match sol_vec_opt { Some(sv) => sv, None => return None };

            // extract new voltages
            let mut v_new = vec![0.0f64; n_var_nodes];
            for ni in 1..n_nodes { v_new[ni - 1] = sol_vec[ni - 1]; }

            // check convergence
            let mut max_diff = 0.0f64;
            for i in 0..n_var_nodes { max_diff = max_diff.max((v_new[i] - v_curr[i]).abs()); }

            v_curr = v_new;

            if max_diff < 1e-9 { // converged
                // fill node_voltages and vs_currents
                let mut node_voltages = vec![0.0; n_nodes];
                for ni in 1..n_nodes { node_voltages[ni] = v_curr[ni - 1]; }
                for k in 0..n_vs { vs_currents[k] = sol_vec[n_var_nodes + k]; }

                // commit memristor states using final voltages
                for (_ci, comp) in self.comps.iter_mut().enumerate() {
                    if let Component::Memristor { n1, n2, mem, .. } = comp {
                        let v1 = if *n1 == 0 { 0.0 } else { node_voltages[*n1] };
                        let v2 = if *n2 == 0 { 0.0 } else { node_voltages[*n2] };
                        let vdrop = v1 - v2;
                        let s_old = mem.state();
                        let s_new = solve_state_given_voltage(mem, vdrop, s_old, dt);
                        mem.set_state(s_new);
                    }
                }

                // update capacitor previous-voltages
                for (ci, comp) in self.comps.iter().enumerate() {
                    if let Component::Capacitor { n1, n2, .. } = comp {
                        let v1 = if *n1 == 0 { 0.0 } else { node_voltages[*n1] };
                        let v2 = if *n2 == 0 { 0.0 } else { node_voltages[*n2] };
                        let vprev = v1 - v2;
                        self.cap_vprev.insert(ci, vprev);
                    }
                }

                return Some(MnaResult { node_voltages, vs_currents });
            }
        }

        // If NR did not converge, return last iterate (best-effort)
        // populate outputs from v_curr
        let mut node_voltages = vec![0.0; n_nodes];
        for ni in 1..n_nodes { node_voltages[ni] = v_curr[ni - 1]; }
        Some(MnaResult { node_voltages, vs_currents })
    }
}
