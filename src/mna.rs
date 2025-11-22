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
            Component::Memristor { id, n1, n2, mem } => write!(f, "M {} {} {} state={:.3}\n", id, n1, n2, mem.state),
            Component::Capacitor { id, n1, n2, c } => write!(f, "C {} {} {}\n", id, n1, n2),
            Component::CurrentSource { id, n_plus, n_minus, i } => write!(f, "I {} {} {}\n", id, n_plus, n_minus),
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

        let mut a = vec![vec![0.0; n]; n];
        let mut b = vec![0.0; n];

        // map voltage source index to its column
        let mut vs_map = Vec::new(); // (comp_index, vs_idx)

        // First pass: stamp resistors and memristors conductance
        for (ci, comp) in self.comps.iter().enumerate() {
                match comp {
                Component::Resistor { n1, n2, r, .. } => {
                    let g = 1.0 / *r;
                    if *n1 != 0 {
                        let i = *n1 - 1;
                        a[i][i] += g;
                    }
                    if *n2 != 0 {
                        let j = *n2 - 1;
                        a[j][j] += g;
                    }
                    if *n1 != 0 && *n2 != 0 {
                        let i = *n1 - 1;
                        let j = *n2 - 1;
                        a[i][j] -= g;
                        a[j][i] -= g;
                    }
                }
                Component::Memristor { n1, n2, mem, .. } => {
                    let r_now = mem.resistance();
                    let g = if r_now <= 0.0 { 0.0 } else { 1.0 / r_now };
                    if *n1 != 0 {
                        let i = *n1 - 1;
                        a[i][i] += g;
                    }
                    if *n2 != 0 {
                        let j = *n2 - 1;
                        a[j][j] += g;
                    }
                    if *n1 != 0 && *n2 != 0 {
                        let i = *n1 - 1;
                        let j = *n2 - 1;
                        a[i][j] -= g;
                        a[j][i] -= g;
                    }
                }
                Component::Capacitor { id: _, n1, n2, c } => {
                    let g = *c / dt;
                    // stamp like a resistor with conductance g and add RHS from previous voltage
                    let vprev = *self.cap_vprev.get(&ci).unwrap_or(&0.0);
                    if *n1 != 0 {
                        let i = *n1 - 1;
                        a[i][i] += g;
                        if *n2 != 0 {
                            let j = *n2 - 1;
                            a[i][j] -= g;
                        }
                        // RHS contribution: g * vprev_diff added to node n1
                        b[*n1 - 1] += g * vprev;
                    }
                    if *n2 != 0 {
                        let j = *n2 - 1;
                        a[j][j] += g;
                        if *n1 != 0 {
                            let i = *n1 - 1;
                            a[j][i] -= g;
                        }
                        // RHS contribution: -g * vprev_diff added to node n2
                        b[*n2 - 1] -= g * vprev;
                    }
                }
                Component::CurrentSource { id: _, n_plus, n_minus, i } => {
                    // current I from n_plus -> n_minus; add to RHS: n_plus -= I, n_minus += I
                    if *n_plus != 0 {
                        b[*n_plus - 1] -= *i;
                    }
                    if *n_minus != 0 {
                        b[*n_minus - 1] += *i;
                    }
                }
                Component::VSource { .. } => {
                    vs_map.push(ci);
                }
            }
        }

        // Second pass: stamp voltage sources (rows/cols for KVL equations)
        for (k, &ci) in vs_map.iter().enumerate() {
            // column/row index for this voltage source in MNA
            let vs_col = n_var_nodes + k;
            if let Component::VSource { n_plus, n_minus, kind, .. } = &self.comps[ci] {
                if *n_plus != 0 {
                    let i = *n_plus - 1;
                    a[i][vs_col] += 1.0;
                    a[vs_col][i] += 1.0;
                }
                if *n_minus != 0 {
                    let j = *n_minus - 1;
                    a[j][vs_col] -= 1.0;
                    a[vs_col][j] -= 1.0;
                }
                // RHS is voltage value
                b[vs_col] = kind.value_at(t);
            }
        }

        // convert a (Vec<Vec>) and b (Vec) into nalgebra matrices and solve with LU
        if n == 0 {
            // nothing to solve
            return Some(MnaResult { node_voltages: vec![0.0; n_nodes], vs_currents: Vec::new() });
        }

        // flatten row-major
        let mut flat = Vec::with_capacity(n * n);
        for row in &a {
            for &v in row.iter() {
                flat.push(v);
            }
        }
        let a_mat = DMatrix::from_row_slice(n, n, &flat);
        let b_vec = DVector::from_row_slice(&b);

        let lu = LU::new(a_mat);
        let sol_vec_opt = lu.solve(&b_vec);
        let sol_vec = match sol_vec_opt {
            Some(sv) => sv,
            None => return None,
        };

        // extract node voltages
        let mut node_voltages = vec![0.0; n_nodes];
        for ni in 1..n_nodes {
            let idx = ni - 1;
            node_voltages[ni] = sol_vec[idx];
        }

        // extract voltage source currents
        let mut vs_currents = Vec::new();
        for k in 0..n_vs {
            vs_currents.push(sol_vec[n_var_nodes + k]);
        }

        // Update memristors' state using solved voltages
        for comp in &mut self.comps {
            if let Component::Memristor { mem, n1, n2, .. } = comp {
                let v1 = if *n1 == 0 { 0.0 } else { node_voltages[*n1] };
                let v2 = if *n2 == 0 { 0.0 } else { node_voltages[*n2] };
                let vdrop = v1 - v2;
                let i = if mem.resistance() <= 0.0 { 0.0 } else { vdrop / mem.resistance() };
                mem.update_state(i, dt);
            }
        }

        Some(MnaResult { node_voltages, vs_currents })
    }
}
