use std::fmt;
pub mod node_graph;
pub mod models;

#[derive(Clone)]
pub struct Resistor {
    pub id: String,
    pub r: f64,
}

// Trait-based memristor models. This lets us swap models (HP/TiO2, VTEAM, Yakopcic, etc.)
pub trait MemristorModel: Send + Sync {
    fn id(&self) -> &str;
    fn set_id(&mut self, id: String);
    fn state(&self) -> f64;
    fn set_state(&mut self, s: f64);

    // resistance at the model's current state
    fn resistance(&self) -> f64;

    // resistance for an arbitrary state (used in inner solves)
    fn resistance_for_state(&self, s: f64) -> f64;

    // Given a candidate old state, voltage across the device and current through it,
    // predict the next state after timestep dt (non-mutating prediction used in inner loops).
    fn predict_state(&self, s_old: f64, vdrop: f64, current: f64, dt: f64) -> f64;

    // clone into a boxed trait object
    fn box_clone(&self) -> Box<dyn MemristorModel>;
}

impl Clone for Box<dyn MemristorModel> {
    fn clone(&self) -> Box<dyn MemristorModel> { self.box_clone() }
}

#[derive(Clone)]
pub struct Memristor {
    pub inner: Box<dyn MemristorModel>,
}

impl Memristor {
    pub fn new<M: MemristorModel + 'static>(m: M) -> Self { Memristor { inner: Box::new(m) } }

    pub fn id(&self) -> &str { self.inner.id() }
    pub fn state(&self) -> f64 { self.inner.state() }
    pub fn set_state(&mut self, s: f64) { self.inner.set_state(s); }
    pub fn resistance(&self) -> f64 { self.inner.resistance() }
    pub fn resistance_for_state(&self, s: f64) -> f64 { self.inner.resistance_for_state(s) }
    pub fn predict_state(&self, s_old: f64, vdrop: f64, current: f64, dt: f64) -> f64 { self.inner.predict_state(s_old, vdrop, current, dt) }
}

// --- HP TiO2 (linear drift) model ---
#[derive(Clone)]
pub struct HpTiO2Model {
    pub id: String,
    pub ron: f64,
    pub roff: f64,
    pub state: f64, // 0..1
    pub mu0: f64,
    pub window_p: f64,
    pub ithreshold: f64,
    pub activation_e: f64,
    pub temperature: f64,
}

impl HpTiO2Model {
    pub fn new(id: impl Into<String>, ron: f64, roff: f64, state: f64) -> Self {
        HpTiO2Model { id: id.into(), ron, roff, state, mu0: 1e3, window_p: 1.0, ithreshold: 0.0, activation_e: 0.6, temperature: 300.0 }
    }
}

impl MemristorModel for HpTiO2Model {
    fn id(&self) -> &str { &self.id }
    fn set_id(&mut self, id: String) { self.id = id; }
    fn state(&self) -> f64 { self.state }
    fn set_state(&mut self, s: f64) { self.state = s.clamp(0.0, 1.0); }
    fn resistance(&self) -> f64 { self.resistance_for_state(self.state) }
    fn resistance_for_state(&self, s: f64) -> f64 { let s = s.clamp(0.0,1.0); self.ron * s + self.roff * (1.0 - s) }
    fn predict_state(&self, s_old: f64, _vdrop: f64, current: f64, dt: f64) -> f64 {
        if current.abs() <= self.ithreshold { return s_old.clamp(0.0,1.0); }
        let s = s_old.clamp(0.0,1.0);
        let two_x_m1 = 2.0 * s - 1.0;
        let f_win = 1.0 - two_x_m1.abs().powf(2.0 * self.window_p);
        let k_b_ev = 8.617333262145e-5_f64;
        let temp_factor = (-self.activation_e / (k_b_ev * self.temperature)).exp();
        let i_eff = (current.abs() - self.ithreshold).max(0.0).powf(1.0) * current.signum();
        let ds_dt = self.mu0 * temp_factor * i_eff * f_win;
        (s_old + ds_dt * dt).clamp(0.0, 1.0)
    }
    fn box_clone(&self) -> Box<dyn MemristorModel> { Box::new(self.clone()) }
}

// Physics-backed model adapter: wraps a boxed `models::MemristorPhysics` and implements
// the existing `MemristorModel` trait so the solver and other code do not need to change.
#[derive(Clone)]
pub struct PhysicsBackedModel {
    pub id: String,
    pub physics: Box<dyn crate::models::MemristorPhysics>,
    pub state: f64,
}

impl PhysicsBackedModel {
    pub fn new(id: impl Into<String>, physics: Box<dyn crate::models::MemristorPhysics>, state: f64) -> Self {
        PhysicsBackedModel { id: id.into(), physics, state }
    }
}

impl MemristorModel for PhysicsBackedModel {
    fn id(&self) -> &str { &self.id }
    fn set_id(&mut self, id: String) { self.id = id; }
    fn state(&self) -> f64 { self.state }
    fn set_state(&mut self, s: f64) { self.state = self.physics.clip_state(s); }
    fn resistance(&self) -> f64 { self.resistance_for_state(self.state) }
    fn resistance_for_state(&self, s: f64) -> f64 { self.physics.resistance(self.physics.clip_state(s)) }
    fn predict_state(&self, s_old: f64, vdrop: f64, current: f64, dt: f64) -> f64 {
        let ds_dt = self.physics.derivative(s_old, vdrop, current);
        self.physics.clip_state(s_old + ds_dt * dt)
    }
    fn box_clone(&self) -> Box<dyn MemristorModel> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub enum Component {
    R(Resistor),
    M(Memristor),
    Series(Vec<Component>),
    Parallel(Vec<Component>),
}

impl fmt::Debug for Component {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Component::R(r) => write!(f, "R({}:{:.3}Ω)", r.id, r.r),
            Component::M(m) => write!(f, "M({}: state={:.3})", m.id(), m.state()),
            Component::Series(v) => write!(f, "Series({} items)", v.len()),
            Component::Parallel(v) => write!(f, "Parallel({} items)", v.len()),
        }
    }
}

impl Resistor {
    pub fn eq_resistance(&self) -> f64 {
        self.r
    }
}

impl Memristor {
    // Mutating state update using the voltage across the device (v = vdrop)
    // This computes current = v / R(s) then uses the model's predictor to step the state.
    pub fn update_state_by_voltage(&mut self, vdrop: f64, dt: f64) {
        let s_old = self.state();
        let r_now = self.resistance_for_state(s_old).max(1e-300);
        let current = if r_now.is_infinite() { 0.0 } else { vdrop / r_now };
        let s_new = self.predict_state(s_old, vdrop, current, dt);
        self.set_state(s_new);
    }

    // Mutating update by providing current directly
    pub fn update_state_by_current(&mut self, current: f64, dt: f64) {
        let s_old = self.state();
        let s_new = self.predict_state(s_old, 0.0, current, dt);
        self.set_state(s_new);
    }

    pub fn set_id(&mut self, id: String) { self.inner.set_id(id); }
}

impl std::fmt::Debug for Memristor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Memristor({}: state={:.6})", self.id(), self.state())
    }
}

impl Component {
    pub fn eq_resistance(&self) -> f64 {
        match self {
            Component::R(r) => r.eq_resistance(),
            Component::M(m) => m.resistance(),
            Component::Series(items) => items.iter().map(|c| c.eq_resistance()).sum(),
            Component::Parallel(items) => {
                let mut inv = 0.0;
                for c in items {
                    let r = c.eq_resistance();
                    if r <= 0.0 {
                        return 0.0;
                    }
                    inv += 1.0 / r;
                }
                if inv == 0.0 {
                    f64::INFINITY
                } else {
                    1.0 / inv
                }
            }
        }
    }

    // Simulate a step: given voltage across this component, compute current through it,
    // update any memristor states using that current (and dt), and optionally record values.
    // Returns total current through component.
    pub fn step(&mut self, voltage: f64, dt: f64, rec: &mut Recorder, path: &str) -> f64 {
        match self {
            Component::R(r) => {
                let i = voltage / r.r;
                rec.record(path, voltage, i, None);
                i
            }
            Component::M(m) => {
                let r_now = m.resistance();
                let i = if r_now == 0.0 { 0.0 } else { voltage / r_now };
                // update state using voltage across device (mutates model)
                m.update_state_by_voltage(voltage, dt);
                rec.record(path, voltage, i, Some(m.state()));
                i
            }
            Component::Series(items) => {
                // series: same current through all; first compute equivalent R
                let r_eq = items.iter().map(|c| c.eq_resistance()).sum::<f64>();
                let i = if r_eq.is_infinite() { 0.0 } else { voltage / r_eq };
                // distribute voltages proportionally
                for (idx, child) in items.iter_mut().enumerate() {
                    let r = child.eq_resistance();
                    let v_child = i * r;
                    let child_path = format!("{}/S{}", path, idx);
                    child.step(v_child, dt, rec, &child_path);
                }
                rec.record(path, voltage, i, None);
                i
            }
            Component::Parallel(items) => {
                // parallel: same voltage across each child
                let mut i_total = 0.0;
                for (idx, child) in items.iter_mut().enumerate() {
                    let child_path = format!("{}/P{}", path, idx);
                    let i_child = child.step(voltage, dt, rec, &child_path);
                    i_total += i_child;
                }
                rec.record(path, voltage, i_total, None);
                i_total
            }
        }
    }
}

#[derive(Debug)]
pub struct Sample {
    pub time: f64,
    pub path: String,
    pub voltage: f64,
    pub current: f64,
    pub state: Option<f64>,
}

pub struct Recorder {
    pub samples: Vec<Sample>,
}

impl Recorder {
    pub fn new() -> Self {
        Recorder { samples: Vec::new() }
    }

    pub fn record(&mut self, path: &str, voltage: f64, current: f64, state: Option<f64>) {
        let t = 0.0; // placeholder, time is set by simulator wrapper
        self.samples.push(Sample {
            time: t,
            path: path.to_string(),
            voltage,
            current,
            state,
        });
    }

    pub fn clear(&mut self) {
        self.samples.clear();
    }
}

pub struct Simulator {
    pub top: Component,
    pub recorder: Recorder,
    pub time: f64,
}

impl Simulator {
    pub fn new(top: Component) -> Self {
        Simulator {
            top,
            recorder: Recorder::new(),
            time: 0.0,
        }
    }

    // Simulate over time with a drive function that gives voltage at time t
    pub fn run<F: Fn(f64) -> f64>(&mut self, tstop: f64, dt: f64, drive: F) {
        let mut t = 0.0;
        while t <= tstop {
            let v = drive(t);
            self.recorder.clear();
            // step the network with this voltage
            let top = &mut self.top;
            let _current = top.step(v, dt, &mut self.recorder, "top");

            // stamp time into recorded samples
            for s in &mut self.recorder.samples {
                s.time = t;
            }

            // output CSV line per sample to stdout for this time step
            // (the CLI will call run and collect recorder data across time steps)

            // advance memristor internal states already done in step
            t += dt;
            self.time = t;
        }
    }
}

pub mod mna;

// Run a small MNA example (same netlist as `cli`) and return CSV as a String.
// This is intended for in-process use (e.g. GUI) so we don't need to spawn the CLI binary.
pub fn run_mna_example_csv(tstop: f64, dt: f64) -> String {
    use crate::mna;

    let mut net = mna::Netlist::new();
    // R1 between node 1 and ground (1k)
    net.add(mna::Component::Resistor { id: "R1".into(), n1: 1, n2: 0, r: 1e3 });
    // Memristor M1 between node 2 and ground (HP/TiO2 default for example)
    net.add(mna::Component::Memristor { id: "M1".into(), n1: 2, n2: 0, mem: Memristor::new(HpTiO2Model::new("M1", 100.0, 16000.0, 0.1)) });
    // Series branch: R2 (2k) between node1 and node2
    net.add(mna::Component::Resistor { id: "R2".into(), n1: 1, n2: 2, r: 2e3 });
    // Voltage source between node 0 and node 1 (drive across R1+branch)
    net.add(mna::Component::VSource { id: "V1".into(), n_plus: 1, n_minus: 0, kind: mna::VoltageKind::DC(1.0) });

    // build CSV in-memory
    let mut s = String::new();
    s.push_str("time,node_0,node_1,node_2,M1_state\n");

    let mut t = 0.0;
    while t <= tstop {
        if let Some(res) = net.solve(t, dt) {
            let v0 = res.node_voltages.get(0).copied().unwrap_or(0.0);
            let v1 = res.node_voltages.get(1).copied().unwrap_or(0.0);
            let v2 = res.node_voltages.get(2).copied().unwrap_or(0.0);
            // find M1 state
            let mut m1_state = 0.0;
            for c in &net.comps {
                if let mna::Component::Memristor { id, mem, .. } = c {
                    if id == "M1" { m1_state = mem.state(); }
                }
            }
            s.push_str(&format!("{:.9},{:.6e},{:.6e},{:.6e},{:.6}\n", t, v0, v1, v2, m1_state));
        } else {
            s.push_str(&format!("{:.9},ERR,ERR,ERR,ERR\n", t));
        }
        t += dt;
    }

    s
}

// Stream MNA example results incrementally. Sends tuples (time, node2_voltage, m1_state)
// over the provided sender. The function checks `cancel` periodically and returns early
// if cancellation is requested. Caller should handle channel closure as completion.
pub fn run_mna_stream(
    tstop: f64,
    dt: f64,
    tx: std::sync::mpsc::Sender<(f64, f64, f64)>,
    cancel: std::sync::Arc<std::sync::atomic::AtomicBool>,
    mem_template: Memristor,
    vs_kind: crate::mna::VoltageKind,
) {
    use crate::mna;

    let mut net = mna::Netlist::new();
    net.add(mna::Component::Resistor { id: "R1".into(), n1: 1, n2: 0, r: 1e3 });
    let mut mem = mem_template.clone();
    mem.set_id("M1".into());
    net.add(mna::Component::Memristor { id: "M1".into(), n1: 2, n2: 0, mem });
    net.add(mna::Component::Resistor { id: "R2".into(), n1: 1, n2: 2, r: 2e3 });
    net.add(mna::Component::VSource { id: "V1".into(), n_plus: 1, n_minus: 0, kind: vs_kind });

    // compute a stable integer step count to avoid cumulative floating-point error
    let steps_f = (tstop / dt).floor();
    let steps = if steps_f.is_finite() && steps_f >= 0.0 { steps_f as usize } else { 0 };
    // iterate 0..=steps and ensure final sample time equals tstop
    for i in 0..=steps {
        if cancel.load(std::sync::atomic::Ordering::Relaxed) {
            break;
        }
        let t = if i == steps { tstop } else { (i as f64) * dt };

        if let Some(res) = net.solve(t, dt) {
            let v2 = res.node_voltages.get(2).copied().unwrap_or(0.0);
            // find M1 state
            let mut m1_state = 0.0;
            for c in &net.comps {
                if let mna::Component::Memristor { id, mem, .. } = c {
                    if id == "M1" { m1_state = mem.state(); }
                }
            }
            // ignore send errors (receiver dropped)
            let _ = tx.send((t, v2, m1_state));
        } else {
            // send NaNs on error
            let _ = tx.send((t, f64::NAN, f64::NAN));
        }
    }
}

// Stream a provided Netlist incrementally. Sends tuples (time, monitored_node_voltage, mem_state)
// `monitor_node` is the node index to sample (0..n_nodes-1). `mem_id` is the memristor id to report state for.
pub fn run_mna_stream_from_netlist(
    mut net: crate::mna::Netlist,
    monitor_node: usize,
    mem_id: String,
    tstop: f64,
    dt: f64,
    tx: std::sync::mpsc::Sender<(f64, f64, f64)>,
    cancel: std::sync::Arc<std::sync::atomic::AtomicBool>,
) {
    // compute a stable integer step count
    let steps_f = (tstop / dt).floor();
    let steps = if steps_f.is_finite() && steps_f >= 0.0 { steps_f as usize } else { 0 };
    for i in 0..=steps {
        if cancel.load(std::sync::atomic::Ordering::Relaxed) { break; }
        let t = if i == steps { tstop } else { (i as f64) * dt };
        if let Some(res) = net.solve(t, dt) {
            let v_mon = res.node_voltages.get(monitor_node).copied().unwrap_or(0.0);
            // find mem state with given id
            let mut m_state = 0.0;
            for c in &net.comps {
                if let crate::mna::Component::Memristor { id, mem, .. } = c {
                    if id == &mem_id { m_state = mem.state(); }
                }
            }
            let _ = tx.send((t, v_mon, m_state));
        } else {
            let _ = tx.send((if i==steps { tstop } else { (i as f64) * dt }, f64::NAN, f64::NAN));
        }
    }
}
