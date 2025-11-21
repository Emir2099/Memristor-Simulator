use std::fmt;

#[derive(Clone)]
pub struct Resistor {
    pub id: String,
    pub r: f64,
}

#[derive(Clone)]
pub struct Memristor {
    pub id: String,
    pub ron: f64,
    pub roff: f64,
    pub state: f64, // between 0..1
    // mobility-like parameter (controls rate of state change)
    pub mu: f64,
    // window exponent for boundary enforcement (Joglekar-like window)
    pub window_p: f64,
    // current threshold below which state doesn't change
    pub ithreshold: f64,
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
            Component::M(m) => write!(f, "M({}: state={:.3})", m.id, m.state),
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
    pub fn resistance(&self) -> f64 {
        // linear interpolation between ron (state=1) and roff (state=0)
        let s = self.state.clamp(0.0, 1.0);
        self.ron * s + self.roff * (1.0 - s)
    }

    pub fn update_state(&mut self, current: f64, dt: f64) {
        // Nonlinear drift with a window function to slow state change near boundaries.
        // Use a Joglekar-like window f(x) = 1 - (2x-1)^(2*p)
        if current.abs() < self.ithreshold {
            return;
        }
        let s = self.state.clamp(0.0, 1.0);
        let two_x_m1 = 2.0 * s - 1.0;
        let f_win = 1.0 - two_x_m1.abs().powf(2.0 * self.window_p);
        // state derivative proportional to current, mobility-like factor, and window
        let ds = self.mu * current * f_win * dt;
        self.state = (self.state + ds).clamp(0.0, 1.0);
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
                let i = voltage / r_now;
                // update state using current
                m.update_state(i, dt);
                rec.record(path, voltage, i, Some(m.state));
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
            let mut top = &mut self.top;
            let current = top.step(v, dt, &mut self.recorder, "top");

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
