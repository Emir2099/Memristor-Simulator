use eframe::egui;
use memristor_sim::node_graph::{Graph, NodeKind};
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread;

struct App {
    tstop: f64,
    dt: f64,
    running: bool,
    rx: Option<Receiver<(f64, f64, f64)>>,
    cancel: Option<Arc<AtomicBool>>,
    times: Vec<f64>,
    node2: Vec<f64>,
    m1_state: Vec<f64>,
    // params
    mem_ron: f64,
    mem_roff: f64,
    mem_state_init: f64,
    mem_mu0: f64,
    mem_n: f64,
    mem_window_p: f64,
    mem_ithreshold: f64,
    drive_is_sine: bool,
    drive_amp: f64,
    drive_freq: f64,
    // simple in-app graph editor state
    graph: Graph,
    link_from: usize,
    link_to: usize,
}

impl Default for App {
    fn default() -> Self {
        Self {
            tstop: 1e-3,
            dt: 1e-6,
            running: false,
            rx: None,
            cancel: None,
            times: Vec::new(),
            node2: Vec::new(),
            m1_state: Vec::new(),
            mem_ron: 100.0,
            mem_roff: 16000.0,
            mem_state_init: 0.1,
            mem_mu0: 1e3,
            mem_n: 1.0,
            mem_window_p: 1.0,
            mem_ithreshold: 0.0,
            drive_is_sine: false,
            drive_amp: 1.0,
            drive_freq: 1000.0,
            graph: Graph::default(),
            link_from: 0,
            link_to: 0,
        }
    }
}

impl App {
    fn start_sim(&mut self) {
        if self.running { return; }
        self.running = true;
        self.times.clear(); self.node2.clear(); self.m1_state.clear();

        let tstop = self.tstop;
        let dt = self.dt;
        let (tx, rx) = channel::<(f64, f64, f64)>();
        let cancel_flag = Arc::new(AtomicBool::new(false));
        self.rx = Some(rx);
        self.cancel = Some(cancel_flag.clone());

        let mem_templ = memristor_sim::Memristor {
            id: "M1".into(), ron: self.mem_ron, roff: self.mem_roff,
            state: self.mem_state_init, mu0: self.mem_mu0, n: self.mem_n,
            window_p: self.mem_window_p, ithreshold: self.mem_ithreshold,
            activation_e: 0.6, temperature: 300.0
        };
        let vs_kind = if self.drive_is_sine {
            memristor_sim::mna::VoltageKind::Sine { amp: self.drive_amp, freq: self.drive_freq, phase: 0.0 }
        } else {
            memristor_sim::mna::VoltageKind::DC(self.drive_amp)
        };

        thread::spawn(move || {
            memristor_sim::run_mna_stream(tstop, dt, tx, cancel_flag, mem_templ, vs_kind);
        });
    }

    fn start_sim_from_netlist(&mut self, net: memristor_sim::mna::Netlist, monitor_node: usize, mem_id: String) {
        if self.running { return; }
        self.running = true;
        self.times.clear(); self.node2.clear(); self.m1_state.clear();

        let tstop = self.tstop;
        let dt = self.dt;
        let (tx, rx) = std::sync::mpsc::channel::<(f64, f64, f64)>();
        let cancel_flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.rx = Some(rx);
        self.cancel = Some(cancel_flag.clone());

        thread::spawn(move || {
            memristor_sim::run_mna_stream_from_netlist(net, monitor_node, mem_id, tstop, dt, tx, cancel_flag);
        });
    }

    fn try_collect_stream(&mut self) {
        if let Some(rx) = &self.rx {
            loop {
                match rx.try_recv() {
                    Ok((t, v2, s)) => {
                        if v2.is_finite() && s.is_finite() {
                            self.times.push(t);
                            self.node2.push(v2);
                            self.m1_state.push(s);
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        let n = self.times.len();
                        let (t_first, t_last) = if n >= 1 { (self.times.first().copied().unwrap_or(0.0), self.times.last().copied().unwrap_or(0.0)) } else { (0.0, 0.0) };
                        let (v_first, v_last) = if n >= 1 { (self.node2.first().copied().unwrap_or(0.0), self.node2.last().copied().unwrap_or(0.0)) } else { (0.0, 0.0) };
                        let v_min = self.node2.iter().cloned().fold(f64::INFINITY, f64::min);
                        let v_max = self.node2.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                        println!("[sim debug] stream disconnected: samples={} t_first={:.9} t_last={:.9} v_first={:.9e} v_last={:.9e} v_min={:.9e} v_max={:.9e}", n, t_first, t_last, v_first, v_last, v_min, v_max);
                        self.running = false; self.rx = None; self.cancel = None; break;
                    }
                }
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.try_collect_stream();
        // Top controls and status
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("Memristor Simulator (native GUI)");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.add_enabled(!self.running, egui::Button::new("Run")).clicked() { self.start_sim(); }
                    if ui.add_enabled(self.running, egui::Button::new("Stop")).clicked() {
                        if let Some(cancel) = &self.cancel { cancel.store(true, Ordering::Relaxed); }
                    }
                });
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("tstop (s):"); ui.add(egui::DragValue::new(&mut self.tstop).speed(1e-6));
                ui.label("dt (s):"); ui.add(egui::DragValue::new(&mut self.dt).speed(1e-7));
                if ui.button("Save CSV").clicked() {
                    if !self.times.is_empty() {
                        let mut s = String::new(); s.push_str("time,node_2,M1_state\n");
                        for i in 0..self.times.len() { s.push_str(&format!("{:.9},{:.6e},{:.6}\n", self.times[i], self.node2[i], self.m1_state[i])); }
                        if let Some(path) = rfd::FileDialog::new().save_file() { let _ = std::fs::write(&path, &s); }
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(format!("Status: {}", if self.running { "Running..." } else { "Idle" }));
                });
            });
            let progress = if let Some(&t) = self.times.last() { (t / self.tstop).clamp(0.0, 1.0) as f32 } else { 0.0f32 };
            ui.add(egui::ProgressBar::new(progress).text(format!("{:.1}%", (progress as f64) * 100.0)).desired_width(400.0));
        });

        // Left side: sample list and some stats
        egui::SidePanel::left("left_panel").min_width(260.0).resizable(true).show(ctx, |ui| {
            ui.label("Last samples (time, node_2, state):");
            egui::ScrollArea::vertical().max_height(280.0).show(ui, |ui| {
                let n = self.times.len();
                let start = if n > 16 { n - 16 } else { 0 };
                for i in start..n { ui.label(format!("{:.6}\t{:.6e}\t{:.6}", self.times[i], self.node2[i], self.m1_state[i])); }
                if n == 0 { ui.label("(no samples yet)"); }
            });

            ui.separator();
            ui.label(format!("Samples: {}", self.times.len()));
            if self.times.len() >= 2 {
                let first = self.times.first().copied().unwrap_or(0.0);
                let last = self.times.last().copied().unwrap_or(0.0);
                let rate = (self.times.len() - 1) as f64 / (last - first).max(1e-12);
                ui.label(format!("{:.0} sps", rate));
            }

            ui.separator();
            ui.label("Memristor params:");
            ui.add(egui::DragValue::new(&mut self.mem_ron)); ui.label("ron");
            ui.add(egui::DragValue::new(&mut self.mem_roff)); ui.label("roff");

            ui.separator();
            ui.label("Node Graph (simple):");
            ui.horizontal(|ui| {
                if ui.button("Add Memristor").clicked() {
                    self.graph.add_node(NodeKind::Memristor { id: format!("m{}", self.graph.nodes.len()), ron: self.mem_ron, roff: self.mem_roff, state: self.mem_state_init, mu0: self.mem_mu0, n: self.mem_n, window_p: self.mem_window_p, ithreshold: self.mem_ithreshold });
                }
                if ui.button("Add VSource").clicked() {
                    self.graph.add_node(NodeKind::VSource { id: format!("v{}", self.graph.nodes.len()), amp: self.drive_amp, freq: self.drive_freq, is_sine: self.drive_is_sine });
                }
            });

            ui.label("Nodes:");
            for n in &self.graph.nodes { ui.label(format!("{}: {:?}", n.id, n.kind)); }

            ui.label("Links:");
            for l in &self.graph.links { ui.label(format!("{} -> {}", l.a, l.b)); }

            ui.horizontal(|ui| {
                ui.label("From idx:"); ui.add(egui::DragValue::new(&mut self.link_from).clamp_range(0..=100));
                ui.label("To idx:"); ui.add(egui::DragValue::new(&mut self.link_to).clamp_range(0..=100));
                if ui.button("Add Link").clicked() {
                    if self.link_from < self.graph.nodes.len() && self.link_to < self.graph.nodes.len() {
                        self.graph.add_link(self.link_from, self.link_to);
                    }
                }
            });

            ui.separator();
            if ui.button("Build Netlist & Run").clicked() {
                let net = self.graph.to_netlist();
                // choose monitor node: prefer node 2 if exists (first mem), else 0
                let monitor = if net.max_node >= 2 { 2 } else { 0 };
                // default mem id M1
                let mem_id = "M1".to_string();
                self.start_sim_from_netlist(net, monitor, mem_id);
            }
        });

        // Central: plot area
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                let available = ui.available_size();
                let width = available.x.max(320.0);
                let height = available.y.max(200.0).min(720.0);
                let (rect, _resp) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
                ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(24,26,30));

                if self.times.is_empty() {
                    ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, "No data yet", egui::FontId::proportional(18.0), egui::Color32::LIGHT_GRAY);
                } else {
                    let t_min = *self.times.first().unwrap_or(&0.0);
                    let t_max = *self.times.last().unwrap_or(&1.0);
                    let mut v_min = self.node2.iter().cloned().fold(f64::INFINITY, f64::min);
                    let mut v_max = self.node2.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    if !v_min.is_finite() || !v_max.is_finite() { v_min = 0.0; v_max = 1.0; }
                    let orig_span = v_max - v_min;
                    if orig_span.abs() < 1e-12 { let m = (v_max.abs().max(1.0)) * 1e-3; v_min -= m; v_max += m; } else { let span = v_max - v_min; v_min -= 0.06 * span; v_max += 0.06 * span; }
                    let v_range = (v_max - v_min).max(1e-9);
                    let t_range = (t_max - t_min).max(1e-12);

                    // vertical grid lines
                    for i in 0..=4 { let fx = i as f32 / 4.0; let x = rect.left() + fx * rect.width(); ui.painter().line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], egui::Stroke::new(1.0, egui::Color32::from_rgb(50,50,55))); }

                    let mut pts: Vec<egui::Pos2> = Vec::with_capacity(self.times.len());
                    for (t, v) in self.times.iter().zip(self.node2.iter()) {
                        let fx = ((*t - t_min) / t_range) as f32;
                        let fy = ((*v - v_min) / v_range) as f32;
                        let x = rect.left() + fx * rect.width();
                        let y = rect.bottom() - fy * rect.height();
                        pts.push(egui::pos2(x,y));
                    }

                    if pts.len() > 1 { ui.painter().add(egui::Shape::line(pts.clone(), egui::Stroke::new(3.0, egui::Color32::from_rgb(0,200,255)))); }
                    for p in pts.iter().step_by((pts.len()/120).max(1)) { ui.painter().circle_filled(*p, 3.0, egui::Color32::from_rgb(0,200,255)); }
                    if let (Some(f), Some(l)) = (pts.first(), pts.last()) { ui.painter().circle_filled(*f, 6.0, egui::Color32::from_rgb(0,255,0)); ui.painter().circle_filled(*l, 6.0, egui::Color32::from_rgb(255,0,0)); }

                    if orig_span.abs() < 1e-9 {
                        if let Some(&v0) = self.node2.first() {
                            let fy0 = ((v0 - v_min) / v_range) as f32;
                            let y0 = rect.bottom() - fy0 * rect.height();
                            ui.painter().line_segment([egui::pos2(rect.left(), y0), egui::pos2(rect.right(), y0)], egui::Stroke::new(4.0, egui::Color32::from_rgb(255,200,0)));
                            for p in pts.iter().step_by((pts.len()/40).max(1)) { ui.painter().circle_filled(*p, 6.0, egui::Color32::from_rgb(255,200,0)); }
                        }
                    }

                    let dbg_show = 12usize.min(self.times.len()); let mut y_off = rect.top() + 8.0;
                    for i in 0..dbg_show { let line = format!("{:.6}: {:.6e} st={:.3}", self.times[i], self.node2[i], self.m1_state[i]); ui.painter().text(rect.left_top() + egui::vec2(8.0, y_off - rect.top()), egui::Align2::LEFT_TOP, line, egui::FontId::proportional(12.0), egui::Color32::from_rgba_unmultiplied(220,220,220,230)); y_off += 14.0; }

                    ui.painter().text(rect.right_top() - egui::vec2(8.0, -4.0), egui::Align2::RIGHT_TOP, format!("max={:.6}", v_max), egui::FontId::proportional(12.0), egui::Color32::LIGHT_GRAY);
                    ui.painter().text(rect.right_bottom() - egui::vec2(8.0, 18.0), egui::Align2::RIGHT_BOTTOM, format!("min={:.6}", v_min), egui::FontId::proportional(12.0), egui::Color32::LIGHT_GRAY);
                }

                ui.painter().rect_stroke(rect, 2.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(80,80,90)));
            });
        });

        if self.running { ctx.request_repaint_after(std::time::Duration::from_millis(100)); }
    }
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    let _ = eframe::run_native("Memristor Simulator", native_options, Box::new(|_cc| Box::new(App::default())));
}
