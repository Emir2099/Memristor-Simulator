use eframe::egui;
use memristor_sim::three_view;
use memristor_sim::node_graph::{Graph, NodeKind};
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use std::thread;
use serde_json;
use std::fs;

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
    selected_link: Option<usize>,
    // last generated netlist preview and options
    last_net_preview: String,
    last_net_warnings: Vec<String>,
    last_net: Option<memristor_sim::mna::Netlist>,
    monitor_node: usize,
    mem_options: Vec<String>,
    selected_mem_idx: usize,
    // visual editor state
    node_positions: Vec<[f32;2]>,
    selected_node: Option<usize>,
    dragging_node: Option<usize>,
    drag_offset: egui::Vec2,
    // canvas transform
    pan: egui::Vec2,
    zoom: f32,
    // model inspector params (global defaults)
    vteam_von: f64,
    vteam_voff: f64,
    yak_a1: f64,
    yak_a2: f64,
    yak_b: f64,
    // model selection for adding nodes
    model_choices: Vec<String>,
    selected_model_choice: usize,
    // embedded 3D view renderer (optional)
    three_view: Option<three_view::ThreeViewRenderer>,
    // editor helpers
    connect_mode: bool,
    connect_src: Option<usize>,
    // dragging a link from a node's output pin
    link_dragging: bool,
}

impl Default for App {
    fn default() -> Self {
        let mut app = Self {
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
            selected_link: None,
            last_net_preview: String::new(),
            last_net_warnings: Vec::new(),
            last_net: None,
            monitor_node: 0,
            mem_options: Vec::new(),
            selected_mem_idx: 0,
            node_positions: Vec::new(),
            selected_node: None,
            dragging_node: None,
            drag_offset: egui::vec2(0.0,0.0),
            pan: egui::vec2(0.0, 0.0),
            zoom: 1.0,
            vteam_von: -1.0,
            vteam_voff: 1.0,
            yak_a1: 1e-3,
            yak_a2: 1e-3,
            yak_b: 1.0,
            model_choices: vec!["HP".into(), "VTEAM".into(), "YAKOPCIC".into()],
            selected_model_choice: 0,
            three_view: None,
            connect_mode: false,
            connect_src: None,
            link_dragging: false,
        };

        // try to load saved node positions
        app.load_layout();

        app
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut app = App::default();
        // initialize the three_view renderer using the full creation context (future GPU init)
        app.three_view = Some(three_view::ThreeViewRenderer::new_with_ctx(Some(cc)));
        app
    }
}

// Configure neuromorphic dark theme and styles
fn configure_styles(ctx: &egui::Context) {
    let mut style = (*ctx.style()).clone();
    let mut ts = style.text_styles.clone();
    ts.insert(egui::TextStyle::Heading, egui::FontId::new(20.0, egui::FontFamily::Proportional));
    ts.insert(egui::TextStyle::Body, egui::FontId::new(14.0, egui::FontFamily::Proportional));
    ts.insert(egui::TextStyle::Monospace, egui::FontId::new(12.0, egui::FontFamily::Monospace));
    ts.insert(egui::TextStyle::Button, egui::FontId::new(14.0, egui::FontFamily::Proportional));
    ts.insert(egui::TextStyle::Small, egui::FontId::new(11.0, egui::FontFamily::Proportional));
    style.text_styles = ts;
    style.visuals = egui::Visuals::dark();
    style.visuals.window_rounding = egui::Rounding::same(10.0);
    style.visuals.widgets.noninteractive.bg_fill = egui::Color32::from_rgb(20, 20, 25);
    style.visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(40, 40, 50);
    style.visuals.selection.bg_fill = egui::Color32::from_rgb(0, 180, 255);
    ctx.set_style(style);
}

impl App {
    fn save_layout(&self) {
        // persist both graph structure and node positions
        if self.node_positions.is_empty() { return; }
        #[derive(serde::Serialize)]
        struct Layout<'a> { graph: &'a memristor_sim::node_graph::Graph, positions: &'a Vec<[f32;2]> }
        let layout = Layout { graph: &self.graph, positions: &self.node_positions };
        if let Ok(s) = serde_json::to_string_pretty(&layout) { let _ = fs::write(".node_layout.json", s); }
    }

    fn load_layout(&mut self) {
        if let Ok(s) = fs::read_to_string(".node_layout.json") {
            #[derive(serde::Deserialize)]
            struct Layout { graph: memristor_sim::node_graph::Graph, positions: Vec<[f32;2]> }
            if let Ok(layout) = serde_json::from_str::<Layout>(&s) {
                self.graph = layout.graph;
                self.node_positions = layout.positions;
            }
        }
    }

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

        let mem_templ = memristor_sim::Memristor::new(memristor_sim::HpTiO2Model::new("M1", self.mem_ron, self.mem_roff, self.mem_state_init));
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

    // remove selected node (by selected_node) and persist layout
    fn delete_selected_node(&mut self) {
        if let Some(idx) = self.selected_node {
            self.graph.remove_node(idx);
            if idx < self.node_positions.len() { self.node_positions.remove(idx); }
            self.selected_node = None;
            self.save_layout();
        }
    }

    // remove selected link index
    fn delete_link_index(&mut self, link_idx: usize) {
        self.graph.remove_link(link_idx);
    }

    // populate a simple example: V1 -> R2 -> M1
    fn populate_example(&mut self) {
        self.graph.nodes.clear();
        self.graph.links.clear();
        self.node_positions.clear();
        // add vsource at idx 0
        self.graph.add_node(NodeKind::VSource { id: "v0".into(), amp: self.drive_amp, freq: self.drive_freq, is_sine: self.drive_is_sine });
        self.node_positions.push([40.0, 40.0]);
        // add memristor at idx 1
        self.graph.add_node(NodeKind::Memristor { id: "m0".into(), ron: self.mem_ron, roff: self.mem_roff, state: self.mem_state_init, mu0: self.mem_mu0, n: self.mem_n, window_p: self.mem_window_p, ithreshold: self.mem_ithreshold, model: "HP".into() });
        self.node_positions.push([220.0, 40.0]);
        // add link 0->1
        self.graph.add_link(0,1);
        self.save_layout();
        // auto-build preview
        let (preview, warnings) = self.graph.to_netlist_preview();
        let net = self.graph.to_netlist();
        let mut mems = Vec::new();
        for c in &net.comps { if let memristor_sim::mna::Component::Memristor { id, .. } = c { mems.push(id.clone()); } }
        self.mem_options = mems;
        self.monitor_node = if net.max_node >= 2 { 2 } else { 0 };
        self.selected_mem_idx = 0;
        self.last_net_preview = preview;
        self.last_net_warnings = warnings;
        self.last_net = Some(net);
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
        // Narrow the side panel slightly to give more space to the central canvas
        egui::SidePanel::left("left_panel").min_width(200.0).resizable(true).show(ctx, |ui| {
            ui.label("Last samples (time, node_2, state):");
            egui::ScrollArea::vertical().id_source("left_samples_scroll").max_height(280.0).show(ui, |ui| {
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
                // model selection combo box for new memristor nodes
                egui::ComboBox::from_id_source("model_choice").selected_text(self.model_choices[self.selected_model_choice].clone()).show_ui(ui, |ui| {
                    for (i, m) in self.model_choices.iter().enumerate() {
                        ui.selectable_value(&mut self.selected_model_choice, i, m);
                    }
                });
                if ui.button("Add Memristor").clicked() {
                    let chosen = self.model_choices[self.selected_model_choice].clone();
                    self.graph.add_node(NodeKind::Memristor { id: format!("m{}", self.graph.nodes.len()), ron: self.mem_ron, roff: self.mem_roff, state: self.mem_state_init, mu0: self.mem_mu0, n: self.mem_n, window_p: self.mem_window_p, ithreshold: self.mem_ithreshold, model: chosen });
                }
                if ui.button("Add VSource").clicked() {
                    self.graph.add_node(NodeKind::VSource { id: format!("v{}", self.graph.nodes.len()), amp: self.drive_amp, freq: self.drive_freq, is_sine: self.drive_is_sine });
                }
            });

            ui.label("Nodes:");
            for n in &self.graph.nodes { ui.label(format!("{}: {:?}", n.id, n.kind)); }

            ui.label("Links:");
            for (i, l) in self.graph.links.iter().enumerate() {
                let is_selected = self.selected_link == Some(i);
                if ui.selectable_label(is_selected, format!("{}: {} -> {}", i, l.a, l.b)).clicked() {
                    self.selected_link = Some(i);
                }
            }

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
                if ui.horizontal(|ui| ui.button("Build Netlist").clicked()).inner {
                // build netlist using inspector parameters
                let net = self.graph.to_netlist_with_params(self.vteam_von, self.vteam_voff, self.yak_a1, self.yak_a2, self.yak_b);
                // construct preview string
                let mut preview = String::new();
                for c in &net.comps { preview.push_str(&format!("{}", c)); }
                // simple validation warnings
                let mut warnings = Vec::new();
                for c in &net.comps {
                    if let memristor_sim::mna::Component::Resistor { id, n1, n2, .. } = c {
                        if n1 == n2 { warnings.push(format!("Resistor {} has identical terminals ({} == {})", id, n1, n2)); }
                    }
                }
                // populate mem options
                let mut mems = Vec::new();
                for c in &net.comps {
                    if let memristor_sim::mna::Component::Memristor { id, .. } = c {
                        mems.push(id.clone());
                    }
                }
                self.mem_options = mems;
                // choose default monitor node: first mem node if exists (node 2) else 0
                self.monitor_node = if net.max_node >= 2 { 2 } else { 0 };
                self.selected_mem_idx = 0;
                self.last_net_preview = preview;
                self.last_net_warnings = warnings;
                self.last_net = Some(net);
            }

                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Delete Selected Node").clicked() {
                        self.delete_selected_node();
                    }
                    if ui.button("Delete Selected Link").clicked() {
                        if let Some(link_idx) = self.selected_link { self.delete_link_index(link_idx); self.selected_link = None; }
                    }
                });

                if ui.button("Populate Example").clicked() {
                    self.populate_example();
                }

            if let Some(_) = &self.last_net {
                ui.separator();
                ui.label("Netlist preview:");
                egui::ScrollArea::vertical().id_source("netlist_preview_scroll").max_height(160.0).show(ui, |ui| {
                    ui.code(&self.last_net_preview);
                    if !self.last_net_warnings.is_empty() {
                        ui.separator();
                        for w in &self.last_net_warnings { ui.colored_label(egui::Color32::YELLOW, w); }
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Monitor node:");
                    // simple numeric selector for monitor node
                    ui.add(egui::DragValue::new(&mut self.monitor_node).clamp_range(0..=(self.last_net.as_ref().unwrap().max_node)));
                });

                ui.horizontal(|ui| {
                    ui.label("Memristor:");
                    if self.mem_options.is_empty() {
                        ui.label("(none)");
                    } else {
                        egui::ComboBox::from_id_source("mem_select").selected_text(self.mem_options[self.selected_mem_idx].clone()).show_ui(ui, |ui| {
                            for (i, id) in self.mem_options.iter().enumerate() {
                                ui.selectable_value(&mut self.selected_mem_idx, i, id);
                            }
                        });
                    }
                });

                if ui.button("Run Netlist").clicked() {
                    if let Some(net) = self.last_net.take() {
                        let mem_id = if self.mem_options.is_empty() { "M1".to_string() } else { self.mem_options[self.selected_mem_idx].clone() };
                        self.start_sim_from_netlist(net, self.monitor_node, mem_id);
                    }
                }
            }
        });

        // Central: visual editor (full width, plot moved to bottom panel)
        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_size();
            ui.horizontal(|ui| {
                ui.heading("Visual Node Editor");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label("Zoom:");
                    if ui.small_button("-").clicked() { self.zoom = (self.zoom * 0.9).max(0.3); }
                    ui.add_sized(egui::vec2(80.0, 20.0), egui::Slider::new(&mut self.zoom, 0.3..=2.0).show_value(false));
                    if ui.small_button("+").clicked() { self.zoom = (self.zoom * 1.1).min(2.0); }
                    ui.add_space(10.0);
                    if ui.small_button("Reset View").clicked() { self.pan = egui::vec2(0.0, 0.0); self.zoom = 1.0; }
                    ui.add_space(6.0);
                    ui.checkbox(&mut self.connect_mode, "Connect");
                });
            });
            ui.add_space(4.0);

            let height = (available.y - 40.0).max(300.0);
            let (editor_rect, _resp) = ui.allocate_exact_size(egui::vec2(available.x, height), egui::Sense::click_and_drag());
            
            // draw blueprint grid background (respect pan/zoom)
            let grid_color = egui::Color32::from_rgb(30, 36, 44);
            ui.painter().rect_filled(editor_rect, 4.0, egui::Color32::from_rgb(18,18,22));
            let grid_spacing = (28.0 * self.zoom).max(8.0);
            let left = editor_rect.left(); 
            let top = editor_rect.top();
            let right = editor_rect.right(); 
            let bottom = editor_rect.bottom();
            // vertical lines
            let pan_x = self.pan.x % grid_spacing;
            let pan_y = self.pan.y % grid_spacing;
            let mut x = left - pan_x;
            while x < right {
                ui.painter().line_segment([egui::pos2(x, top), egui::pos2(x, bottom)], egui::Stroke::new(1.0, grid_color));
                x += grid_spacing;
            }
            // horizontal lines
            let mut y = top - pan_y;
            while y < bottom {
                ui.painter().line_segment([egui::pos2(left, y), egui::pos2(right, y)], egui::Stroke::new(1.0, grid_color));
                y += grid_spacing;
            }

            // ensure node_positions matches nodes
                    while self.node_positions.len() < self.graph.nodes.len() { let idx = self.node_positions.len(); self.node_positions.push([20.0 + (idx as f32)*140.0, 20.0 + ((idx/4) as f32)*100.0]); }

            let editor_resp = _resp;
            let pointer_pos = editor_resp.hover_pos();
            let mut released_over: Option<usize> = None;
            let mut last_dragged: Option<usize> = None;

            for (i, n) in self.graph.nodes.iter().enumerate() {
                let pos = self.node_positions[i];
                let node_size = egui::vec2(120.0, 60.0) * self.zoom;
                let top_left = editor_rect.left_top() + egui::vec2(pos[0] * self.zoom + self.pan.x, pos[1] * self.zoom + self.pan.y);
                let node_rect = egui::Rect::from_min_size(top_left, node_size);

                // draw node background and border (highlight if selected)
                let selected = self.selected_node == Some(i);
                // draw shadow
                ui.painter().rect_filled(node_rect.shrink(-6.0), 10.0, egui::Color32::from_rgba_unmultiplied(0,0,0,180));
                let bg = if selected { egui::Color32::from_rgb(38,48,64) } else { egui::Color32::from_rgb(34,36,40) };
                let stroke_col = if selected { egui::Color32::from_rgb(0,180,255) } else { egui::Color32::from_rgb(90,100,110) };
                let stroke_w = if selected { 2.5 } else { 1.0 };
                // header
                let header_h = 18.0 * self.zoom;
                let header_rect = egui::Rect::from_min_max(node_rect.min, egui::pos2(node_rect.max.x, node_rect.min.y + header_h));
                ui.painter().rect_filled(node_rect, 8.0, bg);
                ui.painter().rect_filled(header_rect, 8.0, egui::Color32::from_rgb(0, 120, 160));
                ui.painter().rect_stroke(node_rect, 8.0, egui::Stroke::new(stroke_w, stroke_col));
                ui.painter().text(header_rect.center() + egui::vec2(0.0,0.0), egui::Align2::CENTER_CENTER, format!("{}", i), egui::FontId::proportional(12.0 * self.zoom), egui::Color32::WHITE);

                // draw small pins on left (input) and right (output)
                let left_pin = egui::pos2(node_rect.left() + 8.0 * self.zoom, node_rect.center().y);
                let right_pin = egui::pos2(node_rect.right() - 8.0 * self.zoom, node_rect.center().y);
                ui.painter().circle_filled(left_pin, 6.0, egui::Color32::from_rgb(200,200,200));
                ui.painter().circle_filled(right_pin, 6.0, egui::Color32::from_rgb(200,200,200));

                // If this node is the current connect source, draw an outline to indicate it
                if self.connect_src == Some(i) {
                    ui.painter().rect_stroke(node_rect.shrink(-2.0), 10.0, egui::Stroke::new(3.0, egui::Color32::from_rgb(0,200,120)));
                }

                // icon/text per kind
                match &n.kind {
                    memristor_sim::node_graph::NodeKind::VSource { id, .. } => {
                        // draw a small circle icon with + sign
                        let icon_center = node_rect.center() - egui::vec2(0.0, 4.0);
                        ui.painter().circle_filled(icon_center, 14.0, egui::Color32::from_rgb(220,180,80));
                        ui.painter().text(icon_center, egui::Align2::CENTER_CENTER, "+", egui::FontId::proportional(14.0), egui::Color32::BLACK);
                        ui.painter().text(node_rect.center() + egui::vec2(0.0,18.0), egui::Align2::CENTER_TOP, format!("V: {}", id), egui::FontId::proportional(12.0), egui::Color32::LIGHT_GRAY);
                    }
                    memristor_sim::node_graph::NodeKind::Memristor { id, .. } => {
                        // draw a zig-zag-like icon (approx with polyline)
                        let ic = node_rect.center() - egui::vec2(0.0, 4.0);
                        let pts = [ic + egui::vec2(-14.0 * self.zoom, -6.0 * self.zoom), ic + egui::vec2(-6.0 * self.zoom, 6.0 * self.zoom), ic + egui::vec2(0.0, -6.0 * self.zoom), ic + egui::vec2(6.0 * self.zoom, 6.0 * self.zoom), ic + egui::vec2(14.0 * self.zoom, -6.0 * self.zoom)];
                        ui.painter().line_segment([pts[0], pts[1]], egui::Stroke::new(2.0, egui::Color32::from_rgb(100,200,180)));
                        ui.painter().line_segment([pts[1], pts[2]], egui::Stroke::new(2.0, egui::Color32::from_rgb(100,200,180)));
                        ui.painter().line_segment([pts[2], pts[3]], egui::Stroke::new(2.0, egui::Color32::from_rgb(100,200,180)));
                        ui.painter().line_segment([pts[3], pts[4]], egui::Stroke::new(2.0, egui::Color32::from_rgb(100,200,180)));
                        ui.painter().text(node_rect.center() + egui::vec2(0.0,18.0), egui::Align2::CENTER_TOP, format!("M: {}", id), egui::FontId::proportional(12.0), egui::Color32::LIGHT_GRAY);
                    }
                }

                // detect drag start on editor area
                let inside = pointer_pos.map_or(false, |p| node_rect.contains(p));
                if editor_resp.drag_started() && inside {
                    // If drag started near the right pin, interpret as "link drag" (create link by dragging out)
                    if let Some(pp) = pointer_pos {
                        // compute actual distance to right_pin
                        let dx = pp.x - right_pin.x;
                        let dy = pp.y - right_pin.y;
                        let dist = (dx*dx + dy*dy).sqrt();
                        if dist < 14.0 * self.zoom {
                            // begin link dragging from this node; record it as last_dragged for later link creation
                            self.link_dragging = true;
                            last_dragged = Some(i);
                        } else {
                            self.dragging_node = Some(i);
                            self.drag_offset = pp.to_vec2() - top_left.to_vec2();
                        }
                    }
                }
                
                // Handle pan with middle mouse or space+drag
                if editor_resp.drag_started_by(egui::PointerButton::Middle) || 
                   (ctx.input(|i| i.key_pressed(egui::Key::Space)) && editor_resp.drag_started()) {
                    if let Some(pp) = pointer_pos {
                        self.pan = pp.to_vec2() - editor_rect.left_top().to_vec2();
                    }
                }

                if let Some(dn) = self.dragging_node {
                    if dn == i {
                        let delta = editor_resp.drag_delta();
                        if delta != egui::Vec2::ZERO {
                            // move node by drag delta (account for zoom)
                            let mut np = egui::pos2(self.node_positions[i][0], self.node_positions[i][1]);
                            np += delta / self.zoom;
                            self.node_positions[i] = [np.x.max(0.0), np.y.max(0.0)];
                        }
                        if editor_resp.drag_stopped() {
                            // pointer released - check for drop over other node
                            if let Some(pp) = pointer_pos {
                                for (j, _m) in self.graph.nodes.iter().enumerate() {
                                    if j != i {
                                        let other_top = editor_rect.left_top() + egui::vec2(self.node_positions[j][0] * self.zoom + self.pan.x, self.node_positions[j][1] * self.zoom + self.pan.y);
                                        let other_rect = egui::Rect::from_min_size(other_top, egui::vec2(120.0,60.0) * self.zoom);
                                        if other_rect.contains(pp) {
                                            released_over = Some(j);
                                            break;
                                        }
                                    }
                                }
                            }
                            // remember which node was dragged before clearing
                            last_dragged = Some(i);
                            self.dragging_node = None;
                        }
                    }
                }
                // If link-dragging was started from this node, handle drag stop here as well
                if self.link_dragging && editor_resp.drag_stopped() {
                    // if released over some other node, create a link
                    if let Some(pp) = pointer_pos {
                        for (j, _m) in self.graph.nodes.iter().enumerate() {
                            if j != i {
                                let other_top = editor_rect.left_top() + egui::vec2(self.node_positions[j][0] * self.zoom + self.pan.x, self.node_positions[j][1] * self.zoom + self.pan.y);
                                let other_rect = egui::Rect::from_min_size(other_top, egui::vec2(120.0,60.0) * self.zoom);
                                if other_rect.contains(pp) {
                                    released_over = Some(j);
                                    break;
                                }
                            }
                        }
                    }
                    // link creation will be handled after the loop using last_dragged and released_over
                    self.link_dragging = false;
                }
            }

            // draw links between centers with cubic bezier curves, glow, and labels
            for (li, l) in self.graph.links.iter().enumerate() {
                if l.a < self.node_positions.len() && l.b < self.node_positions.len() {
                    let a_pos = editor_rect.left_top() + egui::vec2(self.node_positions[l.a][0] * self.zoom + self.pan.x + 60.0 * self.zoom, self.node_positions[l.a][1] * self.zoom + self.pan.y + 30.0 * self.zoom);
                    let b_pos = editor_rect.left_top() + egui::vec2(self.node_positions[l.b][0] * self.zoom + self.pan.x + 60.0 * self.zoom, self.node_positions[l.b][1] * self.zoom + self.pan.y + 30.0 * self.zoom);
                    // determine if this link is selected
                    let mut col = egui::Color32::from_rgb(200,200,90);
                    let mut w: f32 = 2.0;
                    // highlight selected link
                    if self.selected_link == Some(li) { col = egui::Color32::from_rgb(255,180,40); w = 3.5; }

                    // hover detection: compute distance and a hover factor [0..1]
                    let mut hover_factor: f32 = 0.0;
                    let mut t_on_seg: f32 = 0.5;
                    if let Some(pp) = pointer_pos {
                        let seg = b_pos - a_pos;
                        let seg_len2 = seg.x*seg.x + seg.y*seg.y;
                        if seg_len2 > 1e-6 {
                            let t = (((pp.x - a_pos.x) * seg.x + (pp.y - a_pos.y) * seg.y) / seg_len2) as f32;
                            let t_clamped = t.clamp(0.0, 1.0);
                            t_on_seg = t_clamped;
                            let proj = a_pos + seg * t_clamped;
                            let dx = pp.x - proj.x; let dy = pp.y - proj.y;
                            let dist2 = dx*dx + dy*dy;
                            let thresh2 = 100.0; // 10px radius
                            if dist2 < thresh2 {
                                hover_factor = (1.0 - (dist2 / thresh2) as f32).clamp(0.0, 1.0);
                            }
                        }
                    }

                    // Only animate if link is selected or hovered (performance optimization)
                    let strength = if self.selected_link == Some(li) || hover_factor > 0.1 {
                        let time = ctx.input(|i| i.time);
                        let pulse = (0.5 * (1.0 + (time * 6.0).sin())) as f32;
                        let sel_factor = if self.selected_link == Some(li) { 1.0 } else { 0.0 };
                        (0.15 * sel_factor) + (hover_factor * 0.85 * pulse)
                    } else {
                        0.0
                    };

                    // glow (draw thicker translucent bezier behind)
                    let dir = b_pos - a_pos;
                    let c1 = a_pos + egui::vec2(dir.x * 0.25, dir.y * 0.0);
                    let c2 = b_pos - egui::vec2(dir.x * 0.25, dir.y * 0.0);
                    // Optimize bezier curve sampling - fewer points when zoomed out
                    let steps = (16.0 * self.zoom.min(1.5)).max(8.0) as usize;
                    let mut bezier_pts: Vec<egui::Pos2> = Vec::with_capacity(steps + 1);
                    for i in 0..=steps {
                        let t = i as f32 / steps as f32;
                        let it = 1.0 - t;
                        let p = a_pos.to_vec2() * it*it*it
                            + c1.to_vec2() * 3.0*it*it*t
                            + c2.to_vec2() * 3.0*it*t*t
                            + b_pos.to_vec2() * t*t*t;
                        bezier_pts.push(egui::pos2(p.x, p.y));
                    }
                    if strength > 0.01 {
                        let glow_col = egui::Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), (80.0 * strength) as u8);
                        ui.painter().add(egui::Shape::line(bezier_pts.clone(), egui::Stroke::new(w + 8.0 * strength, glow_col)));
                    }
                    let main_w = w + 2.0 * strength;
                    ui.painter().add(egui::Shape::line(bezier_pts.clone(), egui::Stroke::new(main_w, col)));

                    // draw arrowhead as small triangle
                    let len = (dir.x*dir.x + dir.y*dir.y).sqrt().max(1.0);
                    let nd = dir / len;
                    let perp = egui::vec2(-nd.y, nd.x);
                    let ah = 8.0f32 * self.zoom; // arrowhead length
                    let p1 = b_pos - nd * ah + perp * (ah * 0.5);
                    let p2 = b_pos - nd * ah - perp * (ah * 0.5);
                    ui.painter().add(egui::Shape::convex_polygon(vec![b_pos, p1, p2], col, egui::Stroke::new(0.0, col)));

                    // label at midpoint offset by perpendicular direction
                    let seg = b_pos - a_pos;
                    let midpoint = a_pos + seg * t_on_seg;
                    let label_pos = midpoint + perp * 14.0;
                    let label = format!("{}→{} ({})", l.a, l.b, li);
                    ui.painter().text(label_pos, egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(12.0), egui::Color32::LIGHT_GRAY);
                }
            }

            // If currently dragging a link from a node's output, draw a temporary preview from source -> cursor
            if self.link_dragging {
                if let Some(src) = last_dragged {
                    if src < self.node_positions.len() {
                        let a_pos = editor_rect.left_top() + egui::vec2(self.node_positions[src][0] * self.zoom + self.pan.x + 60.0 * self.zoom, self.node_positions[src][1] * self.zoom + self.pan.y + 30.0 * self.zoom);
                        if let Some(pp) = pointer_pos {
                            // simple straight preview line with arrow
                            ui.painter().line_segment([a_pos, pp], egui::Stroke::new(2.5, egui::Color32::from_rgb(220,200,60)));
                            // small arrow at tip
                            let dir = pp - a_pos;
                            let len = (dir.x*dir.x + dir.y*dir.y).sqrt().max(1.0);
                            let nd = dir / len;
                            let perp = egui::vec2(-nd.y, nd.x);
                            let ah = 8.0f32 * self.zoom;
                            let p1 = pp - nd * ah + perp * (ah * 0.5);
                            let p2 = pp - nd * ah - perp * (ah * 0.5);
                            ui.painter().add(egui::Shape::convex_polygon(vec![pp, p1, p2], egui::Color32::from_rgb(220,200,60), egui::Stroke::new(0.0, egui::Color32::from_rgb(220,200,60))));
                        }
                    }
                }
            }

            if let Some(j) = released_over {
                // create link from last-dragged node -> j (fall back to selected_node)
                if let Some(i) = last_dragged.or(self.selected_node) {
                    if i != j { self.graph.add_link(i, j); }
                }
            }

            // Handle mouse wheel zoom
            let scroll_delta = editor_resp.hover_pos().map_or(0.0, |_| {
                ctx.input(|i| i.raw_scroll_delta.y)
            });
            if scroll_delta != 0.0 {
                let zoom_factor = if scroll_delta > 0.0 { 1.1 } else { 0.9 };
                self.zoom = (self.zoom * zoom_factor).clamp(0.3, 2.0);
            }
            
            // click selection / connect-mode: if editor area was clicked (no drag), set selected_node or handle connect flow
            if editor_resp.clicked() && self.dragging_node.is_none() {
                if let Some(pp) = pointer_pos {
                    let mut found = false;
                    for (i, _n) in self.graph.nodes.iter().enumerate() {
                        let top = editor_rect.left_top() + egui::vec2(self.node_positions[i][0] * self.zoom + self.pan.x, self.node_positions[i][1] * self.zoom + self.pan.y);
                        let rect = egui::Rect::from_min_size(top, egui::vec2(120.0,60.0) * self.zoom);
                        if rect.contains(pp) {
                            found = true;
                            // If connect mode is enabled, perform click-to-connect: click first node to select source, click second to create link
                            if self.connect_mode {
                                if let Some(src) = self.connect_src {
                                    if src != i {
                                        self.graph.add_link(src, i);
                                    }
                                    self.connect_src = None; // reset after connecting
                                } else {
                                    self.connect_src = Some(i);
                                }
                                self.selected_node = Some(i);
                            } else {
                                // normal selection
                                self.selected_node = Some(i);
                            }
                            break;
                        }
                    }
                    if !found {
                        // clicked empty space - clear selection or cancel connect source depending on mode
                        if self.connect_mode { self.connect_src = None; }
                        else { self.selected_node = None; }
                    }
                }
            }
        });

        // Right-side inspector (3D view + params)
        egui::SidePanel::right("inspector").min_width(300.0).resizable(true).show(ctx, |ui| {
            ui.heading("Inspector");
            ui.separator();
            // Embedded 3D view (fallback CPU image if no GPU renderer available)
            let state_s = self.m1_state.last().copied().unwrap_or(self.mem_state_init) as f32;
            let view_size = egui::vec2(260.0, 160.0);
            let (rect, _resp) = ui.allocate_exact_size(view_size, egui::Sense::hover());
            ui.painter().rect_filled(rect, 8.0, egui::Color32::from_rgb(16,16,20));
            if let Some(rend) = &mut self.three_view {
                let tid = rend.ensure_texture(ui.ctx(), state_s, view_size);
                let uv = egui::Rect::from_min_max(egui::pos2(0.0,0.0), egui::pos2(1.0,1.0));
                ui.painter().add(egui::Shape::image(tid, rect, uv, egui::Color32::WHITE));
            } else {
                // fallback: simple painted cylinder like before
                let r = (255.0 * state_s) as u8;
                let g = (200.0 * state_s) as u8;
                let color = egui::Color32::from_rgb(r, g, 0);
                let body_rect = egui::Rect::from_min_max(rect.left_top() + egui::vec2(40.0, 30.0), rect.right_bottom() - egui::vec2(40.0, 30.0));
                ui.painter().rect_filled(body_rect, 8.0, color);
                let top_center = egui::pos2((body_rect.left() + body_rect.right()) * 0.5, body_rect.top());
                let bottom_center = egui::pos2((body_rect.left() + body_rect.right()) * 0.5, body_rect.bottom());
                let top_rect = egui::Rect::from_center_size(top_center, egui::vec2(body_rect.width()*0.9, 12.0));
                let bottom_rect = egui::Rect::from_center_size(bottom_center, egui::vec2(body_rect.width()*0.9, 12.0));
                ui.painter().rect_filled(top_rect, 12.0, color);
                ui.painter().rect_filled(bottom_rect, 12.0, egui::Color32::from_rgba_unmultiplied((r/2).max(10), (g/2).max(10), 0, 200));
            }
            ui.separator();
            ui.label("VTEAM params:");
            ui.add(egui::Slider::new(&mut self.vteam_von, -5.0..=0.0).text("v_on"));
            ui.add(egui::Slider::new(&mut self.vteam_voff, 0.0..=5.0).text("v_off"));
            ui.separator();
            ui.label("Yakopcic params:");
            ui.add(egui::Slider::new(&mut self.yak_a1, 0.0..=1e-1).text("a1"));
            ui.add(egui::Slider::new(&mut self.yak_a2, 0.0..=1e-1).text("a2"));
            ui.add(egui::Slider::new(&mut self.yak_b, 0.0..=10.0).text("b"));
        });

        // Bottom oscilloscope (full-width, improved layout)
        egui::TopBottomPanel::bottom("oscilloscope").min_height(180.0).resizable(true).show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Oscilloscope");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("Clear").clicked() {
                        self.times.clear();
                        self.node2.clear();
                        self.m1_state.clear();
                    }
                });
            });
            ui.separator();
            let available = ui.available_size();
            let rect = ui.allocate_exact_size(egui::vec2(available.x, (available.y - 30.0).max(150.0)), egui::Sense::hover()).0;
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

                // Draw grid (optimized - fewer lines)
                let grid_lines = 8;
                for i in 0..=grid_lines { 
                    let fx = i as f32 / grid_lines as f32; 
                    let x = rect.left() + fx * rect.width(); 
                    ui.painter().line_segment(
                        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], 
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(40,40,45))
                    ); 
                }
                // Horizontal grid lines
                for i in 0..=4 {
                    let fy = i as f32 / 4.0;
                    let y = rect.top() + fy * rect.height();
                    ui.painter().line_segment(
                        [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], 
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(40,40,45))
                    );
                }

                // Optimize plotting - sample points if too many
                let max_points = 2000;
                let step = if self.times.len() > max_points { 
                    self.times.len() / max_points 
                } else { 
                    1 
                };
                
                let mut pts: Vec<egui::Pos2> = Vec::with_capacity((self.times.len() / step).min(max_points));
                for (idx, (t, v)) in self.times.iter().zip(self.node2.iter()).enumerate() {
                    if idx % step == 0 {
                        let fx = ((*t - t_min) / t_range) as f32;
                        let fy = ((*v - v_min) / v_range) as f32;
                        let x = rect.left() + fx * rect.width();
                        let y = rect.bottom() - fy * rect.height();
                        pts.push(egui::pos2(x,y));
                    }
                }
                if pts.len() > 1 { 
                    ui.painter().add(egui::Shape::line(pts, egui::Stroke::new(2.5, egui::Color32::from_rgb(0,200,255)))); 
                }
                
                // Draw state curve if available
                if !self.m1_state.is_empty() {
                    let mut state_pts: Vec<egui::Pos2> = Vec::with_capacity((self.m1_state.len() / step).min(max_points));
                    let s_min = 0.0;
                    let s_max = 1.0;
                    let s_range = f64::max(s_max - s_min, 1e-9);
                    for (idx, s) in self.m1_state.iter().enumerate() {
                        if idx % step == 0 && idx < self.times.len() {
                            let t = self.times[idx];
                            let fx = ((t - t_min) / t_range) as f32;
                            let fy = ((*s - s_min) / s_range) as f32;
                            let x = rect.left() + fx * rect.width();
                            let y = rect.bottom() - fy * rect.height();
                            state_pts.push(egui::pos2(x, y));
                        }
                    }
                    if state_pts.len() > 1 {
                        ui.painter().add(egui::Shape::line(state_pts, egui::Stroke::new(2.0, egui::Color32::from_rgb(255,180,40))));
                    }
                }
            }
        });

        // Only request repaint when actually needed (simulation running or user interaction)
        if self.running { 
            ctx.request_repaint_after(std::time::Duration::from_millis(50)); // Reduced from 100ms
        }
    }
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    let _ = eframe::run_native("Memristor Simulator", native_options, Box::new(|cc| { configure_styles(&cc.egui_ctx); Box::new(App::new(cc)) }));
}
