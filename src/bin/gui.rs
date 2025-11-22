use eframe::egui;
use std::sync::mpsc::{channel, Receiver};
use std::thread;

struct App {
    // simulation parameters
    tstop: f64,
    dt: f64,
    // state
    running: bool,
    // receiver for background thread result
    rx: Option<Receiver<String>>,
    // latest CSV output
    csv: String,
    // parsed series for plotting (time, node2 voltage)
    times: Vec<f64>,
    node2: Vec<f64>,
    m1_state: Vec<f64>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            tstop: 1e-3,
            dt: 1e-6,
            running: false,
            rx: None,
            csv: String::new(),
            times: Vec::new(),
            node2: Vec::new(),
            m1_state: Vec::new(),
        }
    }
}

impl App {
    fn start_sim(&mut self) {
        if self.running {
            return;
        }
        self.running = true;
        self.csv.clear();
        self.times.clear();
        self.node2.clear();
        self.m1_state.clear();

        let tstop = self.tstop;
        let dt = self.dt;

        let (tx, rx) = channel();
        self.rx = Some(rx);

        thread::spawn(move || {
            // run simulator in background
            let csv = memristor_sim::run_mna_example_csv(tstop, dt);
            // send CSV back to UI thread
            let _ = tx.send(csv);
        });
    }

    fn try_collect_result(&mut self) {
        if let Some(rx) = &self.rx {
            match rx.try_recv() {
                Ok(csv) => {
                    self.csv = csv;
                    self.parse_csv_for_plot();
                    self.running = false;
                    self.rx = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    // still running
                }
                Err(_) => {
                    self.running = false;
                    self.rx = None;
                }
            }
        }
    }

    fn parse_csv_for_plot(&mut self) {
        self.times.clear();
        self.node2.clear();
        self.m1_state.clear();

        for (i, line) in self.csv.lines().enumerate() {
            if i == 0 { continue; } // skip header
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 5 {
                if let (Ok(t), Ok(_n0), Ok(_n1), Ok(n2), Ok(s)) = (
                    parts[0].parse::<f64>(),
                    parts[1].parse::<f64>(),
                    parts[2].parse::<f64>(),
                    parts[3].parse::<f64>(),
                    parts[4].parse::<f64>(),
                ) {
                    self.times.push(t);
                    self.node2.push(n2);
                    self.m1_state.push(s);
                }
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // poll for background result
        self.try_collect_result();

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Memristor Simulator (native GUI)");

            ui.horizontal(|ui| {
                ui.label("tstop (s):");
                ui.add(egui::DragValue::new(&mut self.tstop).speed(1e-6));
                ui.label("dt (s):");
                ui.add(egui::DragValue::new(&mut self.dt).speed(1e-7));
                if ui.add_enabled(!self.running, egui::Button::new("Run")).clicked() {
                    self.start_sim();
                }
                if ui.add_enabled(self.running, egui::Button::new("Stop")).clicked() {
                    // stopping thread gracefully isn't implemented; just mark as not running
                    // TODO: add cancellation support in simulator
                    self.running = false;
                    self.rx = None;
                }
                if ui.button("Save CSV").clicked() {
                    if !self.csv.is_empty() {
                        if let Some(path) = rfd::FileDialog::new().save_file() {
                            if let Err(e) = std::fs::write(&path, &self.csv) {
                                eprintln!("Failed to save CSV: {}", e);
                            }
                        }
                    }
                }
            });

            ui.separator();

            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label(format!("Status: {}", if self.running { "Running..." } else { "Idle" }));
                    ui.label(format!("Samples: {}", self.times.len()));
                    egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
                        ui.monospace(if self.csv.len() > 2000 { &self.csv[..2000] } else { &self.csv });
                    });
                });

                // improved plotting area using painter: background, grid, axes, two traces
                let (rect, _resp) = ui.allocate_exact_size(egui::vec2(700.0, 360.0), egui::Sense::hover());
                // dark plot background to make lines visible
                ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(24, 26, 30));

                if self.times.is_empty() {
                    ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, "No data yet", egui::FontId::proportional(18.0), egui::Color32::LIGHT_GRAY);
                } else {
                    let t_min = *self.times.first().unwrap_or(&0.0);
                    let t_max = *self.times.last().unwrap_or(&1.0);
                    let v_min = self.node2.iter().cloned().fold(f64::INFINITY, f64::min);
                    let v_max = self.node2.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    let v_min = if v_min.is_finite() { v_min } else { 0.0 };
                    let v_max = if v_max.is_finite() { v_max } else { 1.0 };
                    let v_range = (v_max - v_min).max(1e-12);
                    let t_range = (t_max - t_min).max(1e-12);

                    // draw grid lines (5 vertical, 4 horizontal)
                    for i in 0..=5 {
                        let fx = i as f32 / 5.0;
                        let x = rect.left() + fx * rect.width();
                        ui.painter().line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], egui::Stroke::new(1.0, egui::Color32::from_rgb(50,50,55)));
                    }
                    for i in 0..=4 {
                        let fy = i as f32 / 4.0;
                        let y = rect.top() + fy * rect.height();
                        ui.painter().line_segment([egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], egui::Stroke::new(1.0, egui::Color32::from_rgb(50,50,55)));
                    }

                    // draw node2 trace (bright cyan)
                    if self.times.len() > 1 {
                        let mut points: Vec<egui::Pos2> = Vec::with_capacity(self.times.len());
                        for (t, v) in self.times.iter().zip(self.node2.iter()) {
                            let fx = ((*t - t_min) / t_range) as f32;
                            let fy = ((*v - v_min) / v_range) as f32;
                            let x = rect.left() + fx * rect.width();
                            let y = rect.bottom() - fy * rect.height();
                            points.push(egui::pos2(x, y));
                        }
                        ui.painter().add(egui::Shape::line(points.clone(), egui::Stroke::new(2.5, egui::Color32::from_rgb(0, 200, 255))));
                    }

                    // draw M1 state trace (orange) on secondary axis normalized 0..1
                    if self.times.len() > 1 && !self.m1_state.is_empty() {
                        let mut state_points: Vec<egui::Pos2> = Vec::with_capacity(self.m1_state.len());
                        for (t, s) in self.times.iter().zip(self.m1_state.iter()) {
                            let fx = ((*t - t_min) / t_range) as f32;
                            let fy = *s as f32; // states already 0..1
                            let x = rect.left() + fx * rect.width();
                            // map state to lower part of plot (e.g., bottom 25%) so it doesn't overlap voltage
                            let y = rect.bottom() - (0.2 * fy) * rect.height();
                            state_points.push(egui::pos2(x, y));
                        }
                        ui.painter().add(egui::Shape::line(state_points.clone(), egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 165, 0))));
                    }

                    // axes labels and ticks
                    let label_color = egui::Color32::LIGHT_GRAY;
                    ui.painter().text(rect.left_top() + egui::vec2(6.0, 6.0), egui::Align2::LEFT_TOP, "node_2 (V)", egui::FontId::proportional(14.0), label_color);
                    ui.painter().text(rect.right_bottom() - egui::vec2(60.0, 20.0), egui::Align2::LEFT_BOTTOM, format!("t [{:.3}..{:.3}]s", t_min, t_max), egui::FontId::proportional(12.0), label_color);
                    // legend
                    let legend_pos = rect.left_top() + egui::vec2(8.0, 28.0);
                    let legend_rect = egui::Rect::from_min_size(legend_pos, egui::vec2(140.0, 20.0));
                    ui.painter().rect_filled(legend_rect, egui::Rounding::same(4.0), egui::Color32::from_rgba_unmultiplied(10,10,10,180));
                    ui.painter().text(legend_pos + egui::vec2(6.0, 2.0), egui::Align2::LEFT_TOP, "● node_2", egui::FontId::proportional(12.0), egui::Color32::from_rgb(0,200,255));
                    ui.painter().text(legend_pos + egui::vec2(86.0, 2.0), egui::Align2::LEFT_TOP, "● M1_state", egui::FontId::proportional(12.0), egui::Color32::from_rgb(255,165,0));
                }
                // border
                ui.painter().rect_stroke(rect, 2.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(80,80,90)));
            });
        });
        // request repaint while running to animate status
        if self.running {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
    }
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    let _ = eframe::run_native("Memristor Simulator", native_options, Box::new(|_cc| Box::new(App::default())));
}
