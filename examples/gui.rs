// GUI example (not built by default).
// To enable and run this, add `eframe = "0.24"` to `[dependencies]` in Cargo.toml
// and run `cargo run --example gui`.

use eframe::{egui, epi};

struct App {
    last_run: f64,
}

impl Default for App {
    fn default() -> Self { Self { last_run: 0.0 } }
}

impl epi::App for App {
    fn name(&self) -> &str { "Memristor Simulator GUI (example)" }

    fn update(&mut self, ctx: &egui::Context, _frame: &epi::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Memristor Simulator (example)");
            ui.label(format!("Last run: {:.6}", self.last_run));
            if ui.button("Run single step").clicked() {
                self.last_run += 1.0;
            }
            ui.separator();
            ui.label("This is a UI scaffold. Add eframe dependency to run.");
        });
    }
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native("Memristor Simulator", native_options, Box::new(|_cc| Box::new(App::default())));
}
