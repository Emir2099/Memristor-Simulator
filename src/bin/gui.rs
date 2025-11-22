use eframe::egui;

struct App {
    output: String,
}

impl Default for App {
    fn default() -> Self {
        Self { output: String::new() }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Memristor Simulator (native GUI)");
            if ui.button("Run Simulation").clicked() {
                self.output = String::from("Running...");
                // Run the simulator (in-process)
                let csv = memristor_sim::run_mna_example_csv(1e-3, 1e-6);
                self.output = csv;
            }
            ui.separator();
            ui.label("Output (first 2000 chars):");
            egui::ScrollArea::vertical().show(ui, |ui| {
                let display = if self.output.len() > 2000 { &self.output[..2000] } else { &self.output };
                ui.monospace(display);
            });
        });
    }
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    let _ = eframe::run_native("Memristor Simulator", native_options, Box::new(|_cc| Box::new(App::default())));
}
