use eframe::egui;

pub struct ThreeViewRenderer {
    texture: Option<egui::TextureHandle>,
    last_size: [usize; 2],
    rotation: f32,
}

impl ThreeViewRenderer {
    pub fn new(_ctx: &egui::Context) -> Self {
        Self { texture: None, last_size: [0, 0], rotation: 0.0 }
    }

    fn generate_image(s: f32, w: usize, h: usize) -> egui::ColorImage {
        let mut pixels: Vec<egui::Color32> = Vec::with_capacity(w * h);
        // color mapping: s=0 -> dark red, s=1 -> bright yellow
        let r = (255.0 * s).clamp(0.0, 255.0) as u8;
        let g = (200.0 * s).clamp(0.0, 255.0) as u8;
        let base = egui::Color32::from_rgb(r, g, 24);

        // simple CPU fallback: draw gradient background + bright square in center representing a cube
        for y in 0..h {
            for x in 0..w {
                let fx = x as f32 / (w as f32).max(1.0);
                let fy = y as f32 / (h as f32).max(1.0);
                // slight radial darkening
                let dx = fx - 0.5;
                let dy = fy - 0.5;
                let dist = (dx*dx + dy*dy).sqrt();
                let dark = (1.0 - dist).clamp(0.0, 1.0);
                let mut col = base;
                // blend with subtle blue tint on one corner to hint 3D
                let blend = ((fx + fy) * 0.5) as f32;
                let br = ((col.r() as f32) * (0.6 + 0.4 * dark * blend)) as u8;
                let bgc = ((col.g() as f32) * (0.6 + 0.4 * dark * blend)) as u8;
                let bb = ((24u8 as f32) * (0.6 + 0.4 * dark * blend)) as u8;
                col = egui::Color32::from_rgb(br, bgc, bb);

                pixels.push(col);
            }
        }

        // Draw a centered square (the "cube" face) brighter depending on s
        let cw = (w as f32 * 0.4).max(4.0) as usize;
        let ch = (h as f32 * 0.4).max(4.0) as usize;
        let cx = w / 2;
        let cy = h / 2;
        let bright = egui::Color32::from_rgb((255.0 * s) as u8, (255.0 * (0.7 + 0.3*s)) as u8, 80);
        for yy in (cy.saturating_sub(ch/2))..(cy + ch/2).min(h) {
            for xx in (cx.saturating_sub(cw/2))..(cx + cw/2).min(w) {
                let idx = yy * w + xx;
                if idx < pixels.len() { pixels[idx] = bright; }
            }
        }

        // convert to ColorImage
        let mut raw: Vec<[u8;4]> = Vec::with_capacity(w*h);
        for c in pixels {
            raw.push([c.r(), c.g(), c.b(), c.a()]);
        }
        // flatten
        let mut flat: Vec<u8> = Vec::with_capacity(w*h*4);
        for px in raw { flat.extend_from_slice(&px); }
        egui::ColorImage::from_rgba_unmultiplied([w, h], &flat)
    }

    pub fn render(&mut self, ui: &mut egui::Ui, s: f32, size: egui::Vec2) {
        // Back-compat wrapper: just ensure texture is up-to-date. Actual painting
        // into a rect is done by caller (so we can use painter::image with a rect).
        let ctx = ui.ctx();
        let _ = self.ensure_texture(ctx, s, size);
    }

    /// Ensure the texture for the given state/size exists and is up to date. Returns the texture id.
    pub fn ensure_texture(&mut self, ctx: &egui::Context, s: f32, size: egui::Vec2) -> egui::TextureId {
        let w = size.x.max(1.0) as usize;
        let h = size.y.max(1.0) as usize;
        let need_update = self.texture.is_none() || self.last_size != [w,h];
        // always (re)load the texture from the fresh image; this avoids relying on TextureHandle::set
        let img = Self::generate_image(s, w, h);
        let tex = ctx.load_texture("three_view_tex", img, egui::TextureOptions::LINEAR);
        self.texture = Some(tex);
        self.last_size = [w, h];
        self.texture.as_ref().unwrap().id()
    }
}

fn main() {
    // this file exists only to keep cargo happy (src/bin/*.rs files are treated as binaries).
}
