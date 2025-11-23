use eframe::egui;

// GPU view enabled; three-d is optional but we render with WGSL SDF by default.

pub struct ThreeViewRenderer {
    texture: Option<egui::TextureHandle>,
    last_size: [usize; 2],
    rotation: f32,
    last_state: f32,
    #[cfg(feature = "gpu_view")]
    gpu: Option<GpuCtx>,
}

impl ThreeViewRenderer {
    pub fn new(_ctx: &egui::Context) -> Self {
        Self { 
            texture: None, 
            last_size: [0, 0], 
            rotation: 0.0, 
            last_state: -1.0,
            #[cfg(feature = "gpu_view")] 
            gpu: None 
        }
    }

    /// Initialize renderer using optional eframe CreationContext. When the `gpu_view`
    /// feature is enabled we will later attempt to initialize GPU resources from
    /// the provided context; for now this accepts the context for future use.
    pub fn new_with_ctx(cc: Option<&eframe::CreationContext<'_>>) -> Self {
        // Do not move `egui_ctx` out of `cc`; borrow it when available.
        let s = if let Some(cc) = cc {
            // clone the egui context to avoid moving it out of `cc`
            let ctx_clone = cc.egui_ctx.clone();
            Self::new(&ctx_clone)
        } else {
            Self::new(&egui::Context::default())
        };

        #[cfg(feature = "gpu_view")]
        {
            // Try to initialize an offscreen wgpu device/queue. This is separate
            // from eframe's device and is used as an isolated renderer for now.
            if let Ok(gpu) = GpuCtx::new() {
                let mut s = s;
                s.gpu = Some(gpu);
                return s;
            }
        }

        s
    }

    fn generate_image(s: f32, w: usize, h: usize) -> egui::ColorImage {
        // Improved 3D-looking memristor visualization with better lighting
        let mut pixels = vec![0u8; w * h * 4];
        
        let center_x = w as f32 * 0.5;
        let center_y = h as f32 * 0.5;
        let radius = (w.min(h) as f32 * 0.35).min(center_x.min(center_y) - 10.0);
        
        // Color based on state: dark red (low) -> orange -> yellow (high)
        let base_r = (128.0 + 127.0 * s) as u8;
        let base_g = (50.0 + 150.0 * s) as u8;
        let base_b = (20 + (30.0 * s) as u8).min(50);
        
        for y in 0..h {
            for x in 0..w {
                let fx = (x as f32 - center_x) / radius;
                let fy = (y as f32 - center_y) / radius;
                let dist2 = fx * fx + fy * fy;
                
                let idx = (y * w + x) * 4;
                
                if dist2 < 1.0 {
                    // Inside the circle - draw 3D cylinder effect
                    let depth = (1.0 - dist2).sqrt();
                    let angle = fy.atan2(fx);
                    
                    // Simulate lighting from top-left
                    let _light_dir = -0.707; // 45 degrees (unused but kept for future use)
                    let light_factor = (angle.cos() * 0.5 + 0.5).max(0.3);
                    let highlight = if depth > 0.85 { 1.2 } else { 1.0 };
                    
                    let r = ((base_r as f32) * light_factor * highlight).min(255.0) as u8;
                    let g = ((base_g as f32) * light_factor * highlight).min(255.0) as u8;
                    let b = ((base_b as f32) * light_factor * highlight).min(255.0) as u8;
                    
                    // Add rim lighting
                    let rim = if dist2 > 0.85 { 1.3 } else { 1.0 };
                    
                    pixels[idx] = (r as f32 * rim).min(255.0) as u8;
                    pixels[idx + 1] = (g as f32 * rim).min(255.0) as u8;
                    pixels[idx + 2] = (b as f32 * rim).min(255.0) as u8;
                    pixels[idx + 3] = 255;
                } else {
                    // Background - dark gradient
                    let bg_factor = (1.0 - (dist2 - 1.0).min(2.0) * 0.3).max(0.1);
                    pixels[idx] = (20.0 * bg_factor) as u8;
                    pixels[idx + 1] = (20.0 * bg_factor) as u8;
                    pixels[idx + 2] = (25.0 * bg_factor) as u8;
                    pixels[idx + 3] = 255;
                }
            }
        }
        
        egui::ColorImage::from_rgba_unmultiplied([w, h], &pixels)
    }

    pub fn render(&mut self, ui: &mut egui::Ui, s: f32, size: egui::Vec2) {
        // Back-compat wrapper: just ensure texture is up-to-date. Actual painting
        // into a rect is done by caller (so we can use painter::image with a rect).
        let ctx = ui.ctx();
        // If we have a GPU context available we could render on the GPU here.
        // For now, keep the existing CPU->egui path while we expand GPU rendering.
        // advance rotation for simple animation
        self.rotation = (self.rotation + 0.03) % (std::f32::consts::PI * 2.0);
        let _ = self.ensure_texture(ctx, s, size);
    }

    /// Ensure the texture for the given state/size exists and is up to date. Returns the texture id.
    pub fn ensure_texture(&mut self, ctx: &egui::Context, s: f32, size: egui::Vec2) -> egui::TextureId {
        let w = size.x.max(1.0) as usize;
        let h = size.y.max(1.0) as usize;
        
        // Only regenerate if size changed or state changed significantly (cache optimization)
        let state_changed = (s - self.last_state).abs() > 0.01;
        let size_changed = self.last_size != [w, h];
        let need_update = size_changed || state_changed || self.texture.is_none();
        
        if !need_update {
            return self.texture.as_ref().unwrap().id();
        }
        
        self.last_state = s;
        
        // If we have an offscreen GPU context, render on GPU and read back.
        #[cfg(feature = "gpu_view")]
        {
            if let Some(gpu) = &mut self.gpu {
                if let Some(img) = gpu.render_clear_to_image(s, self.rotation, w as u32, h as u32) {
                    let tex = ctx.load_texture("three_view_tex", img, egui::TextureOptions::LINEAR);
                    self.texture = Some(tex);
                    self.last_size = [w, h];
                    return self.texture.as_ref().unwrap().id();
                }
            }
        }

        // Fallback: generate CPU image only when needed
        let img = Self::generate_image(s, w, h);
        let tex = ctx.load_texture("three_view_tex", img, egui::TextureOptions::LINEAR);
        self.texture = Some(tex);
        self.last_size = [w, h];
        self.texture.as_ref().unwrap().id()
    }
}

#[cfg(feature = "gpu_view")]
struct GpuCtx {
    instance: wgpu::Instance,
    adapter: wgpu::Adapter,
    device: wgpu::Device,
    queue: wgpu::Queue,
}

#[cfg(feature = "gpu_view")]
impl GpuCtx {
    pub fn new() -> Result<Self, ()> {
        // Create an instance descriptor and instance
        let instance_desc = wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            dx12_shader_compiler: Default::default(),
            flags: wgpu::InstanceFlags::empty(),
            gles_minor_version: wgpu::Gles3MinorVersion::default(),
        };
        let instance = wgpu::Instance::new(instance_desc);

        // Request an adapter
        let adapter_opt = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }));
        let adapter = match adapter_opt {
            Some(a) => a,
            None => return Err(()),
        };

        // Request device and queue
        let req = wgpu::DeviceDescriptor {
            label: Some("three_view_device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
        };
        let (device, queue) = match pollster::block_on(adapter.request_device(&req, None)) {
            Ok(pair) => pair,
            Err(_) => return Err(()),
        };

        Ok(Self { instance, adapter, device, queue })
    }

    /// Render a simple clear color into a GPU texture of size (w,h), read back the RGBA8 bytes,
    /// and return an `egui::ColorImage`. This is a synchronous blocking helper using `pollster`.
    pub fn render_clear_to_image(&mut self, s: f32, rotation: f32, w: u32, h: u32) -> Option<egui::ColorImage> {
        use std::borrow::Cow;
        // no DeviceExt used here currently
        println!("three_view: render_clear_to_image start {}x{} rot={} s={}", w, h, rotation, s);

        // choose format and create texture
        let format = wgpu::TextureFormat::Rgba8UnormSrgb;
        let size = wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 };
        let texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("three_view_offscreen"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Map state s to a clear color
        let r = (s.clamp(0.0, 1.0) * 255.0) as f64 / 255.0;
        let g = (s.clamp(0.0, 1.0) * 200.0) as f64 / 255.0;
        let clear_color = wgpu::Color { r, g, b: 0.05, a: 1.0 };

        // Render using a WGSL SDF raymarch shader: two crossing cylinders + central cube,
        // with basic lighting, soft shadows and ambient occlusion. This runs on our
        // offscreen wgpu device and avoids needing `three-d` integration.
        {
            let shader_source = r#"
@group(0) @binding(0) var<uniform> u_params: vec4<f32>; // x=width, y=height, z=rotation, w=s

@vertex fn vs_main(@builtin(vertex_index) vid: u32) -> @builtin(position) vec4<f32> {
    var pts = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
    let p = pts[vid];
    return vec4<f32>(p, 0.0, 1.0);
}

fn sd_box(p: vec3<f32>, b: vec3<f32>) -> f32 {
    let q = abs(p) - b;
    let outside = max(q, vec3<f32>(0.0));
    return length(outside) + min(max(q.x, max(q.y, q.z)), 0.0);
}

fn sd_cyl_x(p: vec3<f32>, r: f32) -> f32 { // cylinder along X axis
    return length(vec2<f32>(p.y, p.z)) - r;
}

fn sd_cyl_z(p: vec3<f32>, r: f32) -> f32 { // cylinder along Z axis
    return length(vec2<f32>(p.x, p.y)) - r;
}

fn op_union(a: f32, b: f32) -> f32 { return min(a, b); }

fn map(p: vec3<f32>) -> f32 {
    let bx = sd_cyl_x(p, 0.35);
    let bz = sd_cyl_z(p, 0.35);
    let cb = sd_box(p, vec3<f32>(0.18, 0.18, 0.18));
    return op_union(op_union(bx, bz), cb);
}

fn estimate_normal(p: vec3<f32>) -> vec3<f32> {
    let e = 0.0008;
    let dx = map(p + vec3<f32>(e,0,0)) - map(p - vec3<f32>(e,0,0));
    let dy = map(p + vec3<f32>(0,e,0)) - map(p - vec3<f32>(0,e,0));
    let dz = map(p + vec3<f32>(0,0,e)) - map(p - vec3<f32>(0,0,e));
    return normalize(vec3<f32>(dx, dy, dz));
}

fn softshadow(ro: vec3<f32>, rd: vec3<f32>, k: f32) -> f32 {
    var res: f32 = 1.0;
    var t: f32 = 0.02;
    for (var i: i32 = 0; i < 32; i = i + 1) {
        let h = map(ro + rd * t);
        if (h < 0.001) { return 0.0; }
        res = min(res, k * h / t);
        t = t + clamp(h, 0.01, 0.5);
        if (t > 10.0) { break; }
    }
    return clamp(res, 0.0, 1.0);
}

@fragment fn fs_main(@builtin(position) pos: vec4<f32>) -> @location(0) vec4<f32> {
    let w = u_params.x;
    let h = u_params.y;
    let rot = u_params.z;
    let s = u_params.w;

    let uv = (pos.xy / vec2<f32>(w, h));
    let aspect = w / h;
    var p = (uv * 2.0 - vec2<f32>(1.0, 1.0));
    p.x *= aspect;

    // Camera
    let cam_pos = vec3<f32>(0.0, 0.0, 3.0);
    let look_at = vec3<f32>(0.0, 0.0, 0.0);
    let forward = normalize(look_at - cam_pos);
    let right = normalize(cross(vec3<f32>(0.0,1.0,0.0), forward));
    let up = cross(forward, right);

    let fov = 45.0 * 3.14159265 / 180.0;
    let rd = normalize(forward + right * p.x * tan(fov*0.5) + up * p.y * tan(fov*0.5));

    // rotate scene
    let ca = cos(rot);
    let sa = sin(rot);
    let rotm = mat3x3<f32>(vec3<f32>(ca,0,sa), vec3<f32>(0,1,0), vec3<f32>(-sa,0,ca));

    // raymarch
    var t: f32 = 0.0;
    var dist: f32 = 0.0;
    var hit: i32 = 0;
    for (var i: i32 = 0; i < 128; i = i + 1) {
        let ppos = cam_pos + rd * t;
        let sp = rotm * ppos; // rotate object space
        dist = map(sp);
        if (dist < 0.001) { hit = 1; break; }
        t = t + dist;
        if (t > 10.0) { break; }
    }

    var col = vec3<f32>(0.08, 0.08, 0.1);
    if (hit == 1) {
        let ppos = cam_pos + rd * t;
        let sp = rotm * ppos;
        let n = estimate_normal(sp);
        let light_dir = normalize(vec3<f32>(0.5, 0.8, 0.6));
        let diff = max(dot(n, light_dir), 0.0);
        let sh = softshadow(sp + n*0.001, light_dir, 16.0);
        let ao = 1.0 - 0.5 * clamp(t*0.05, 0.0, 1.0);
        let base = vec3<f32>(0.2 + s*0.8, 0.15 + s*0.6, 0.05);
        let spec = pow(max(dot(reflect(-light_dir, n), normalize(cam_pos - ppos)), 0.0), 32.0);
        col = base * (0.2 + 0.8 * diff * sh) * ao + vec3<f32>(1.0) * 0.3 * spec;
    }

    // vignette
    let d = length(p);
    col = mix(col, col * (1.0 - d*0.6), 0.5);

    return vec4<f32>(col, 1.0);
}
"#;

            let module = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("three_view_shader"),
                source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(shader_source)),
            });
            println!("three_view: shader module created");

            // uniform buffer: vec4 (w,h,rotation,s)
            let mut ub_bytes: Vec<u8> = Vec::with_capacity(16);
            ub_bytes.extend_from_slice(&(w as f32).to_le_bytes());
            ub_bytes.extend_from_slice(&(h as f32).to_le_bytes());
            ub_bytes.extend_from_slice(&rotation.to_le_bytes());
            ub_bytes.extend_from_slice(&s.to_le_bytes());

            let uniform_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("three_view_uniform"),
                size: ub_bytes.len() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.queue.write_buffer(&uniform_buf, 0, &ub_bytes);
            println!("three_view: uniform buffer written ({} bytes)", ub_bytes.len());

            let bind_layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("three_view_bind_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                }],
            });

            let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("three_view_bind_group"),
                layout: &bind_layout,
                entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::Buffer(uniform_buf.as_entire_buffer_binding()) }],
            });

            let pipeline_layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("three_view_pipeline_layout"),
                bind_group_layouts: &[&bind_layout],
                push_constant_ranges: &[],
            });

            let pipeline = self.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("three_view_pipeline"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState { module: &module, entry_point: "vs_main", buffers: &[] },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState { format, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                primitive: wgpu::PrimitiveState::default(),
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                multiview: None,
            });
            println!("three_view: pipeline created");

            // encode draw
            let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("three_view_encoder") });
            {
                let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("three_view_pass"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(clear_color), store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: None,
                    occlusion_query_set: None,
                    timestamp_writes: None,
                });
                rpass.set_pipeline(&pipeline);
                rpass.set_bind_group(0, &bind_group, &[]);
                rpass.draw(0..3, 0..1);
            }

            self.queue.submit(Some(encoder.finish()));
            println!("three_view: submitted render pass");
            self.device.poll(wgpu::Maintain::Wait);
        }

        // prepare buffer for copy with padded bytes_per_row
        let bytes_per_pixel = 4u32;
        let unpadded_bytes_per_row = w * bytes_per_pixel;
        let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as u32; // 256
        let padded_bytes_per_row = ((unpadded_bytes_per_row + align - 1) / align) * align;

        let buffer_size = (padded_bytes_per_row as u64) * (h as u64);
        let dst_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("three_view_readback"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // encode copy from texture to buffer
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("three_view_copy_encoder") });
        encoder.copy_texture_to_buffer(wgpu::ImageCopyTexture { texture: &texture, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All }, wgpu::ImageCopyBuffer { buffer: &dst_buf, layout: wgpu::ImageDataLayout { offset: 0, bytes_per_row: Some(padded_bytes_per_row), rows_per_image: Some(h) } }, size);

            self.queue.submit(Some(encoder.finish()));
            println!("three_view: submitted copy to buffer");
            // ensure work completes so map_async callback will be invoked
            self.device.poll(wgpu::Maintain::Wait);

        // map buffer and read
        let slice = dst_buf.slice(..);
        // map_async now takes a callback; use a blocking std mpsc channel to wait for the callback
        let (tx, rx) = std::sync::mpsc::channel();
        println!("three_view: mapping readback buffer");
        slice.map_async(wgpu::MapMode::Read, move |res| { let _ = tx.send(res); });
        // Poll the device to ensure the map_async callback is invoked on this thread
        self.device.poll(wgpu::Maintain::Wait);
        match rx.recv_timeout(std::time::Duration::from_secs(5)) {
            Ok(Ok(())) => { println!("three_view: mapping completed"); },
            Ok(Err(e)) => { println!("three_view: mapping error: {:?}", e); return None; },
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => { println!("three_view: mapping timed out"); return None; },
            Err(e) => { println!("three_view: mapping recv error: {:?}", e); return None; },
        }
        let data = slice.get_mapped_range();

        // copy rows into compact RGBA Vec<u8>
        let mut pixels: Vec<u8> = Vec::with_capacity((w * h * 4) as usize);
        for row in 0..h as usize {
            let start = (row as u64 * padded_bytes_per_row as u64) as usize;
            let end = start + (unpadded_bytes_per_row as usize);
            pixels.extend_from_slice(&data[start..end]);
        }

        // unmap buffer
        drop(data);
        dst_buf.unmap();

        // Diagnostic: log readback size and a small sample of bytes
        println!("three_view: readback pixels len={}", pixels.len());
        let sample_len = pixels.len().min(32);
        let sample_bytes: Vec<String> = pixels.iter().take(sample_len).map(|b| format!("{:02x}", b)).collect();
        println!("three_view: sample bytes: {}", sample_bytes.join(" "));

        // Optional: write a debug PNG to target/three_view_debug.png so we can inspect the output
        if let Err(e) = (|| -> Result<(), Box<dyn std::error::Error>> {
            use image::{ImageBuffer, Rgba};
            let path = std::path::Path::new("target/three_view_debug.png");
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            // ImageBuffer::from_raw takes ownership of the pixel vec when successful
            if let Some(img_buf) = ImageBuffer::<Rgba<u8>, _>::from_raw(w, h, pixels.clone()) {
                img_buf.save(path)?;
                println!("three_view: wrote debug PNG to {:?}", path);
            } else {
                println!("three_view: failed to create ImageBuffer from raw pixels (size mismatch)");
            }
            Ok(())
        })() {
            println!("three_view: error writing debug PNG: {}", e);
        }

        // Build egui::ColorImage from RGBA bytes
        Some(egui::ColorImage::from_rgba_unmultiplied([w as usize, h as usize], &pixels))
    }
}
