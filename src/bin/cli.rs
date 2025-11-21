use memristor_sim::*;
use std::io::{self, Write};

fn main() -> io::Result<()> {
    // Build a sample network:
    // top = Series( [ R(1k), Parallel( [ M1, Series([R2, M2]) ]) ])

    let r1 = Component::R(Resistor { id: "R1".into(), r: 1e3 });

    let m1 = Component::M(Memristor { id: "M1".into(), ron: 100.0, roff: 16000.0, state: 0.1, k: 1e3 });
    let r2 = Component::R(Resistor { id: "R2".into(), r: 2e3 });
    let m2 = Component::M(Memristor { id: "M2".into(), ron: 100.0, roff: 16000.0, state: 0.5, k: 1e3 });

    let branch = Component::Series(vec![r2, m2]);
    let parallel = Component::Parallel(vec![m1, branch]);

    let top = Component::Series(vec![r1, parallel]);

    let mut sim = Simulator::new(top);

    // drive: step to 1.0 V at t>=0
    let drive = |_: f64| -> f64 { 1.0 };

    let tstop = 1e-3; // 1 ms
    let dt = 1e-6; // 1 us -> 1000 steps

    // Print CSV header
    println!("time,path,voltage,current,state");

    let mut t = 0.0;
    while t <= tstop {
        let v = drive(t);
        sim.recorder.clear();
        let _i = sim.top.step(v, dt, &mut sim.recorder, "top");

        for s in &sim.recorder.samples {
            println!("{:.9},{},{:.6e},{:.6e},{}", s.time, s.path, s.voltage, s.current, match s.state { Some(x) => format!("{:.6}", x), None => "".into() });
        }

        // advance time and keep memristor states updated (done in step)
        t += dt;
    }

    Ok(())
}
