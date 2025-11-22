use memristor_sim::{mna, Memristor};
use std::io;
use std::fs::File;
use std::io::Write;

fn main() -> io::Result<()> {
    // Build an MNA netlist example:
    // Node numbering: 0 = ground, 1 = after R1, 2 = branch node
    let mut net = mna::Netlist::new();
    // R1 between node 1 and ground (1k)
    net.add(mna::Component::Resistor { id: "R1".into(), n1: 1, n2: 0, r: 1e3 });
    // Memristor M1 between node 2 and ground
    net.add(mna::Component::Memristor { id: "M1".into(), n1: 2, n2: 0, mem: Memristor { id: "M1".into(), ron: 100.0, roff: 16000.0, state: 0.1, mu0: 1e3, n: 1.0, window_p: 1.0, ithreshold: 0.0, activation_e: 0.6, temperature: 300.0 } });
    // Series branch: R2 (2k) between node1 and node2
    net.add(mna::Component::Resistor { id: "R2".into(), n1: 1, n2: 2, r: 2e3 });
    // Voltage source between node 0 and node 1 (drive across R1+branch)
    net.add(mna::Component::VSource { id: "V1".into(), n_plus: 1, n_minus: 0, kind: mna::VoltageKind::DC(1.0) });

    // Prepare CSV file
    let mut file = File::create("mna_output.csv")?;
    // header: time, node_0, node_1, node_2, M1_state
    writeln!(file, "time,node_0,node_1,node_2,M1_state")?;

    let tstop = 1e-3;
    let dt = 1e-6;
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
                    if id == "M1" { m1_state = mem.state; }
                }
            }
            writeln!(file, "{:.9},{:.6e},{:.6e},{:.6e},{:.6}", t, v0, v1, v2, m1_state)?;
        } else {
            writeln!(file, "{:.9},ERR,ERR,ERR,ERR", t)?;
        }
        t += dt;
    }

    println!("Wrote MNA simulation to mna_output.csv");
    Ok(())
}
