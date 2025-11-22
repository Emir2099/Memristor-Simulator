use std::collections::HashMap;
use crate::mna;
use serde::{Serialize, Deserialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NodeKind {
    VSource { id: String, amp: f64, freq: f64, is_sine: bool },
    Memristor { id: String, ron: f64, roff: f64, state: f64, mu0: f64, n: f64, window_p: f64, ithreshold: f64 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Node {
    pub id: usize,
    pub kind: NodeKind,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Link {
    // connect node_a.output -> node_b.input 
    pub a: usize,
    pub b: usize,
}

#[derive(Default, Serialize, Deserialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub links: Vec<Link>,
}

impl Graph {
    pub fn add_node(&mut self, kind: NodeKind) -> usize {
        let id = self.nodes.len();
        self.nodes.push(Node { id, kind });
        id
    }

    pub fn add_link(&mut self, a: usize, b: usize) {
        self.links.push(Link { a, b });
    }

    // Remove a node by index. This will remove any links referencing it and renumber subsequent node ids.
    pub fn remove_node(&mut self, idx: usize) {
        if idx >= self.nodes.len() { return; }
        // remove the node
        self.nodes.remove(idx);
        // renumber node ids
        for (i, n) in self.nodes.iter_mut().enumerate() { n.id = i; }
        // remove links that reference the removed node
        self.links.retain(|l| l.a != idx && l.b != idx);
        // decrement indices greater than removed
        for l in &mut self.links {
            if l.a > idx { l.a -= 1; }
            if l.b > idx { l.b -= 1; }
        }
    }

    // Remove a link by index in the links vector
    pub fn remove_link(&mut self, link_idx: usize) {
        if link_idx >= self.links.len() { return; }
        self.links.remove(link_idx);
    }

    // This implementation assumes a very small subset of workflows:
    // - A single VSource node connected to one or more Memristor nodes.
    // - Memristor nodes not connected to a source are assumed connected to ground.
    // - Also add a series resistor between source node and memristor node (R2) and a resistor to ground (R1)
    // The goal is to demonstrate traversal and netlist construction; a full generic mapper would be more involved.
    pub fn to_netlist(&self) -> mna::Netlist {
        let mut net = mna::Netlist::new();

        // We'll create node numbers as follows:
        // ground = 0
        // source node (if any) = 1
        // mem node(s) = 2.. etc

        // find the first VSource (if any)
        let mut vs_node: Option<usize> = None;
        for n in &self.nodes {
            if let NodeKind::VSource { .. } = n.kind {
                vs_node = Some(n.id);
                break;
            }
        }

        // map node id -> net index
        let mut net_map: HashMap<usize, usize> = HashMap::new();
        let mut next_net = 2usize; // reserve 1 for source node if present

        if let Some(vs) = vs_node {
            net_map.insert(vs, 1);
        }

        // mem nodes always get unique nets (distinct from source). If a memristor is linked to the source
        // we will connect it via a resistor (R2) rather than collapsing it into the same net.
        for n in &self.nodes {
            if let NodeKind::Memristor { .. } = &n.kind {
                net_map.insert(n.id, next_net);
                next_net += 1;
            }
        }

        // Now build a small netlist inspired by the example: R1 (node1-ground), M1 (nodeN-ground), R2 (node1-nodeN), V1 (node1-ground)
        // For each memristor node we create an M component between its net and ground.

        // Add R1 at node1 (if source exists)
        if vs_node.is_some() {
            net.add(mna::Component::Resistor { id: "R1".into(), n1: 1, n2: 0, r: 1e3 });
        }

        // Add memristors and R2 between source and mem node (only if the mem node net differs from source)
        let mut mem_count = 0;
        for n in &self.nodes {
            if let NodeKind::Memristor { id: _, ron, roff, state, mu0, n: expn, window_p, ithreshold } = &n.kind {
                mem_count += 1;
                let net_idx = *net_map.get(&n.id).unwrap_or(&0);
                let mem_id = format!("M{}", mem_count);
                net.add(mna::Component::Memristor { id: mem_id.clone(), n1: net_idx, n2: 0, mem: crate::Memristor { id: mem_id.clone(), ron: *ron, roff: *roff, state: *state, mu0: *mu0, n: *expn, window_p: *window_p, ithreshold: *ithreshold, activation_e: 0.6, temperature: 300.0 } });
                // if there is a source, add R2 between source node (1) and this mem node, but only if nets differ
                if let Some(_) = vs_node {
                    if net_idx != 1 {
                        let r2id = format!("R2_{}", mem_count);
                        net.add(mna::Component::Resistor { id: r2id, n1: 1, n2: net_idx, r: 2e3 });
                    }
                }
            }
        }

        // Add a voltage source if present (connect to node 1)
        if let Some(vn) = vs_node {
            if let NodeKind::VSource { id: _, amp, freq, is_sine } = &self.nodes[vn].kind {
                let kind = if *is_sine { mna::VoltageKind::Sine { amp: *amp, freq: *freq, phase: 0.0 } } else { mna::VoltageKind::DC(*amp) };
                net.add(mna::Component::VSource { id: "V1".into(), n_plus: 1, n_minus: 0, kind });
            }
        }

        net
    }

    // Return a textual preview of the netlist and simple validation warnings.
    pub fn to_netlist_preview(&self) -> (String, Vec<String>) {
        let net = self.to_netlist();
        let mut s = String::new();
        for c in &net.comps {
            s.push_str(&format!("{}", c));
        }
        // simple validation: detect any resistor stamped between same node
        let mut warnings = Vec::new();
        for c in &net.comps {
            if let mna::Component::Resistor { id, n1, n2, .. } = c {
                if n1 == n2 {
                    warnings.push(format!("Resistor {} has identical terminals ({} == {})", id, n1, n2));
                }
            }
        }
        (s, warnings)
    }
}
