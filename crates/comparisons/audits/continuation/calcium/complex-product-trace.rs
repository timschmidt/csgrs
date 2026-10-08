#[allow(dead_code)]
mod corpus { include!("complex-product-corpus.rs"); }
use corpus::*;
fn main() {
    for bits in [32,128,129,192,256,512,2048,16_384,65_536] {
        for family in FAMILIES { for scale in ["integer","dyadic","odd"] {
            for mask in [0,2,6,15] { for route in ["product","quotient","lattice"] {
                let f=fixture(bits,family,scale,mask);
                let input=f.inputs();
                let expected=f.expected(route=="quotient");
                for stage in ["first","repeated"] {
                    hyperreal::dispatch_trace::reset();
                    let guard=hyperreal::dispatch_trace::recording_scope();
                    let out=calculate(&input,route);
                    drop(guard);
                    let trace=hyperreal::dispatch_trace::take();
                    assert_eq!([as_ratio(&out.0),as_ratio(&out.1)],expected);
                    let selected=trace.iter().filter(|e|e.path=="paired-common-scale-three-product").map(|e|e.count).sum::<u64>();
                    let fused=trace.iter().filter(|e|e.path=="mul-components-fused-cold-exact-rational").map(|e|e.count).sum::<u64>();
                    let reuse=trace.iter().filter(|e|e.path=="mul-components-three-product-exact-rational").map(|e|e.count).sum::<u64>();
                    println!("{{\"bits\":{bits},\"family\":\"{family}\",\"scale\":\"{scale}\",\"mask\":{mask},\"route\":\"{route}\",\"stage\":\"{stage}\",\"candidateSelections\":{selected},\"latticeFused\":{fused},\"latticeReuse\":{reuse}}}");
                }
            }}
        }}
    }
}
