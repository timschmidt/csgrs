from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent
target=Path(sys.argv[1]);prior=json.loads((A/'native-path-shims-20260928-v707-terminal.json').read_text());baseline=Path(prior['source_directory'])
old='RationalBezierOverlapOrientation2';new='CurveOverlapOrientation2'
definition='''/// Relative parameter orientation of a certified shared rational-Bezier image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveOverlapOrientation2 {
    /// Both parameter domains traverse the shared image in the same direction.
    Same,
    /// The second parameter domain traverses the shared image in reverse.
    Reversed,
}

'''
base=json.loads((A/'overlap-orientation-base-v706.json').read_text())
for name,sha in base.items():
 data=(baseline/name).read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name
 expected=data.decode().replace(old,new)
 if name.endswith('/rational_bezier_general.rs'):
  assert expected.count(definition)==1;expected=expected.replace(definition,'').replace('CurveOperation2, CurveParameter2,','CurveOperation2, CurveOverlapOrientation2, CurveParameter2,',1)
 elif name.endswith('/curve_intersection.rs'):
  expected=expected.replace('    CurveOverlapOrientation2, UncertaintyReason,','    UncertaintyReason,',1)
  anchor='/// Certified positive-length overlap between two retained curve spans.'
  expected=expected.replace(anchor,definition.replace('certified shared rational-Bezier image','certified shared curve image')+anchor)
 elif name.endswith('/lib.rs'):
  expected=expected.replace('CurveLocation2, CurveParameterSet2,','CurveLocation2, CurveOverlapOrientation2, CurveParameterSet2,',1).replace('    RationalBezierIntersectionTopology2, CurveOverlapOrientation2,\n','    RationalBezierIntersectionTopology2,\n',1)
 actual=(target/name).read_text();assert old not in actual,name
 formatted=subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--emit','stdout'],input=expected,text=True,capture_output=True,check=True).stdout
 assert formatted==actual,name
print('Verified 12 source transformations: identifier, enum ownership/import/export and formatting only')
