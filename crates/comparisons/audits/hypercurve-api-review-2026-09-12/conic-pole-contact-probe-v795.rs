use hypercurve::{BezierParameter2, Classification, CurveContext, ExactCurveError, Point2, RationalBezier2, RationalBezierIntersectionContacts2, Real, UncertaintyReason};
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn point_status(curve:&RationalBezier2,parameter:&BezierParameter2,policy:&CurveContext)-> &'static str {
 match parameter {
  BezierParameter2::Exact(p)=> match curve.point_at(p,policy){Ok(_)=>"finite",Err(ExactCurveError::Blocked(b)) if b.reason()==UncertaintyReason::Boundary=>"pole",Err(_)=>"unresolved"},
  BezierParameter2::Algebraic(p)=>match curve.point_at_algebraic_parameter(p,policy){Ok(Classification::Decided(_))=>"finite",Ok(Classification::Uncertain(UncertaintyReason::Boundary))=>"pole",_=>"unresolved"},
 }
}
fn main(){
 let mut cases=0;let mut finite_controls=0;let mut pole_contacts=0;
 for (pi,policy)in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  for middle in [2,-1,-2]{
   for (dx,dy)in [(1,0),(0,1)]{
    let curve=|dx:i32,dy:i32|RationalBezier2::try_new(vec![Point2::from_values(dx,dy),Point2::from_values(1+dx,1+dy),Point2::from_values(2+dx,dy)],vec![Real::one(),Real::from(middle),Real::one()]).unwrap();
    let first=curve(0,0);let second=curve(dx,dy);
    for c in [&first,&second]{assert!(c.point_at(&q(1,4),policy).is_ok());finite_controls+=1;}
    println!("begin policy={pi} middle={middle} shift={dx},{dy}");
    match first.intersection_contacts(&second,policy){
     Ok(result)=>{
      let kind=match &result{RationalBezierIntersectionContacts2::NoIntersection=>"none",RationalBezierIntersectionContacts2::Contacts(_)=>"complete",RationalBezierIntersectionContacts2::Overlap(_)=>"overlap",RationalBezierIntersectionContacts2::ContactsAndOverlap{..}=>"contacts_and_overlap",RationalBezierIntersectionContacts2::Incomplete{..}=>"incomplete",RationalBezierIntersectionContacts2::DegenerateResultant=>"degenerate"};
      println!("result={kind} contacts={}",result.isolated_contacts().len());
      for (index,contact)in result.isolated_contacts().iter().enumerate(){
       let a=point_status(&first,contact.first_parameter(),policy);let b=point_status(&second,contact.second_parameter(),policy);
       println!("contact={index} first={a} second={b} transverse={}",contact.is_certified_transverse());
       if a=="pole"||b=="pole"{pole_contacts+=1;}
      }
     },
     Err(ExactCurveError::Blocked(b))=>println!("blocked={:?}",b.reason()),
     Err(_)=>println!("invalid"),
    }
    cases+=1;
   }
  }
 }
 println!("completed_cases={cases} finite_controls={finite_controls} pole_contacts={pole_contacts}");
}
