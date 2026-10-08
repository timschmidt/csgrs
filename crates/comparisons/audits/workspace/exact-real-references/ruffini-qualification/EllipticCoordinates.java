import dk.jonaslindstrom.ruffini.elliptic.elements.*;
import dk.jonaslindstrom.ruffini.elliptic.structures.*;
import dk.jonaslindstrom.ruffini.finitefields.PrimeField;
import java.util.*;
import java.util.function.Supplier;

/** Local numerical group-law/coordinate controls, not a cryptographic assessment. */
public class EllipticCoordinates {
    static int count;
    static int mod(int x,int p){return Math.floorMod(x,p);}
    static int inv(int x,int p){for(int y=1;y<p;y++)if(mod(x*y,p)==1)return y;throw new ArithmeticException();}
    static AffinePoint<Integer> point(int x,int y,int p){return new AffinePoint<>(mod(x,p),mod(y,p));}
    static AffinePoint<Integer> reference(AffinePoint<Integer> a,AffinePoint<Integer> b,int p,boolean montgomery){
        if(a.isPointAtInfinity())return b;if(b.isPointAtInfinity())return a;
        int x=a.x(),y=a.y(),u=b.x(),v=b.y();
        if(x==u&&mod(y+v,p)==0)return AffinePoint.pointAtInfinity();
        int slope=x==u?mod((3*x*x+(montgomery?2*x:0)+1)*inv(2*y,p),p):mod((v-y)*inv(u-x,p),p);
        int resultX=mod(slope*slope-x-u-(montgomery?1:0),p);
        return point(resultX,slope*(x-resultX)-y,p);
    }
    static List<AffinePoint<Integer>> points(int p,boolean montgomery){
        List<AffinePoint<Integer>> result=new ArrayList<>();result.add(AffinePoint.pointAtInfinity());
        for(int x=0;x<p;x++)for(int y=0;y<p;y++)if(mod(y*y-x*x*x-(montgomery?x*x:x)-(montgomery?x:1),p)==0)result.add(point(x,y,p));
        return result;
    }
    static void probe(String id,Supplier<Boolean> body){
        count++;try{System.out.println((body.get()?"PASS":"WRONG")+"\t"+id);}catch(Exception e){System.out.println("EXCEPTION\t"+id+"\t"+e.getClass().getSimpleName());}
    }
    public static void main(String[] args){
        for(int p:new int[]{5,7,11}){
            PrimeField field=new PrimeField(p);
            var affine=new ShortWeierstrassCurveAffine<>(field,1,1);
            var projective=new ShortWeierstrassCurveProjective<>(field,1,1);
            var montgomery=new MontgomeryCurve<>(field,1,1);
            List<AffinePoint<Integer>> values=points(p,false);
            for(int i=0;i<values.size();i++){
                final int index=i;var a=values.get(i);
                probe("affine-negate-"+p+"-"+i,()->affine.equals(affine.add(a,affine.negate(a)),affine.zero()));
                for(int scale=1;scale<p;scale++){
                    final int s=scale;
                    var hp=a.isPointAtInfinity()?ProjectivePoint.pointAtInfinity(field):new ProjectivePoint<>(mod(a.x()*s,p),mod(a.y()*s,p),s);
                    var jp=a.isPointAtInfinity()?JacobianPoint.pointAtInfinity(field):new JacobianPoint<>(mod(a.x()*s*s,p),mod(a.y()*s*s*s,p),s);
                    probe("projective-conversion-"+p+"-"+index+"-"+s,()->a.equals(hp.toAffinePoint(field)));
                    probe("jacobian-conversion-"+p+"-"+index+"-"+s,()->a.equals(jp.toAffinePoint(field)));
                    probe("projective-doubling-"+p+"-"+index+"-"+s,()->reference(a,a,p,false).equals(projective.doubling(hp).toAffinePoint(field)));
                }
                for(int j=0;j<values.size();j++){
                    var b=values.get(j);var expected=reference(a,b,p,false);
                    probe("affine-add-"+p+"-"+i+"-"+j,()->expected.equals(affine.add(a,b)));
                    probe("projective-add-"+p+"-"+i+"-"+j,()->expected.equals(projective.add(a.toProjectivePoint(field),b.toProjectivePoint(field)).toAffinePoint(field)));
                }
            }
            var invalid=new ProjectivePoint<>(0,0,0);
            probe("projective-invalid-equality-"+p,()->!projective.equals(invalid,values.get(1).toProjectivePoint(field)));
            var montPoints=points(p,true);
            for(int i=0;i<montPoints.size();i++){
                var a=montPoints.get(i);
                probe("montgomery-negate-"+p+"-"+i,()->montgomery.equals(montgomery.add(a,montgomery.negate(a)),montgomery.zero()));
                for(int j=0;j<montPoints.size();j++){
                    var b=montPoints.get(j);var expected=reference(a,b,p,true);
                    probe("montgomery-add-"+p+"-"+i+"-"+j,()->expected.equals(montgomery.add(a,b)));
                }
            }
            // Derived by substituting a=(3-A²)/(3B²),
            // b=(2A³-9A)/(27B³) into the short-Weierstrass invariant, A=B=1.
            int invariant=mod(2048*inv(3,p),p);
            probe("montgomery-invariant-"+p,()->field.equals(montgomery.jInvariant(),invariant));
        }
        System.out.println("SUMMARY\t"+count);
    }
}
