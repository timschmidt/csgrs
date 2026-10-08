import dk.jonaslindstrom.ruffini.quadraticform.*;
import dk.jonaslindstrom.ruffini.integers.structures.BigIntegers;
import java.math.BigInteger;

public final class ClassGroupContracts {
    static int count;
    interface Check {boolean run();}
    static void probe(String id,Check check){try{System.out.println((check.run()?"PASS":"WRONG")+"\t"+id);}catch(Throwable t){System.out.println("EXCEPTION\t"+id+"\t"+t.getClass().getSimpleName());}count++;}
    static QuadraticForm<BigInteger,BigIntegers> form(int a,int b,int c){return new QuadraticForm<>(BigIntegers.getInstance(),BigInteger.valueOf(a),BigInteger.valueOf(b),BigInteger.valueOf(c));}
    public static void main(String[] args){
        for(int magnitude=1;magnitude<=128;magnitude++){
            final int d=-magnitude;boolean valid=Math.floorMod(d,4)==0||Math.floorMod(d,4)==1;
            probe("discriminant-"+magnitude+"-"+(valid?"valid":"invalid"),()->{try{var group=new ClassGroup(BigInteger.valueOf(d));return valid&&group.identity().discriminant().equals(BigInteger.valueOf(d));}catch(IllegalArgumentException e){return !valid;}});
        }
        var a=form(1,1,6);var b=form(1,1,6);var group=new ClassGroup(BigInteger.valueOf(-23));
        probe("equal-before-cache",()->a.equals(b));a.discriminant();
        probe("equal-one-cache",()->a.equals(b));
        probe("group-equality-one-cache-control",()->group.equals(a,b));b.discriminant();
        probe("equal-both-caches",()->a.equals(b));
        probe("equal-hash-contract",()->!a.equals(b)||a.hashCode()==b.hashCode());
        probe("original-reduction-values",()->{var q=form(11,49,55).reduce();return q.getA().equals(BigInteger.ONE)&&q.getB().equals(BigInteger.ONE)&&q.getC().equals(BigInteger.valueOf(5))&&q.discriminant().equals(BigInteger.valueOf(-19));});
        var p=form(2,-1,3);
        probe("group-inverse-coefficient-control",()->group.equals(group.multiply(p,group.invert(p)),group.identity()));
        probe("group-cube-coefficient-control",()->group.equals(group.multiply(p,p,p),group.identity()));
        for(int d:new int[]{-23,-251}) {
            var g=new ClassGroup(BigInteger.valueOf(d));var x=d==-23?form(2,-1,3):form(5,-3,13);int order=d==-23?3:7;
            for(int exponent=1;exponent<=order;exponent++){final int n=exponent;probe("group-power-"+(-d)+"-"+n,()->g.equals(g.power(x,n),g.identity())==(n==order));}
        }
        System.out.println("SUMMARY\t"+count);
    }
}
