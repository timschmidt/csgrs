import dk.jonaslindstrom.ruffini.reals.elements.ConstructiveReal;
import dk.jonaslindstrom.ruffini.reals.structures.ConstructiveReals;
import dk.jonaslindstrom.ruffini.reals.structures.RealNumbers;
import dk.jonaslindstrom.ruffini.common.algorithms.IntegerRingEmbedding;
import dk.jonaslindstrom.ruffini.common.algorithms.Multiply;
import dk.jonaslindstrom.ruffini.common.algorithms.Power;
import java.math.BigInteger;
import java.math.BigDecimal;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicInteger;

/** Qualification only: unchanged donor classes, exact rational output checked externally. */
public final class RuffiniBoundary {
    static BigInteger floor(BigInteger n, BigInteger d) {
        BigInteger[] qr=n.divideAndRemainder(d);
        return qr[1].signum()<0 ? qr[0].subtract(BigInteger.ONE) : qr[0];
    }
    static ConstructiveReal rational(BigInteger n, BigInteger d) {
        return new ConstructiveReal(i -> i>=0 ? floor(n.shiftLeft(i),d) : floor(n,d.shiftLeft(-i)), n+"/"+d);
    }
    static void emit(String kind, String id, Object... values) {
        StringBuilder s=new StringBuilder(kind+"\t"+id);
        for(Object value:values) s.append('\t').append(value);
        System.out.println(s);
    }
    static final class ProbeStop extends RuntimeException {}
    public static void main(String[] args) throws Exception {
        if(args[0].equals("negative-reciprocal")) {
            System.out.println("ENTER\tpublic-negative-reciprocal");
            System.out.flush();
            emit("RETURN","public-negative-reciprocal",ConstructiveReal.reciprocal(ConstructiveReal.of(-1)).apply(17));
            return;
        }
        if(args[0].equals("signed-search")) {
            AtomicInteger calls=new AtomicInteger(),last=new AtomicInteger();
            ConstructiveReal x=new ConstructiveReal(i->{last.set(i); if(calls.incrementAndGet()>4096) throw new ProbeStop(); return BigInteger.ONE.negate().shiftLeft(i);},"-1");
            try { ConstructiveReal.reciprocal(x); emit("RETURN","signed-search","unexpected"); }
            catch(ProbeStop ex) { emit("BOUNDED","signed-search",calls.get(),last.get()); }
            return;
        }
        if(args[0].equals("integer-boundaries")) {
            RealNumbers r=RealNumbers.getInstance();
            for(int n:new int[]{-2147483648,-1000,-17,-2,-1,0,1,2,17,1000,2147483647}) {
                emit("SCALE","int-"+n,n,new Multiply<Double>(r).apply(n,3.0));
                try { emit("SCALE","big-"+n,n,new Multiply<Double>(r).apply(BigInteger.valueOf(n),3.0)); }
                catch(StackOverflowError ex) { emit("STACK","big-"+n); }
            }
            try { emit("RETURN","embedding-min",new IntegerRingEmbedding<Double>(r).apply(Integer.MIN_VALUE)); }
            catch(StackOverflowError ex) { emit("STACK","embedding-min"); }
            try { emit("RETURN","power-min",new Power<Double>(r).apply(1.0,Integer.MIN_VALUE)); }
            catch(StackOverflowError ex) { emit("STACK","power-min"); }
            emit("CONTROL","power-big-min",new Power<Double>(r).apply(1.0,BigInteger.valueOf(Integer.MIN_VALUE)));
            return;
        }
        if(args[0].equals("equality")) {
            ConstructiveReal a=rational(BigInteger.ZERO,BigInteger.ONE);
            ConstructiveReal b=rational(BigInteger.ONE,BigInteger.ONE.shiftLeft(17));
            ConstructiveReal c=rational(BigInteger.TWO,BigInteger.ONE.shiftLeft(17));
            emit("EQUALITY","nontransitive",a.equals(b),b.equals(c),a.equals(c),new ConstructiveReals().equals(a,b));
            ConstructiveReal one=ConstructiveReal.of(1), another=ConstructiveReal.of(1);
            HashSet<ConstructiveReal> set=new HashSet<>(); set.add(one); set.add(another);
            emit("HASH","same-integer",one.equals(another),one.hashCode(),another.hashCode(),set.size());
            AtomicInteger calls=new AtomicInteger();
            ConstructiveReal counted=new ConstructiveReal(i->{calls.incrementAndGet();return BigInteger.ONE.shiftLeft(i);},"one");
            int constructed=calls.get(); counted.apply(256); int high=calls.get();
            for(int i=0;i<10;i++) counted.apply(17);
            int coarse=calls.get();
            for(int i=0;i<10;i++) counted.equals(one);
            emit("CACHE","call-count",constructed,high,coarse,calls.get());
            return;
        }
        if(args[0].equals("format")) {
            for(int m:new int[]{0,1,2,3,4,5,8,16,17,32,64,128,256})
                for(int n:new int[]{-1000000,-3,-1,0,1,3,1000000})
                    emit("DECIMAL","int-"+n+"-"+m,m,n,ConstructiveReal.of(n).estimate(m).toPlainString());
            for(double x:new double[]{0.1,-0.1,0.5,-0.5,Math.nextUp(1.0),Double.MIN_VALUE,Double.MAX_VALUE}) {
                BigDecimal d=BigDecimal.valueOf(x);
                emit("IMPORT","double-"+Double.toHexString(x),128,d.unscaledValue(),d.scale(),ConstructiveReal.of(x).apply(128));
            }
            return;
        }
        if(args[0].equals("cache")) {
            // Floor and ceiling estimators both obey strict <1 scaled-error at every precision.
            for(int high:new int[]{16,17,32,64,128}) {
                BigInteger d=BigInteger.ONE.shiftLeft(high+2);
                for(int sign:new int[]{-1,1}) for(int delta:new int[]{-1,0,1}) {
                    BigInteger n=d.add(BigInteger.valueOf(delta)).multiply(BigInteger.valueOf(sign));
                    for(String direction:new String[]{"floor","ceil"}) {
                        ConstructiveReal x=new ConstructiveReal(i->direction.equals("floor") ? floor(n.shiftLeft(i),d) : floor(n.negate().shiftLeft(i),d).negate(),direction+"-near-integer");
                        x.apply(high);
                        for(int low:new int[]{0,1,2,8,15})
                            emit("APPROX","cache-"+direction+"-"+high+"-"+sign+"-"+delta+"-"+low,low,n,d,x.apply(low));
                    }
                }
            }
            return;
        }
        if(!args[0].equals("corpus")) throw new IllegalArgumentException("unknown mode");
        for(String row:Files.readAllLines(Path.of(args[1]))) {
            String[] f=row.split("\t"); if(f[0].equals("id")) continue;
            String id=f[0],op=f[1]; int bits=Integer.parseInt(f[2]);
            BigInteger an=new BigInteger(f[3]),ad=new BigInteger(f[4]),bn=new BigInteger(f[5]),bd=new BigInteger(f[6]);
            ConstructiveReal a=rational(an,ad),b=rational(bn,bd),v;
            if(op.equals("add")) v=ConstructiveReal.add(a,b);
            else if(op.equals("mul")) v=ConstructiveReal.multiply(a,b);
            else if(op.equals("neg")) v=ConstructiveReal.negate(a);
            else if(op.equals("inv")) v=ConstructiveReal.reciprocal(a);
            else if(op.equals("public-mul")) v=ConstructiveReal.multiply(ConstructiveReal.of(an.intValueExact()),ConstructiveReal.of(Double.parseDouble(f[7])));
            else throw new IllegalArgumentException(op);
            emit("CORPUS",id,bits,v.apply(bits));
        }
    }
}
