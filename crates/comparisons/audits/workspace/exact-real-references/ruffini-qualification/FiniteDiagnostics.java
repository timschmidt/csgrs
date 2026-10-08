import dk.jonaslindstrom.ruffini.finitefields.BigPrimeField;
import dk.jonaslindstrom.ruffini.finitefields.algorithms.*;
import dk.jonaslindstrom.ruffini.polynomials.elements.Polynomial;
import java.math.BigInteger;

/** Confirm exception identities/messages; do not relabel unrelated errors as search exhaustion. */
public final class FiniteDiagnostics {
    public static void main(String[] args) {
        int count=0;
        for(int p:new int[]{3,5,7})for(int seed:new int[]{17,42,149})for(int b=0;b<p;b++)for(int c=0;c<p;c++){
            String id="berlekamp/"+p+"/"+seed+"/"+b+"/"+c;
            try{int out=new BerlekampRabinAlgorithm(p,8,new FiniteBoundaries.BoundedRandom(seed)).apply(Polynomial.of(c,b,1));System.out.println("DETAIL\t"+id+"\tVALUE\t"+out);}
            catch(Throwable e){
                String message=String.valueOf(e.getMessage()).replace('\t',' ').replace('\n',' ');
                System.out.println("DETAIL\t"+id+"\t"+e.getClass().getSimpleName()+"\t"+message);
                if(e instanceof NullPointerException)System.out.println("FRAME\t"+id+"\t"+e.getStackTrace()[0]);
            }
            count++;
        }
        try{
            BigInteger out=new BigTonelliShanks(new BigPrimeField(new BigInteger("184683593729"))).apply(BigInteger.valueOf(9));
            System.out.println("HIGH\tVALUE\t"+out);
        }catch(Throwable e){
            System.out.println("HIGH\t"+e.getClass().getSimpleName());
            var trace=e.getStackTrace();for(int i=0;i<Math.min(3,trace.length);i++)System.out.println("HIGHFRAME\t"+trace[i]);
        }
        System.out.println("SUMMARY\t"+count);
    }
}
