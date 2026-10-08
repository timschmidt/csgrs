import {readFileSync,writeFileSync,lstatSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,repo=resolve(root,'../Ruffini');
const read=new Set(['.github/workflows/javadoc.yml','.github/workflows/maven.yml','.gitignore','CITATION.cff','LICENSE','README.md','pom.xml','reals/pom.xml','reals/src/main/java/dk/jonaslindstrom/ruffini/reals/elements/ConstructiveReal.java']);
// Generated diagram: complete envelopes and decoded SVG/UML read; opaque encoded
// bytes verified, not claimed as manually read source (root scope convention).
const reviewedAssets=new Set(['abstractions.svg']);
for (const path of [
 'common/pom.xml',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/structures/ConstructiveReals.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/structures/RealNumbers.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/elements/ComplexNumber.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/algorithms/RungeKutta.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/structures/ComplexNumbers.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/structures/RealCoordinateSpace.java',
 'reals/src/main/java/dk/jonaslindstrom/ruffini/reals/structures/ComplexCoordinateSpace.java',
 ...['AdditiveGroup','CommutativeMonoid','EuclideanDomain','Field','Group','InnerProductSpace','Module','Monoid','NormedVectorSpace','OrderedSet','Ring','SemiRing','Semigroup','Set','VectorSpace'].map(name => 'common/src/main/java/dk/jonaslindstrom/ruffini/common/abstractions/'+name+'.java'),
 'common/src/main/java/dk/jonaslindstrom/ruffini/common/util/Pair.java',
 'common/src/main/java/dk/jonaslindstrom/ruffini/common/algorithms/Multiply.java',
 'common/src/main/java/dk/jonaslindstrom/ruffini/common/algorithms/Power.java',
]) read.add(path);
for(const name of ['IntegerRingEmbedding','Sum','Product','DotProduct','QuadraticEquation','BarrettReduction','BinaryGCD','BitLength','EuclideanAlgorithm']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/algorithms/'+name+'.java');
for(const name of ['AbstractModule','AbstractVectorSpace','VectorSpaceOverField','VectorGroup','FieldOfFractions','QuotientRing']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/structures/'+name+'.java');
for(const name of ['BaseVector','ConcreteVector','ConstructiveVector','Vector']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/vector/'+name+'.java');
for(const path of ['common/src/main/java/dk/jonaslindstrom/ruffini/common/util/SamePair.java','common/src/main/java/dk/jonaslindstrom/ruffini/common/elements/Fraction.java','demos/src/main/java/demo/ConstructiveRealsDemo.java','integers/pom.xml']) read.add(path);
for(const name of ['BigIntegers','BigIntegersModuloN','BigRationals','Integers','IntegersModuloN','Rationals']) read.add('integers/src/main/java/dk/jonaslindstrom/ruffini/integers/structures/'+name+'.java');
for(const name of ['Calculator','MultiOperator','NullSafeRing','PerformanceLoggingField','PerformanceLoggingRing']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/helpers/'+name+'.java');
for(const name of ['ChineseRemainderTheorem','Projection','JacobiSymbol','BigLegendreSymbol']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/algorithms/'+name+'.java');
for(const name of ['TestUtils','SamplingUtils']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/util/'+name+'.java');
read.add('common/src/test/java/AlgorithmsTests.java');
for(const name of ['DiscreteFourierTransform','InverseDiscreteFourierTransform']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/algorithms/'+name+'.java');
for(const name of ['Determinant','GaussianElimination','GramMatrix','GramSchmidt','GramSchmidtOverRing','KroneckerProduct','MatrixAddition','MatrixInversion','MatrixMultiplication','QRDecomposition','StrassenMultiplication']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/matrices/algorithms/'+name+'.java');
for(const name of ['BaseMatrix','ConcreteMatrix','Matrix','MatrixView','MutableMatrix','SparseMatrix']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/matrices/elements/'+name+'.java');
for(const name of ['GeneralLinearGroup','MatrixRing']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/matrices/structures/'+name+'.java');
for(const name of ['ArrayUtils','MultiDimensionalArray','ArgMax','DataConversionPrimitives','EncodingUtils','MathUtils','MatrixIndex','PermutationUtils','StreamUtils','StringUtils','Triple']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/util/'+name+'.java');
for(const name of ['InvalidParametersException','NotASquareException','NotInvertibleException']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/exceptions/'+name+'.java');
for(const name of ['IntBinaryFunction','TernaryOperator','TriFunction']) read.add('common/src/main/java/dk/jonaslindstrom/ruffini/common/functional/'+name+'.java');
for(const path of ['IntegerPolynomial.java','algorithms/CongruenceSolver.java','algorithms/EulersTotientFunction.java','algorithms/ModularSquareRoot.java','algorithms/factorize/Factorize.java','algorithms/factorize/PollardRho.java','structures/limbs/BigElement.java','structures/limbs/BigElements.java']) read.add('integers/src/main/java/dk/jonaslindstrom/ruffini/integers/'+path);
read.add('integers/src/test/java/TestIntegers.java');
for(const path of ['pom.xml','src/main/java/dk/jonaslindstrom/arithmeticparser/EvaluationException.java','src/main/java/dk/jonaslindstrom/arithmeticparser/Evaluator.java','src/main/java/dk/jonaslindstrom/arithmeticparser/MultiOperator.java','src/main/java/dk/jonaslindstrom/arithmeticparser/NumberParser.java','src/main/java/dk/jonaslindstrom/arithmeticparser/Parser.java','src/main/java/dk/jonaslindstrom/arithmeticparser/Token.java','src/test/java/dk/jonaslindstrom/arithmeticparser/TestParser.java']) read.add('parser/'+path);
for(const path of ['pom.xml','src/main/java/dk/jonaslindstrom/ruffini/permutations/algorithms/RandomDerangement.java','src/main/java/dk/jonaslindstrom/ruffini/permutations/elements/Permutation.java','src/main/java/dk/jonaslindstrom/ruffini/permutations/structures/SymmetricGroup.java','src/test/java/PermutationTests.java']) read.add('permutations/'+path);
read.add('finite-fields/pom.xml');
for(const path of ['AlgebraicFieldExtension','BigFiniteField','BigPrimeField','FiniteField','GaussianRationals','PrimeField','QuadraticField','algorithms/BerlekampRabinAlgorithm','algorithms/BigTonelliShanks','algorithms/TonelliShanks']) read.add('finite-fields/src/main/java/dk/jonaslindstrom/ruffini/finitefields/'+path+'.java');
read.add('finite-fields/src/test/java/AlgorithmTests.java');
read.add('polynomials/pom.xml');
for(const path of ['algorithms/BatchPolynomialEvaluation','algorithms/BinaryTree','algorithms/CharacteristicPolynomial','algorithms/FastDivision','algorithms/GröbnerBasis','algorithms/Inversion','algorithms/KaratsubaAlgorithm','algorithms/LagrangePolynomial','algorithms/Modulus','algorithms/MultivariatePolynomialDivision','algorithms/PolynomialInterpolation','algorithms/Remainder','elements/Monomial','elements/MultivariatePolynomial','elements/Polynomial','elements/recursive/Constant','elements/recursive/Monomial','elements/recursive/Polynomial','ordering/GradedLexicographicalOrdering','ordering/LexicographicalOrdering','ordering/MonomialOrdering','structures/MultivariatePolynomialRing','structures/MultivariatePolynomialRingOverRing','structures/PolynomialRing','structures/PolynomialRingFFT','structures/PolynomialRingKaratsuba','structures/PolynomialRingOverRing']) read.add('polynomials/src/main/java/dk/jonaslindstrom/ruffini/polynomials/'+path+'.java');
read.add('polynomials/src/test/java/PolynomialTests.java');
for(const path of ['pom.xml','src/main/java/dk/jonaslindstrom/ruffini/quadraticform/ClassGroup.java','src/main/java/dk/jonaslindstrom/ruffini/quadraticform/QuadraticForm.java','test/java/QuadraticFormTests.java']) read.add('class-group/'+path);
read.add('elliptic/pom.xml');
for(const path of ['algorithms/MillersAlgorithm','algorithms/OptimalAtePairing','algorithms/WeilPairing','elements/AffinePoint','elements/EdwardsPoint','elements/JacobianPoint','elements/ProjectivePoint','structures/Curve25519','structures/EdwardsCurve','structures/MontgomeryCurve','structures/ShortWeierstrassCurveAffine','structures/ShortWeierstrassCurveProjective','structures/bls12381/BLS12381','structures/bls12381/Serialization','structures/bn254/BN254']) read.add('elliptic/src/main/java/dk/jonaslindstrom/ruffini/elliptic/'+path+'.java');
read.add('elliptic/src/test/java/TestCurve25519.java');
read.add('demos/pom.xml');
for(const path of ['AKS','BLS12381','BellPolynomials','CauchyMatrix','DeterminantFormula','ECMDemo','HadamardMatrix','MultivariatePolynomials','PolynomialMultiplication','StrassenDemo','WeilPairingDemo','poseidon/Constants','poseidon/Poseidon']) read.add('demos/src/main/java/demo/'+path+'.java');
for(const name of ['matrix00','matrix01','matrix02','matrix03','matrix04','matrix05','matrix06','matrix07','matrix08','matrix09','matrix10','matrix11','matrix12','matrix13','matrix14','matrix15']) read.add('demos/src/main/java/demo/poseidon/poseidon_constants/'+name);
for(const name of ['constants00','constants01','constants02','constants03','constants04','constants05','constants06','constants07','constants08','constants09','constants10','constants11','constants12','constants13','constants14','constants15']) read.add('demos/src/main/java/demo/poseidon/poseidon_constants/'+name);
const inventory=[],coverage=[],groups={};let totalLines=0,totalRead=0;
for(const row of readFileSync(root+'/git-index-unquoted.tsv','utf8').trimEnd().split('\n')){
 const m=/^(\d+) ([a-f0-9]{40}) 0\t(.+)$/.exec(row);if(!m||m[1]!=='100644')throw Error('unexpected index mode');
 const path=m[3],full=repo+'/'+path;if(!lstatSync(full).isFile())throw Error('nonregular file');
 const bytes=readFileSync(full),blob=createHash('sha1').update(Buffer.from('blob '+bytes.length+'\0')).update(bytes).digest('hex');
 if(blob!==m[2])throw Error('checkout differs from frozen index: '+path);
 const text=bytes.toString('utf8');if(text.includes('\0')||!Buffer.from(text).equals(bytes))throw Error('non-UTF8/text file requires separate inventory');
 const lines=text.length===0?0:text.split('\n').length-(text.endsWith('\n')?1:0),sha=createHash('sha256').update(bytes).digest('hex');
 inventory.push([path,bytes.length,lines,sha,read.has(path)?'READ':reviewedAssets.has(path)?'REVIEWED_ASSET':'UNREAD'].join('\t'));
 if(read.has(path)){coverage.push([path,lines,sha,'1-'+lines].join('\t'));totalRead+=lines;}
 const group=path.includes('/')?path.split('/')[0]:'root';groups[group]??={files:0,lines:0};groups[group].files++;groups[group].lines+=lines;totalLines+=lines;
}
if(inventory.length!==245||coverage.length!==read.size)throw Error('inventory coverage mismatch');
if(inventory.filter(row=>row.endsWith('\tREVIEWED_ASSET')).length!==reviewedAssets.size||[...reviewedAssets].some(path=>read.has(path)))throw Error('asset coverage mismatch');
writeFileSync(resolve(root,'../RUFFINI_FILE_INVENTORY.tsv'),'path\tbytes\tphysical_lines\tsha256\tread_status\n'+inventory.join('\n')+'\n');
writeFileSync(resolve(root,'../RUFFINI_READ_COVERAGE.tsv'),'path\tphysical_lines\tsha256\tread_ranges\n'+coverage.join('\n')+'\n');
console.log(JSON.stringify({pin:'82d552fee22d92e493936183fab8672517694e56',files:245,physicalLines:totalLines,readFiles:coverage.length,readLines:totalRead,reviewedAssets:[...reviewedAssets],unreadFiles:245-coverage.length-reviewedAssets.size,groups,status:'OPEN'},null,2));
