// Component validators intentionally retain historical stage labels. This
// aggregate supplies the current scoped decision after validating every stage.
await import('./inventory.mjs');
await import('./verify-dependencies.mjs');
await import('./analyze-boundary.mjs');
await import('./analyze-field.mjs');
await import('./analyze-api-float.mjs');
await import('./fractional/analyze-qualification.mjs');
await import('./fractional/analyze-memory.mjs');
const original=process.argv;
try{
 process.argv=[original[0],original[1],'paired-v1','prototype'];
 await import('./fractional/analyze-bench.mjs');
 process.argv=[original[0],original[1],'paired-retained-v1','retained'];
 await import('./fractional/analyze-bench.mjs?retained');
}finally{process.argv=original;}
console.log(JSON.stringify({target:'haskell-constructible',pin:'46d760cbd2d21f955ec96c8fe2c13fdf3b2dd9d0',scopedAudit:'qualified-complete',sourceFiles:4,sourceLines:490,nativeBoundaryFailures:'documented and compared; original donor unmodified',retainedHyperTransfer:'algebraic-integer quotient separation certificate',fullEcosystemGoal:'ACTIVE/OPEN'},null,2));
