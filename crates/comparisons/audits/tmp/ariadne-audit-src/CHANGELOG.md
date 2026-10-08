## Changelog

Each release lists the issues it addresses. The issue number is followed by the kind of feature (in parentheses) and a short description. 

Legenda for the issue kind:

- N: new feature
- A: addition to a feature
- C: change to the behavior of a feature
- F: fixed feature
- R: removed feature

### 2.5.3

*Date: 03/07/2025*

- [#34](https://github.com/ariadne-cps/ariadne/issues/34) (N) Support multivalued functions (for Interval Taylor models support)
- [#328](https://github.com/ariadne-cps/ariadne/issues/328) (N) Add atan support for NormedAlgebra
- [#796](https://github.com/ariadne-cps/ariadne/issues/796) (N) Add Foundations module for (topo)logical notions
- [#569](https://github.com/ariadne-cps/ariadne/issues/569) (A) Adding multiple points simulator
- [#685](https://github.com/ariadne-cps/ariadne/issues/685) (A) Support ARM by making it properly fall into C99 rounding
- [#686](https://github.com/ariadne-cps/ariadne/issues/686) (A) Support GCC 12 on all platforms
- [#334](https://github.com/ariadne-cps/ariadne/issues/334) (C) Clean-up function models, notably template parameters
- [#337](https://github.com/ariadne-cps/ariadne/issues/337) (C) Rename partial functions to FunctionPatch
- [#665](https://github.com/ariadne-cps/ariadne/issues/665) (C) Remove Value<FloatXX>, and use plain FloatXX as exact object
- [#666](https://github.com/ariadne-cps/ariadne/issues/666) (C) Template Float classes
- [#681](https://github.com/ariadne-cps/ariadne/issues/681) (C) Refactor the logger as a ConcLog logger submodule 
- [#688](https://github.com/ariadne-cps/ariadne/issues/688) (C) Modify atan to avoid using long double, to support ARM architecture
- [#702](https://github.com/ariadne-cps/ariadne/issues/702) (C) Refactor the concurrency modules as a BetterThreads concurrency submodule
- [#788](https://github.com/ariadne-cps/ariadne/issues/788) (C) Add explicit suggest(...) for step-size suggestion to integrator classes
- [#698](https://github.com/ariadne-cps/ariadne/issues/698) (C) Use snprintf instead of the deprecated sprintf
- [#198](https://github.com/ariadne-cps/ariadne/issues/198) (C,F) Modify differential inclusion code to conform to other evolvers, fix reach set returned to be rigorous
- [#678](https://github.com/ariadne-cps/ariadne/issues/678) (F) Address warnings using AppleClang, which is now usable on C++20 Concepts code
- [#710](https://github.com/ariadne-cps/ariadne/issues/710) (F) Change use of char to short to address issue that ARM GCC char is unsigned
- [#735](https://github.com/ariadne-cps/ariadne/issues/735) (F) Fix bug in multiplication of Taylor models causing under-approximation of error
- [#741](https://github.com/ariadne-cps/ariadne/issues/741) (F) Fix fault in VectorFieldSimulator by using Approximation<FLT> instead of Real step size

### 2.4.1

*Date: 07/08/2021*

- [#357](https://github.com/ariadne-cps/ariadne/issues/357) (N) Introduce a TaylorSeriesBounderIntegrator, that identifies the bounds from the expansion without resorting to a Bounder initially
- [#613](https://github.com/ariadne-cps/ariadne/issues/613) (A) Add Python bindings for Variables2d, Projection2d fields, HybridEnclosure space accessors
- [#617](https://github.com/ariadne-cps/ariadne/issues/617) (A) Add is_polynomial_in predicate for symbolic expressions
- [#622](https://github.com/ariadne-cps/ariadne/issues/622) (A) Implement GradedTaylorPicardIterator that avoids initial use of Bounder
- [#626](https://github.com/ariadne-cps/ariadne/issues/626) (A) Allow 'none' graphics backend or drawer from CLI, to temporarily switch off graphics when desired  
- [#628](https://github.com/ariadne-cps/ariadne/issues/628) (A) Implement some methods for Polynomial, in particular for vectors thereof
- [#636](https://github.com/ariadne-cps/ariadne/issues/636) (A) Add Python bindings for hybrid automaton target method
- [#642](https://github.com/ariadne-cps/ariadne/issues/642) (A) Add Python bindings for hybrid automaton name and symbolic expressions in dynamics/guards/invariants/resets
- [#643](https://github.com/ariadne-cps/ariadne/issues/643) (A) Add C++ and Python support for iteration through components of a CompositeHybridAutomaton  
- [#612](https://github.com/ariadne-cps/ariadne/issues/612) (C) Remove unused legend from Gnuplot output
- [#639](https://github.com/ariadne-cps/ariadne/issues/639) (C) Use a finer (3/4) refinement strategy for the step size when computing a flow step based on an accuracy threshold
- [#616](https://github.com/ariadne-cps/ariadne/issues/616) (F) Fix behavior of unary symbolic expression predicates
- [#620](https://github.com/ariadne-cps/ariadne/issues/620) (F) EulerBounder did not reset the bounding domain between refinements to the step size
- [#631](https://github.com/ariadne-cps/ariadne/issues/631) (F) Conditional disabling of graphics in absence of both Cairo and Gnuplot was incorrect
- [#645](https://github.com/ariadne-cps/ariadne/issues/645) (F) Use only static const Generator objects, addressing linking issue for the whole library
- [#623](https://github.com/ariadne-cps/ariadne/issues/623) (R) Simplify IntegratorInterface with only one flow step, removing flow/flow_to and hiding flow_bounds

### 2.3

*Date: 24/05/2021*

- [#538](https://github.com/ariadne-cps/ariadne/issues/538) (N) Introduce a concurrency module designed for parallel execution of internal tasks
- [#549](https://github.com/ariadne-cps/ariadne/issues/549) (N) Introduce a Least Recently Used cache utility for holding a limited number of homogeneous objects
- [#559](https://github.com/ariadne-cps/ariadne/issues/559) (N) Introduce an optional ariadne_main.hpp that offers an ariadne_main() function handling all header tasks for an executable   
- [#562](https://github.com/ariadne-cps/ariadne/issues/562) (N) Introduce a command line interface class for acquiring CLI input arguments setting logger verbosity, theme and scheduler, along with concurrency, graphics backend and drawer  
- [#561](https://github.com/ariadne-cps/ariadne/issues/561) (A) Add Python bindings for TaskManager in order to control concurrency
- [#570](https://github.com/ariadne-cps/ariadne/issues/570) (A) Add Python bindings for GraphicsManager in order to control drawer and graphics backend  
- [#571](https://github.com/ariadne-cps/ariadne/issues/571) (A) Add Python bindings for Logger in order to expose (themed) logging
- [#599](https://github.com/ariadne-cps/ariadne/issues/599) (A) Add extra Python bindings for HybridEnclosure, add Python subscripting for ListSet<HybridEnclosure> 
- [#602](https://github.com/ariadne-cps/ariadne/issues/602) (A) Add extra Python bindings for Enclosure, add Python bindings for Point2d for graphics
- [#605](https://github.com/ariadne-cps/ariadne/issues/605) (A) Add Python bindings for CommandLineInterface, to acquire arguments to a Python script  
- [#551](https://github.com/ariadne-cps/ariadne/issues/551) (C) Make VectorFieldEvolver process sets in parallel (when splitting initially or during evolution)
- [#556](https://github.com/ariadne-cps/ariadne/issues/556) (C) Disallow unsafe default value of upper semantics for orbit methods
- [#565](https://github.com/ariadne-cps/ariadne/issues/565) (C) Draw lists of (Labelled/Hybrid)Enclosure in parallel  
- [#563](https://github.com/ariadne-cps/ariadne/issues/563) (C) Rename the 'output' module into the 'io' module to support future classes related to input  
- [#566](https://github.com/ariadne-cps/ariadne/issues/566) (C) Make HybridEvolver process sets in parallel (due to either splitting or multiple trajectories)
- [#573](https://github.com/ariadne-cps/ariadne/issues/573) (C) Use an LRU cache for modes in CompositeHybridAutomaton, avoid exhausting a given mode in HybridEvolver before changing mode
- [#580](https://github.com/ariadne-cps/ariadne/issues/580) (C) VectorFieldEvolver, IteratedMapEvolver and HybridEvolver now check that the initial enclosure is consistent
- [#592](https://github.com/ariadne-cps/ariadne/issues/592) (C) Enclosure now uses the global GraphicsManager drawer instead of having a dedicated configuration field
- [#595](https://github.com/ariadne-cps/ariadne/issues/595) (C) Modify examples to use ariadne_main function for simplicity, tutorials are not changed  
- [#539](https://github.com/ariadne-cps/ariadne/issues/539) (F) A segmentation fault sometimes would be issued when terminating the executable, due to logging
- [#557](https://github.com/ariadne-cps/ariadne/issues/557) (F) Fix behavior of StopWatch utility for concurrent code, enhance the class for choosing a duration type
- [#567](https://github.com/ariadne-cps/ariadne/issues/567) (F) Fix problem with state_time space creation when state space already contains the 't' variable
- [#576](https://github.com/ariadne-cps/ariadne/issues/576) (F) Fix HybridEnclosure state_set() incorrectly working when an auxiliary function is present
- [#578](https://github.com/ariadne-cps/ariadne/issues/578) (F) Fix VectorFieldEvolver not storing the auxiliary mapping to the initial enclosure created from an expression set
- [#581](https://github.com/ariadne-cps/ariadne/issues/581) (F) Fix Enclosure splittings not carrying over the auxiliary mapping  
- [#558](https://github.com/ariadne-cps/ariadne/issues/558) (R) Remove unnecessary IteratedMapEvolver::enclosure methods
- [#552](https://github.com/ariadne-cps/ariadne/issues/552) (R) Remove evolve/reach/reach_evolve methods from EvolverInterface, relying on orbit generation only
- [#553](https://github.com/ariadne-cps/ariadne/issues/553) (R) Remove ability to write an evolver object to the standard output, since it was implemented as a fixed string for all evolvers anyway

### 2.2

*Date: 25/04/2021*

- [#441](https://github.com/ariadne-cps/ariadne/issues/441) (N) Introduce support for Gnuplot output, including animated gif plot of sets and tridimensional plots for PDEs
- [#507](https://github.com/ariadne-cps/ariadne/issues/507) (N) Introduce a simulator for vector field dynamics
- [#514](https://github.com/ariadne-cps/ariadne/issues/514) (N) Introduce Python examples in python/examples
- [#538](https://github.com/ariadne-cps/ariadne/issues/538) (N) Introduce a concurrency module with basic primitives for parallel execution
- [#509](https://github.com/ariadne-cps/ariadne/issues/509) (A) Add support for a set as input in simulators, using the midpoint as the effective point
- [#513](https://github.com/ariadne-cps/ariadne/issues/513) (A) Add missing Python bindings for verify_safety in (Hybrid)ReachabilityAnalyser
- [#516](https://github.com/ariadne-cps/ariadne/issues/516) (A) Add missing Python bindings for Real predicates to be used in automata specification
- [#518](https://github.com/ariadne-cps/ariadne/issues/518) (A) Add missing Python bindings for evolver configuration and initial set assignment
- [#520](https://github.com/ariadne-cps/ariadne/issues/520) (A) Add missing Python bindings for plotting using HybridFigure
- [#543](https://github.com/ariadne-cps/ariadne/issues/543) (A) Add missing Python bindings for iterating across ListSet of Enclosure classes
- [#527](https://github.com/ariadne-cps/ariadne/issues/527) (A) Allow to draw a Labelled/Hybrid orbit directly to a Labelled/Hybrid figure
- [#492](https://github.com/ariadne-cps/ariadne/issues/492) (C) Change SFINAE code to use C++20 concepts, currently preventing AppleClang compilation under macOS until the compiler supports Concepts
- [#529](https://github.com/ariadne-cps/ariadne/issues/529) (C) Disallow construction of VectorField and IteratedMap from a Function, since it was broken
- [#533](https://github.com/ariadne-cps/ariadne/issues/533) (C) Map<K,V> now checks for existing key using ARIADNE_ASSERT, yielding errors also for Release builds
- [#447](https://github.com/ariadne-cps/ariadne/issues/447) (F) Check that a VectorField is defined with dynamics for all involved variables, fixes a segfault within evolution
- [#532](https://github.com/ariadne-cps/ariadne/issues/532) (F) RealExpressionBoundedConstraintSet could be constructed in an incoherent way, due to missing checks
- [#536](https://github.com/ariadne-cps/ariadne/issues/536) (F) Discrete evolution did not update the enclosure time, resulting in incorrect plots vs time  
- [#211](https://github.com/ariadne-cps/ariadne/issues/211) (R) Remove various deprecated functions

### 2.1 

*Date: 09/03/2021*

- [#412](https://github.com/ariadne-cps/ariadne/issues/412) (N) Supply Aptitude and Homebrew packages for installation
- [#267](https://github.com/ariadne-cps/ariadne/issues/267) (A) Complete Python bindings for dynamics and hybrid modules
- [#477](https://github.com/ariadne-cps/ariadne/issues/477) (C) Make the tutorials the same for C++ and Python

### 2.0

*Date: 17/04/2020*

First release.
