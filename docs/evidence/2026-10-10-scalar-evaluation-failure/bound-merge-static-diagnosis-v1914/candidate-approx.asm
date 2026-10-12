
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1850-bound-merge-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a6260 <<hyperreal::computable::node::Computable>::approx_signal>:
   a6260:	push   %rbp
   a6261:	push   %r15
   a6263:	push   %r14
   a6265:	push   %r13
   a6267:	push   %r12
   a6269:	push   %rbx
   a626a:	sub    $0x3a8,%rsp
   a6271:	mov    %ecx,%ebp
   a6273:	mov    %rsi,%r13
   a6276:	mov    %rdi,%rbx
   a6279:	mov    (%rdx),%r14
   a627c:	test   %r14,%r14
   a627f:	je     a628e <<hyperreal::computable::node::Computable>::approx_signal+0x2e>
   a6281:	movzbl 0x10(%r14),%eax
   a6286:	test   %al,%al
   a6288:	jne    a6430 <<hyperreal::computable::node::Computable>::approx_signal+0x1d0>
   a628e:	mov    %rdx,0x40(%rsp)
   a6293:	data16 data16 data16 mov %fs:0x0,%rax
   a629f:	lea    -0x120(%rax),%r15
   a62a6:	movzbl -0x110(%rax),%eax
   a62ad:	test   %eax,%eax
   a62af:	jne    a7230 <<hyperreal::computable::node::Computable>::approx_signal+0xfd0>
   a62b5:	movabs $0x7fffffffffffffff,%rax
   a62bf:	mov    (%r15),%rcx
   a62c2:	cmp    %rax,%rcx
   a62c5:	jae    a7260 <<hyperreal::computable::node::Computable>::approx_signal+0x1000>
   a62cb:	lea    0x1(%rcx),%rax
   a62cf:	mov    %rax,(%r15)
   a62d2:	mov    0x8(%r15),%rdx
   a62d6:	test   %rdx,%rdx
   a62d9:	je     a62f3 <<hyperreal::computable::node::Computable>::approx_signal+0x93>
   a62db:	nopl   0x0(%rax,%rax,1)
   a62e0:	cmpb   $0x0,0x20(%rdx)
   a62e4:	jne    a642d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a62ea:	mov    0x18(%rdx),%rdx
   a62ee:	test   %rdx,%rdx
   a62f1:	jne    a62e0 <<hyperreal::computable::node::Computable>::approx_signal+0x80>
   a62f3:	mov    %rcx,(%r15)
   a62f6:	mov    0x0(%r13),%r12
   a62fa:	movzbl 0x10(%r12),%edx
   a6300:	test   %edx,%edx
   a6302:	jne    a6323 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a6304:	cmpb   $0x1,0x30(%r12)
   a630a:	jne    a6323 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a630c:	movups -0x72ce3(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   a6313:	movups %xmm0,0x10(%rbx)
   a6317:	movups -0x72cfe(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   a631e:	jmp    a64ce <<hyperreal::computable::node::Computable>::approx_signal+0x26e>
   a6323:	cmp    $0xfff00000,%ebp
   a6329:	jge    a633c <<hyperreal::computable::node::Computable>::approx_signal+0xdc>
   a632b:	neg    %ebp
   a632d:	setno  %al
   a6330:	add    %al,%al
   a6332:	add    $0xa,%al
   a6334:	mov    %al,0x8(%rbx)
   a6337:	jmp    a6434 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a633c:	cmp    $0x2,%edx
   a633f:	je     a6450 <<hyperreal::computable::node::Computable>::approx_signal+0x1f0>
   a6345:	cmp    $0x1,%edx
   a6348:	je     a63af <<hyperreal::computable::node::Computable>::approx_signal+0x14f>
   a634a:	test   %edx,%edx
   a634c:	jne    a647a <<hyperreal::computable::node::Computable>::approx_signal+0x21a>
   a6352:	mov    %rax,(%r15)
   a6355:	mov    0x8(%r15),%rdx
   a6359:	test   %rdx,%rdx
   a635c:	je     a638a <<hyperreal::computable::node::Computable>::approx_signal+0x12a>
   a635e:	mov    %rdx,%rsi
   a6361:	cmpb   $0x0,0x20(%rsi)
   a6365:	jne    a63a0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a6367:	mov    0x10(%rsi),%rdi
   a636b:	mov    0x18(%rdi),%rax
   a636f:	nop
   a6370:	test   %rax,%rax
   a6373:	je     a63a0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a6375:	lea    -0x1(%rax),%r8
   a6379:	lock cmpxchg %r8,0x18(%rdi)
   a637f:	jne    a6370 <<hyperreal::computable::node::Computable>::approx_signal+0x110>
   a6381:	mov    0x18(%rsi),%rsi
   a6385:	test   %rsi,%rsi
   a6388:	jne    a6361 <<hyperreal::computable::node::Computable>::approx_signal+0x101>
   a638a:	mov    %rcx,(%r15)
   a638d:	add    $0x18,%r12
   a6391:	mov    %rbx,%rdi
   a6394:	mov    %r12,%rsi
   a6397:	jmp    a6409 <<hyperreal::computable::node::Computable>::approx_signal+0x1a9>
   a6399:	nopl   0x0(%rax)
   a63a0:	movb   $0x1,0x20(%rdx)
   a63a4:	mov    0x18(%rdx),%rdx
   a63a8:	test   %rdx,%rdx
   a63ab:	jne    a63a0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a63ad:	jmp    a642d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a63af:	mov    %rax,(%r15)
   a63b2:	mov    0x8(%r15),%rdx
   a63b6:	test   %rdx,%rdx
   a63b9:	je     a63ea <<hyperreal::computable::node::Computable>::approx_signal+0x18a>
   a63bb:	mov    %rdx,%rsi
   a63be:	cmpb   $0x0,0x20(%rsi)
   a63c2:	jne    a6420 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a63c4:	mov    0x10(%rsi),%rdi
   a63c8:	mov    0x18(%rdi),%rax
   a63cc:	nopl   0x0(%rax)
   a63d0:	test   %rax,%rax
   a63d3:	je     a6420 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a63d5:	lea    -0x1(%rax),%r8
   a63d9:	lock cmpxchg %r8,0x18(%rdi)
   a63df:	jne    a63d0 <<hyperreal::computable::node::Computable>::approx_signal+0x170>
   a63e1:	mov    0x18(%rsi),%rsi
   a63e5:	test   %rsi,%rsi
   a63e8:	jne    a63be <<hyperreal::computable::node::Computable>::approx_signal+0x15e>
   a63ea:	mov    %rcx,(%r15)
   a63ed:	lea    0x11461c(%rip),%rdi        # 1baa10 <hyperreal::computable::node::signed::ONE+0x20>
   a63f4:	lea    0x1145f5(%rip),%r14        # 1ba9f0 <hyperreal::computable::node::signed::ONE>
   a63fb:	mov    %r14,%rsi
   a63fe:	call   63070 <<std::sync::once::Once>::call_once_force::<<std::sync::lazy_lock::LazyLock<num_bigint::bigint::BigInt>>::force::{closure#0}>>
   a6403:	mov    %rbx,%rdi
   a6406:	mov    %r14,%rsi
   a6409:	mov    %ebp,%edx
   a640b:	call   be7e0 <hyperreal::computable::node::scale_at_precision_ref>
   a6410:	jmp    a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a6412:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a6420:	movb   $0x1,0x20(%rdx)
   a6424:	mov    0x18(%rdx),%rdx
   a6428:	test   %rdx,%rdx
   a642b:	jne    a6420 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a642d:	mov    %rcx,(%r15)
   a6430:	movb   $0xc,0x8(%rbx)
   a6434:	movq   $0xffffffffffffffff,(%rbx)
   a643b:	mov    %rbx,%rax
   a643e:	add    $0x3a8,%rsp
   a6445:	pop    %rbx
   a6446:	pop    %r12
   a6448:	pop    %r13
   a644a:	pop    %r14
   a644c:	pop    %r15
   a644e:	pop    %rbp
   a644f:	ret
   a6450:	movzbl 0x11(%r12),%esi
   a6456:	lea    0x210(%rsp),%rdi
   a645e:	mov    %ebp,%edx
   a6460:	call   b4f40 <<hyperreal::computable::node::Computable>::cached_constant_at_precision>
   a6465:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a646e:	jne    a64ba <<hyperreal::computable::node::Computable>::approx_signal+0x25a>
   a6470:	movl   $0x0,0xc(%rsp)
   a6478:	jmp    a64dc <<hyperreal::computable::node::Computable>::approx_signal+0x27c>
   a647a:	mov    0x38(%r12),%rax
   a647f:	test   $0x800,%eax
   a6484:	jne    a649b <<hyperreal::computable::node::Computable>::approx_signal+0x23b>
   a6486:	mov    %r13,%rdi
   a6489:	mov    0x40(%rsp),%rsi
   a648e:	call   87600 <<hyperreal::computable::node::Computable>::ensure_finite_real>
   a6493:	cmp    $0xff,%al
   a6495:	jne    a6334 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a649b:	lea    0x40(%r12),%rsi
   a64a0:	lea    0x210(%rsp),%rdi
   a64a8:	mov    %ebp,%edx
   a64aa:	call   bd300 <<hyperreal::computable::node::EvaluationCache>::at_precision>
   a64af:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a64b8:	je     a64d6 <<hyperreal::computable::node::Computable>::approx_signal+0x276>
   a64ba:	movups 0x210(%rsp),%xmm0
   a64c2:	movups 0x220(%rsp),%xmm1
   a64ca:	movups %xmm1,0x10(%rbx)
   a64ce:	movups %xmm0,(%rbx)
   a64d1:	jmp    a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a64d6:	mov    $0x1,%al
   a64d8:	mov    %eax,0xc(%rsp)
   a64dc:	lea    0x10(%r12),%rax
   a64e1:	mov    %rax,0x38(%rsp)
   a64e6:	movzbl (%rax),%eax
   a64e9:	add    $0xffffffdc,%eax
   a64ec:	cmp    $0x1a,%eax
   a64ef:	ja     a658b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a64f5:	lea    -0x71fa4(%rip),%rcx        # 34558 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0xf38>
   a64fc:	movslq (%rcx,%rax,4),%rax
   a6500:	add    %rcx,%rax
   a6503:	jmp    *%rax
   a6505:	movl   $0x1,0x30(%rsp)
   a650d:	jmp    a652b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a650f:	movl   $0x0,0x30(%rsp)
   a6517:	jmp    a652b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a6519:	movl   $0x2,0x30(%rsp)
   a6521:	jmp    a652b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a6523:	movl   $0x3,0x30(%rsp)
   a652b:	lea    0x18(%r12),%rdi
   a6530:	call   bbd50 <<hyperreal::computable::node::Computable>::half_pi_index_when_real>
   a6535:	test   $0x1,%al
   a6537:	je     a658b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a6539:	movzbl %dl,%edi
   a653c:	movl   $0xff000100,0x210(%rsp)
   a6547:	cmp    $0x3,%dil
   a654b:	ja     a7414 <<hyperreal::computable::node::Computable>::approx_signal+0x11b4>
   a6551:	movzbl 0x210(%rsp,%rdi,1),%ecx
   a6559:	movl   $0xff0001,0x210(%rsp)
   a6564:	movzbl 0x210(%rsp,%rdi,1),%eax
   a656c:	mov    0x30(%rsp),%edx
   a6570:	lea    -0x71fb3(%rip),%rsi        # 345c4 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0xfa4>
   a6577:	movslq (%rsi,%rdx,4),%rdx
   a657b:	add    %rsi,%rdx
   a657e:	jmp    *%rdx
   a6580:	mov    %ecx,%eax
   a6582:	movsbq %al,%rsi
   a6586:	jmp    a6f64 <<hyperreal::computable::node::Computable>::approx_signal+0xd04>
   a658b:	cmpb   $0x0,0xc(%rsp)
   a6590:	je     a65c5 <<hyperreal::computable::node::Computable>::approx_signal+0x365>
   a6592:	lea    0x210(%rsp),%rdi
   a659a:	mov    %r13,%rsi
   a659d:	mov    0x40(%rsp),%rdx
   a65a2:	mov    %ebp,%ecx
   a65a4:	call   a5580 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a65a9:	mov    0x210(%rsp),%r12
   a65b1:	cmp    $0xffffffffffffffff,%r12
   a65b5:	je     a7032 <<hyperreal::computable::node::Computable>::approx_signal+0xdd2>
   a65bb:	cmp    $0xfffffffffffffffe,%r12
   a65bf:	jne    a7128 <<hyperreal::computable::node::Computable>::approx_signal+0xec8>
   a65c5:	mov    0x38(%rsp),%rsi
   a65ca:	movzbl (%rsi),%eax
   a65cd:	cmp    $0xa,%eax
   a65d0:	ja     a703f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a65d6:	mov    $0x430,%ecx
   a65db:	bt     %eax,%ecx
   a65de:	jae    a703f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a65e4:	movl   $0xffffffff,0x108(%rsp)
   a65ef:	movl   $0xffffffff,0x118(%rsp)
   a65fa:	movl   $0xffffffff,0x128(%rsp)
   a6605:	movl   $0xffffffff,0x138(%rsp)
   a6610:	movl   $0xffffffff,0x148(%rsp)
   a661b:	movl   $0xffffffff,0x158(%rsp)
   a6626:	movl   $0xffffffff,0x168(%rsp)
   a6631:	movl   $0xffffffff,0x178(%rsp)
   a663c:	movl   $0xffffffff,0x188(%rsp)
   a6647:	movl   $0xffffffff,0x198(%rsp)
   a6652:	movl   $0xffffffff,0x1a8(%rsp)
   a665d:	movl   $0xffffffff,0x1b8(%rsp)
   a6668:	movl   $0xffffffff,0x1c8(%rsp)
   a6673:	movl   $0xffffffff,0x1d8(%rsp)
   a667e:	movl   $0xffffffff,0x1e8(%rsp)
   a6689:	movl   $0xffffffff,0x1f8(%rsp)
   a6694:	movq   $0x0,0x208(%rsp)
   a66a0:	movq   $0x0,0xf0(%rsp)
   a66ac:	movq   $0x8,0xf8(%rsp)
   a66b8:	movq   $0x0,0x100(%rsp)
   a66c4:	movq   $0xffffffffffffffff,0x228(%rsp)
   a66d0:	movq   $0xffffffffffffffff,0x248(%rsp)
   a66dc:	movq   $0xffffffffffffffff,0x268(%rsp)
   a66e8:	movq   $0xffffffffffffffff,0x288(%rsp)
   a66f4:	movq   $0xffffffffffffffff,0x2a8(%rsp)
   a6700:	movq   $0xffffffffffffffff,0x2c8(%rsp)
   a670c:	movq   $0xffffffffffffffff,0x2e8(%rsp)
   a6718:	movq   $0xffffffffffffffff,0x308(%rsp)
   a6724:	movq   $0x0,0x328(%rsp)
   a6730:	movq   $0x0,0x210(%rsp)
   a673c:	movq   $0x8,0x218(%rsp)
   a6748:	movq   $0x0,0x220(%rsp)
   a6754:	mov    %r13,0x18(%rsp)
   a6759:	mov    %ebp,0x14(%rsp)
   a675d:	movl   $0x0,0x10(%rsp)
   a6765:	lea    0xf0(%rsp),%rdi
   a676d:	lea    0x10(%rsp),%rsi
   a6772:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6777:	lea    0xdec22(%rip),%rax        # 1853a0 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x30>
   a677e:	mov    %rax,0x38(%rsp)
   a6783:	lea    -0x732ca(%rip),%rax        # 334c0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x360>
   a678a:	mov    %rax,0x30(%rsp)
   a678f:	mov    0x100(%rsp),%rax
   a6797:	test   %rax,%rax
   a679a:	je     a67c3 <<hyperreal::computable::node::Computable>::approx_signal+0x563>
   a679c:	dec    %rax
   a679f:	mov    %rax,0x100(%rsp)
   a67a7:	mov    0xf8(%rsp),%rcx
   a67af:	shl    $0x4,%rax
   a67b3:	mov    (%rcx,%rax,1),%r12d
   a67b7:	mov    0x4(%rcx,%rax,1),%r9d
   a67bc:	mov    0x8(%rcx,%rax,1),%r13
   a67c1:	jmp    a680b <<hyperreal::computable::node::Computable>::approx_signal+0x5ab>
   a67c3:	mov    0x208(%rsp),%rax
   a67cb:	test   %rax,%rax
   a67ce:	je     a71fc <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a67d4:	lea    -0x1(%rax),%rdi
   a67d8:	mov    %rdi,0x208(%rsp)
   a67e0:	cmp    $0x10,%rax
   a67e4:	ja     a7426 <<hyperreal::computable::node::Computable>::approx_signal+0x11c6>
   a67ea:	shl    $0x4,%rdi
   a67ee:	lea    0x108(%rsp),%rax
   a67f6:	mov    (%rax,%rdi,1),%r12d
   a67fa:	mov    0x4(%rax,%rdi,1),%r9d
   a67ff:	mov    0x8(%rax,%rdi,1),%r13
   a6804:	movl   $0xffffffff,(%rax,%rdi,1)
   a680b:	cmp    $0xffffffff,%r12d
   a680f:	je     a71fc <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a6815:	test   %r14,%r14
   a6818:	je     a6827 <<hyperreal::computable::node::Computable>::approx_signal+0x5c7>
   a681a:	movzbl 0x10(%r14),%eax
   a681f:	test   %al,%al
   a6821:	jne    a6f20 <<hyperreal::computable::node::Computable>::approx_signal+0xcc0>
   a6827:	movzbl 0x10(%r15),%eax
   a682c:	test   %eax,%eax
   a682e:	jne    a6eb6 <<hyperreal::computable::node::Computable>::approx_signal+0xc56>
   a6834:	mov    (%r15),%rcx
   a6837:	movabs $0x7fffffffffffffff,%rax
   a6841:	cmp    %rax,%rcx
   a6844:	jae    a6ef0 <<hyperreal::computable::node::Computable>::approx_signal+0xc90>
   a684a:	lea    0x1(%rcx),%rax
   a684e:	mov    %rax,(%r15)
   a6851:	mov    0x8(%r15),%rdx
   a6855:	test   %rdx,%rdx
   a6858:	je     a688e <<hyperreal::computable::node::Computable>::approx_signal+0x62e>
   a685a:	mov    %rdx,%rsi
   a685d:	cmpb   $0x0,0x20(%rsi)
   a6861:	jne    a6f10 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a6867:	mov    0x10(%rsi),%rdi
   a686b:	mov    0x18(%rdi),%rax
   a686f:	nop
   a6870:	test   %rax,%rax
   a6873:	je     a6f10 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a6879:	lea    -0x1(%rax),%r8
   a687d:	lock cmpxchg %r8,0x18(%rdi)
   a6883:	jne    a6870 <<hyperreal::computable::node::Computable>::approx_signal+0x610>
   a6885:	mov    0x18(%rsi),%rsi
   a6889:	test   %rsi,%rsi
   a688c:	jne    a685d <<hyperreal::computable::node::Computable>::approx_signal+0x5fd>
   a688e:	mov    %r9d,0xc(%rsp)
   a6893:	mov    %rcx,(%r15)
   a6896:	mov    %r12d,%eax
   a6899:	lea    -0x722cc(%rip),%rcx        # 345d4 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0xfb4>
   a68a0:	movslq (%rcx,%rax,4),%rax
   a68a4:	add    %rcx,%rax
   a68a7:	jmp    *%rax
   a68a9:	mov    0x0(%r13),%rsi
   a68ad:	cmpb   $0x0,0x10(%rsi)
   a68b1:	jne    a68c0 <<hyperreal::computable::node::Computable>::approx_signal+0x660>
   a68b3:	cmpb   $0x1,0x30(%rsi)
   a68b7:	lea    -0x7329e(%rip),%rax        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   a68be:	je     a68f4 <<hyperreal::computable::node::Computable>::approx_signal+0x694>
   a68c0:	mov    0xc(%rsp),%edx
   a68c4:	cmp    $0xffefffff,%edx
   a68ca:	jle    a72f5 <<hyperreal::computable::node::Computable>::approx_signal+0x1095>
   a68d0:	lea    0x388(%rsp),%rdi
   a68d8:	call   b2960 <<hyperreal::computable::node::Computable>::cached_at_precision>
   a68dd:	cmpq   $0xffffffffffffffff,0x388(%rsp)
   a68e6:	lea    0x388(%rsp),%rax
   a68ee:	je     a6b83 <<hyperreal::computable::node::Computable>::approx_signal+0x923>
   a68f4:	lea    0x210(%rsp),%rdi
   a68fc:	mov    %rax,%rsi
   a68ff:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6904:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6909:	lea    0x10(%rsp),%rdi
   a690e:	lea    0x210(%rsp),%rsi
   a6916:	call   a5af0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a691b:	mov    %r13,0x68(%rsp)
   a6920:	mov    0x10(%rsp),%r12
   a6925:	cmp    $0xffffffffffffffff,%r12
   a6929:	je     a739b <<hyperreal::computable::node::Computable>::approx_signal+0x113b>
   a692f:	mov    0x18(%rsp),%r13
   a6934:	lea    0x29(%rsp),%rax
   a6939:	movups -0x9(%rax),%xmm0
   a693d:	movaps %xmm0,0x350(%rsp)
   a6945:	lea    0x10(%rsp),%rdi
   a694a:	lea    0x210(%rsp),%rsi
   a6952:	call   a5af0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a6957:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a695d:	je     a73c1 <<hyperreal::computable::node::Computable>::approx_signal+0x1161>
   a6963:	movups 0x10(%rsp),%xmm0
   a6968:	movups 0x20(%rsp),%xmm1
   a696d:	movaps %xmm1,0x370(%rsp)
   a6975:	movaps %xmm0,0x360(%rsp)
   a697d:	mov    %r12,0x10(%rsp)
   a6982:	mov    %r13,0x18(%rsp)
   a6987:	movaps 0x350(%rsp),%xmm0
   a698f:	lea    0x29(%rsp),%rax
   a6994:	movups %xmm0,-0x9(%rax)
   a6998:	lea    0x48(%rsp),%rbp
   a699d:	mov    %rbp,%rdi
   a69a0:	lea    0x360(%rsp),%rsi
   a69a8:	lea    0x10(%rsp),%rdx
   a69ad:	call   c25e0 <<num_bigint::bigint::BigInt as core::ops::arith::Add>::add>
   a69b2:	lea    0x330(%rsp),%r13
   a69ba:	mov    %r13,%rdi
   a69bd:	mov    %rbp,%rsi
   a69c0:	mov    $0xfffffffe,%edx
   a69c5:	call   becc0 <hyperreal::computable::node::scale>
   a69ca:	movzbl 0x348(%rsp),%r12d
   a69d3:	mov    %rbp,%rdi
   a69d6:	mov    %r13,%rsi
   a69d9:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a69de:	mov    0x58(%rsp),%rax
   a69e3:	mov    %rax,0x20(%rsp)
   a69e8:	movups 0x48(%rsp),%xmm0
   a69ed:	movaps %xmm0,0x10(%rsp)
   a69f2:	mov    %r12b,0x28(%rsp)
   a69f7:	mov    0x68(%rsp),%rdi
   a69fc:	mov    %r14,%rsi
   a69ff:	mov    0xc(%rsp),%edx
   a6a03:	lea    0x10(%rsp),%rcx
   a6a08:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6a0d:	cmp    $0xff,%al
   a6a0f:	jne    a72be <<hyperreal::computable::node::Computable>::approx_signal+0x105e>
   a6a15:	lea    0x210(%rsp),%rdi
   a6a1d:	mov    %r13,%rsi
   a6a20:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6a25:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6a2a:	lea    0x10(%rsp),%rdi
   a6a2f:	lea    0x210(%rsp),%rsi
   a6a37:	call   a5af0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a6a3c:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a6a42:	je     a737c <<hyperreal::computable::node::Computable>::approx_signal+0x111c>
   a6a48:	movups 0x10(%rsp),%xmm0
   a6a4d:	movups 0x20(%rsp),%xmm1
   a6a52:	movaps %xmm1,0xa0(%rsp)
   a6a5a:	movaps %xmm0,0x90(%rsp)
   a6a62:	movzbl 0xa8(%rsp),%r12d
   a6a6b:	lea    0x48(%rsp),%rdi
   a6a70:	lea    0x90(%rsp),%rsi
   a6a78:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a6a7d:	mov    0xc(%rsp),%edx
   a6a81:	mov    0x58(%rsp),%rax
   a6a86:	mov    %rax,0x20(%rsp)
   a6a8b:	movups 0x48(%rsp),%xmm0
   a6a90:	movaps %xmm0,0x10(%rsp)
   a6a95:	mov    %r12b,0x28(%rsp)
   a6a9a:	mov    %r13,%rdi
   a6a9d:	mov    %r14,%rsi
   a6aa0:	lea    0x10(%rsp),%rcx
   a6aa5:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6aaa:	cmp    $0xff,%al
   a6aac:	jne    a727a <<hyperreal::computable::node::Computable>::approx_signal+0x101a>
   a6ab2:	lea    0x210(%rsp),%rdi
   a6aba:	lea    0x90(%rsp),%rsi
   a6ac2:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6ac7:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6acc:	lea    0x10(%rsp),%rdi
   a6ad1:	lea    0x210(%rsp),%rsi
   a6ad9:	call   a5af0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a6ade:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a6ae4:	je     a73ba <<hyperreal::computable::node::Computable>::approx_signal+0x115a>
   a6aea:	mov    0x20(%rsp),%rax
   a6aef:	mov    %rax,0x80(%rsp)
   a6af7:	movups 0x10(%rsp),%xmm0
   a6afc:	movaps %xmm0,0x70(%rsp)
   a6b01:	lea    0x29(%rsp),%rcx
   a6b06:	mov    (%rcx),%eax
   a6b08:	mov    $0x2,%r12b
   a6b0b:	sub    0x28(%rsp),%r12b
   a6b10:	mov    0x3(%rcx),%ecx
   a6b13:	lea    0x89(%rsp),%rdx
   a6b1b:	mov    %ecx,0x3(%rdx)
   a6b1e:	mov    %eax,(%rdx)
   a6b20:	mov    %r12b,0x88(%rsp)
   a6b28:	lea    0x48(%rsp),%rdi
   a6b2d:	lea    0x70(%rsp),%rsi
   a6b32:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a6b37:	mov    0xc(%rsp),%edx
   a6b3b:	mov    0x58(%rsp),%rax
   a6b40:	mov    %rax,0x20(%rsp)
   a6b45:	movups 0x48(%rsp),%xmm0
   a6b4a:	movaps %xmm0,0x10(%rsp)
   a6b4f:	mov    %r12b,0x28(%rsp)
   a6b54:	mov    %r13,%rdi
   a6b57:	mov    %r14,%rsi
   a6b5a:	lea    0x10(%rsp),%rcx
   a6b5f:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6b64:	cmp    $0xff,%al
   a6b66:	jne    a729f <<hyperreal::computable::node::Computable>::approx_signal+0x103f>
   a6b6c:	lea    0x210(%rsp),%rdi
   a6b74:	lea    0x70(%rsp),%rsi
   a6b79:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6b7e:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6b83:	lea    0x10(%rsp),%rdi
   a6b88:	mov    %r13,%rsi
   a6b8b:	mov    0x40(%rsp),%rdx
   a6b90:	mov    0xc(%rsp),%ecx
   a6b94:	call   a5580 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a6b99:	mov    0x10(%rsp),%r12
   a6b9e:	cmp    $0xfffffffffffffffe,%r12
   a6ba2:	je     a6c4f <<hyperreal::computable::node::Computable>::approx_signal+0x9ef>
   a6ba8:	cmp    $0xffffffffffffffff,%r12
   a6bac:	je     a7306 <<hyperreal::computable::node::Computable>::approx_signal+0x10a6>
   a6bb2:	mov    %r13,%rbp
   a6bb5:	movzbl 0x18(%rsp),%eax
   a6bba:	lea    0x29(%rsp),%rdx
   a6bbf:	mov    -0x1(%rdx),%rcx
   a6bc3:	lea    0xb9(%rsp),%rsi
   a6bcb:	mov    %rcx,0xf(%rsi)
   a6bcf:	movups -0x10(%rdx),%xmm0
   a6bd3:	movups %xmm0,(%rsi)
   a6bd6:	mov    %r12,0xb0(%rsp)
   a6bde:	mov    %al,0xb8(%rsp)
   a6be5:	movzbl 0xc8(%rsp),%r13d
   a6bee:	lea    0x48(%rsp),%rdi
   a6bf3:	lea    0xb0(%rsp),%rsi
   a6bfb:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a6c00:	mov    0x58(%rsp),%rax
   a6c05:	mov    %rax,0x20(%rsp)
   a6c0a:	movups 0x48(%rsp),%xmm0
   a6c0f:	movaps %xmm0,0x10(%rsp)
   a6c14:	mov    %r13b,0x28(%rsp)
   a6c19:	mov    %rbp,%rdi
   a6c1c:	mov    %r14,%rsi
   a6c1f:	mov    0xc(%rsp),%edx
   a6c23:	lea    0x10(%rsp),%rcx
   a6c28:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6c2d:	cmp    $0xff,%al
   a6c2f:	jne    a7313 <<hyperreal::computable::node::Computable>::approx_signal+0x10b3>
   a6c35:	lea    0x210(%rsp),%rdi
   a6c3d:	lea    0xb0(%rsp),%rsi
   a6c45:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6c4a:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6c4f:	mov    0x0(%r13),%r12
   a6c53:	movzbl 0x10(%r12),%eax
   a6c59:	cmp    $0xa,%eax
   a6c5c:	je     a6d86 <<hyperreal::computable::node::Computable>::approx_signal+0xb26>
   a6c62:	cmp    $0x5,%eax
   a6c65:	je     a6cc7 <<hyperreal::computable::node::Computable>::approx_signal+0xa67>
   a6c67:	cmp    $0x4,%eax
   a6c6a:	jne    a6df0 <<hyperreal::computable::node::Computable>::approx_signal+0xb90>
   a6c70:	mov    %r13,0x18(%rsp)
   a6c75:	mov    0xc(%rsp),%eax
   a6c79:	mov    %eax,0x14(%rsp)
   a6c7d:	movl   $0x1,0x10(%rsp)
   a6c85:	lea    0xf0(%rsp),%rdi
   a6c8d:	lea    0x10(%rsp),%rsi
   a6c92:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6c97:	add    $0x18,%r12
   a6c9b:	mov    %r12,0x18(%rsp)
   a6ca0:	mov    0xc(%rsp),%eax
   a6ca4:	mov    %eax,0x14(%rsp)
   a6ca8:	movl   $0x0,0x10(%rsp)
   a6cb0:	lea    0xf0(%rsp),%rdi
   a6cb8:	lea    0x10(%rsp),%rsi
   a6cbd:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6cc2:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6cc7:	mov    %r13,0x18(%rsp)
   a6ccc:	mov    0xc(%rsp),%eax
   a6cd0:	mov    %eax,0x14(%rsp)
   a6cd4:	movl   $0x2,0x10(%rsp)
   a6cdc:	lea    0xf0(%rsp),%rdi
   a6ce4:	lea    0x10(%rsp),%rsi
   a6ce9:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6cee:	lea    0x28(%r12),%rdx
   a6cf3:	lea    0x18(%r12),%rbp
   a6cf8:	mov    0x28(%r12),%rax
   a6cfd:	mov    0x38(%rax),%r13
   a6d01:	shr    $0x10,%r13d
   a6d05:	mov    0x18(%r12),%rax
   a6d0a:	mov    0x38(%rax),%r12
   a6d0e:	shr    $0x10,%r12d
   a6d12:	mov    0xc(%rsp),%ecx
   a6d16:	add    $0xfffffffe,%ecx
   a6d19:	cmp    %r12w,%r13w
   a6d1d:	mov    %rdx,0x68(%rsp)
   a6d22:	mov    %rdx,%rax
   a6d25:	cmovg  %rbp,%rax
   a6d29:	mov    %rax,0x18(%rsp)
   a6d2e:	mov    %ecx,0xc(%rsp)
   a6d32:	mov    %ecx,0x14(%rsp)
   a6d36:	movl   $0x0,0x10(%rsp)
   a6d3e:	lea    0xf0(%rsp),%rdi
   a6d46:	lea    0x10(%rsp),%rsi
   a6d4b:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6d50:	cmp    %r12w,%r13w
   a6d54:	cmovg  0x68(%rsp),%rbp
   a6d5a:	mov    %rbp,0x18(%rsp)
   a6d5f:	mov    0xc(%rsp),%eax
   a6d63:	mov    %eax,0x14(%rsp)
   a6d67:	movl   $0x0,0x10(%rsp)
   a6d6f:	lea    0xf0(%rsp),%rdi
   a6d77:	lea    0x10(%rsp),%rsi
   a6d7c:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6d81:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6d86:	mov    %r13,0x18(%rsp)
   a6d8b:	mov    0xc(%rsp),%eax
   a6d8f:	mov    %eax,0x14(%rsp)
   a6d93:	movl   $0x3,0x10(%rsp)
   a6d9b:	lea    0xf0(%rsp),%rdi
   a6da3:	lea    0x10(%rsp),%rsi
   a6da8:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6dad:	mov    0xc(%rsp),%eax
   a6db1:	sub    0x14(%r12),%eax
   a6db6:	mov    %eax,0xc(%rsp)
   a6dba:	jo     a740b <<hyperreal::computable::node::Computable>::approx_signal+0x11ab>
   a6dc0:	add    $0x18,%r12
   a6dc4:	mov    %r12,0x18(%rsp)
   a6dc9:	mov    0xc(%rsp),%eax
   a6dcd:	mov    %eax,0x14(%rsp)
   a6dd1:	movl   $0x0,0x10(%rsp)
   a6dd9:	lea    0xf0(%rsp),%rdi
   a6de1:	lea    0x10(%rsp),%rsi
   a6de6:	call   a5c60 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a6deb:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6df0:	add    $0x10,%r12
   a6df4:	lea    0x10(%rsp),%rdi
   a6df9:	mov    %r12,%rsi
   a6dfc:	mov    0x40(%rsp),%rdx
   a6e01:	mov    0xc(%rsp),%ecx
   a6e05:	call   ec6a0 <<hyperreal::computable::approximation::Approximation>::approximate>
   a6e0a:	mov    0x10(%rsp),%r12
   a6e0f:	movzbl 0x18(%rsp),%eax
   a6e14:	cmp    $0xffffffffffffffff,%r12
   a6e18:	je     a7357 <<hyperreal::computable::node::Computable>::approx_signal+0x10f7>
   a6e1e:	mov    %r13,%rbp
   a6e21:	lea    0x29(%rsp),%rdx
   a6e26:	mov    -0x1(%rdx),%rcx
   a6e2a:	lea    0xd9(%rsp),%rsi
   a6e32:	mov    %rcx,0xf(%rsi)
   a6e36:	movups -0x10(%rdx),%xmm0
   a6e3a:	movups %xmm0,(%rsi)
   a6e3d:	mov    %r12,0xd0(%rsp)
   a6e45:	mov    %al,0xd8(%rsp)
   a6e4c:	movzbl 0xe8(%rsp),%r13d
   a6e55:	lea    0x48(%rsp),%rdi
   a6e5a:	lea    0xd0(%rsp),%rsi
   a6e62:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a6e67:	mov    0x58(%rsp),%rax
   a6e6c:	mov    %rax,0x20(%rsp)
   a6e71:	movups 0x48(%rsp),%xmm0
   a6e76:	movaps %xmm0,0x10(%rsp)
   a6e7b:	mov    %r13b,0x28(%rsp)
   a6e80:	mov    %rbp,%rdi
   a6e83:	mov    %r14,%rsi
   a6e86:	mov    0xc(%rsp),%edx
   a6e8a:	lea    0x10(%rsp),%rcx
   a6e8f:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6e94:	cmp    $0xff,%al
   a6e96:	jne    a735f <<hyperreal::computable::node::Computable>::approx_signal+0x10ff>
   a6e9c:	lea    0x210(%rsp),%rdi
   a6ea4:	lea    0xd0(%rsp),%rsi
   a6eac:	call   a5b80 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a6eb1:	jmp    a678f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a6eb6:	cmp    $0x2,%eax
   a6eb9:	je     a7345 <<hyperreal::computable::node::Computable>::approx_signal+0x10e5>
   a6ebf:	mov    %r9d,%ebp
   a6ec2:	mov    %r15,%rdi
   a6ec5:	lea    0x850d4(%rip),%rsi        # 12bfa0 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a6ecc:	call   *0xe267e(%rip)        # 189550 <_DYNAMIC+0x3d0>
   a6ed2:	movb   $0x0,0x10(%r15)
   a6ed7:	mov    %ebp,%r9d
   a6eda:	mov    (%r15),%rcx
   a6edd:	movabs $0x7fffffffffffffff,%rax
   a6ee7:	cmp    %rax,%rcx
   a6eea:	jb     a684a <<hyperreal::computable::node::Computable>::approx_signal+0x5ea>
   a6ef0:	lea    0xdf821(%rip),%rdi        # 186718 <anon.953f3db8f5a996f677fb6a53175227a4.23.llvm.5533634630175563929>
   a6ef7:	call   *0xe265b(%rip)        # 189558 <_DYNAMIC+0x3d8>
   a6efd:	jmp    a7438 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a6f02:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a6f10:	movb   $0x1,0x20(%rdx)
   a6f14:	mov    0x18(%rdx),%rdx
   a6f18:	test   %rdx,%rdx
   a6f1b:	jne    a6f10 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a6f1d:	mov    %rcx,(%r15)
   a6f20:	movb   $0xc,0x8(%rbx)
   a6f24:	movq   $0xffffffffffffffff,(%rbx)
   a6f2b:	lea    0x210(%rsp),%rdi
   a6f33:	call   80140 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a6f38:	mov    0xf0(%rsp),%rsi
   a6f40:	test   %rsi,%rsi
   a6f43:	je     a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a6f49:	mov    0xf8(%rsp),%rdi
   a6f51:	shl    $0x4,%rsi
   a6f55:	jmp    a7011 <<hyperreal::computable::node::Computable>::approx_signal+0xdb1>
   a6f5a:	test   %al,%al
   a6f5c:	je     a7029 <<hyperreal::computable::node::Computable>::approx_signal+0xdc9>
   a6f62:	xor    %esi,%esi
   a6f64:	lea    0x10(%rsp),%rdi
   a6f69:	call   c2c50 <<num_bigint::bigint::BigInt as core::convert::From<i64>>::from>
   a6f6e:	add    $0x38,%r12
   a6f72:	mov    0x28(%rsp),%esi
   a6f76:	mov    %r12,%rdi
   a6f79:	call   a87c0 <<hyperreal::computable::node::AtomicFacts>::publish_exact_sign>
   a6f7e:	lea    0xf0(%rsp),%r15
   a6f86:	lea    0x10(%rsp),%rsi
   a6f8b:	mov    %r15,%rdi
   a6f8e:	mov    %ebp,%edx
   a6f90:	call   be600 <hyperreal::computable::node::scale_at_precision>
   a6f95:	movzbl 0x108(%rsp),%r12d
   a6f9e:	lea    0x48(%rsp),%rdi
   a6fa3:	mov    %r15,%rsi
   a6fa6:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a6fab:	mov    0x58(%rsp),%rax
   a6fb0:	mov    %rax,0x220(%rsp)
   a6fb8:	movups 0x48(%rsp),%xmm0
   a6fbd:	movaps %xmm0,0x210(%rsp)
   a6fc5:	mov    %r12b,0x228(%rsp)
   a6fcd:	lea    0x210(%rsp),%rcx
   a6fd5:	mov    %r13,%rdi
   a6fd8:	mov    %r14,%rsi
   a6fdb:	mov    %ebp,%edx
   a6fdd:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a6fe2:	cmp    $0xff,%al
   a6fe4:	je     a71e7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a6fea:	mov    %al,0x8(%rbx)
   a6fed:	movq   $0xffffffffffffffff,(%rbx)
   a6ff4:	mov    0xf0(%rsp),%rsi
   a6ffc:	test   %rsi,%rsi
   a6fff:	je     a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a7005:	mov    0xf8(%rsp),%rdi
   a700d:	shl    $0x3,%rsi
   a7011:	mov    $0x8,%edx
   a7016:	call   *0xe23ec(%rip)        # 189408 <_DYNAMIC+0x288>
   a701c:	jmp    a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a7021:	test   %cl,%cl
   a7023:	jne    a6f62 <<hyperreal::computable::node::Computable>::approx_signal+0xd02>
   a7029:	movb   $0x2,0x8(%rbx)
   a702d:	jmp    a6434 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a7032:	movzbl 0x218(%rsp),%eax
   a703a:	jmp    a6334 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a703f:	lea    0x210(%rsp),%rdi
   a7047:	mov    0x40(%rsp),%rdx
   a704c:	mov    %ebp,%ecx
   a704e:	call   ec6a0 <<hyperreal::computable::approximation::Approximation>::approximate>
   a7053:	mov    0x210(%rsp),%r15
   a705b:	movzbl 0x218(%rsp),%eax
   a7063:	cmp    $0xffffffffffffffff,%r15
   a7067:	je     a6334 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a706d:	mov    0x228(%rsp),%rcx
   a7075:	mov    %rcx,0x108(%rsp)
   a707d:	movups 0x219(%rsp),%xmm0
   a7085:	movups %xmm0,0xf9(%rsp)
   a708d:	mov    %r15,0xf0(%rsp)
   a7095:	mov    %al,0xf8(%rsp)
   a709c:	movzbl 0x108(%rsp),%r12d
   a70a5:	lea    0x10(%rsp),%rdi
   a70aa:	lea    0xf0(%rsp),%rsi
   a70b2:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a70b7:	mov    0x20(%rsp),%rax
   a70bc:	mov    %rax,0x220(%rsp)
   a70c4:	movups 0x10(%rsp),%xmm0
   a70c9:	movaps %xmm0,0x210(%rsp)
   a70d1:	mov    %r12b,0x228(%rsp)
   a70d9:	lea    0x210(%rsp),%rcx
   a70e1:	mov    %r13,%rdi
   a70e4:	mov    %r14,%rsi
   a70e7:	mov    %ebp,%edx
   a70e9:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a70ee:	cmp    $0xff,%al
   a70f0:	je     a71e7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a70f6:	mov    %al,0x8(%rbx)
   a70f9:	movq   $0xffffffffffffffff,(%rbx)
   a7100:	test   %r15,%r15
   a7103:	je     a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a7109:	mov    0xf8(%rsp),%rdi
   a7111:	shl    $0x3,%r15
   a7115:	mov    $0x8,%edx
   a711a:	mov    %r15,%rsi
   a711d:	call   *0xe22e5(%rip)        # 189408 <_DYNAMIC+0x288>
   a7123:	jmp    a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a7128:	movzbl 0x218(%rsp),%eax
   a7130:	mov    0x228(%rsp),%rcx
   a7138:	mov    %rcx,0x108(%rsp)
   a7140:	movups 0x219(%rsp),%xmm0
   a7148:	movups %xmm0,0xf9(%rsp)
   a7150:	mov    %r12,0xf0(%rsp)
   a7158:	mov    %al,0xf8(%rsp)
   a715f:	movzbl 0x108(%rsp),%r15d
   a7168:	lea    0x10(%rsp),%rdi
   a716d:	lea    0xf0(%rsp),%rsi
   a7175:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a717a:	mov    0x20(%rsp),%rax
   a717f:	mov    %rax,0x220(%rsp)
   a7187:	movups 0x10(%rsp),%xmm0
   a718c:	movaps %xmm0,0x210(%rsp)
   a7194:	mov    %r15b,0x228(%rsp)
   a719c:	lea    0x210(%rsp),%rcx
   a71a4:	mov    %r13,%rdi
   a71a7:	mov    %r14,%rsi
   a71aa:	mov    %ebp,%edx
   a71ac:	call   b22f0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a71b1:	cmp    $0xff,%al
   a71b3:	je     a71e7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a71b5:	mov    %al,0x8(%rbx)
   a71b8:	movq   $0xffffffffffffffff,(%rbx)
   a71bf:	test   %r12,%r12
   a71c2:	je     a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a71c8:	mov    0xf8(%rsp),%rdi
   a71d0:	shl    $0x3,%r12
   a71d4:	mov    $0x8,%edx
   a71d9:	mov    %r12,%rsi
   a71dc:	call   *0xe2226(%rip)        # 189408 <_DYNAMIC+0x288>
   a71e2:	jmp    a643b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a71e7:	movups 0xf0(%rsp),%xmm0
   a71ef:	movups 0x100(%rsp),%xmm1
   a71f7:	jmp    a64ca <<hyperreal::computable::node::Computable>::approx_signal+0x26a>
   a71fc:	lea    0x10(%rsp),%rdi
   a7201:	lea    0x210(%rsp),%rsi
   a7209:	call   a5af0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a720e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a7214:	je     a73dc <<hyperreal::computable::node::Computable>::approx_signal+0x117c>
   a721a:	movups 0x10(%rsp),%xmm0
   a721f:	movups 0x20(%rsp),%xmm1
   a7224:	movups %xmm1,0x10(%rbx)
   a7228:	movups %xmm0,(%rbx)
   a722b:	jmp    a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a7230:	cmp    $0x1,%eax
   a7233:	jne    a726d <<hyperreal::computable::node::Computable>::approx_signal+0x100d>
   a7235:	lea    0x84d64(%rip),%rsi        # 12bfa0 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a723c:	mov    %r15,%rdi
   a723f:	call   *0xe230b(%rip)        # 189550 <_DYNAMIC+0x3d0>
   a7245:	movb   $0x0,0x10(%r15)
   a724a:	movabs $0x7fffffffffffffff,%rax
   a7254:	mov    (%r15),%rcx
   a7257:	cmp    %rax,%rcx
   a725a:	jb     a62cb <<hyperreal::computable::node::Computable>::approx_signal+0x6b>
   a7260:	lea    0xdf4c9(%rip),%rdi        # 186730 <anon.953f3db8f5a996f677fb6a53175227a4.24.llvm.5533634630175563929>
   a7267:	call   *0xe22eb(%rip)        # 189558 <_DYNAMIC+0x3d8>
   a726d:	lea    0xdf3f4(%rip),%rdi        # 186668 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.5533634630175563929>
   a7274:	call   *0xe22e6(%rip)        # 189560 <_DYNAMIC+0x3e0>
   a727a:	mov    %al,0x8(%rbx)
   a727d:	movq   $0xffffffffffffffff,(%rbx)
   a7284:	mov    0x90(%rsp),%rsi
   a728c:	test   %rsi,%rsi
   a728f:	je     a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a7295:	mov    0x98(%rsp),%rdi
   a729d:	jmp    a72e1 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a729f:	mov    %al,0x8(%rbx)
   a72a2:	movq   $0xffffffffffffffff,(%rbx)
   a72a9:	mov    0x70(%rsp),%rsi
   a72ae:	test   %rsi,%rsi
   a72b1:	je     a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a72b7:	mov    0x78(%rsp),%rdi
   a72bc:	jmp    a72e1 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a72be:	mov    %al,0x8(%rbx)
   a72c1:	movq   $0xffffffffffffffff,(%rbx)
   a72c8:	mov    0x330(%rsp),%rsi
   a72d0:	test   %rsi,%rsi
   a72d3:	je     a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a72d9:	mov    0x338(%rsp),%rdi
   a72e1:	shl    $0x3,%rsi
   a72e5:	mov    $0x8,%edx
   a72ea:	call   *0xe2118(%rip)        # 189408 <_DYNAMIC+0x288>
   a72f0:	jmp    a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a72f5:	neg    %edx
   a72f7:	setno  %al
   a72fa:	add    %al,%al
   a72fc:	add    $0xa,%al
   a72fe:	mov    %al,0x8(%rbx)
   a7301:	jmp    a6f24 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a7306:	movzbl 0x18(%rsp),%eax
   a730b:	mov    %al,0x8(%rbx)
   a730e:	jmp    a6f24 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a7313:	mov    %al,0x8(%rbx)
   a7316:	movq   $0xffffffffffffffff,(%rbx)
   a731d:	test   %r12,%r12
   a7320:	je     a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a7326:	mov    0xb8(%rsp),%rdi
   a732e:	shl    $0x3,%r12
   a7332:	mov    $0x8,%edx
   a7337:	mov    %r12,%rsi
   a733a:	call   *0xe20c8(%rip)        # 189408 <_DYNAMIC+0x288>
   a7340:	jmp    a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a7345:	lea    0xdf31c(%rip),%rdi        # 186668 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.5533634630175563929>
   a734c:	call   *0xe220e(%rip)        # 189560 <_DYNAMIC+0x3e0>
   a7352:	jmp    a7438 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a7357:	mov    %al,0x8(%rbx)
   a735a:	jmp    a6f24 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a735f:	mov    %al,0x8(%rbx)
   a7362:	movq   $0xffffffffffffffff,(%rbx)
   a7369:	test   %r12,%r12
   a736c:	je     a6f2b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a7372:	mov    0xd8(%rsp),%rdi
   a737a:	jmp    a732e <<hyperreal::computable::node::Computable>::approx_signal+0x10ce>
   a737c:	mov    $0x20,%esi
   a7381:	lea    -0x73e88(%rip),%rax        # 33500 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x3a0>
   a7388:	mov    %rax,0x30(%rsp)
   a738d:	lea    0xde054(%rip),%rax        # 1853e8 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x78>
   a7394:	mov    %rax,0x38(%rsp)
   a7399:	jmp    a73f9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a739b:	mov    $0x1b,%esi
   a73a0:	lea    -0x717f9(%rip),%rax        # 35bae <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.2438011856832709387+0x6b>
   a73a7:	mov    %rax,0x30(%rsp)
   a73ac:	lea    0xde005(%rip),%rax        # 1853b8 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x48>
   a73b3:	mov    %rax,0x38(%rsp)
   a73b8:	jmp    a73f9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a73ba:	mov    $0x20,%esi
   a73bf:	jmp    a73f9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a73c1:	lea    -0x717ff(%rip),%rdi        # 35bc9 <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.2438011856832709387+0x86>
   a73c8:	lea    0xde001(%rip),%rdx        # 1853d0 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x60>
   a73cf:	mov    $0x1b,%esi
   a73d4:	call   *0xe2266(%rip)        # 189640 <_DYNAMIC+0x4c0>
   a73da:	jmp    a7438 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a73dc:	lea    0xde01d(%rip),%rax        # 185400 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x90>
   a73e3:	mov    %rax,0x38(%rsp)
   a73e8:	mov    $0x22,%esi
   a73ed:	lea    -0x71810(%rip),%rax        # 35be4 <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.2438011856832709387+0xa1>
   a73f4:	mov    %rax,0x30(%rsp)
   a73f9:	mov    0x30(%rsp),%rdi
   a73fe:	mov    0x38(%rsp),%rdx
   a7403:	call   *0xe2237(%rip)        # 189640 <_DYNAMIC+0x4c0>
   a7409:	jmp    a7438 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a740b:	movb   $0xa,0x8(%rbx)
   a740f:	jmp    a6f24 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a7414:	lea    0xde26d(%rip),%rdx        # 185688 <anon.9023bb4654a08944d5043612f590d0ff.326.llvm.2438011856832709387+0x258>
   a741b:	mov    $0x4,%esi
   a7420:	call   *0xe20ba(%rip)        # 1894e0 <_DYNAMIC+0x360>
   a7426:	lea    0xddf5b(%rip),%rdx        # 185388 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.2438011856832709387+0x18>
   a742d:	mov    $0x10,%esi
   a7432:	call   *0xe20a8(%rip)        # 1894e0 <_DYNAMIC+0x360>
   a7438:	ud2
   a743a:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a743f:	mov    %rax,%rbx
   a7442:	test   %r12,%r12
   a7445:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a744b:	mov    0xd8(%rsp),%rdi
   a7453:	jmp    a746e <<hyperreal::computable::node::Computable>::approx_signal+0x120e>
   a7455:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a745a:	mov    %rax,%rbx
   a745d:	test   %r12,%r12
   a7460:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a7466:	mov    0xb8(%rsp),%rdi
   a746e:	shl    $0x3,%r12
   a7472:	mov    $0x8,%edx
   a7477:	jmp    a758d <<hyperreal::computable::node::Computable>::approx_signal+0x132d>
   a747c:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a7481:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a7486:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a748b:	jmp    a7579 <<hyperreal::computable::node::Computable>::approx_signal+0x1319>
   a7490:	mov    %rax,%rbx
   a7493:	mov    0x330(%rsp),%rsi
   a749b:	test   %rsi,%rsi
   a749e:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a74a4:	mov    0x338(%rsp),%rdi
   a74ac:	jmp    a74e7 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a74ae:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a74b3:	mov    %rax,%rbx
   a74b6:	mov    0x70(%rsp),%rsi
   a74bb:	test   %rsi,%rsi
   a74be:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a74c4:	mov    0x78(%rsp),%rdi
   a74c9:	jmp    a74e7 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a74cb:	mov    %rax,%rbx
   a74ce:	mov    0x90(%rsp),%rsi
   a74d6:	test   %rsi,%rsi
   a74d9:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a74df:	mov    0x98(%rsp),%rdi
   a74e7:	shl    $0x3,%rsi
   a74eb:	mov    $0x8,%edx
   a74f0:	call   *0xe1f12(%rip)        # 189408 <_DYNAMIC+0x288>
   a74f6:	jmp    a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a74fb:	mov    %rax,%rbx
   a74fe:	test   %r15,%r15
   a7501:	je     a75ce <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a7507:	mov    0xf8(%rsp),%rdi
   a750f:	shl    $0x3,%r15
   a7513:	mov    $0x8,%edx
   a7518:	mov    %r15,%rsi
   a751b:	jmp    a75c8 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a7520:	mov    %rax,%rbx
   a7523:	test   %r12,%r12
   a7526:	je     a75ce <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a752c:	mov    0xf8(%rsp),%rdi
   a7534:	shl    $0x3,%r12
   a7538:	mov    $0x8,%edx
   a753d:	mov    %r12,%rsi
   a7540:	jmp    a75c8 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a7545:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a7547:	mov    %rax,%rbx
   a754a:	mov    0x10(%rsp),%rsi
   a754f:	test   %rsi,%rsi
   a7552:	je     a75ce <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a7554:	mov    0x18(%rsp),%rdi
   a7559:	jmp    a7573 <<hyperreal::computable::node::Computable>::approx_signal+0x1313>
   a755b:	mov    %rax,%rbx
   a755e:	mov    0xf0(%rsp),%rsi
   a7566:	test   %rsi,%rsi
   a7569:	je     a75ce <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a756b:	mov    0xf8(%rsp),%rdi
   a7573:	shl    $0x3,%rsi
   a7577:	jmp    a75c3 <<hyperreal::computable::node::Computable>::approx_signal+0x1363>
   a7579:	mov    %rax,%rbx
   a757c:	test   %r12,%r12
   a757f:	je     a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a7581:	shl    $0x3,%r12
   a7585:	mov    $0x8,%edx
   a758a:	mov    %r13,%rdi
   a758d:	mov    %r12,%rsi
   a7590:	call   *0xe1e72(%rip)        # 189408 <_DYNAMIC+0x288>
   a7596:	jmp    a759d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a7598:	jmp    a759a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a759a:	mov    %rax,%rbx
   a759d:	lea    0x210(%rsp),%rdi
   a75a5:	call   80140 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a75aa:	mov    0xf0(%rsp),%rsi
   a75b2:	test   %rsi,%rsi
   a75b5:	je     a75ce <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a75b7:	mov    0xf8(%rsp),%rdi
   a75bf:	shl    $0x4,%rsi
   a75c3:	mov    $0x8,%edx
   a75c8:	call   *0xe1e3a(%rip)        # 189408 <_DYNAMIC+0x288>
   a75ce:	mov    %rbx,%rdi
   a75d1:	call   183780 <_Unwind_Resume@plt>
