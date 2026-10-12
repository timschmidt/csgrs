
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v2015-nth-root-enclosure-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a30b0 <<hyperreal::computable::node::Computable>::approx_signal>:
   a30b0:	push   %rbp
   a30b1:	push   %r15
   a30b3:	push   %r14
   a30b5:	push   %r13
   a30b7:	push   %r12
   a30b9:	push   %rbx
   a30ba:	sub    $0x3a8,%rsp
   a30c1:	mov    %ecx,%ebp
   a30c3:	mov    %rsi,%r13
   a30c6:	mov    %rdi,%rbx
   a30c9:	mov    (%rdx),%r14
   a30cc:	test   %r14,%r14
   a30cf:	je     a30de <<hyperreal::computable::node::Computable>::approx_signal+0x2e>
   a30d1:	movzbl 0x10(%r14),%eax
   a30d6:	test   %al,%al
   a30d8:	jne    a3280 <<hyperreal::computable::node::Computable>::approx_signal+0x1d0>
   a30de:	mov    %rdx,0x40(%rsp)
   a30e3:	data16 data16 data16 mov %fs:0x0,%rax
   a30ef:	lea    -0x120(%rax),%r15
   a30f6:	movzbl -0x110(%rax),%eax
   a30fd:	test   %eax,%eax
   a30ff:	jne    a4080 <<hyperreal::computable::node::Computable>::approx_signal+0xfd0>
   a3105:	movabs $0x7fffffffffffffff,%rax
   a310f:	mov    (%r15),%rcx
   a3112:	cmp    %rax,%rcx
   a3115:	jae    a40b0 <<hyperreal::computable::node::Computable>::approx_signal+0x1000>
   a311b:	lea    0x1(%rcx),%rax
   a311f:	mov    %rax,(%r15)
   a3122:	mov    0x8(%r15),%rdx
   a3126:	test   %rdx,%rdx
   a3129:	je     a3143 <<hyperreal::computable::node::Computable>::approx_signal+0x93>
   a312b:	nopl   0x0(%rax,%rax,1)
   a3130:	cmpb   $0x0,0x20(%rdx)
   a3134:	jne    a327d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a313a:	mov    0x18(%rdx),%rdx
   a313e:	test   %rdx,%rdx
   a3141:	jne    a3130 <<hyperreal::computable::node::Computable>::approx_signal+0x80>
   a3143:	mov    %rcx,(%r15)
   a3146:	mov    0x0(%r13),%r12
   a314a:	movzbl 0x10(%r12),%edx
   a3150:	test   %edx,%edx
   a3152:	jne    a3173 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a3154:	cmpb   $0x1,0x30(%r12)
   a315a:	jne    a3173 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a315c:	movups -0x703d3(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   a3163:	movups %xmm0,0x10(%rbx)
   a3167:	movups -0x703ee(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   a316e:	jmp    a331e <<hyperreal::computable::node::Computable>::approx_signal+0x26e>
   a3173:	cmp    $0xfff00000,%ebp
   a3179:	jge    a318c <<hyperreal::computable::node::Computable>::approx_signal+0xdc>
   a317b:	neg    %ebp
   a317d:	setno  %al
   a3180:	add    %al,%al
   a3182:	add    $0xa,%al
   a3184:	mov    %al,0x8(%rbx)
   a3187:	jmp    a3284 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a318c:	cmp    $0x2,%edx
   a318f:	je     a32a0 <<hyperreal::computable::node::Computable>::approx_signal+0x1f0>
   a3195:	cmp    $0x1,%edx
   a3198:	je     a31ff <<hyperreal::computable::node::Computable>::approx_signal+0x14f>
   a319a:	test   %edx,%edx
   a319c:	jne    a32ca <<hyperreal::computable::node::Computable>::approx_signal+0x21a>
   a31a2:	mov    %rax,(%r15)
   a31a5:	mov    0x8(%r15),%rdx
   a31a9:	test   %rdx,%rdx
   a31ac:	je     a31da <<hyperreal::computable::node::Computable>::approx_signal+0x12a>
   a31ae:	mov    %rdx,%rsi
   a31b1:	cmpb   $0x0,0x20(%rsi)
   a31b5:	jne    a31f0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a31b7:	mov    0x10(%rsi),%rdi
   a31bb:	mov    0x18(%rdi),%rax
   a31bf:	nop
   a31c0:	test   %rax,%rax
   a31c3:	je     a31f0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a31c5:	lea    -0x1(%rax),%r8
   a31c9:	lock cmpxchg %r8,0x18(%rdi)
   a31cf:	jne    a31c0 <<hyperreal::computable::node::Computable>::approx_signal+0x110>
   a31d1:	mov    0x18(%rsi),%rsi
   a31d5:	test   %rsi,%rsi
   a31d8:	jne    a31b1 <<hyperreal::computable::node::Computable>::approx_signal+0x101>
   a31da:	mov    %rcx,(%r15)
   a31dd:	add    $0x18,%r12
   a31e1:	mov    %rbx,%rdi
   a31e4:	mov    %r12,%rsi
   a31e7:	jmp    a3259 <<hyperreal::computable::node::Computable>::approx_signal+0x1a9>
   a31e9:	nopl   0x0(%rax)
   a31f0:	movb   $0x1,0x20(%rdx)
   a31f4:	mov    0x18(%rdx),%rdx
   a31f8:	test   %rdx,%rdx
   a31fb:	jne    a31f0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a31fd:	jmp    a327d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a31ff:	mov    %rax,(%r15)
   a3202:	mov    0x8(%r15),%rdx
   a3206:	test   %rdx,%rdx
   a3209:	je     a323a <<hyperreal::computable::node::Computable>::approx_signal+0x18a>
   a320b:	mov    %rdx,%rsi
   a320e:	cmpb   $0x0,0x20(%rsi)
   a3212:	jne    a3270 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a3214:	mov    0x10(%rsi),%rdi
   a3218:	mov    0x18(%rdi),%rax
   a321c:	nopl   0x0(%rax)
   a3220:	test   %rax,%rax
   a3223:	je     a3270 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a3225:	lea    -0x1(%rax),%r8
   a3229:	lock cmpxchg %r8,0x18(%rdi)
   a322f:	jne    a3220 <<hyperreal::computable::node::Computable>::approx_signal+0x170>
   a3231:	mov    0x18(%rsi),%rsi
   a3235:	test   %rsi,%rsi
   a3238:	jne    a320e <<hyperreal::computable::node::Computable>::approx_signal+0x15e>
   a323a:	mov    %rcx,(%r15)
   a323d:	lea    0x113414(%rip),%rdi        # 1b6658 <hyperreal::computable::node::signed::ONE+0x20>
   a3244:	lea    0x1133ed(%rip),%r14        # 1b6638 <hyperreal::computable::node::signed::ONE>
   a324b:	mov    %r14,%rsi
   a324e:	call   600d0 <<std::sync::once::Once>::call_once_force::<<std::sync::lazy_lock::LazyLock<num_bigint::bigint::BigInt>>::force::{closure#0}>>
   a3253:	mov    %rbx,%rdi
   a3256:	mov    %r14,%rsi
   a3259:	mov    %ebp,%edx
   a325b:	call   bb700 <hyperreal::computable::node::scale_at_precision_ref>
   a3260:	jmp    a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3262:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a3270:	movb   $0x1,0x20(%rdx)
   a3274:	mov    0x18(%rdx),%rdx
   a3278:	test   %rdx,%rdx
   a327b:	jne    a3270 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a327d:	mov    %rcx,(%r15)
   a3280:	movb   $0xc,0x8(%rbx)
   a3284:	movq   $0xffffffffffffffff,(%rbx)
   a328b:	mov    %rbx,%rax
   a328e:	add    $0x3a8,%rsp
   a3295:	pop    %rbx
   a3296:	pop    %r12
   a3298:	pop    %r13
   a329a:	pop    %r14
   a329c:	pop    %r15
   a329e:	pop    %rbp
   a329f:	ret
   a32a0:	movzbl 0x11(%r12),%esi
   a32a6:	lea    0x210(%rsp),%rdi
   a32ae:	mov    %ebp,%edx
   a32b0:	call   b1b60 <<hyperreal::computable::node::Computable>::cached_constant_at_precision>
   a32b5:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a32be:	jne    a330a <<hyperreal::computable::node::Computable>::approx_signal+0x25a>
   a32c0:	movl   $0x0,0xc(%rsp)
   a32c8:	jmp    a332c <<hyperreal::computable::node::Computable>::approx_signal+0x27c>
   a32ca:	mov    0x38(%r12),%rax
   a32cf:	test   $0x800,%eax
   a32d4:	jne    a32eb <<hyperreal::computable::node::Computable>::approx_signal+0x23b>
   a32d6:	mov    %r13,%rdi
   a32d9:	mov    0x40(%rsp),%rsi
   a32de:	call   84450 <<hyperreal::computable::node::Computable>::ensure_finite_real>
   a32e3:	cmp    $0xff,%al
   a32e5:	jne    a3184 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a32eb:	lea    0x40(%r12),%rsi
   a32f0:	lea    0x210(%rsp),%rdi
   a32f8:	mov    %ebp,%edx
   a32fa:	call   ba220 <<hyperreal::computable::node::EvaluationCache>::at_precision>
   a32ff:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a3308:	je     a3326 <<hyperreal::computable::node::Computable>::approx_signal+0x276>
   a330a:	movups 0x210(%rsp),%xmm0
   a3312:	movups 0x220(%rsp),%xmm1
   a331a:	movups %xmm1,0x10(%rbx)
   a331e:	movups %xmm0,(%rbx)
   a3321:	jmp    a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3326:	mov    $0x1,%al
   a3328:	mov    %eax,0xc(%rsp)
   a332c:	lea    0x10(%r12),%rax
   a3331:	mov    %rax,0x38(%rsp)
   a3336:	movzbl (%rax),%eax
   a3339:	add    $0xffffffdc,%eax
   a333c:	cmp    $0x1a,%eax
   a333f:	ja     a33db <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a3345:	lea    -0x6f56c(%rip),%rcx        # 33de0 <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.4922951029947715914+0xe08>
   a334c:	movslq (%rcx,%rax,4),%rax
   a3350:	add    %rcx,%rax
   a3353:	jmp    *%rax
   a3355:	movl   $0x1,0x30(%rsp)
   a335d:	jmp    a337b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a335f:	movl   $0x0,0x30(%rsp)
   a3367:	jmp    a337b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a3369:	movl   $0x2,0x30(%rsp)
   a3371:	jmp    a337b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a3373:	movl   $0x3,0x30(%rsp)
   a337b:	lea    0x18(%r12),%rdi
   a3380:	call   b8970 <<hyperreal::computable::node::Computable>::half_pi_index_when_real>
   a3385:	test   $0x1,%al
   a3387:	je     a33db <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a3389:	movzbl %dl,%edi
   a338c:	movl   $0xff000100,0x210(%rsp)
   a3397:	cmp    $0x3,%dil
   a339b:	ja     a4264 <<hyperreal::computable::node::Computable>::approx_signal+0x11b4>
   a33a1:	movzbl 0x210(%rsp,%rdi,1),%ecx
   a33a9:	movl   $0xff0001,0x210(%rsp)
   a33b4:	movzbl 0x210(%rsp,%rdi,1),%eax
   a33bc:	mov    0x30(%rsp),%edx
   a33c0:	lea    -0x6f57b(%rip),%rsi        # 33e4c <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.4922951029947715914+0xe74>
   a33c7:	movslq (%rsi,%rdx,4),%rdx
   a33cb:	add    %rsi,%rdx
   a33ce:	jmp    *%rdx
   a33d0:	mov    %ecx,%eax
   a33d2:	movsbq %al,%rsi
   a33d6:	jmp    a3db4 <<hyperreal::computable::node::Computable>::approx_signal+0xd04>
   a33db:	cmpb   $0x0,0xc(%rsp)
   a33e0:	je     a3415 <<hyperreal::computable::node::Computable>::approx_signal+0x365>
   a33e2:	lea    0x210(%rsp),%rdi
   a33ea:	mov    %r13,%rsi
   a33ed:	mov    0x40(%rsp),%rdx
   a33f2:	mov    %ebp,%ecx
   a33f4:	call   a23d0 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a33f9:	mov    0x210(%rsp),%r12
   a3401:	cmp    $0xffffffffffffffff,%r12
   a3405:	je     a3e82 <<hyperreal::computable::node::Computable>::approx_signal+0xdd2>
   a340b:	cmp    $0xfffffffffffffffe,%r12
   a340f:	jne    a3f78 <<hyperreal::computable::node::Computable>::approx_signal+0xec8>
   a3415:	mov    0x38(%rsp),%rsi
   a341a:	movzbl (%rsi),%eax
   a341d:	cmp    $0xa,%eax
   a3420:	ja     a3e8f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a3426:	mov    $0x430,%ecx
   a342b:	bt     %eax,%ecx
   a342e:	jae    a3e8f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a3434:	movl   $0xffffffff,0x108(%rsp)
   a343f:	movl   $0xffffffff,0x118(%rsp)
   a344a:	movl   $0xffffffff,0x128(%rsp)
   a3455:	movl   $0xffffffff,0x138(%rsp)
   a3460:	movl   $0xffffffff,0x148(%rsp)
   a346b:	movl   $0xffffffff,0x158(%rsp)
   a3476:	movl   $0xffffffff,0x168(%rsp)
   a3481:	movl   $0xffffffff,0x178(%rsp)
   a348c:	movl   $0xffffffff,0x188(%rsp)
   a3497:	movl   $0xffffffff,0x198(%rsp)
   a34a2:	movl   $0xffffffff,0x1a8(%rsp)
   a34ad:	movl   $0xffffffff,0x1b8(%rsp)
   a34b8:	movl   $0xffffffff,0x1c8(%rsp)
   a34c3:	movl   $0xffffffff,0x1d8(%rsp)
   a34ce:	movl   $0xffffffff,0x1e8(%rsp)
   a34d9:	movl   $0xffffffff,0x1f8(%rsp)
   a34e4:	movq   $0x0,0x208(%rsp)
   a34f0:	movq   $0x0,0xf0(%rsp)
   a34fc:	movq   $0x8,0xf8(%rsp)
   a3508:	movq   $0x0,0x100(%rsp)
   a3514:	movq   $0xffffffffffffffff,0x228(%rsp)
   a3520:	movq   $0xffffffffffffffff,0x248(%rsp)
   a352c:	movq   $0xffffffffffffffff,0x268(%rsp)
   a3538:	movq   $0xffffffffffffffff,0x288(%rsp)
   a3544:	movq   $0xffffffffffffffff,0x2a8(%rsp)
   a3550:	movq   $0xffffffffffffffff,0x2c8(%rsp)
   a355c:	movq   $0xffffffffffffffff,0x2e8(%rsp)
   a3568:	movq   $0xffffffffffffffff,0x308(%rsp)
   a3574:	movq   $0x0,0x328(%rsp)
   a3580:	movq   $0x0,0x210(%rsp)
   a358c:	movq   $0x8,0x218(%rsp)
   a3598:	movq   $0x0,0x220(%rsp)
   a35a4:	mov    %r13,0x18(%rsp)
   a35a9:	mov    %ebp,0x14(%rsp)
   a35ad:	movl   $0x0,0x10(%rsp)
   a35b5:	lea    0xf0(%rsp),%rdi
   a35bd:	lea    0x10(%rsp),%rsi
   a35c2:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a35c7:	lea    0xddb2a(%rip),%rax        # 1810f8 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x30>
   a35ce:	mov    %rax,0x38(%rsp)
   a35d3:	lea    -0x709ba(%rip),%rax        # 32c20 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x190>
   a35da:	mov    %rax,0x30(%rsp)
   a35df:	mov    0x100(%rsp),%rax
   a35e7:	test   %rax,%rax
   a35ea:	je     a3613 <<hyperreal::computable::node::Computable>::approx_signal+0x563>
   a35ec:	dec    %rax
   a35ef:	mov    %rax,0x100(%rsp)
   a35f7:	mov    0xf8(%rsp),%rcx
   a35ff:	shl    $0x4,%rax
   a3603:	mov    (%rcx,%rax,1),%r12d
   a3607:	mov    0x4(%rcx,%rax,1),%r9d
   a360c:	mov    0x8(%rcx,%rax,1),%r13
   a3611:	jmp    a365b <<hyperreal::computable::node::Computable>::approx_signal+0x5ab>
   a3613:	mov    0x208(%rsp),%rax
   a361b:	test   %rax,%rax
   a361e:	je     a404c <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a3624:	lea    -0x1(%rax),%rdi
   a3628:	mov    %rdi,0x208(%rsp)
   a3630:	cmp    $0x10,%rax
   a3634:	ja     a4276 <<hyperreal::computable::node::Computable>::approx_signal+0x11c6>
   a363a:	shl    $0x4,%rdi
   a363e:	lea    0x108(%rsp),%rax
   a3646:	mov    (%rax,%rdi,1),%r12d
   a364a:	mov    0x4(%rax,%rdi,1),%r9d
   a364f:	mov    0x8(%rax,%rdi,1),%r13
   a3654:	movl   $0xffffffff,(%rax,%rdi,1)
   a365b:	cmp    $0xffffffff,%r12d
   a365f:	je     a404c <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a3665:	test   %r14,%r14
   a3668:	je     a3677 <<hyperreal::computable::node::Computable>::approx_signal+0x5c7>
   a366a:	movzbl 0x10(%r14),%eax
   a366f:	test   %al,%al
   a3671:	jne    a3d70 <<hyperreal::computable::node::Computable>::approx_signal+0xcc0>
   a3677:	movzbl 0x10(%r15),%eax
   a367c:	test   %eax,%eax
   a367e:	jne    a3d06 <<hyperreal::computable::node::Computable>::approx_signal+0xc56>
   a3684:	mov    (%r15),%rcx
   a3687:	movabs $0x7fffffffffffffff,%rax
   a3691:	cmp    %rax,%rcx
   a3694:	jae    a3d40 <<hyperreal::computable::node::Computable>::approx_signal+0xc90>
   a369a:	lea    0x1(%rcx),%rax
   a369e:	mov    %rax,(%r15)
   a36a1:	mov    0x8(%r15),%rdx
   a36a5:	test   %rdx,%rdx
   a36a8:	je     a36de <<hyperreal::computable::node::Computable>::approx_signal+0x62e>
   a36aa:	mov    %rdx,%rsi
   a36ad:	cmpb   $0x0,0x20(%rsi)
   a36b1:	jne    a3d60 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a36b7:	mov    0x10(%rsi),%rdi
   a36bb:	mov    0x18(%rdi),%rax
   a36bf:	nop
   a36c0:	test   %rax,%rax
   a36c3:	je     a3d60 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a36c9:	lea    -0x1(%rax),%r8
   a36cd:	lock cmpxchg %r8,0x18(%rdi)
   a36d3:	jne    a36c0 <<hyperreal::computable::node::Computable>::approx_signal+0x610>
   a36d5:	mov    0x18(%rsi),%rsi
   a36d9:	test   %rsi,%rsi
   a36dc:	jne    a36ad <<hyperreal::computable::node::Computable>::approx_signal+0x5fd>
   a36de:	mov    %r9d,0xc(%rsp)
   a36e3:	mov    %rcx,(%r15)
   a36e6:	mov    %r12d,%eax
   a36e9:	lea    -0x6f894(%rip),%rcx        # 33e5c <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.4922951029947715914+0xe84>
   a36f0:	movslq (%rcx,%rax,4),%rax
   a36f4:	add    %rcx,%rax
   a36f7:	jmp    *%rax
   a36f9:	mov    0x0(%r13),%rsi
   a36fd:	cmpb   $0x0,0x10(%rsi)
   a3701:	jne    a3710 <<hyperreal::computable::node::Computable>::approx_signal+0x660>
   a3703:	cmpb   $0x1,0x30(%rsi)
   a3707:	lea    -0x7098e(%rip),%rax        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   a370e:	je     a3744 <<hyperreal::computable::node::Computable>::approx_signal+0x694>
   a3710:	mov    0xc(%rsp),%edx
   a3714:	cmp    $0xffefffff,%edx
   a371a:	jle    a4145 <<hyperreal::computable::node::Computable>::approx_signal+0x1095>
   a3720:	lea    0x388(%rsp),%rdi
   a3728:	call   af580 <<hyperreal::computable::node::Computable>::cached_at_precision>
   a372d:	cmpq   $0xffffffffffffffff,0x388(%rsp)
   a3736:	lea    0x388(%rsp),%rax
   a373e:	je     a39d3 <<hyperreal::computable::node::Computable>::approx_signal+0x923>
   a3744:	lea    0x210(%rsp),%rdi
   a374c:	mov    %rax,%rsi
   a374f:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3754:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3759:	lea    0x10(%rsp),%rdi
   a375e:	lea    0x210(%rsp),%rsi
   a3766:	call   a2940 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a376b:	mov    %r13,0x68(%rsp)
   a3770:	mov    0x10(%rsp),%r12
   a3775:	cmp    $0xffffffffffffffff,%r12
   a3779:	je     a41eb <<hyperreal::computable::node::Computable>::approx_signal+0x113b>
   a377f:	mov    0x18(%rsp),%r13
   a3784:	lea    0x29(%rsp),%rax
   a3789:	movups -0x9(%rax),%xmm0
   a378d:	movaps %xmm0,0x350(%rsp)
   a3795:	lea    0x10(%rsp),%rdi
   a379a:	lea    0x210(%rsp),%rsi
   a37a2:	call   a2940 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a37a7:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a37ad:	je     a4211 <<hyperreal::computable::node::Computable>::approx_signal+0x1161>
   a37b3:	movups 0x10(%rsp),%xmm0
   a37b8:	movups 0x20(%rsp),%xmm1
   a37bd:	movaps %xmm1,0x370(%rsp)
   a37c5:	movaps %xmm0,0x360(%rsp)
   a37cd:	mov    %r12,0x10(%rsp)
   a37d2:	mov    %r13,0x18(%rsp)
   a37d7:	movaps 0x350(%rsp),%xmm0
   a37df:	lea    0x29(%rsp),%rax
   a37e4:	movups %xmm0,-0x9(%rax)
   a37e8:	lea    0x48(%rsp),%rbp
   a37ed:	mov    %rbp,%rdi
   a37f0:	lea    0x360(%rsp),%rsi
   a37f8:	lea    0x10(%rsp),%rdx
   a37fd:	call   bf500 <<num_bigint::bigint::BigInt as core::ops::arith::Add>::add>
   a3802:	lea    0x330(%rsp),%r13
   a380a:	mov    %r13,%rdi
   a380d:	mov    %rbp,%rsi
   a3810:	mov    $0xfffffffe,%edx
   a3815:	call   bbbe0 <hyperreal::computable::node::scale>
   a381a:	movzbl 0x348(%rsp),%r12d
   a3823:	mov    %rbp,%rdi
   a3826:	mov    %r13,%rsi
   a3829:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a382e:	mov    0x58(%rsp),%rax
   a3833:	mov    %rax,0x20(%rsp)
   a3838:	movups 0x48(%rsp),%xmm0
   a383d:	movaps %xmm0,0x10(%rsp)
   a3842:	mov    %r12b,0x28(%rsp)
   a3847:	mov    0x68(%rsp),%rdi
   a384c:	mov    %r14,%rsi
   a384f:	mov    0xc(%rsp),%edx
   a3853:	lea    0x10(%rsp),%rcx
   a3858:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a385d:	cmp    $0xff,%al
   a385f:	jne    a410e <<hyperreal::computable::node::Computable>::approx_signal+0x105e>
   a3865:	lea    0x210(%rsp),%rdi
   a386d:	mov    %r13,%rsi
   a3870:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3875:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a387a:	lea    0x10(%rsp),%rdi
   a387f:	lea    0x210(%rsp),%rsi
   a3887:	call   a2940 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a388c:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a3892:	je     a41cc <<hyperreal::computable::node::Computable>::approx_signal+0x111c>
   a3898:	movups 0x10(%rsp),%xmm0
   a389d:	movups 0x20(%rsp),%xmm1
   a38a2:	movaps %xmm1,0xa0(%rsp)
   a38aa:	movaps %xmm0,0x90(%rsp)
   a38b2:	movzbl 0xa8(%rsp),%r12d
   a38bb:	lea    0x48(%rsp),%rdi
   a38c0:	lea    0x90(%rsp),%rsi
   a38c8:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a38cd:	mov    0xc(%rsp),%edx
   a38d1:	mov    0x58(%rsp),%rax
   a38d6:	mov    %rax,0x20(%rsp)
   a38db:	movups 0x48(%rsp),%xmm0
   a38e0:	movaps %xmm0,0x10(%rsp)
   a38e5:	mov    %r12b,0x28(%rsp)
   a38ea:	mov    %r13,%rdi
   a38ed:	mov    %r14,%rsi
   a38f0:	lea    0x10(%rsp),%rcx
   a38f5:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a38fa:	cmp    $0xff,%al
   a38fc:	jne    a40ca <<hyperreal::computable::node::Computable>::approx_signal+0x101a>
   a3902:	lea    0x210(%rsp),%rdi
   a390a:	lea    0x90(%rsp),%rsi
   a3912:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3917:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a391c:	lea    0x10(%rsp),%rdi
   a3921:	lea    0x210(%rsp),%rsi
   a3929:	call   a2940 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a392e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a3934:	je     a420a <<hyperreal::computable::node::Computable>::approx_signal+0x115a>
   a393a:	mov    0x20(%rsp),%rax
   a393f:	mov    %rax,0x80(%rsp)
   a3947:	movups 0x10(%rsp),%xmm0
   a394c:	movaps %xmm0,0x70(%rsp)
   a3951:	lea    0x29(%rsp),%rcx
   a3956:	mov    (%rcx),%eax
   a3958:	mov    $0x2,%r12b
   a395b:	sub    0x28(%rsp),%r12b
   a3960:	mov    0x3(%rcx),%ecx
   a3963:	lea    0x89(%rsp),%rdx
   a396b:	mov    %ecx,0x3(%rdx)
   a396e:	mov    %eax,(%rdx)
   a3970:	mov    %r12b,0x88(%rsp)
   a3978:	lea    0x48(%rsp),%rdi
   a397d:	lea    0x70(%rsp),%rsi
   a3982:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3987:	mov    0xc(%rsp),%edx
   a398b:	mov    0x58(%rsp),%rax
   a3990:	mov    %rax,0x20(%rsp)
   a3995:	movups 0x48(%rsp),%xmm0
   a399a:	movaps %xmm0,0x10(%rsp)
   a399f:	mov    %r12b,0x28(%rsp)
   a39a4:	mov    %r13,%rdi
   a39a7:	mov    %r14,%rsi
   a39aa:	lea    0x10(%rsp),%rcx
   a39af:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a39b4:	cmp    $0xff,%al
   a39b6:	jne    a40ef <<hyperreal::computable::node::Computable>::approx_signal+0x103f>
   a39bc:	lea    0x210(%rsp),%rdi
   a39c4:	lea    0x70(%rsp),%rsi
   a39c9:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a39ce:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a39d3:	lea    0x10(%rsp),%rdi
   a39d8:	mov    %r13,%rsi
   a39db:	mov    0x40(%rsp),%rdx
   a39e0:	mov    0xc(%rsp),%ecx
   a39e4:	call   a23d0 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a39e9:	mov    0x10(%rsp),%r12
   a39ee:	cmp    $0xfffffffffffffffe,%r12
   a39f2:	je     a3a9f <<hyperreal::computable::node::Computable>::approx_signal+0x9ef>
   a39f8:	cmp    $0xffffffffffffffff,%r12
   a39fc:	je     a4156 <<hyperreal::computable::node::Computable>::approx_signal+0x10a6>
   a3a02:	mov    %r13,%rbp
   a3a05:	movzbl 0x18(%rsp),%eax
   a3a0a:	lea    0x29(%rsp),%rdx
   a3a0f:	mov    -0x1(%rdx),%rcx
   a3a13:	lea    0xb9(%rsp),%rsi
   a3a1b:	mov    %rcx,0xf(%rsi)
   a3a1f:	movups -0x10(%rdx),%xmm0
   a3a23:	movups %xmm0,(%rsi)
   a3a26:	mov    %r12,0xb0(%rsp)
   a3a2e:	mov    %al,0xb8(%rsp)
   a3a35:	movzbl 0xc8(%rsp),%r13d
   a3a3e:	lea    0x48(%rsp),%rdi
   a3a43:	lea    0xb0(%rsp),%rsi
   a3a4b:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3a50:	mov    0x58(%rsp),%rax
   a3a55:	mov    %rax,0x20(%rsp)
   a3a5a:	movups 0x48(%rsp),%xmm0
   a3a5f:	movaps %xmm0,0x10(%rsp)
   a3a64:	mov    %r13b,0x28(%rsp)
   a3a69:	mov    %rbp,%rdi
   a3a6c:	mov    %r14,%rsi
   a3a6f:	mov    0xc(%rsp),%edx
   a3a73:	lea    0x10(%rsp),%rcx
   a3a78:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3a7d:	cmp    $0xff,%al
   a3a7f:	jne    a4163 <<hyperreal::computable::node::Computable>::approx_signal+0x10b3>
   a3a85:	lea    0x210(%rsp),%rdi
   a3a8d:	lea    0xb0(%rsp),%rsi
   a3a95:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3a9a:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3a9f:	mov    0x0(%r13),%r12
   a3aa3:	movzbl 0x10(%r12),%eax
   a3aa9:	cmp    $0xa,%eax
   a3aac:	je     a3bd6 <<hyperreal::computable::node::Computable>::approx_signal+0xb26>
   a3ab2:	cmp    $0x5,%eax
   a3ab5:	je     a3b17 <<hyperreal::computable::node::Computable>::approx_signal+0xa67>
   a3ab7:	cmp    $0x4,%eax
   a3aba:	jne    a3c40 <<hyperreal::computable::node::Computable>::approx_signal+0xb90>
   a3ac0:	mov    %r13,0x18(%rsp)
   a3ac5:	mov    0xc(%rsp),%eax
   a3ac9:	mov    %eax,0x14(%rsp)
   a3acd:	movl   $0x1,0x10(%rsp)
   a3ad5:	lea    0xf0(%rsp),%rdi
   a3add:	lea    0x10(%rsp),%rsi
   a3ae2:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3ae7:	add    $0x18,%r12
   a3aeb:	mov    %r12,0x18(%rsp)
   a3af0:	mov    0xc(%rsp),%eax
   a3af4:	mov    %eax,0x14(%rsp)
   a3af8:	movl   $0x0,0x10(%rsp)
   a3b00:	lea    0xf0(%rsp),%rdi
   a3b08:	lea    0x10(%rsp),%rsi
   a3b0d:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3b12:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3b17:	mov    %r13,0x18(%rsp)
   a3b1c:	mov    0xc(%rsp),%eax
   a3b20:	mov    %eax,0x14(%rsp)
   a3b24:	movl   $0x2,0x10(%rsp)
   a3b2c:	lea    0xf0(%rsp),%rdi
   a3b34:	lea    0x10(%rsp),%rsi
   a3b39:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3b3e:	lea    0x28(%r12),%rdx
   a3b43:	lea    0x18(%r12),%rbp
   a3b48:	mov    0x28(%r12),%rax
   a3b4d:	mov    0x38(%rax),%r13
   a3b51:	shr    $0x10,%r13d
   a3b55:	mov    0x18(%r12),%rax
   a3b5a:	mov    0x38(%rax),%r12
   a3b5e:	shr    $0x10,%r12d
   a3b62:	mov    0xc(%rsp),%ecx
   a3b66:	add    $0xfffffffe,%ecx
   a3b69:	cmp    %r12w,%r13w
   a3b6d:	mov    %rdx,0x68(%rsp)
   a3b72:	mov    %rdx,%rax
   a3b75:	cmovg  %rbp,%rax
   a3b79:	mov    %rax,0x18(%rsp)
   a3b7e:	mov    %ecx,0xc(%rsp)
   a3b82:	mov    %ecx,0x14(%rsp)
   a3b86:	movl   $0x0,0x10(%rsp)
   a3b8e:	lea    0xf0(%rsp),%rdi
   a3b96:	lea    0x10(%rsp),%rsi
   a3b9b:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3ba0:	cmp    %r12w,%r13w
   a3ba4:	cmovg  0x68(%rsp),%rbp
   a3baa:	mov    %rbp,0x18(%rsp)
   a3baf:	mov    0xc(%rsp),%eax
   a3bb3:	mov    %eax,0x14(%rsp)
   a3bb7:	movl   $0x0,0x10(%rsp)
   a3bbf:	lea    0xf0(%rsp),%rdi
   a3bc7:	lea    0x10(%rsp),%rsi
   a3bcc:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3bd1:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3bd6:	mov    %r13,0x18(%rsp)
   a3bdb:	mov    0xc(%rsp),%eax
   a3bdf:	mov    %eax,0x14(%rsp)
   a3be3:	movl   $0x3,0x10(%rsp)
   a3beb:	lea    0xf0(%rsp),%rdi
   a3bf3:	lea    0x10(%rsp),%rsi
   a3bf8:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3bfd:	mov    0xc(%rsp),%eax
   a3c01:	sub    0x14(%r12),%eax
   a3c06:	mov    %eax,0xc(%rsp)
   a3c0a:	jo     a425b <<hyperreal::computable::node::Computable>::approx_signal+0x11ab>
   a3c10:	add    $0x18,%r12
   a3c14:	mov    %r12,0x18(%rsp)
   a3c19:	mov    0xc(%rsp),%eax
   a3c1d:	mov    %eax,0x14(%rsp)
   a3c21:	movl   $0x0,0x10(%rsp)
   a3c29:	lea    0xf0(%rsp),%rdi
   a3c31:	lea    0x10(%rsp),%rsi
   a3c36:	call   a2ab0 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3c3b:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3c40:	add    $0x10,%r12
   a3c44:	lea    0x10(%rsp),%rdi
   a3c49:	mov    %r12,%rsi
   a3c4c:	mov    0x40(%rsp),%rdx
   a3c51:	mov    0xc(%rsp),%ecx
   a3c55:	call   e95c0 <<hyperreal::computable::approximation::Approximation>::approximate>
   a3c5a:	mov    0x10(%rsp),%r12
   a3c5f:	movzbl 0x18(%rsp),%eax
   a3c64:	cmp    $0xffffffffffffffff,%r12
   a3c68:	je     a41a7 <<hyperreal::computable::node::Computable>::approx_signal+0x10f7>
   a3c6e:	mov    %r13,%rbp
   a3c71:	lea    0x29(%rsp),%rdx
   a3c76:	mov    -0x1(%rdx),%rcx
   a3c7a:	lea    0xd9(%rsp),%rsi
   a3c82:	mov    %rcx,0xf(%rsi)
   a3c86:	movups -0x10(%rdx),%xmm0
   a3c8a:	movups %xmm0,(%rsi)
   a3c8d:	mov    %r12,0xd0(%rsp)
   a3c95:	mov    %al,0xd8(%rsp)
   a3c9c:	movzbl 0xe8(%rsp),%r13d
   a3ca5:	lea    0x48(%rsp),%rdi
   a3caa:	lea    0xd0(%rsp),%rsi
   a3cb2:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3cb7:	mov    0x58(%rsp),%rax
   a3cbc:	mov    %rax,0x20(%rsp)
   a3cc1:	movups 0x48(%rsp),%xmm0
   a3cc6:	movaps %xmm0,0x10(%rsp)
   a3ccb:	mov    %r13b,0x28(%rsp)
   a3cd0:	mov    %rbp,%rdi
   a3cd3:	mov    %r14,%rsi
   a3cd6:	mov    0xc(%rsp),%edx
   a3cda:	lea    0x10(%rsp),%rcx
   a3cdf:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3ce4:	cmp    $0xff,%al
   a3ce6:	jne    a41af <<hyperreal::computable::node::Computable>::approx_signal+0x10ff>
   a3cec:	lea    0x210(%rsp),%rdi
   a3cf4:	lea    0xd0(%rsp),%rsi
   a3cfc:	call   a29d0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3d01:	jmp    a35df <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3d06:	cmp    $0x2,%eax
   a3d09:	je     a4195 <<hyperreal::computable::node::Computable>::approx_signal+0x10e5>
   a3d0f:	mov    %r9d,%ebp
   a3d12:	mov    %r15,%rdi
   a3d15:	lea    0x84ef4(%rip),%rsi        # 128c10 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a3d1c:	call   *0xe1456(%rip)        # 185178 <_DYNAMIC+0x388>
   a3d22:	movb   $0x0,0x10(%r15)
   a3d27:	mov    %ebp,%r9d
   a3d2a:	mov    (%r15),%rcx
   a3d2d:	movabs $0x7fffffffffffffff,%rax
   a3d37:	cmp    %rax,%rcx
   a3d3a:	jb     a369a <<hyperreal::computable::node::Computable>::approx_signal+0x5ea>
   a3d40:	lea    0xde6f9(%rip),%rdi        # 182440 <anon.047ef208c0d43d81aa529f346c7b9e9a.23.llvm.15529955461551244247>
   a3d47:	call   *0xe1433(%rip)        # 185180 <_DYNAMIC+0x390>
   a3d4d:	jmp    a4288 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a3d52:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a3d60:	movb   $0x1,0x20(%rdx)
   a3d64:	mov    0x18(%rdx),%rdx
   a3d68:	test   %rdx,%rdx
   a3d6b:	jne    a3d60 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a3d6d:	mov    %rcx,(%r15)
   a3d70:	movb   $0xc,0x8(%rbx)
   a3d74:	movq   $0xffffffffffffffff,(%rbx)
   a3d7b:	lea    0x210(%rsp),%rdi
   a3d83:	call   7d1a0 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a3d88:	mov    0xf0(%rsp),%rsi
   a3d90:	test   %rsi,%rsi
   a3d93:	je     a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3d99:	mov    0xf8(%rsp),%rdi
   a3da1:	shl    $0x4,%rsi
   a3da5:	jmp    a3e61 <<hyperreal::computable::node::Computable>::approx_signal+0xdb1>
   a3daa:	test   %al,%al
   a3dac:	je     a3e79 <<hyperreal::computable::node::Computable>::approx_signal+0xdc9>
   a3db2:	xor    %esi,%esi
   a3db4:	lea    0x10(%rsp),%rdi
   a3db9:	call   bfb70 <<num_bigint::bigint::BigInt as core::convert::From<i64>>::from>
   a3dbe:	add    $0x38,%r12
   a3dc2:	mov    0x28(%rsp),%esi
   a3dc6:	mov    %r12,%rdi
   a3dc9:	call   a53e0 <<hyperreal::computable::node::AtomicFacts>::publish_exact_sign>
   a3dce:	lea    0xf0(%rsp),%r15
   a3dd6:	lea    0x10(%rsp),%rsi
   a3ddb:	mov    %r15,%rdi
   a3dde:	mov    %ebp,%edx
   a3de0:	call   bb520 <hyperreal::computable::node::scale_at_precision>
   a3de5:	movzbl 0x108(%rsp),%r12d
   a3dee:	lea    0x48(%rsp),%rdi
   a3df3:	mov    %r15,%rsi
   a3df6:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3dfb:	mov    0x58(%rsp),%rax
   a3e00:	mov    %rax,0x220(%rsp)
   a3e08:	movups 0x48(%rsp),%xmm0
   a3e0d:	movaps %xmm0,0x210(%rsp)
   a3e15:	mov    %r12b,0x228(%rsp)
   a3e1d:	lea    0x210(%rsp),%rcx
   a3e25:	mov    %r13,%rdi
   a3e28:	mov    %r14,%rsi
   a3e2b:	mov    %ebp,%edx
   a3e2d:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3e32:	cmp    $0xff,%al
   a3e34:	je     a4037 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a3e3a:	mov    %al,0x8(%rbx)
   a3e3d:	movq   $0xffffffffffffffff,(%rbx)
   a3e44:	mov    0xf0(%rsp),%rsi
   a3e4c:	test   %rsi,%rsi
   a3e4f:	je     a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3e55:	mov    0xf8(%rsp),%rdi
   a3e5d:	shl    $0x3,%rsi
   a3e61:	mov    $0x8,%edx
   a3e66:	call   *0xe120c(%rip)        # 185078 <_DYNAMIC+0x288>
   a3e6c:	jmp    a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3e71:	test   %cl,%cl
   a3e73:	jne    a3db2 <<hyperreal::computable::node::Computable>::approx_signal+0xd02>
   a3e79:	movb   $0x2,0x8(%rbx)
   a3e7d:	jmp    a3284 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a3e82:	movzbl 0x218(%rsp),%eax
   a3e8a:	jmp    a3184 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a3e8f:	lea    0x210(%rsp),%rdi
   a3e97:	mov    0x40(%rsp),%rdx
   a3e9c:	mov    %ebp,%ecx
   a3e9e:	call   e95c0 <<hyperreal::computable::approximation::Approximation>::approximate>
   a3ea3:	mov    0x210(%rsp),%r15
   a3eab:	movzbl 0x218(%rsp),%eax
   a3eb3:	cmp    $0xffffffffffffffff,%r15
   a3eb7:	je     a3184 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a3ebd:	mov    0x228(%rsp),%rcx
   a3ec5:	mov    %rcx,0x108(%rsp)
   a3ecd:	movups 0x219(%rsp),%xmm0
   a3ed5:	movups %xmm0,0xf9(%rsp)
   a3edd:	mov    %r15,0xf0(%rsp)
   a3ee5:	mov    %al,0xf8(%rsp)
   a3eec:	movzbl 0x108(%rsp),%r12d
   a3ef5:	lea    0x10(%rsp),%rdi
   a3efa:	lea    0xf0(%rsp),%rsi
   a3f02:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3f07:	mov    0x20(%rsp),%rax
   a3f0c:	mov    %rax,0x220(%rsp)
   a3f14:	movups 0x10(%rsp),%xmm0
   a3f19:	movaps %xmm0,0x210(%rsp)
   a3f21:	mov    %r12b,0x228(%rsp)
   a3f29:	lea    0x210(%rsp),%rcx
   a3f31:	mov    %r13,%rdi
   a3f34:	mov    %r14,%rsi
   a3f37:	mov    %ebp,%edx
   a3f39:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3f3e:	cmp    $0xff,%al
   a3f40:	je     a4037 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a3f46:	mov    %al,0x8(%rbx)
   a3f49:	movq   $0xffffffffffffffff,(%rbx)
   a3f50:	test   %r15,%r15
   a3f53:	je     a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3f59:	mov    0xf8(%rsp),%rdi
   a3f61:	shl    $0x3,%r15
   a3f65:	mov    $0x8,%edx
   a3f6a:	mov    %r15,%rsi
   a3f6d:	call   *0xe1105(%rip)        # 185078 <_DYNAMIC+0x288>
   a3f73:	jmp    a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a3f78:	movzbl 0x218(%rsp),%eax
   a3f80:	mov    0x228(%rsp),%rcx
   a3f88:	mov    %rcx,0x108(%rsp)
   a3f90:	movups 0x219(%rsp),%xmm0
   a3f98:	movups %xmm0,0xf9(%rsp)
   a3fa0:	mov    %r12,0xf0(%rsp)
   a3fa8:	mov    %al,0xf8(%rsp)
   a3faf:	movzbl 0x108(%rsp),%r15d
   a3fb8:	lea    0x10(%rsp),%rdi
   a3fbd:	lea    0xf0(%rsp),%rsi
   a3fc5:	call   128280 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3fca:	mov    0x20(%rsp),%rax
   a3fcf:	mov    %rax,0x220(%rsp)
   a3fd7:	movups 0x10(%rsp),%xmm0
   a3fdc:	movaps %xmm0,0x210(%rsp)
   a3fe4:	mov    %r15b,0x228(%rsp)
   a3fec:	lea    0x210(%rsp),%rcx
   a3ff4:	mov    %r13,%rdi
   a3ff7:	mov    %r14,%rsi
   a3ffa:	mov    %ebp,%edx
   a3ffc:	call   aef10 <<hyperreal::computable::node::Computable>::store_cache_value>
   a4001:	cmp    $0xff,%al
   a4003:	je     a4037 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a4005:	mov    %al,0x8(%rbx)
   a4008:	movq   $0xffffffffffffffff,(%rbx)
   a400f:	test   %r12,%r12
   a4012:	je     a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a4018:	mov    0xf8(%rsp),%rdi
   a4020:	shl    $0x3,%r12
   a4024:	mov    $0x8,%edx
   a4029:	mov    %r12,%rsi
   a402c:	call   *0xe1046(%rip)        # 185078 <_DYNAMIC+0x288>
   a4032:	jmp    a328b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a4037:	movups 0xf0(%rsp),%xmm0
   a403f:	movups 0x100(%rsp),%xmm1
   a4047:	jmp    a331a <<hyperreal::computable::node::Computable>::approx_signal+0x26a>
   a404c:	lea    0x10(%rsp),%rdi
   a4051:	lea    0x210(%rsp),%rsi
   a4059:	call   a2940 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a405e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a4064:	je     a422c <<hyperreal::computable::node::Computable>::approx_signal+0x117c>
   a406a:	movups 0x10(%rsp),%xmm0
   a406f:	movups 0x20(%rsp),%xmm1
   a4074:	movups %xmm1,0x10(%rbx)
   a4078:	movups %xmm0,(%rbx)
   a407b:	jmp    a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4080:	cmp    $0x1,%eax
   a4083:	jne    a40bd <<hyperreal::computable::node::Computable>::approx_signal+0x100d>
   a4085:	lea    0x84b84(%rip),%rsi        # 128c10 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a408c:	mov    %r15,%rdi
   a408f:	call   *0xe10e3(%rip)        # 185178 <_DYNAMIC+0x388>
   a4095:	movb   $0x0,0x10(%r15)
   a409a:	movabs $0x7fffffffffffffff,%rax
   a40a4:	mov    (%r15),%rcx
   a40a7:	cmp    %rax,%rcx
   a40aa:	jb     a311b <<hyperreal::computable::node::Computable>::approx_signal+0x6b>
   a40b0:	lea    0xde3a1(%rip),%rdi        # 182458 <anon.047ef208c0d43d81aa529f346c7b9e9a.24.llvm.15529955461551244247>
   a40b7:	call   *0xe10c3(%rip)        # 185180 <_DYNAMIC+0x390>
   a40bd:	lea    0xde2cc(%rip),%rdi        # 182390 <anon.047ef208c0d43d81aa529f346c7b9e9a.4.llvm.15529955461551244247>
   a40c4:	call   *0xe10be(%rip)        # 185188 <_DYNAMIC+0x398>
   a40ca:	mov    %al,0x8(%rbx)
   a40cd:	movq   $0xffffffffffffffff,(%rbx)
   a40d4:	mov    0x90(%rsp),%rsi
   a40dc:	test   %rsi,%rsi
   a40df:	je     a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a40e5:	mov    0x98(%rsp),%rdi
   a40ed:	jmp    a4131 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a40ef:	mov    %al,0x8(%rbx)
   a40f2:	movq   $0xffffffffffffffff,(%rbx)
   a40f9:	mov    0x70(%rsp),%rsi
   a40fe:	test   %rsi,%rsi
   a4101:	je     a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4107:	mov    0x78(%rsp),%rdi
   a410c:	jmp    a4131 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a410e:	mov    %al,0x8(%rbx)
   a4111:	movq   $0xffffffffffffffff,(%rbx)
   a4118:	mov    0x330(%rsp),%rsi
   a4120:	test   %rsi,%rsi
   a4123:	je     a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4129:	mov    0x338(%rsp),%rdi
   a4131:	shl    $0x3,%rsi
   a4135:	mov    $0x8,%edx
   a413a:	call   *0xe0f38(%rip)        # 185078 <_DYNAMIC+0x288>
   a4140:	jmp    a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4145:	neg    %edx
   a4147:	setno  %al
   a414a:	add    %al,%al
   a414c:	add    $0xa,%al
   a414e:	mov    %al,0x8(%rbx)
   a4151:	jmp    a3d74 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a4156:	movzbl 0x18(%rsp),%eax
   a415b:	mov    %al,0x8(%rbx)
   a415e:	jmp    a3d74 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a4163:	mov    %al,0x8(%rbx)
   a4166:	movq   $0xffffffffffffffff,(%rbx)
   a416d:	test   %r12,%r12
   a4170:	je     a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4176:	mov    0xb8(%rsp),%rdi
   a417e:	shl    $0x3,%r12
   a4182:	mov    $0x8,%edx
   a4187:	mov    %r12,%rsi
   a418a:	call   *0xe0ee8(%rip)        # 185078 <_DYNAMIC+0x288>
   a4190:	jmp    a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4195:	lea    0xde1f4(%rip),%rdi        # 182390 <anon.047ef208c0d43d81aa529f346c7b9e9a.4.llvm.15529955461551244247>
   a419c:	call   *0xe0fe6(%rip)        # 185188 <_DYNAMIC+0x398>
   a41a2:	jmp    a4288 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a41a7:	mov    %al,0x8(%rbx)
   a41aa:	jmp    a3d74 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a41af:	mov    %al,0x8(%rbx)
   a41b2:	movq   $0xffffffffffffffff,(%rbx)
   a41b9:	test   %r12,%r12
   a41bc:	je     a3d7b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a41c2:	mov    0xd8(%rsp),%rdi
   a41ca:	jmp    a417e <<hyperreal::computable::node::Computable>::approx_signal+0x10ce>
   a41cc:	mov    $0x20,%esi
   a41d1:	lea    -0x71578(%rip),%rax        # 32c60 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x1d0>
   a41d8:	mov    %rax,0x30(%rsp)
   a41dd:	lea    0xdcf5c(%rip),%rax        # 181140 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x78>
   a41e4:	mov    %rax,0x38(%rsp)
   a41e9:	jmp    a4249 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a41eb:	mov    $0x1b,%esi
   a41f0:	lea    -0x6edd1(%rip),%rax        # 35426 <anon.f651f877c5f1b0c83bd435f3f6e617b1.257.llvm.9350274297799828109+0x6b>
   a41f7:	mov    %rax,0x30(%rsp)
   a41fc:	lea    0xdcf0d(%rip),%rax        # 181110 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x48>
   a4203:	mov    %rax,0x38(%rsp)
   a4208:	jmp    a4249 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a420a:	mov    $0x20,%esi
   a420f:	jmp    a4249 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a4211:	lea    -0x6edd7(%rip),%rdi        # 35441 <anon.f651f877c5f1b0c83bd435f3f6e617b1.257.llvm.9350274297799828109+0x86>
   a4218:	lea    0xdcf09(%rip),%rdx        # 181128 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x60>
   a421f:	mov    $0x1b,%esi
   a4224:	call   *0xe103e(%rip)        # 185268 <_DYNAMIC+0x478>
   a422a:	jmp    a4288 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a422c:	lea    0xdcf25(%rip),%rax        # 181158 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x90>
   a4233:	mov    %rax,0x38(%rsp)
   a4238:	mov    $0x22,%esi
   a423d:	lea    -0x6ede8(%rip),%rax        # 3545c <anon.f651f877c5f1b0c83bd435f3f6e617b1.257.llvm.9350274297799828109+0xa1>
   a4244:	mov    %rax,0x30(%rsp)
   a4249:	mov    0x30(%rsp),%rdi
   a424e:	mov    0x38(%rsp),%rdx
   a4253:	call   *0xe100f(%rip)        # 185268 <_DYNAMIC+0x478>
   a4259:	jmp    a4288 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a425b:	movb   $0xa,0x8(%rbx)
   a425f:	jmp    a3d74 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a4264:	lea    0xdd175(%rip),%rdx        # 1813e0 <anon.f651f877c5f1b0c83bd435f3f6e617b1.326.llvm.9350274297799828109+0x258>
   a426b:	mov    $0x4,%esi
   a4270:	call   *0xe0eca(%rip)        # 185140 <_DYNAMIC+0x350>
   a4276:	lea    0xdce63(%rip),%rdx        # 1810e0 <anon.f651f877c5f1b0c83bd435f3f6e617b1.258.llvm.9350274297799828109+0x18>
   a427d:	mov    $0x10,%esi
   a4282:	call   *0xe0eb8(%rip)        # 185140 <_DYNAMIC+0x350>
   a4288:	ud2
   a428a:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a428f:	mov    %rax,%rbx
   a4292:	test   %r12,%r12
   a4295:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a429b:	mov    0xd8(%rsp),%rdi
   a42a3:	jmp    a42be <<hyperreal::computable::node::Computable>::approx_signal+0x120e>
   a42a5:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a42aa:	mov    %rax,%rbx
   a42ad:	test   %r12,%r12
   a42b0:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a42b6:	mov    0xb8(%rsp),%rdi
   a42be:	shl    $0x3,%r12
   a42c2:	mov    $0x8,%edx
   a42c7:	jmp    a43dd <<hyperreal::computable::node::Computable>::approx_signal+0x132d>
   a42cc:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a42d1:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a42d6:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a42db:	jmp    a43c9 <<hyperreal::computable::node::Computable>::approx_signal+0x1319>
   a42e0:	mov    %rax,%rbx
   a42e3:	mov    0x330(%rsp),%rsi
   a42eb:	test   %rsi,%rsi
   a42ee:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a42f4:	mov    0x338(%rsp),%rdi
   a42fc:	jmp    a4337 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a42fe:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4303:	mov    %rax,%rbx
   a4306:	mov    0x70(%rsp),%rsi
   a430b:	test   %rsi,%rsi
   a430e:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4314:	mov    0x78(%rsp),%rdi
   a4319:	jmp    a4337 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a431b:	mov    %rax,%rbx
   a431e:	mov    0x90(%rsp),%rsi
   a4326:	test   %rsi,%rsi
   a4329:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a432f:	mov    0x98(%rsp),%rdi
   a4337:	shl    $0x3,%rsi
   a433b:	mov    $0x8,%edx
   a4340:	call   *0xe0d32(%rip)        # 185078 <_DYNAMIC+0x288>
   a4346:	jmp    a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a434b:	mov    %rax,%rbx
   a434e:	test   %r15,%r15
   a4351:	je     a441e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a4357:	mov    0xf8(%rsp),%rdi
   a435f:	shl    $0x3,%r15
   a4363:	mov    $0x8,%edx
   a4368:	mov    %r15,%rsi
   a436b:	jmp    a4418 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a4370:	mov    %rax,%rbx
   a4373:	test   %r12,%r12
   a4376:	je     a441e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a437c:	mov    0xf8(%rsp),%rdi
   a4384:	shl    $0x3,%r12
   a4388:	mov    $0x8,%edx
   a438d:	mov    %r12,%rsi
   a4390:	jmp    a4418 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a4395:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4397:	mov    %rax,%rbx
   a439a:	mov    0x10(%rsp),%rsi
   a439f:	test   %rsi,%rsi
   a43a2:	je     a441e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a43a4:	mov    0x18(%rsp),%rdi
   a43a9:	jmp    a43c3 <<hyperreal::computable::node::Computable>::approx_signal+0x1313>
   a43ab:	mov    %rax,%rbx
   a43ae:	mov    0xf0(%rsp),%rsi
   a43b6:	test   %rsi,%rsi
   a43b9:	je     a441e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a43bb:	mov    0xf8(%rsp),%rdi
   a43c3:	shl    $0x3,%rsi
   a43c7:	jmp    a4413 <<hyperreal::computable::node::Computable>::approx_signal+0x1363>
   a43c9:	mov    %rax,%rbx
   a43cc:	test   %r12,%r12
   a43cf:	je     a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a43d1:	shl    $0x3,%r12
   a43d5:	mov    $0x8,%edx
   a43da:	mov    %r13,%rdi
   a43dd:	mov    %r12,%rsi
   a43e0:	call   *0xe0c92(%rip)        # 185078 <_DYNAMIC+0x288>
   a43e6:	jmp    a43ed <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a43e8:	jmp    a43ea <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a43ea:	mov    %rax,%rbx
   a43ed:	lea    0x210(%rsp),%rdi
   a43f5:	call   7d1a0 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a43fa:	mov    0xf0(%rsp),%rsi
   a4402:	test   %rsi,%rsi
   a4405:	je     a441e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a4407:	mov    0xf8(%rsp),%rdi
   a440f:	shl    $0x4,%rsi
   a4413:	mov    $0x8,%edx
   a4418:	call   *0xe0c5a(%rip)        # 185078 <_DYNAMIC+0x288>
   a441e:	mov    %rbx,%rdi
   a4421:	call   17f640 <_Unwind_Resume@plt>
