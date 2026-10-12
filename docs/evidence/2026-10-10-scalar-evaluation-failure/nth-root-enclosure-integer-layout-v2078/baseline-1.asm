
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1774-domain-predicate-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a3330 <<hyperreal::computable::node::Computable>::approx_signal>:
   a3330:	push   %rbp
   a3331:	push   %r15
   a3333:	push   %r14
   a3335:	push   %r13
   a3337:	push   %r12
   a3339:	push   %rbx
   a333a:	sub    $0x3a8,%rsp
   a3341:	mov    %ecx,%ebp
   a3343:	mov    %rsi,%r13
   a3346:	mov    %rdi,%rbx
   a3349:	mov    (%rdx),%r14
   a334c:	test   %r14,%r14
   a334f:	je     a335e <<hyperreal::computable::node::Computable>::approx_signal+0x2e>
   a3351:	movzbl 0x10(%r14),%eax
   a3356:	test   %al,%al
   a3358:	jne    a3500 <<hyperreal::computable::node::Computable>::approx_signal+0x1d0>
   a335e:	mov    %rdx,0x40(%rsp)
   a3363:	data16 data16 data16 mov %fs:0x0,%rax
   a336f:	lea    -0x120(%rax),%r15
   a3376:	movzbl -0x110(%rax),%eax
   a337d:	test   %eax,%eax
   a337f:	jne    a4300 <<hyperreal::computable::node::Computable>::approx_signal+0xfd0>
   a3385:	movabs $0x7fffffffffffffff,%rax
   a338f:	mov    (%r15),%rcx
   a3392:	cmp    %rax,%rcx
   a3395:	jae    a4330 <<hyperreal::computable::node::Computable>::approx_signal+0x1000>
   a339b:	lea    0x1(%rcx),%rax
   a339f:	mov    %rax,(%r15)
   a33a2:	mov    0x8(%r15),%rdx
   a33a6:	test   %rdx,%rdx
   a33a9:	je     a33c3 <<hyperreal::computable::node::Computable>::approx_signal+0x93>
   a33ab:	nopl   0x0(%rax,%rax,1)
   a33b0:	cmpb   $0x0,0x20(%rdx)
   a33b4:	jne    a34fd <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a33ba:	mov    0x18(%rdx),%rdx
   a33be:	test   %rdx,%rdx
   a33c1:	jne    a33b0 <<hyperreal::computable::node::Computable>::approx_signal+0x80>
   a33c3:	mov    %rcx,(%r15)
   a33c6:	mov    0x0(%r13),%r12
   a33ca:	movzbl 0x10(%r12),%edx
   a33d0:	test   %edx,%edx
   a33d2:	jne    a33f3 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a33d4:	cmpb   $0x1,0x30(%r12)
   a33da:	jne    a33f3 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a33dc:	movups -0x70613(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   a33e3:	movups %xmm0,0x10(%rbx)
   a33e7:	movups -0x7062e(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   a33ee:	jmp    a359e <<hyperreal::computable::node::Computable>::approx_signal+0x26e>
   a33f3:	cmp    $0xfff00000,%ebp
   a33f9:	jge    a340c <<hyperreal::computable::node::Computable>::approx_signal+0xdc>
   a33fb:	neg    %ebp
   a33fd:	setno  %al
   a3400:	add    %al,%al
   a3402:	add    $0xa,%al
   a3404:	mov    %al,0x8(%rbx)
   a3407:	jmp    a3504 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a340c:	cmp    $0x2,%edx
   a340f:	je     a3520 <<hyperreal::computable::node::Computable>::approx_signal+0x1f0>
   a3415:	cmp    $0x1,%edx
   a3418:	je     a347f <<hyperreal::computable::node::Computable>::approx_signal+0x14f>
   a341a:	test   %edx,%edx
   a341c:	jne    a354a <<hyperreal::computable::node::Computable>::approx_signal+0x21a>
   a3422:	mov    %rax,(%r15)
   a3425:	mov    0x8(%r15),%rdx
   a3429:	test   %rdx,%rdx
   a342c:	je     a345a <<hyperreal::computable::node::Computable>::approx_signal+0x12a>
   a342e:	mov    %rdx,%rsi
   a3431:	cmpb   $0x0,0x20(%rsi)
   a3435:	jne    a3470 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a3437:	mov    0x10(%rsi),%rdi
   a343b:	mov    0x18(%rdi),%rax
   a343f:	nop
   a3440:	test   %rax,%rax
   a3443:	je     a3470 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a3445:	lea    -0x1(%rax),%r8
   a3449:	lock cmpxchg %r8,0x18(%rdi)
   a344f:	jne    a3440 <<hyperreal::computable::node::Computable>::approx_signal+0x110>
   a3451:	mov    0x18(%rsi),%rsi
   a3455:	test   %rsi,%rsi
   a3458:	jne    a3431 <<hyperreal::computable::node::Computable>::approx_signal+0x101>
   a345a:	mov    %rcx,(%r15)
   a345d:	add    $0x18,%r12
   a3461:	mov    %rbx,%rdi
   a3464:	mov    %r12,%rsi
   a3467:	jmp    a34d9 <<hyperreal::computable::node::Computable>::approx_signal+0x1a9>
   a3469:	nopl   0x0(%rax)
   a3470:	movb   $0x1,0x20(%rdx)
   a3474:	mov    0x18(%rdx),%rdx
   a3478:	test   %rdx,%rdx
   a347b:	jne    a3470 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a347d:	jmp    a34fd <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a347f:	mov    %rax,(%r15)
   a3482:	mov    0x8(%r15),%rdx
   a3486:	test   %rdx,%rdx
   a3489:	je     a34ba <<hyperreal::computable::node::Computable>::approx_signal+0x18a>
   a348b:	mov    %rdx,%rsi
   a348e:	cmpb   $0x0,0x20(%rsi)
   a3492:	jne    a34f0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a3494:	mov    0x10(%rsi),%rdi
   a3498:	mov    0x18(%rdi),%rax
   a349c:	nopl   0x0(%rax)
   a34a0:	test   %rax,%rax
   a34a3:	je     a34f0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a34a5:	lea    -0x1(%rax),%r8
   a34a9:	lock cmpxchg %r8,0x18(%rdi)
   a34af:	jne    a34a0 <<hyperreal::computable::node::Computable>::approx_signal+0x170>
   a34b1:	mov    0x18(%rsi),%rsi
   a34b5:	test   %rsi,%rsi
   a34b8:	jne    a348e <<hyperreal::computable::node::Computable>::approx_signal+0x15e>
   a34ba:	mov    %rcx,(%r15)
   a34bd:	lea    0x1134d4(%rip),%rdi        # 1b6998 <hyperreal::computable::node::signed::ONE+0x20>
   a34c4:	lea    0x1134ad(%rip),%r14        # 1b6978 <hyperreal::computable::node::signed::ONE>
   a34cb:	mov    %r14,%rsi
   a34ce:	call   60140 <<std::sync::once::Once>::call_once_force::<<std::sync::lazy_lock::LazyLock<num_bigint::bigint::BigInt>>::force::{closure#0}>>
   a34d3:	mov    %rbx,%rdi
   a34d6:	mov    %r14,%rsi
   a34d9:	mov    %ebp,%edx
   a34db:	call   bb760 <hyperreal::computable::node::scale_at_precision_ref>
   a34e0:	jmp    a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a34e2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a34f0:	movb   $0x1,0x20(%rdx)
   a34f4:	mov    0x18(%rdx),%rdx
   a34f8:	test   %rdx,%rdx
   a34fb:	jne    a34f0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a34fd:	mov    %rcx,(%r15)
   a3500:	movb   $0xc,0x8(%rbx)
   a3504:	movq   $0xffffffffffffffff,(%rbx)
   a350b:	mov    %rbx,%rax
   a350e:	add    $0x3a8,%rsp
   a3515:	pop    %rbx
   a3516:	pop    %r12
   a3518:	pop    %r13
   a351a:	pop    %r14
   a351c:	pop    %r15
   a351e:	pop    %rbp
   a351f:	ret
   a3520:	movzbl 0x11(%r12),%esi
   a3526:	lea    0x210(%rsp),%rdi
   a352e:	mov    %ebp,%edx
   a3530:	call   b1ec0 <<hyperreal::computable::node::Computable>::cached_constant_at_precision>
   a3535:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a353e:	jne    a358a <<hyperreal::computable::node::Computable>::approx_signal+0x25a>
   a3540:	movl   $0x0,0xc(%rsp)
   a3548:	jmp    a35ac <<hyperreal::computable::node::Computable>::approx_signal+0x27c>
   a354a:	mov    0x38(%r12),%rax
   a354f:	test   $0x800,%eax
   a3554:	jne    a356b <<hyperreal::computable::node::Computable>::approx_signal+0x23b>
   a3556:	mov    %r13,%rdi
   a3559:	mov    0x40(%rsp),%rsi
   a355e:	call   846d0 <<hyperreal::computable::node::Computable>::ensure_finite_real>
   a3563:	cmp    $0xff,%al
   a3565:	jne    a3404 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a356b:	lea    0x40(%r12),%rsi
   a3570:	lea    0x210(%rsp),%rdi
   a3578:	mov    %ebp,%edx
   a357a:	call   ba280 <<hyperreal::computable::node::EvaluationCache>::at_precision>
   a357f:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a3588:	je     a35a6 <<hyperreal::computable::node::Computable>::approx_signal+0x276>
   a358a:	movups 0x210(%rsp),%xmm0
   a3592:	movups 0x220(%rsp),%xmm1
   a359a:	movups %xmm1,0x10(%rbx)
   a359e:	movups %xmm0,(%rbx)
   a35a1:	jmp    a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a35a6:	mov    $0x1,%al
   a35a8:	mov    %eax,0xc(%rsp)
   a35ac:	lea    0x10(%r12),%rax
   a35b1:	mov    %rax,0x38(%rsp)
   a35b6:	movzbl (%rax),%eax
   a35b9:	add    $0xffffffdc,%eax
   a35bc:	cmp    $0x1a,%eax
   a35bf:	ja     a365b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a35c5:	lea    -0x6f7ac(%rip),%rcx        # 33e20 <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.310272158230796301+0xe08>
   a35cc:	movslq (%rcx,%rax,4),%rax
   a35d0:	add    %rcx,%rax
   a35d3:	jmp    *%rax
   a35d5:	movl   $0x1,0x30(%rsp)
   a35dd:	jmp    a35fb <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a35df:	movl   $0x0,0x30(%rsp)
   a35e7:	jmp    a35fb <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a35e9:	movl   $0x2,0x30(%rsp)
   a35f1:	jmp    a35fb <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a35f3:	movl   $0x3,0x30(%rsp)
   a35fb:	lea    0x18(%r12),%rdi
   a3600:	call   b8cd0 <<hyperreal::computable::node::Computable>::half_pi_index_when_real>
   a3605:	test   $0x1,%al
   a3607:	je     a365b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a3609:	movzbl %dl,%edi
   a360c:	movl   $0xff000100,0x210(%rsp)
   a3617:	cmp    $0x3,%dil
   a361b:	ja     a44e4 <<hyperreal::computable::node::Computable>::approx_signal+0x11b4>
   a3621:	movzbl 0x210(%rsp,%rdi,1),%ecx
   a3629:	movl   $0xff0001,0x210(%rsp)
   a3634:	movzbl 0x210(%rsp,%rdi,1),%eax
   a363c:	mov    0x30(%rsp),%edx
   a3640:	lea    -0x6f7bb(%rip),%rsi        # 33e8c <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.310272158230796301+0xe74>
   a3647:	movslq (%rsi,%rdx,4),%rdx
   a364b:	add    %rsi,%rdx
   a364e:	jmp    *%rdx
   a3650:	mov    %ecx,%eax
   a3652:	movsbq %al,%rsi
   a3656:	jmp    a4034 <<hyperreal::computable::node::Computable>::approx_signal+0xd04>
   a365b:	cmpb   $0x0,0xc(%rsp)
   a3660:	je     a3695 <<hyperreal::computable::node::Computable>::approx_signal+0x365>
   a3662:	lea    0x210(%rsp),%rdi
   a366a:	mov    %r13,%rsi
   a366d:	mov    0x40(%rsp),%rdx
   a3672:	mov    %ebp,%ecx
   a3674:	call   a2650 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a3679:	mov    0x210(%rsp),%r12
   a3681:	cmp    $0xffffffffffffffff,%r12
   a3685:	je     a4102 <<hyperreal::computable::node::Computable>::approx_signal+0xdd2>
   a368b:	cmp    $0xfffffffffffffffe,%r12
   a368f:	jne    a41f8 <<hyperreal::computable::node::Computable>::approx_signal+0xec8>
   a3695:	mov    0x38(%rsp),%rsi
   a369a:	movzbl (%rsi),%eax
   a369d:	cmp    $0xa,%eax
   a36a0:	ja     a410f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a36a6:	mov    $0x430,%ecx
   a36ab:	bt     %eax,%ecx
   a36ae:	jae    a410f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a36b4:	movl   $0xffffffff,0x108(%rsp)
   a36bf:	movl   $0xffffffff,0x118(%rsp)
   a36ca:	movl   $0xffffffff,0x128(%rsp)
   a36d5:	movl   $0xffffffff,0x138(%rsp)
   a36e0:	movl   $0xffffffff,0x148(%rsp)
   a36eb:	movl   $0xffffffff,0x158(%rsp)
   a36f6:	movl   $0xffffffff,0x168(%rsp)
   a3701:	movl   $0xffffffff,0x178(%rsp)
   a370c:	movl   $0xffffffff,0x188(%rsp)
   a3717:	movl   $0xffffffff,0x198(%rsp)
   a3722:	movl   $0xffffffff,0x1a8(%rsp)
   a372d:	movl   $0xffffffff,0x1b8(%rsp)
   a3738:	movl   $0xffffffff,0x1c8(%rsp)
   a3743:	movl   $0xffffffff,0x1d8(%rsp)
   a374e:	movl   $0xffffffff,0x1e8(%rsp)
   a3759:	movl   $0xffffffff,0x1f8(%rsp)
   a3764:	movq   $0x0,0x208(%rsp)
   a3770:	movq   $0x0,0xf0(%rsp)
   a377c:	movq   $0x8,0xf8(%rsp)
   a3788:	movq   $0x0,0x100(%rsp)
   a3794:	movq   $0xffffffffffffffff,0x228(%rsp)
   a37a0:	movq   $0xffffffffffffffff,0x248(%rsp)
   a37ac:	movq   $0xffffffffffffffff,0x268(%rsp)
   a37b8:	movq   $0xffffffffffffffff,0x288(%rsp)
   a37c4:	movq   $0xffffffffffffffff,0x2a8(%rsp)
   a37d0:	movq   $0xffffffffffffffff,0x2c8(%rsp)
   a37dc:	movq   $0xffffffffffffffff,0x2e8(%rsp)
   a37e8:	movq   $0xffffffffffffffff,0x308(%rsp)
   a37f4:	movq   $0x0,0x328(%rsp)
   a3800:	movq   $0x0,0x210(%rsp)
   a380c:	movq   $0x8,0x218(%rsp)
   a3818:	movq   $0x0,0x220(%rsp)
   a3824:	mov    %r13,0x18(%rsp)
   a3829:	mov    %ebp,0x14(%rsp)
   a382d:	movl   $0x0,0x10(%rsp)
   a3835:	lea    0xf0(%rsp),%rdi
   a383d:	lea    0x10(%rsp),%rsi
   a3842:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3847:	lea    0xddbba(%rip),%rax        # 181408 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x30>
   a384e:	mov    %rax,0x38(%rsp)
   a3853:	lea    -0x70bfa(%rip),%rax        # 32c60 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x190>
   a385a:	mov    %rax,0x30(%rsp)
   a385f:	mov    0x100(%rsp),%rax
   a3867:	test   %rax,%rax
   a386a:	je     a3893 <<hyperreal::computable::node::Computable>::approx_signal+0x563>
   a386c:	dec    %rax
   a386f:	mov    %rax,0x100(%rsp)
   a3877:	mov    0xf8(%rsp),%rcx
   a387f:	shl    $0x4,%rax
   a3883:	mov    (%rcx,%rax,1),%r12d
   a3887:	mov    0x4(%rcx,%rax,1),%r9d
   a388c:	mov    0x8(%rcx,%rax,1),%r13
   a3891:	jmp    a38db <<hyperreal::computable::node::Computable>::approx_signal+0x5ab>
   a3893:	mov    0x208(%rsp),%rax
   a389b:	test   %rax,%rax
   a389e:	je     a42cc <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a38a4:	lea    -0x1(%rax),%rdi
   a38a8:	mov    %rdi,0x208(%rsp)
   a38b0:	cmp    $0x10,%rax
   a38b4:	ja     a44f6 <<hyperreal::computable::node::Computable>::approx_signal+0x11c6>
   a38ba:	shl    $0x4,%rdi
   a38be:	lea    0x108(%rsp),%rax
   a38c6:	mov    (%rax,%rdi,1),%r12d
   a38ca:	mov    0x4(%rax,%rdi,1),%r9d
   a38cf:	mov    0x8(%rax,%rdi,1),%r13
   a38d4:	movl   $0xffffffff,(%rax,%rdi,1)
   a38db:	cmp    $0xffffffff,%r12d
   a38df:	je     a42cc <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a38e5:	test   %r14,%r14
   a38e8:	je     a38f7 <<hyperreal::computable::node::Computable>::approx_signal+0x5c7>
   a38ea:	movzbl 0x10(%r14),%eax
   a38ef:	test   %al,%al
   a38f1:	jne    a3ff0 <<hyperreal::computable::node::Computable>::approx_signal+0xcc0>
   a38f7:	movzbl 0x10(%r15),%eax
   a38fc:	test   %eax,%eax
   a38fe:	jne    a3f86 <<hyperreal::computable::node::Computable>::approx_signal+0xc56>
   a3904:	mov    (%r15),%rcx
   a3907:	movabs $0x7fffffffffffffff,%rax
   a3911:	cmp    %rax,%rcx
   a3914:	jae    a3fc0 <<hyperreal::computable::node::Computable>::approx_signal+0xc90>
   a391a:	lea    0x1(%rcx),%rax
   a391e:	mov    %rax,(%r15)
   a3921:	mov    0x8(%r15),%rdx
   a3925:	test   %rdx,%rdx
   a3928:	je     a395e <<hyperreal::computable::node::Computable>::approx_signal+0x62e>
   a392a:	mov    %rdx,%rsi
   a392d:	cmpb   $0x0,0x20(%rsi)
   a3931:	jne    a3fe0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a3937:	mov    0x10(%rsi),%rdi
   a393b:	mov    0x18(%rdi),%rax
   a393f:	nop
   a3940:	test   %rax,%rax
   a3943:	je     a3fe0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a3949:	lea    -0x1(%rax),%r8
   a394d:	lock cmpxchg %r8,0x18(%rdi)
   a3953:	jne    a3940 <<hyperreal::computable::node::Computable>::approx_signal+0x610>
   a3955:	mov    0x18(%rsi),%rsi
   a3959:	test   %rsi,%rsi
   a395c:	jne    a392d <<hyperreal::computable::node::Computable>::approx_signal+0x5fd>
   a395e:	mov    %r9d,0xc(%rsp)
   a3963:	mov    %rcx,(%r15)
   a3966:	mov    %r12d,%eax
   a3969:	lea    -0x6fad4(%rip),%rcx        # 33e9c <anon.e73eca07880e1ca9ab6c937d4a70853c.7.llvm.310272158230796301+0xe84>
   a3970:	movslq (%rcx,%rax,4),%rax
   a3974:	add    %rcx,%rax
   a3977:	jmp    *%rax
   a3979:	mov    0x0(%r13),%rsi
   a397d:	cmpb   $0x0,0x10(%rsi)
   a3981:	jne    a3990 <<hyperreal::computable::node::Computable>::approx_signal+0x660>
   a3983:	cmpb   $0x1,0x30(%rsi)
   a3987:	lea    -0x70bce(%rip),%rax        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   a398e:	je     a39c4 <<hyperreal::computable::node::Computable>::approx_signal+0x694>
   a3990:	mov    0xc(%rsp),%edx
   a3994:	cmp    $0xffefffff,%edx
   a399a:	jle    a43c5 <<hyperreal::computable::node::Computable>::approx_signal+0x1095>
   a39a0:	lea    0x388(%rsp),%rdi
   a39a8:	call   af8e0 <<hyperreal::computable::node::Computable>::cached_at_precision>
   a39ad:	cmpq   $0xffffffffffffffff,0x388(%rsp)
   a39b6:	lea    0x388(%rsp),%rax
   a39be:	je     a3c53 <<hyperreal::computable::node::Computable>::approx_signal+0x923>
   a39c4:	lea    0x210(%rsp),%rdi
   a39cc:	mov    %rax,%rsi
   a39cf:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a39d4:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a39d9:	lea    0x10(%rsp),%rdi
   a39de:	lea    0x210(%rsp),%rsi
   a39e6:	call   a2bc0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a39eb:	mov    %r13,0x68(%rsp)
   a39f0:	mov    0x10(%rsp),%r12
   a39f5:	cmp    $0xffffffffffffffff,%r12
   a39f9:	je     a446b <<hyperreal::computable::node::Computable>::approx_signal+0x113b>
   a39ff:	mov    0x18(%rsp),%r13
   a3a04:	lea    0x29(%rsp),%rax
   a3a09:	movups -0x9(%rax),%xmm0
   a3a0d:	movaps %xmm0,0x350(%rsp)
   a3a15:	lea    0x10(%rsp),%rdi
   a3a1a:	lea    0x210(%rsp),%rsi
   a3a22:	call   a2bc0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a3a27:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a3a2d:	je     a4491 <<hyperreal::computable::node::Computable>::approx_signal+0x1161>
   a3a33:	movups 0x10(%rsp),%xmm0
   a3a38:	movups 0x20(%rsp),%xmm1
   a3a3d:	movaps %xmm1,0x370(%rsp)
   a3a45:	movaps %xmm0,0x360(%rsp)
   a3a4d:	mov    %r12,0x10(%rsp)
   a3a52:	mov    %r13,0x18(%rsp)
   a3a57:	movaps 0x350(%rsp),%xmm0
   a3a5f:	lea    0x29(%rsp),%rax
   a3a64:	movups %xmm0,-0x9(%rax)
   a3a68:	lea    0x48(%rsp),%rbp
   a3a6d:	mov    %rbp,%rdi
   a3a70:	lea    0x360(%rsp),%rsi
   a3a78:	lea    0x10(%rsp),%rdx
   a3a7d:	call   bf560 <<num_bigint::bigint::BigInt as core::ops::arith::Add>::add>
   a3a82:	lea    0x330(%rsp),%r13
   a3a8a:	mov    %r13,%rdi
   a3a8d:	mov    %rbp,%rsi
   a3a90:	mov    $0xfffffffe,%edx
   a3a95:	call   bbc40 <hyperreal::computable::node::scale>
   a3a9a:	movzbl 0x348(%rsp),%r12d
   a3aa3:	mov    %rbp,%rdi
   a3aa6:	mov    %r13,%rsi
   a3aa9:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3aae:	mov    0x58(%rsp),%rax
   a3ab3:	mov    %rax,0x20(%rsp)
   a3ab8:	movups 0x48(%rsp),%xmm0
   a3abd:	movaps %xmm0,0x10(%rsp)
   a3ac2:	mov    %r12b,0x28(%rsp)
   a3ac7:	mov    0x68(%rsp),%rdi
   a3acc:	mov    %r14,%rsi
   a3acf:	mov    0xc(%rsp),%edx
   a3ad3:	lea    0x10(%rsp),%rcx
   a3ad8:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3add:	cmp    $0xff,%al
   a3adf:	jne    a438e <<hyperreal::computable::node::Computable>::approx_signal+0x105e>
   a3ae5:	lea    0x210(%rsp),%rdi
   a3aed:	mov    %r13,%rsi
   a3af0:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3af5:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3afa:	lea    0x10(%rsp),%rdi
   a3aff:	lea    0x210(%rsp),%rsi
   a3b07:	call   a2bc0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a3b0c:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a3b12:	je     a444c <<hyperreal::computable::node::Computable>::approx_signal+0x111c>
   a3b18:	movups 0x10(%rsp),%xmm0
   a3b1d:	movups 0x20(%rsp),%xmm1
   a3b22:	movaps %xmm1,0xa0(%rsp)
   a3b2a:	movaps %xmm0,0x90(%rsp)
   a3b32:	movzbl 0xa8(%rsp),%r12d
   a3b3b:	lea    0x48(%rsp),%rdi
   a3b40:	lea    0x90(%rsp),%rsi
   a3b48:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3b4d:	mov    0xc(%rsp),%edx
   a3b51:	mov    0x58(%rsp),%rax
   a3b56:	mov    %rax,0x20(%rsp)
   a3b5b:	movups 0x48(%rsp),%xmm0
   a3b60:	movaps %xmm0,0x10(%rsp)
   a3b65:	mov    %r12b,0x28(%rsp)
   a3b6a:	mov    %r13,%rdi
   a3b6d:	mov    %r14,%rsi
   a3b70:	lea    0x10(%rsp),%rcx
   a3b75:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3b7a:	cmp    $0xff,%al
   a3b7c:	jne    a434a <<hyperreal::computable::node::Computable>::approx_signal+0x101a>
   a3b82:	lea    0x210(%rsp),%rdi
   a3b8a:	lea    0x90(%rsp),%rsi
   a3b92:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3b97:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3b9c:	lea    0x10(%rsp),%rdi
   a3ba1:	lea    0x210(%rsp),%rsi
   a3ba9:	call   a2bc0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a3bae:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a3bb4:	je     a448a <<hyperreal::computable::node::Computable>::approx_signal+0x115a>
   a3bba:	mov    0x20(%rsp),%rax
   a3bbf:	mov    %rax,0x80(%rsp)
   a3bc7:	movups 0x10(%rsp),%xmm0
   a3bcc:	movaps %xmm0,0x70(%rsp)
   a3bd1:	lea    0x29(%rsp),%rcx
   a3bd6:	mov    (%rcx),%eax
   a3bd8:	mov    $0x2,%r12b
   a3bdb:	sub    0x28(%rsp),%r12b
   a3be0:	mov    0x3(%rcx),%ecx
   a3be3:	lea    0x89(%rsp),%rdx
   a3beb:	mov    %ecx,0x3(%rdx)
   a3bee:	mov    %eax,(%rdx)
   a3bf0:	mov    %r12b,0x88(%rsp)
   a3bf8:	lea    0x48(%rsp),%rdi
   a3bfd:	lea    0x70(%rsp),%rsi
   a3c02:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3c07:	mov    0xc(%rsp),%edx
   a3c0b:	mov    0x58(%rsp),%rax
   a3c10:	mov    %rax,0x20(%rsp)
   a3c15:	movups 0x48(%rsp),%xmm0
   a3c1a:	movaps %xmm0,0x10(%rsp)
   a3c1f:	mov    %r12b,0x28(%rsp)
   a3c24:	mov    %r13,%rdi
   a3c27:	mov    %r14,%rsi
   a3c2a:	lea    0x10(%rsp),%rcx
   a3c2f:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3c34:	cmp    $0xff,%al
   a3c36:	jne    a436f <<hyperreal::computable::node::Computable>::approx_signal+0x103f>
   a3c3c:	lea    0x210(%rsp),%rdi
   a3c44:	lea    0x70(%rsp),%rsi
   a3c49:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3c4e:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3c53:	lea    0x10(%rsp),%rdi
   a3c58:	mov    %r13,%rsi
   a3c5b:	mov    0x40(%rsp),%rdx
   a3c60:	mov    0xc(%rsp),%ecx
   a3c64:	call   a2650 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a3c69:	mov    0x10(%rsp),%r12
   a3c6e:	cmp    $0xfffffffffffffffe,%r12
   a3c72:	je     a3d1f <<hyperreal::computable::node::Computable>::approx_signal+0x9ef>
   a3c78:	cmp    $0xffffffffffffffff,%r12
   a3c7c:	je     a43d6 <<hyperreal::computable::node::Computable>::approx_signal+0x10a6>
   a3c82:	mov    %r13,%rbp
   a3c85:	movzbl 0x18(%rsp),%eax
   a3c8a:	lea    0x29(%rsp),%rdx
   a3c8f:	mov    -0x1(%rdx),%rcx
   a3c93:	lea    0xb9(%rsp),%rsi
   a3c9b:	mov    %rcx,0xf(%rsi)
   a3c9f:	movups -0x10(%rdx),%xmm0
   a3ca3:	movups %xmm0,(%rsi)
   a3ca6:	mov    %r12,0xb0(%rsp)
   a3cae:	mov    %al,0xb8(%rsp)
   a3cb5:	movzbl 0xc8(%rsp),%r13d
   a3cbe:	lea    0x48(%rsp),%rdi
   a3cc3:	lea    0xb0(%rsp),%rsi
   a3ccb:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3cd0:	mov    0x58(%rsp),%rax
   a3cd5:	mov    %rax,0x20(%rsp)
   a3cda:	movups 0x48(%rsp),%xmm0
   a3cdf:	movaps %xmm0,0x10(%rsp)
   a3ce4:	mov    %r13b,0x28(%rsp)
   a3ce9:	mov    %rbp,%rdi
   a3cec:	mov    %r14,%rsi
   a3cef:	mov    0xc(%rsp),%edx
   a3cf3:	lea    0x10(%rsp),%rcx
   a3cf8:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3cfd:	cmp    $0xff,%al
   a3cff:	jne    a43e3 <<hyperreal::computable::node::Computable>::approx_signal+0x10b3>
   a3d05:	lea    0x210(%rsp),%rdi
   a3d0d:	lea    0xb0(%rsp),%rsi
   a3d15:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3d1a:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3d1f:	mov    0x0(%r13),%r12
   a3d23:	movzbl 0x10(%r12),%eax
   a3d29:	cmp    $0xa,%eax
   a3d2c:	je     a3e56 <<hyperreal::computable::node::Computable>::approx_signal+0xb26>
   a3d32:	cmp    $0x5,%eax
   a3d35:	je     a3d97 <<hyperreal::computable::node::Computable>::approx_signal+0xa67>
   a3d37:	cmp    $0x4,%eax
   a3d3a:	jne    a3ec0 <<hyperreal::computable::node::Computable>::approx_signal+0xb90>
   a3d40:	mov    %r13,0x18(%rsp)
   a3d45:	mov    0xc(%rsp),%eax
   a3d49:	mov    %eax,0x14(%rsp)
   a3d4d:	movl   $0x1,0x10(%rsp)
   a3d55:	lea    0xf0(%rsp),%rdi
   a3d5d:	lea    0x10(%rsp),%rsi
   a3d62:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3d67:	add    $0x18,%r12
   a3d6b:	mov    %r12,0x18(%rsp)
   a3d70:	mov    0xc(%rsp),%eax
   a3d74:	mov    %eax,0x14(%rsp)
   a3d78:	movl   $0x0,0x10(%rsp)
   a3d80:	lea    0xf0(%rsp),%rdi
   a3d88:	lea    0x10(%rsp),%rsi
   a3d8d:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3d92:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3d97:	mov    %r13,0x18(%rsp)
   a3d9c:	mov    0xc(%rsp),%eax
   a3da0:	mov    %eax,0x14(%rsp)
   a3da4:	movl   $0x2,0x10(%rsp)
   a3dac:	lea    0xf0(%rsp),%rdi
   a3db4:	lea    0x10(%rsp),%rsi
   a3db9:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3dbe:	lea    0x28(%r12),%rdx
   a3dc3:	lea    0x18(%r12),%rbp
   a3dc8:	mov    0x28(%r12),%rax
   a3dcd:	mov    0x38(%rax),%r13
   a3dd1:	shr    $0x10,%r13d
   a3dd5:	mov    0x18(%r12),%rax
   a3dda:	mov    0x38(%rax),%r12
   a3dde:	shr    $0x10,%r12d
   a3de2:	mov    0xc(%rsp),%ecx
   a3de6:	add    $0xfffffffe,%ecx
   a3de9:	cmp    %r12w,%r13w
   a3ded:	mov    %rdx,0x68(%rsp)
   a3df2:	mov    %rdx,%rax
   a3df5:	cmovg  %rbp,%rax
   a3df9:	mov    %rax,0x18(%rsp)
   a3dfe:	mov    %ecx,0xc(%rsp)
   a3e02:	mov    %ecx,0x14(%rsp)
   a3e06:	movl   $0x0,0x10(%rsp)
   a3e0e:	lea    0xf0(%rsp),%rdi
   a3e16:	lea    0x10(%rsp),%rsi
   a3e1b:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3e20:	cmp    %r12w,%r13w
   a3e24:	cmovg  0x68(%rsp),%rbp
   a3e2a:	mov    %rbp,0x18(%rsp)
   a3e2f:	mov    0xc(%rsp),%eax
   a3e33:	mov    %eax,0x14(%rsp)
   a3e37:	movl   $0x0,0x10(%rsp)
   a3e3f:	lea    0xf0(%rsp),%rdi
   a3e47:	lea    0x10(%rsp),%rsi
   a3e4c:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3e51:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3e56:	mov    %r13,0x18(%rsp)
   a3e5b:	mov    0xc(%rsp),%eax
   a3e5f:	mov    %eax,0x14(%rsp)
   a3e63:	movl   $0x3,0x10(%rsp)
   a3e6b:	lea    0xf0(%rsp),%rdi
   a3e73:	lea    0x10(%rsp),%rsi
   a3e78:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3e7d:	mov    0xc(%rsp),%eax
   a3e81:	sub    0x14(%r12),%eax
   a3e86:	mov    %eax,0xc(%rsp)
   a3e8a:	jo     a44db <<hyperreal::computable::node::Computable>::approx_signal+0x11ab>
   a3e90:	add    $0x18,%r12
   a3e94:	mov    %r12,0x18(%rsp)
   a3e99:	mov    0xc(%rsp),%eax
   a3e9d:	mov    %eax,0x14(%rsp)
   a3ea1:	movl   $0x0,0x10(%rsp)
   a3ea9:	lea    0xf0(%rsp),%rdi
   a3eb1:	lea    0x10(%rsp),%rsi
   a3eb6:	call   a2d30 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a3ebb:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3ec0:	add    $0x10,%r12
   a3ec4:	lea    0x10(%rsp),%rdi
   a3ec9:	mov    %r12,%rsi
   a3ecc:	mov    0x40(%rsp),%rdx
   a3ed1:	mov    0xc(%rsp),%ecx
   a3ed5:	call   e9620 <<hyperreal::computable::approximation::Approximation>::approximate>
   a3eda:	mov    0x10(%rsp),%r12
   a3edf:	movzbl 0x18(%rsp),%eax
   a3ee4:	cmp    $0xffffffffffffffff,%r12
   a3ee8:	je     a4427 <<hyperreal::computable::node::Computable>::approx_signal+0x10f7>
   a3eee:	mov    %r13,%rbp
   a3ef1:	lea    0x29(%rsp),%rdx
   a3ef6:	mov    -0x1(%rdx),%rcx
   a3efa:	lea    0xd9(%rsp),%rsi
   a3f02:	mov    %rcx,0xf(%rsi)
   a3f06:	movups -0x10(%rdx),%xmm0
   a3f0a:	movups %xmm0,(%rsi)
   a3f0d:	mov    %r12,0xd0(%rsp)
   a3f15:	mov    %al,0xd8(%rsp)
   a3f1c:	movzbl 0xe8(%rsp),%r13d
   a3f25:	lea    0x48(%rsp),%rdi
   a3f2a:	lea    0xd0(%rsp),%rsi
   a3f32:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a3f37:	mov    0x58(%rsp),%rax
   a3f3c:	mov    %rax,0x20(%rsp)
   a3f41:	movups 0x48(%rsp),%xmm0
   a3f46:	movaps %xmm0,0x10(%rsp)
   a3f4b:	mov    %r13b,0x28(%rsp)
   a3f50:	mov    %rbp,%rdi
   a3f53:	mov    %r14,%rsi
   a3f56:	mov    0xc(%rsp),%edx
   a3f5a:	lea    0x10(%rsp),%rcx
   a3f5f:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a3f64:	cmp    $0xff,%al
   a3f66:	jne    a442f <<hyperreal::computable::node::Computable>::approx_signal+0x10ff>
   a3f6c:	lea    0x210(%rsp),%rdi
   a3f74:	lea    0xd0(%rsp),%rsi
   a3f7c:	call   a2c50 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a3f81:	jmp    a385f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a3f86:	cmp    $0x2,%eax
   a3f89:	je     a4415 <<hyperreal::computable::node::Computable>::approx_signal+0x10e5>
   a3f8f:	mov    %r9d,%ebp
   a3f92:	mov    %r15,%rdi
   a3f95:	lea    0x84f84(%rip),%rsi        # 128f20 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a3f9c:	call   *0xe1516(%rip)        # 1854b8 <_DYNAMIC+0x388>
   a3fa2:	movb   $0x0,0x10(%r15)
   a3fa7:	mov    %ebp,%r9d
   a3faa:	mov    (%r15),%rcx
   a3fad:	movabs $0x7fffffffffffffff,%rax
   a3fb7:	cmp    %rax,%rcx
   a3fba:	jb     a391a <<hyperreal::computable::node::Computable>::approx_signal+0x5ea>
   a3fc0:	lea    0xde7b9(%rip),%rdi        # 182780 <anon.953f3db8f5a996f677fb6a53175227a4.23.llvm.5533634630175563929>
   a3fc7:	call   *0xe14f3(%rip)        # 1854c0 <_DYNAMIC+0x390>
   a3fcd:	jmp    a4508 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a3fd2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a3fe0:	movb   $0x1,0x20(%rdx)
   a3fe4:	mov    0x18(%rdx),%rdx
   a3fe8:	test   %rdx,%rdx
   a3feb:	jne    a3fe0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a3fed:	mov    %rcx,(%r15)
   a3ff0:	movb   $0xc,0x8(%rbx)
   a3ff4:	movq   $0xffffffffffffffff,(%rbx)
   a3ffb:	lea    0x210(%rsp),%rdi
   a4003:	call   7d210 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a4008:	mov    0xf0(%rsp),%rsi
   a4010:	test   %rsi,%rsi
   a4013:	je     a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a4019:	mov    0xf8(%rsp),%rdi
   a4021:	shl    $0x4,%rsi
   a4025:	jmp    a40e1 <<hyperreal::computable::node::Computable>::approx_signal+0xdb1>
   a402a:	test   %al,%al
   a402c:	je     a40f9 <<hyperreal::computable::node::Computable>::approx_signal+0xdc9>
   a4032:	xor    %esi,%esi
   a4034:	lea    0x10(%rsp),%rdi
   a4039:	call   bfbd0 <<num_bigint::bigint::BigInt as core::convert::From<i64>>::from>
   a403e:	add    $0x38,%r12
   a4042:	mov    0x28(%rsp),%esi
   a4046:	mov    %r12,%rdi
   a4049:	call   a5740 <<hyperreal::computable::node::AtomicFacts>::publish_exact_sign>
   a404e:	lea    0xf0(%rsp),%r15
   a4056:	lea    0x10(%rsp),%rsi
   a405b:	mov    %r15,%rdi
   a405e:	mov    %ebp,%edx
   a4060:	call   bb580 <hyperreal::computable::node::scale_at_precision>
   a4065:	movzbl 0x108(%rsp),%r12d
   a406e:	lea    0x48(%rsp),%rdi
   a4073:	mov    %r15,%rsi
   a4076:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a407b:	mov    0x58(%rsp),%rax
   a4080:	mov    %rax,0x220(%rsp)
   a4088:	movups 0x48(%rsp),%xmm0
   a408d:	movaps %xmm0,0x210(%rsp)
   a4095:	mov    %r12b,0x228(%rsp)
   a409d:	lea    0x210(%rsp),%rcx
   a40a5:	mov    %r13,%rdi
   a40a8:	mov    %r14,%rsi
   a40ab:	mov    %ebp,%edx
   a40ad:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a40b2:	cmp    $0xff,%al
   a40b4:	je     a42b7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a40ba:	mov    %al,0x8(%rbx)
   a40bd:	movq   $0xffffffffffffffff,(%rbx)
   a40c4:	mov    0xf0(%rsp),%rsi
   a40cc:	test   %rsi,%rsi
   a40cf:	je     a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a40d5:	mov    0xf8(%rsp),%rdi
   a40dd:	shl    $0x3,%rsi
   a40e1:	mov    $0x8,%edx
   a40e6:	call   *0xe12cc(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a40ec:	jmp    a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a40f1:	test   %cl,%cl
   a40f3:	jne    a4032 <<hyperreal::computable::node::Computable>::approx_signal+0xd02>
   a40f9:	movb   $0x2,0x8(%rbx)
   a40fd:	jmp    a3504 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a4102:	movzbl 0x218(%rsp),%eax
   a410a:	jmp    a3404 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a410f:	lea    0x210(%rsp),%rdi
   a4117:	mov    0x40(%rsp),%rdx
   a411c:	mov    %ebp,%ecx
   a411e:	call   e9620 <<hyperreal::computable::approximation::Approximation>::approximate>
   a4123:	mov    0x210(%rsp),%r15
   a412b:	movzbl 0x218(%rsp),%eax
   a4133:	cmp    $0xffffffffffffffff,%r15
   a4137:	je     a3404 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a413d:	mov    0x228(%rsp),%rcx
   a4145:	mov    %rcx,0x108(%rsp)
   a414d:	movups 0x219(%rsp),%xmm0
   a4155:	movups %xmm0,0xf9(%rsp)
   a415d:	mov    %r15,0xf0(%rsp)
   a4165:	mov    %al,0xf8(%rsp)
   a416c:	movzbl 0x108(%rsp),%r12d
   a4175:	lea    0x10(%rsp),%rdi
   a417a:	lea    0xf0(%rsp),%rsi
   a4182:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a4187:	mov    0x20(%rsp),%rax
   a418c:	mov    %rax,0x220(%rsp)
   a4194:	movups 0x10(%rsp),%xmm0
   a4199:	movaps %xmm0,0x210(%rsp)
   a41a1:	mov    %r12b,0x228(%rsp)
   a41a9:	lea    0x210(%rsp),%rcx
   a41b1:	mov    %r13,%rdi
   a41b4:	mov    %r14,%rsi
   a41b7:	mov    %ebp,%edx
   a41b9:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a41be:	cmp    $0xff,%al
   a41c0:	je     a42b7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a41c6:	mov    %al,0x8(%rbx)
   a41c9:	movq   $0xffffffffffffffff,(%rbx)
   a41d0:	test   %r15,%r15
   a41d3:	je     a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a41d9:	mov    0xf8(%rsp),%rdi
   a41e1:	shl    $0x3,%r15
   a41e5:	mov    $0x8,%edx
   a41ea:	mov    %r15,%rsi
   a41ed:	call   *0xe11c5(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a41f3:	jmp    a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a41f8:	movzbl 0x218(%rsp),%eax
   a4200:	mov    0x228(%rsp),%rcx
   a4208:	mov    %rcx,0x108(%rsp)
   a4210:	movups 0x219(%rsp),%xmm0
   a4218:	movups %xmm0,0xf9(%rsp)
   a4220:	mov    %r12,0xf0(%rsp)
   a4228:	mov    %al,0xf8(%rsp)
   a422f:	movzbl 0x108(%rsp),%r15d
   a4238:	lea    0x10(%rsp),%rdi
   a423d:	lea    0xf0(%rsp),%rsi
   a4245:	call   128590 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a424a:	mov    0x20(%rsp),%rax
   a424f:	mov    %rax,0x220(%rsp)
   a4257:	movups 0x10(%rsp),%xmm0
   a425c:	movaps %xmm0,0x210(%rsp)
   a4264:	mov    %r15b,0x228(%rsp)
   a426c:	lea    0x210(%rsp),%rcx
   a4274:	mov    %r13,%rdi
   a4277:	mov    %r14,%rsi
   a427a:	mov    %ebp,%edx
   a427c:	call   af270 <<hyperreal::computable::node::Computable>::store_cache_value>
   a4281:	cmp    $0xff,%al
   a4283:	je     a42b7 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a4285:	mov    %al,0x8(%rbx)
   a4288:	movq   $0xffffffffffffffff,(%rbx)
   a428f:	test   %r12,%r12
   a4292:	je     a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a4298:	mov    0xf8(%rsp),%rdi
   a42a0:	shl    $0x3,%r12
   a42a4:	mov    $0x8,%edx
   a42a9:	mov    %r12,%rsi
   a42ac:	call   *0xe1106(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a42b2:	jmp    a350b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a42b7:	movups 0xf0(%rsp),%xmm0
   a42bf:	movups 0x100(%rsp),%xmm1
   a42c7:	jmp    a359a <<hyperreal::computable::node::Computable>::approx_signal+0x26a>
   a42cc:	lea    0x10(%rsp),%rdi
   a42d1:	lea    0x210(%rsp),%rsi
   a42d9:	call   a2bc0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a42de:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a42e4:	je     a44ac <<hyperreal::computable::node::Computable>::approx_signal+0x117c>
   a42ea:	movups 0x10(%rsp),%xmm0
   a42ef:	movups 0x20(%rsp),%xmm1
   a42f4:	movups %xmm1,0x10(%rbx)
   a42f8:	movups %xmm0,(%rbx)
   a42fb:	jmp    a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4300:	cmp    $0x1,%eax
   a4303:	jne    a433d <<hyperreal::computable::node::Computable>::approx_signal+0x100d>
   a4305:	lea    0x84c14(%rip),%rsi        # 128f20 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a430c:	mov    %r15,%rdi
   a430f:	call   *0xe11a3(%rip)        # 1854b8 <_DYNAMIC+0x388>
   a4315:	movb   $0x0,0x10(%r15)
   a431a:	movabs $0x7fffffffffffffff,%rax
   a4324:	mov    (%r15),%rcx
   a4327:	cmp    %rax,%rcx
   a432a:	jb     a339b <<hyperreal::computable::node::Computable>::approx_signal+0x6b>
   a4330:	lea    0xde461(%rip),%rdi        # 182798 <anon.953f3db8f5a996f677fb6a53175227a4.24.llvm.5533634630175563929>
   a4337:	call   *0xe1183(%rip)        # 1854c0 <_DYNAMIC+0x390>
   a433d:	lea    0xde38c(%rip),%rdi        # 1826d0 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.5533634630175563929>
   a4344:	call   *0xe117e(%rip)        # 1854c8 <_DYNAMIC+0x398>
   a434a:	mov    %al,0x8(%rbx)
   a434d:	movq   $0xffffffffffffffff,(%rbx)
   a4354:	mov    0x90(%rsp),%rsi
   a435c:	test   %rsi,%rsi
   a435f:	je     a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4365:	mov    0x98(%rsp),%rdi
   a436d:	jmp    a43b1 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a436f:	mov    %al,0x8(%rbx)
   a4372:	movq   $0xffffffffffffffff,(%rbx)
   a4379:	mov    0x70(%rsp),%rsi
   a437e:	test   %rsi,%rsi
   a4381:	je     a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4387:	mov    0x78(%rsp),%rdi
   a438c:	jmp    a43b1 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a438e:	mov    %al,0x8(%rbx)
   a4391:	movq   $0xffffffffffffffff,(%rbx)
   a4398:	mov    0x330(%rsp),%rsi
   a43a0:	test   %rsi,%rsi
   a43a3:	je     a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a43a9:	mov    0x338(%rsp),%rdi
   a43b1:	shl    $0x3,%rsi
   a43b5:	mov    $0x8,%edx
   a43ba:	call   *0xe0ff8(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a43c0:	jmp    a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a43c5:	neg    %edx
   a43c7:	setno  %al
   a43ca:	add    %al,%al
   a43cc:	add    $0xa,%al
   a43ce:	mov    %al,0x8(%rbx)
   a43d1:	jmp    a3ff4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a43d6:	movzbl 0x18(%rsp),%eax
   a43db:	mov    %al,0x8(%rbx)
   a43de:	jmp    a3ff4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a43e3:	mov    %al,0x8(%rbx)
   a43e6:	movq   $0xffffffffffffffff,(%rbx)
   a43ed:	test   %r12,%r12
   a43f0:	je     a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a43f6:	mov    0xb8(%rsp),%rdi
   a43fe:	shl    $0x3,%r12
   a4402:	mov    $0x8,%edx
   a4407:	mov    %r12,%rsi
   a440a:	call   *0xe0fa8(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a4410:	jmp    a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4415:	lea    0xde2b4(%rip),%rdi        # 1826d0 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.5533634630175563929>
   a441c:	call   *0xe10a6(%rip)        # 1854c8 <_DYNAMIC+0x398>
   a4422:	jmp    a4508 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a4427:	mov    %al,0x8(%rbx)
   a442a:	jmp    a3ff4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a442f:	mov    %al,0x8(%rbx)
   a4432:	movq   $0xffffffffffffffff,(%rbx)
   a4439:	test   %r12,%r12
   a443c:	je     a3ffb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a4442:	mov    0xd8(%rsp),%rdi
   a444a:	jmp    a43fe <<hyperreal::computable::node::Computable>::approx_signal+0x10ce>
   a444c:	mov    $0x20,%esi
   a4451:	lea    -0x717b8(%rip),%rax        # 32ca0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x1d0>
   a4458:	mov    %rax,0x30(%rsp)
   a445d:	lea    0xdcfec(%rip),%rax        # 181450 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x78>
   a4464:	mov    %rax,0x38(%rsp)
   a4469:	jmp    a44c9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a446b:	mov    $0x1b,%esi
   a4470:	lea    -0x6f021(%rip),%rax        # 35456 <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.1768662447242434102+0x6b>
   a4477:	mov    %rax,0x30(%rsp)
   a447c:	lea    0xdcf9d(%rip),%rax        # 181420 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x48>
   a4483:	mov    %rax,0x38(%rsp)
   a4488:	jmp    a44c9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a448a:	mov    $0x20,%esi
   a448f:	jmp    a44c9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a4491:	lea    -0x6f027(%rip),%rdi        # 35471 <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.1768662447242434102+0x86>
   a4498:	lea    0xdcf99(%rip),%rdx        # 181438 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x60>
   a449f:	mov    $0x1b,%esi
   a44a4:	call   *0xe10fe(%rip)        # 1855a8 <_DYNAMIC+0x478>
   a44aa:	jmp    a4508 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a44ac:	lea    0xdcfb5(%rip),%rax        # 181468 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x90>
   a44b3:	mov    %rax,0x38(%rsp)
   a44b8:	mov    $0x22,%esi
   a44bd:	lea    -0x6f038(%rip),%rax        # 3548c <anon.9023bb4654a08944d5043612f590d0ff.257.llvm.1768662447242434102+0xa1>
   a44c4:	mov    %rax,0x30(%rsp)
   a44c9:	mov    0x30(%rsp),%rdi
   a44ce:	mov    0x38(%rsp),%rdx
   a44d3:	call   *0xe10cf(%rip)        # 1855a8 <_DYNAMIC+0x478>
   a44d9:	jmp    a4508 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a44db:	movb   $0xa,0x8(%rbx)
   a44df:	jmp    a3ff4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a44e4:	lea    0xdd205(%rip),%rdx        # 1816f0 <anon.9023bb4654a08944d5043612f590d0ff.326.llvm.1768662447242434102+0x258>
   a44eb:	mov    $0x4,%esi
   a44f0:	call   *0xe0f8a(%rip)        # 185480 <_DYNAMIC+0x350>
   a44f6:	lea    0xdcef3(%rip),%rdx        # 1813f0 <anon.9023bb4654a08944d5043612f590d0ff.258.llvm.1768662447242434102+0x18>
   a44fd:	mov    $0x10,%esi
   a4502:	call   *0xe0f78(%rip)        # 185480 <_DYNAMIC+0x350>
   a4508:	ud2
   a450a:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a450f:	mov    %rax,%rbx
   a4512:	test   %r12,%r12
   a4515:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a451b:	mov    0xd8(%rsp),%rdi
   a4523:	jmp    a453e <<hyperreal::computable::node::Computable>::approx_signal+0x120e>
   a4525:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a452a:	mov    %rax,%rbx
   a452d:	test   %r12,%r12
   a4530:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4536:	mov    0xb8(%rsp),%rdi
   a453e:	shl    $0x3,%r12
   a4542:	mov    $0x8,%edx
   a4547:	jmp    a465d <<hyperreal::computable::node::Computable>::approx_signal+0x132d>
   a454c:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4551:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4556:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a455b:	jmp    a4649 <<hyperreal::computable::node::Computable>::approx_signal+0x1319>
   a4560:	mov    %rax,%rbx
   a4563:	mov    0x330(%rsp),%rsi
   a456b:	test   %rsi,%rsi
   a456e:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4574:	mov    0x338(%rsp),%rdi
   a457c:	jmp    a45b7 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a457e:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4583:	mov    %rax,%rbx
   a4586:	mov    0x70(%rsp),%rsi
   a458b:	test   %rsi,%rsi
   a458e:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4594:	mov    0x78(%rsp),%rdi
   a4599:	jmp    a45b7 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a459b:	mov    %rax,%rbx
   a459e:	mov    0x90(%rsp),%rsi
   a45a6:	test   %rsi,%rsi
   a45a9:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a45af:	mov    0x98(%rsp),%rdi
   a45b7:	shl    $0x3,%rsi
   a45bb:	mov    $0x8,%edx
   a45c0:	call   *0xe0df2(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a45c6:	jmp    a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a45cb:	mov    %rax,%rbx
   a45ce:	test   %r15,%r15
   a45d1:	je     a469e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a45d7:	mov    0xf8(%rsp),%rdi
   a45df:	shl    $0x3,%r15
   a45e3:	mov    $0x8,%edx
   a45e8:	mov    %r15,%rsi
   a45eb:	jmp    a4698 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a45f0:	mov    %rax,%rbx
   a45f3:	test   %r12,%r12
   a45f6:	je     a469e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a45fc:	mov    0xf8(%rsp),%rdi
   a4604:	shl    $0x3,%r12
   a4608:	mov    $0x8,%edx
   a460d:	mov    %r12,%rsi
   a4610:	jmp    a4698 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a4615:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a4617:	mov    %rax,%rbx
   a461a:	mov    0x10(%rsp),%rsi
   a461f:	test   %rsi,%rsi
   a4622:	je     a469e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a4624:	mov    0x18(%rsp),%rdi
   a4629:	jmp    a4643 <<hyperreal::computable::node::Computable>::approx_signal+0x1313>
   a462b:	mov    %rax,%rbx
   a462e:	mov    0xf0(%rsp),%rsi
   a4636:	test   %rsi,%rsi
   a4639:	je     a469e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a463b:	mov    0xf8(%rsp),%rdi
   a4643:	shl    $0x3,%rsi
   a4647:	jmp    a4693 <<hyperreal::computable::node::Computable>::approx_signal+0x1363>
   a4649:	mov    %rax,%rbx
   a464c:	test   %r12,%r12
   a464f:	je     a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4651:	shl    $0x3,%r12
   a4655:	mov    $0x8,%edx
   a465a:	mov    %r13,%rdi
   a465d:	mov    %r12,%rsi
   a4660:	call   *0xe0d52(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a4666:	jmp    a466d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a4668:	jmp    a466a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a466a:	mov    %rax,%rbx
   a466d:	lea    0x210(%rsp),%rdi
   a4675:	call   7d210 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a467a:	mov    0xf0(%rsp),%rsi
   a4682:	test   %rsi,%rsi
   a4685:	je     a469e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a4687:	mov    0xf8(%rsp),%rdi
   a468f:	shl    $0x4,%rsi
   a4693:	mov    $0x8,%edx
   a4698:	call   *0xe0d1a(%rip)        # 1853b8 <_DYNAMIC+0x288>
   a469e:	mov    %rbx,%rdi
   a46a1:	call   17f950 <_Unwind_Resume@plt>
