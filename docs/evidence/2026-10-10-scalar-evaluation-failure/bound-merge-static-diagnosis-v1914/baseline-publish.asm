
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1773-domain-predicate-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a8420 <<hyperreal::computable::node::AtomicFacts>::publish_bound>:
   a8420:	push   %rbp
   a8421:	push   %r15
   a8423:	push   %r14
   a8425:	push   %r13
   a8427:	push   %r12
   a8429:	push   %rbx
   a842a:	mov    (%rdi),%rax
   a842d:	mov    (%rsi),%r15d
   a8430:	mov    %r15d,%ecx
   a8433:	sub    $0x2,%ecx
   a8436:	mov    $0x2,%r8d
   a843c:	cmovae %ecx,%r8d
   a8440:	mov    0x4(%rsi),%r9d
   a8444:	movzbl 0x9(%rsi),%ecx
   a8448:	mov    %cl,-0x6(%rsp)
   a844c:	mov    0x8(%rsi),%ebx
   a844f:	xor    %ecx,%ecx
   a8451:	test   $0x1,%r15b
   a8455:	mov    %ebx,%edx
   a8457:	cmove  %ecx,%edx
   a845a:	mov    %edx,-0x4(%rsp)
   a845e:	mov    %r15d,%ecx
   a8461:	xor    $0x1,%cl
   a8464:	mov    %cl,-0x5(%rsp)
   a8468:	nopl   0x0(%rax,%rax,1)
   a8470:	mov    %eax,%ecx
   a8472:	and    $0x3,%ecx
   a8475:	cmp    $0x2,%rcx
   a8479:	jae    a84b0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x90>
   a847b:	cmp    $0xffffffff,%r15d
   a847f:	je     a8521 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x101>
   a8485:	mov    %r15d,%edx
   a8488:	mov    %r9d,%ecx
   a848b:	mov    %ebx,%esi
   a848d:	movzbl -0x6(%rsp),%r10d
   a8493:	mov    %r10d,%r12d
   a8496:	cmp    $0x2,%edx
   a8499:	jae    a858d <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x16d>
   a849f:	jmp    a85d0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1b0>
   a84a4:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   a84b0:	cmp    $0x2,%ecx
   a84b3:	jne    a84d0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0xb0>
   a84b5:	mov    $0x3,%edx
   a84ba:	mov    $0x1,%r10b
   a84bd:	xor    %ecx,%ecx
   a84bf:	test   %r8d,%r8d
   a84c2:	jne    a8503 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0xe3>
   a84c4:	jmp    a8580 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x160>
   a84c9:	nopl   0x0(%rax)
   a84d0:	lea    (%rax,%rax,1),%ecx
   a84d3:	and    $0x18,%cl
   a84d6:	mov    $0x10002ff,%r14d
   a84dc:	shr    %cl,%r14d
   a84df:	mov    %rax,%rbp
   a84e2:	shr    $0x20,%rbp
   a84e6:	mov    %eax,%edx
   a84e8:	shr    $0x4,%edx
   a84eb:	and    $0x1,%edx
   a84ee:	mov    %eax,%r13d
   a84f1:	shr    $0x5,%r13b
   a84f5:	and    $0x1,%r13b
   a84f9:	mov    $0x1,%cl
   a84fb:	xor    %r10d,%r10d
   a84fe:	test   %r8d,%r8d
   a8501:	je     a8580 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x160>
   a8503:	cmp    $0x1,%r8d
   a8507:	jne    a8529 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x109>
   a8509:	mov    $0x2,%r11d
   a850f:	mov    %ebp,%ecx
   a8511:	mov    %r13d,%esi
   a8514:	mov    %r14d,%r12d
   a8517:	test   %r10b,%r10b
   a851a:	je     a8588 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x168>
   a851c:	jmp    a85ff <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1df>
   a8521:	xor    %r11d,%r11d
   a8524:	jmp    a85ff <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1df>
   a8529:	test   %cl,%cl
   a852b:	je     a8580 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x160>
   a852d:	mov    %r15d,%r10d
   a8530:	mov    %r9d,%ecx
   a8533:	mov    -0x4(%rsp),%esi
   a8537:	test   $0x1,%dl
   a853a:	je     a85b5 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x195>
   a853c:	movzbl -0x5(%rsp),%ecx
   a8541:	or     %r13b,%cl
   a8544:	mov    %r13d,%esi
   a8547:	and    $0x1,%sil
   a854b:	mov    %ecx,%r10d
   a854e:	or     %bl,%r10b
   a8551:	test   $0x1,%cl
   a8554:	cmove  %r15d,%edx
   a8558:	mov    %r9d,%ecx
   a855b:	cmovne %ebp,%ecx
   a855e:	movzbl %sil,%esi
   a8562:	mov    $0x1,%r11d
   a8568:	cmove  %r11d,%esi
   a856c:	test   $0x1,%r10b
   a8570:	je     a85a4 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x184>
   a8572:	mov    %edx,%r10d
   a8575:	jmp    a85b5 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x195>
   a8577:	nopw   0x0(%rax,%rax,1)
   a8580:	mov    %ebp,%ecx
   a8582:	mov    %r13d,%esi
   a8585:	mov    %r14d,%r12d
   a8588:	cmp    $0x2,%edx
   a858b:	jb     a85d0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1b0>
   a858d:	je     a859c <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x17c>
   a858f:	cmp    $0x3,%edx
   a8592:	jne    a85d0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1b0>
   a8594:	mov    $0x2,%r11d
   a859a:	jmp    a85ff <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1df>
   a859c:	mov    $0x1,%r11d
   a85a2:	jmp    a85ff <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1df>
   a85a4:	cmp    %ebp,%r9d
   a85a7:	mov    %ebp,%ecx
   a85a9:	cmovl  %r9d,%ecx
   a85ad:	mov    $0x1,%r10d
   a85b3:	xor    %esi,%esi
   a85b5:	cmp    $0xff,%r14b
   a85b9:	movzbl -0x6(%rsp),%edx
   a85be:	movzbl %r14b,%r12d
   a85c2:	cmove  %edx,%r12d
   a85c6:	mov    %r10d,%edx
   a85c9:	cmp    $0x2,%edx
   a85cc:	jae    a858d <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x16d>
   a85ce:	xchg   %ax,%ax
   a85d0:	movsbq %r12b,%r10
   a85d4:	shl    $0x20,%rcx
   a85d8:	or     $0x10,%rcx
   a85dc:	test   $0x1,%dl
   a85df:	mov    $0x0,%edx
   a85e4:	cmove  %rdx,%rcx
   a85e8:	shl    $0x5,%sil
   a85ec:	movzbl %sil,%r11d
   a85f0:	or     %rcx,%r11
   a85f3:	lea    -0x7515a(%rip),%rcx        # 334a0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x340>
   a85fa:	or     0x8(%rcx,%r10,8),%r11
   a85ff:	mov    %eax,%ecx
   a8601:	and    $0xffff0fc0,%ecx
   a8607:	or     %r11,%rcx
   a860a:	cmp    %rax,%rcx
   a860d:	je     a861a <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1fa>
   a860f:	lock cmpxchg %rcx,(%rdi)
   a8614:	jne    a8470 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x50>
   a861a:	pop    %rbx
   a861b:	pop    %r12
   a861d:	pop    %r13
   a861f:	pop    %r14
   a8621:	pop    %r15
   a8623:	pop    %rbp
   a8624:	ret
