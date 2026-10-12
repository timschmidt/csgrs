
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1850-bound-merge-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a8460 <<hyperreal::computable::node::AtomicFacts>::publish_bound>:
   a8460:	push   %rbp
   a8461:	push   %r15
   a8463:	push   %r14
   a8465:	push   %r13
   a8467:	push   %r12
   a8469:	push   %rbx
   a846a:	mov    (%rdi),%r10
   a846d:	mov    (%rsi),%edx
   a846f:	mov    0x4(%rsi),%r8d
   a8473:	mov    0x8(%rsi),%r9d
   a8477:	xor    %ecx,%ecx
   a8479:	test   $0x1,%dl
   a847c:	cmovne %r9d,%ecx
   a8480:	movzbl 0x9(%rsi),%esi
   a8484:	cmp    $0x2,%edx
   a8487:	jae    a8585 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x125>
   a848d:	mov    %edx,%eax
   a848f:	not    %al
   a8491:	or     %r9b,%al
   a8494:	test   $0x1,%al
   a8496:	je     a864b <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1eb>
   a849c:	lea    -0x75003(%rip),%rbx        # 334a0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x340>
   a84a3:	xor    %r11d,%r11d
   a84a6:	movzbl %dl,%ebp
   a84a9:	movzbl %cl,%ecx
   a84ac:	nopl   0x0(%rax)
   a84b0:	mov    %r10d,%eax
   a84b3:	and    $0x3,%eax
   a84b6:	mov    %edx,%r13d
   a84b9:	mov    %r8d,%r14d
   a84bc:	mov    %r9d,%r12d
   a84bf:	mov    %esi,%r15d
   a84c2:	cmp    $0x2,%rax
   a84c6:	jb     a8533 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0xd3>
   a84c8:	cmp    $0x2,%eax
   a84cb:	je     a8559 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0xf9>
   a84d1:	mov    %r10d,%eax
   a84d4:	shr    $0x2,%eax
   a84d7:	and    $0x3,%eax
   a84da:	lea    -0x73ec9(%rip),%r14        # 34618 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0xff8>
   a84e1:	movslq (%r14,%rax,4),%rax
   a84e5:	add    %r14,%rax
   a84e8:	mov    %esi,%r15d
   a84eb:	jmp    *%rax
   a84ed:	mov    $0x2,%r15b
   a84f0:	jmp    a84fa <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x9a>
   a84f2:	xor    %r15d,%r15d
   a84f5:	jmp    a84fa <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x9a>
   a84f7:	mov    $0x1,%r15b
   a84fa:	mov    %r10,%rax
   a84fd:	shr    $0x20,%rax
   a8501:	test   $0x1,%dl
   a8504:	mov    %eax,%r14d
   a8507:	cmovne %r8d,%r14d
   a850b:	test   $0x20,%r10b
   a850f:	mov    $0x1,%r12d
   a8515:	cmove  %ebp,%r12d
   a8519:	cmovne %eax,%r14d
   a851d:	test   $0x10,%r10b
   a8521:	mov    $0x1,%r13d
   a8527:	cmove  %edx,%r13d
   a852b:	cmove  %r8d,%r14d
   a852f:	cmove  %ecx,%r12d
   a8533:	inc    %r15b
   a8536:	movzbl %r15b,%r15d
   a853a:	shl    $0x20,%r14
   a853e:	or     $0x10,%r14
   a8542:	test   $0x1,%r13b
   a8546:	cmove  %r11,%r14
   a854a:	shl    $0x5,%r12b
   a854e:	movzbl %r12b,%eax
   a8552:	or     %r14,%rax
   a8555:	or     (%rbx,%r15,8),%rax
   a8559:	mov    %r10d,%r14d
   a855c:	and    $0xffff0fc0,%r14d
   a8563:	or     %rax,%r14
   a8566:	cmp    %r10,%r14
   a8569:	je     a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a856f:	mov    %r10,%rax
   a8572:	lock cmpxchg %r14,(%rdi)
   a8577:	mov    %rax,%r10
   a857a:	jne    a84b0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x50>
   a8580:	jmp    a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a8585:	cmp    $0xffffffff,%edx
   a8588:	je     a8737 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x2d7>
   a858e:	lea    -0x750f5(%rip),%r14        # 334a0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x340>
   a8595:	xor    %r11d,%r11d
   a8598:	nopl   0x0(%rax,%rax,1)
   a85a0:	mov    %r10d,%ebx
   a85a3:	and    $0x3,%ebx
   a85a6:	cmp    $0x2,%rbx
   a85aa:	jae    a85c0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x160>
   a85ac:	cmp    $0x2,%edx
   a85af:	je     a85ee <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x18e>
   a85b1:	cmp    $0x3,%edx
   a85b4:	jne    a85f5 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x195>
   a85b6:	mov    $0x2,%ebx
   a85bb:	jmp    a8620 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1c0>
   a85bd:	nopl   (%rax)
   a85c0:	cmp    $0x2,%ebx
   a85c3:	je     a8620 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1c0>
   a85c5:	lea    (%r10,%r10,1),%ecx
   a85c9:	and    $0x18,%cl
   a85cc:	mov    $0x10002ff,%eax
   a85d1:	shr    %cl,%eax
   a85d3:	mov    %r10,%rcx
   a85d6:	shr    $0x20,%rcx
   a85da:	mov    %r10d,%ebx
   a85dd:	shr    $0x5,%bl
   a85e0:	and    $0x1,%bl
   a85e3:	mov    %r10d,%ebp
   a85e6:	and    $0x10,%ebp
   a85e9:	shr    $0x4,%ebp
   a85ec:	jmp    a85ff <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x19f>
   a85ee:	mov    $0x1,%ebx
   a85f3:	jmp    a8620 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x1c0>
   a85f5:	xor    %ebp,%ebp
   a85f7:	mov    %r8d,%ecx
   a85fa:	mov    %r9d,%ebx
   a85fd:	mov    %esi,%eax
   a85ff:	inc    %al
   a8601:	movzbl %al,%eax
   a8604:	shl    $0x20,%rcx
   a8608:	or     $0x10,%rcx
   a860c:	test   %bpl,%bpl
   a860f:	cmove  %r11,%rcx
   a8613:	shl    $0x5,%bl
   a8616:	movzbl %bl,%ebx
   a8619:	or     %rcx,%rbx
   a861c:	or     (%r14,%rax,8),%rbx
   a8620:	mov    %r10d,%ecx
   a8623:	and    $0xffff0fc0,%ecx
   a8629:	or     %rbx,%rcx
   a862c:	cmp    %r10,%rcx
   a862f:	je     a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a8635:	mov    %r10,%rax
   a8638:	lock cmpxchg %rcx,(%rdi)
   a863d:	mov    %rax,%r10
   a8640:	jne    a85a0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x140>
   a8646:	jmp    a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a864b:	lea    -0x751b2(%rip),%r13        # 334a0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x340>
   a8652:	xor    %r11d,%r11d
   a8655:	lea    -0x74054(%rip),%rbx        # 34608 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0xfe8>
   a865c:	nopl   0x0(%rax)
   a8660:	mov    %r10d,%eax
   a8663:	and    $0x3,%eax
   a8666:	mov    %edx,%r15d
   a8669:	mov    %r8d,%r14d
   a866c:	mov    %r9d,%r12d
   a866f:	mov    %esi,%ebp
   a8671:	cmp    $0x2,%rax
   a8675:	jae    a86a0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x240>
   a8677:	movsbq %bpl,%rbp
   a867b:	shl    $0x20,%r14
   a867f:	or     $0x10,%r14
   a8683:	test   $0x1,%r15b
   a8687:	cmove  %r11,%r14
   a868b:	shl    $0x5,%r12b
   a868f:	movzbl %r12b,%eax
   a8693:	or     %r14,%rax
   a8696:	or     0x8(%r13,%rbp,8),%rax
   a869b:	jmp    a86a5 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x245>
   a869d:	nopl   (%rax)
   a86a0:	cmp    $0x2,%eax
   a86a3:	jne    a86d0 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x270>
   a86a5:	mov    %r10d,%r14d
   a86a8:	and    $0xffff0fc0,%r14d
   a86af:	or     %rax,%r14
   a86b2:	cmp    %r10,%r14
   a86b5:	je     a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a86bb:	mov    %r10,%rax
   a86be:	lock cmpxchg %r14,(%rdi)
   a86c3:	mov    %rax,%r10
   a86c6:	jne    a8660 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x200>
   a86c8:	jmp    a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a86cd:	nopl   (%rax)
   a86d0:	mov    %r10d,%eax
   a86d3:	shr    $0x2,%eax
   a86d6:	and    $0x3,%eax
   a86d9:	movslq (%rbx,%rax,4),%rax
   a86dd:	add    %rbx,%rax
   a86e0:	mov    %esi,%ebp
   a86e2:	jmp    *%rax
   a86e4:	mov    $0x2,%bpl
   a86e7:	test   $0x10,%r10b
   a86eb:	je     a86f7 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x297>
   a86ed:	jmp    a870e <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x2ae>
   a86ef:	xor    %ebp,%ebp
   a86f1:	test   $0x10,%r10b
   a86f5:	jne    a870e <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x2ae>
   a86f7:	mov    %edx,%r15d
   a86fa:	mov    %r8d,%r14d
   a86fd:	mov    %ecx,%r12d
   a8700:	jmp    a8677 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x217>
   a8705:	mov    $0x1,%bpl
   a8708:	test   $0x10,%r10b
   a870c:	je     a86f7 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x297>
   a870e:	mov    %r10,%r14
   a8711:	shr    $0x20,%r14
   a8715:	mov    $0x1,%r12b
   a8718:	mov    $0x1,%r15d
   a871e:	test   $0x20,%r10b
   a8722:	jne    a8677 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x217>
   a8728:	cmp    %r8d,%r14d
   a872b:	cmovge %r8d,%r14d
   a872f:	xor    %r12d,%r12d
   a8732:	jmp    a8677 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x217>
   a8737:	movabs $0xffffffff00000000,%rcx
   a8741:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a8750:	mov    %r10d,%eax
   a8753:	and    $0x3,%eax
   a8756:	cmp    $0x2,%rax
   a875a:	jae    a8760 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x300>
   a875c:	xor    %eax,%eax
   a875e:	jmp    a8793 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x333>
   a8760:	cmp    $0x2,%eax
   a8763:	je     a8793 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x333>
   a8765:	mov    %r10d,%eax
   a8768:	and    $0xc,%eax
   a876b:	mov    %r10,%rdx
   a876e:	and    %rcx,%rdx
   a8771:	mov    %r10d,%esi
   a8774:	and    $0x20,%esi
   a8777:	mov    %r10,%r8
   a877a:	shl    $0x3b,%r8
   a877e:	sar    $0x3f,%r8
   a8782:	or     $0x10,%rdx
   a8786:	and    %r8,%rdx
   a8789:	or     %rsi,%rdx
   a878c:	add    %rdx,%rax
   a878f:	add    $0x3,%rax
   a8793:	mov    %r10d,%edx
   a8796:	and    $0xffff0fc0,%edx
   a879c:	or     %rax,%rdx
   a879f:	cmp    %r10,%rdx
   a87a2:	je     a87b1 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x351>
   a87a4:	mov    %r10,%rax
   a87a7:	lock cmpxchg %rdx,(%rdi)
   a87ac:	mov    %rax,%r10
   a87af:	jne    a8750 <<hyperreal::computable::node::AtomicFacts>::publish_bound+0x2f0>
   a87b1:	pop    %rbx
   a87b2:	pop    %r12
   a87b4:	pop    %r13
   a87b6:	pop    %r14
   a87b8:	pop    %r15
   a87ba:	pop    %rbp
   a87bb:	ret
