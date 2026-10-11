
target/typed-api-prototype-v1437-acos-residual-targeted-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

000000000005c160 <probe::main>:
   5c160:	push   %rbp
   5c161:	push   %r15
   5c163:	push   %r14
   5c165:	push   %r13
   5c167:	push   %r12
   5c169:	push   %rbx
   5c16a:	sub    $0x168,%rsp
   5c171:	mov    %rsp,%rbx
   5c174:	mov    %rbx,%rdi
   5c177:	call   *0x12a8db(%rip)        # 186a58 <_DYNAMIC+0x288>
   5c17d:	movq   $0x1,0x20(%rsp)
   5c186:	lea    0x120(%rsp),%rdi
   5c18e:	mov    %rbx,%rsi
   5c191:	call   60250 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::skip::Skip<std::env::Args>>>::from_iter>
   5c196:	mov    0x130(%rsp),%rdi
   5c19e:	cmp    $0x1,%rdi
   5c1a2:	je     5dd1f <probe::main+0x1bbf>
   5c1a8:	test   %rdi,%rdi
   5c1ab:	je     5dd28 <probe::main+0x1bc8>
   5c1b1:	mov    0x128(%rsp),%r9
   5c1b9:	mov    0x20(%r9),%rcx
   5c1bd:	mov    0x28(%r9),%rsi
   5c1c1:	cmp    $0x1,%rsi
   5c1c5:	je     5c1d5 <probe::main+0x75>
   5c1c7:	test   %rsi,%rsi
   5c1ca:	je     5dd3a <probe::main+0x1bda>
   5c1d0:	movzbl (%rcx),%edx
   5c1d3:	jmp    5c1ec <probe::main+0x8c>
   5c1d5:	movzbl (%rcx),%edx
   5c1d8:	mov    $0x1,%al
   5c1da:	cmp    $0x2b,%edx
   5c1dd:	je     5db3b <probe::main+0x19db>
   5c1e3:	cmp    $0x2d,%edx
   5c1e6:	je     5db3b <probe::main+0x19db>
   5c1ec:	xor    %eax,%eax
   5c1ee:	cmp    $0x2b,%dl
   5c1f1:	sete   %al
   5c1f4:	mov    %rsi,%rdx
   5c1f7:	sub    %rax,%rdx
   5c1fa:	add    %rax,%rcx
   5c1fd:	neg    %rax
   5c200:	cmp    $0x11,%rdx
   5c204:	jae    5c247 <probe::main+0xe7>
   5c206:	test   %rdx,%rdx
   5c209:	je     5c299 <probe::main+0x139>
   5c20f:	add    %rax,%rsi
   5c212:	neg    %rsi
   5c215:	xor    %r15d,%r15d
   5c218:	xor    %eax,%eax
   5c21a:	nopw   0x0(%rax,%rax,1)
   5c220:	movzbl (%rcx,%rax,1),%edx
   5c224:	add    $0xffffffd0,%edx
   5c227:	cmp    $0x9,%edx
   5c22a:	ja     5dcc7 <probe::main+0x1b67>
   5c230:	lea    (%r15,%r15,4),%rdi
   5c234:	mov    %edx,%edx
   5c236:	lea    (%rdx,%rdi,2),%r15
   5c23a:	inc    %rax
   5c23d:	mov    %rsi,%rdx
   5c240:	add    %rax,%rdx
   5c243:	jne    5c220 <probe::main+0xc0>
   5c245:	jmp    5c29c <probe::main+0x13c>
   5c247:	add    %rax,%rsi
   5c24a:	neg    %rsi
   5c24d:	xor    %r15d,%r15d
   5c250:	mov    $0xa,%edi
   5c255:	xor    %r8d,%r8d
   5c258:	nopl   0x0(%rax,%rax,1)
   5c260:	mov    %rsi,%rax
   5c263:	add    %r8,%rax
   5c266:	je     5c29c <probe::main+0x13c>
   5c268:	mov    %r15,%rax
   5c26b:	mul    %rdi
   5c26e:	movzbl (%rcx,%r8,1),%edx
   5c273:	jo     5db31 <probe::main+0x19d1>
   5c279:	add    $0xffffffd0,%edx
   5c27c:	cmp    $0xa,%edx
   5c27f:	jae    5dcc7 <probe::main+0x1b67>
   5c285:	mov    %rax,%r15
   5c288:	mov    %edx,%eax
   5c28a:	inc    %r8
   5c28d:	add    %rax,%r15
   5c290:	jae    5c260 <probe::main+0x100>
   5c292:	mov    $0x2,%al
   5c294:	jmp    5db3b <probe::main+0x19db>
   5c299:	xor    %r15d,%r15d
   5c29c:	mov    0x10(%r9),%rcx
   5c2a0:	add    $0xfffffffffffffff4,%rcx
   5c2a4:	cmp    $0x3,%rcx
   5c2a8:	ja     5c3da <probe::main+0x27a>
   5c2ae:	mov    0x8(%r9),%rax
   5c2b2:	lea    -0x2c779(%rip),%rdx        # 2fb40 <GCC_except_table0+0x34>
   5c2b9:	movslq (%rdx,%rcx,4),%rcx
   5c2bd:	add    %rdx,%rcx
   5c2c0:	jmp    *%rcx
   5c2c2:	movabs $0x69736f705f64696d,%rcx
   5c2cc:	xor    (%rax),%rcx
   5c2cf:	mov    0x8(%rax),%edx
   5c2d2:	xor    $0x65766974,%rdx
   5c2d9:	mov    $0x1001,%esi
   5c2de:	mov    %rsi,0xc0(%rsp)
   5c2e6:	or     %rcx,%rdx
   5c2e9:	je     5c452 <probe::main+0x2f2>
   5c2ef:	movabs $0x6167656e5f64696d,%rcx
   5c2f9:	xor    (%rax),%rcx
   5c2fc:	mov    0x8(%rax),%eax
   5c2ff:	xor    $0x65766974,%rax
   5c305:	or     %rcx,%rax
   5c308:	je     5c442 <probe::main+0x2e2>
   5c30e:	jmp    5c3da <probe::main+0x27a>
   5c313:	movabs $0x6f705f6c6c616d73,%rdx
   5c31d:	xor    (%rax),%rdx
   5c320:	movabs $0x657669746167656e,%rcx
   5c32a:	lea    0x80c0a02(%rcx),%rsi
   5c331:	xor    0x6(%rax),%rsi
   5c335:	movl   $0xffffffc0,0x70(%rsp)
   5c33d:	mov    $0x8,%edi
   5c342:	mov    %rdi,0xc0(%rsp)
   5c34a:	or     %rdx,%rsi
   5c34d:	je     5c45a <probe::main+0x2fa>
   5c353:	movabs $0x656e5f6c6c616d73,%rdx
   5c35d:	xor    (%rax),%rdx
   5c360:	xor    0x6(%rax),%rcx
   5c364:	or     %rdx,%rcx
   5c367:	jne    5c3da <probe::main+0x27a>
   5c369:	mov    $0x1,%al
   5c36b:	mov    %eax,0x4c(%rsp)
   5c36f:	jmp    5c462 <probe::main+0x302>
   5c374:	movabs $0x705f646568636163,%rdx
   5c37e:	xor    (%rax),%rdx
   5c381:	movabs $0x657669746167656e,%rcx
   5c38b:	lea    0x80c0a02(%rcx),%rsi
   5c392:	xor    0x7(%rax),%rsi
   5c396:	movl   $0x0,0x70(%rsp)
   5c39e:	mov    $0xf4241,%edi
   5c3a3:	mov    %rdi,0xc0(%rsp)
   5c3ab:	or     %rdx,%rsi
   5c3ae:	je     5c8a1 <probe::main+0x741>
   5c3b4:	movabs $0x6e5f646568636163,%rdx
   5c3be:	xor    (%rax),%rdx
   5c3c1:	xor    0x7(%rax),%rcx
   5c3c5:	mov    $0x1,%al
   5c3c7:	mov    %eax,0x4c(%rsp)
   5c3cb:	or     %rdx,%rcx
   5c3ce:	mov    $0x1,%al
   5c3d0:	mov    %eax,0x48(%rsp)
   5c3d4:	je     5c46a <probe::main+0x30a>
   5c3da:	lea    -0x296f1(%rip),%rdi        # 32cf0 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.16730574089061123067+0xf0>
   5c3e1:	lea    0x125e08(%rip),%rdx        # 1821f0 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x60>
   5c3e8:	mov    $0x21,%esi
   5c3ed:	call   *0x12a66d(%rip)        # 186a60 <_DYNAMIC+0x290>
   5c3f3:	jmp    5dd92 <probe::main+0x1c32>
   5c3f8:	movabs $0x657669746167656e,%rcx
   5c402:	movabs $0x736f705f65646977,%rdx
   5c40c:	xor    (%rax),%rdx
   5c40f:	lea    0x80c0a02(%rcx),%rsi
   5c416:	xor    0x5(%rax),%rsi
   5c41a:	mov    $0xf4241,%edi
   5c41f:	mov    %rdi,0xc0(%rsp)
   5c427:	or     %rdx,%rsi
   5c42a:	je     5c452 <probe::main+0x2f2>
   5c42c:	movabs $0x67656e5f65646977,%rdx
   5c436:	xor    (%rax),%rdx
   5c439:	xor    0x5(%rax),%rcx
   5c43d:	or     %rdx,%rcx
   5c440:	jne    5c3da <probe::main+0x27a>
   5c442:	mov    $0x1,%al
   5c444:	mov    %eax,0x4c(%rsp)
   5c448:	movl   $0x0,0x70(%rsp)
   5c450:	jmp    5c462 <probe::main+0x302>
   5c452:	movl   $0x0,0x70(%rsp)
   5c45a:	movl   $0x0,0x4c(%rsp)
   5c462:	movl   $0x0,0x48(%rsp)
   5c46a:	mov    %r9,0x118(%rsp)
   5c472:	call   *0x12a5f0(%rip)        # 186a68 <_DYNAMIC+0x298>
   5c478:	mov    %rax,0x50(%rsp)
   5c47d:	mov    %rdx,0x58(%rsp)
   5c482:	mov    %rsp,%rdi
   5c485:	lea    0x50(%rsp),%rsi
   5c48a:	mov    $0xffffffa0,%edx
   5c48f:	call   *0x12a59b(%rip)        # 186a30 <_DYNAMIC+0x260>
   5c495:	mov    (%rsp),%rsi
   5c499:	cmp    $0xffffffffffffffff,%rsi
   5c49d:	je     5db66 <probe::main+0x1a06>
   5c4a3:	test   %rsi,%rsi
   5c4a6:	je     5c4bc <probe::main+0x35c>
   5c4a8:	mov    0x8(%rsp),%rdi
   5c4ad:	shl    $0x3,%rsi
   5c4b1:	mov    $0x8,%edx
   5c4b6:	call   *0x12a554(%rip)        # 186a10 <_DYNAMIC+0x240>
   5c4bc:	lea    0x58(%rsp),%rbx
   5c4c1:	mov    0x50(%rsp),%rax
   5c4c6:	lock decq (%rax)
   5c4ca:	jne    5c4d7 <probe::main+0x377>
   5c4cc:	lea    0x50(%rsp),%rdi
   5c4d1:	call   *0x12a549(%rip)        # 186a20 <_DYNAMIC+0x250>
   5c4d7:	mov    0x58(%rsp),%rax
   5c4dc:	test   %rax,%rax
   5c4df:	je     5c4f0 <probe::main+0x390>
   5c4e1:	lock decq (%rax)
   5c4e5:	jne    5c4f0 <probe::main+0x390>
   5c4e7:	mov    %rbx,%rdi
   5c4ea:	call   *0x12a538(%rip)        # 186a28 <_DYNAMIC+0x258>
   5c4f0:	call   *0x12a57a(%rip)        # 186a70 <_DYNAMIC+0x2a0>
   5c4f6:	mov    %rax,0x30(%rsp)
   5c4fb:	mov    %rdx,0x38(%rsp)
   5c500:	mov    %rsp,%rdi
   5c503:	lea    0x30(%rsp),%rsi
   5c508:	xor    %edx,%edx
   5c50a:	call   *0x12a520(%rip)        # 186a30 <_DYNAMIC+0x260>
   5c510:	mov    (%rsp),%rax
   5c514:	cmp    $0xffffffffffffffff,%rax
   5c518:	je     5db99 <probe::main+0x1a39>
   5c51e:	mov    0x8(%rsp),%rcx
   5c523:	mov    0x10(%rsp),%rsi
   5c528:	movzbl 0x18(%rsp),%ebx
   5c52d:	test   %rsi,%rsi
   5c530:	je     5c587 <probe::main+0x427>
   5c532:	mov    0xc0(%rsp),%rdx
   5c53a:	dec    %rdx
   5c53d:	mov    %edx,%edi
   5c53f:	and    $0x3f,%dil
   5c543:	shr    $0x6,%rdx
   5c547:	mov    %rax,(%rsp)
   5c54b:	mov    %rcx,0x8(%rsp)
   5c550:	mov    %rsi,0x10(%rsp)
   5c555:	movzbl %dil,%ecx
   5c559:	lea    0x50(%rsp),%rdi
   5c55e:	mov    %rsp,%rsi
   5c561:	call   *0x12a511(%rip)        # 186a78 <_DYNAMIC+0x2a8>
   5c567:	cmp    $0x1,%bl
   5c56a:	jne    5c59f <probe::main+0x43f>
   5c56c:	mov    0x50(%rsp),%rsi
   5c571:	mov    $0x1,%bpl
   5c574:	cmp    $0x4,%rsi
   5c578:	jae    5c5c1 <probe::main+0x461>
   5c57a:	mov    %rsi,0xf0(%rsp)
   5c582:	xor    %r14d,%r14d
   5c585:	jmp    5c5ed <probe::main+0x48d>
   5c587:	mov    %rax,0x50(%rsp)
   5c58c:	mov    %rcx,0x58(%rsp)
   5c591:	movq   $0x0,0x60(%rsp)
   5c59a:	cmp    $0x1,%bl
   5c59d:	je     5c56c <probe::main+0x40c>
   5c59f:	mov    0x50(%rsp),%rax
   5c5a4:	mov    %rax,0xf0(%rsp)
   5c5ac:	mov    0x60(%rsp),%r14
   5c5b1:	test   %r14,%r14
   5c5b4:	movzbl %bl,%eax
   5c5b7:	mov    $0x1,%ebp
   5c5bc:	cmovne %eax,%ebp
   5c5bf:	jmp    5c5ed <probe::main+0x48d>
   5c5c1:	mov    0x58(%rsp),%rdi
   5c5c6:	shl    $0x3,%rsi
   5c5ca:	mov    $0x8,%edx
   5c5cf:	call   *0x12a43b(%rip)        # 186a10 <_DYNAMIC+0x240>
   5c5d5:	movq   $0x8,0x58(%rsp)
   5c5de:	xor    %r14d,%r14d
   5c5e1:	movq   $0x0,0xf0(%rsp)
   5c5ed:	lea    0x38(%rsp),%rbx
   5c5f2:	mov    0x58(%rsp),%rax
   5c5f7:	mov    %rax,0xf8(%rsp)
   5c5ff:	mov    0x30(%rsp),%rax
   5c604:	lock decq (%rax)
   5c608:	jne    5c615 <probe::main+0x4b5>
   5c60a:	lea    0x30(%rsp),%rdi
   5c60f:	call   *0x12a40b(%rip)        # 186a20 <_DYNAMIC+0x250>
   5c615:	mov    0x38(%rsp),%rax
   5c61a:	test   %rax,%rax
   5c61d:	je     5c62e <probe::main+0x4ce>
   5c61f:	lock decq (%rax)
   5c623:	jne    5c62e <probe::main+0x4ce>
   5c625:	mov    %rbx,%rdi
   5c628:	call   *0x12a3fa(%rip)        # 186a28 <_DYNAMIC+0x258>
   5c62e:	lea    0x0(,%r14,8),%r13
   5c636:	test   %r14,%r14
   5c639:	je     5c671 <probe::main+0x511>
   5c63b:	call   *0x12a43f(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5c641:	mov    $0x8,%esi
   5c646:	mov    %r13,%rdi
   5c649:	call   *0x12a439(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5c64f:	test   %rax,%rax
   5c652:	je     5dd41 <probe::main+0x1be1>
   5c658:	mov    %rax,%r12
   5c65b:	mov    %rax,%rdi
   5c65e:	mov    0xf8(%rsp),%rsi
   5c666:	mov    %r13,%rdx
   5c669:	call   *0x12a421(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5c66f:	jmp    5c677 <probe::main+0x517>
   5c671:	mov    $0x8,%r12d
   5c677:	movzbl %bpl,%eax
   5c67b:	test   %al,%al
   5c67d:	mov    %ebp,0x98(%rsp)
   5c684:	mov    %eax,0xa8(%rsp)
   5c68b:	je     5c717 <probe::main+0x5b7>
   5c691:	lea    0x8(%rsp),%rbx
   5c696:	cmp    $0x1,%eax
   5c699:	jne    5c740 <probe::main+0x5e0>
   5c69f:	mov    %r13,0xa0(%rsp)
   5c6a7:	movups -0x299ae(%rip),%xmm0        # 32d00 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.16730574089061123067+0x100>
   5c6ae:	movaps %xmm0,(%rsp)
   5c6b2:	movq   $0x0,0x10(%rsp)
   5c6bb:	mov    %rsp,%rdi
   5c6be:	call   *0x12a3d4(%rip)        # 186a98 <_DYNAMIC+0x2c8>
   5c6c4:	mov    0x8(%rsp),%rax
   5c6c9:	movq   $0x1,(%rax)
   5c6d0:	mov    (%rsp),%r13
   5c6d4:	mov    0x8(%rsp),%rbx
   5c6d9:	mov    $0x1,%ebp
   5c6de:	movl   $0x0,0x88(%rsp)
   5c6e9:	test   %r14,%r14
   5c6ec:	je     5c704 <probe::main+0x5a4>
   5c6ee:	mov    $0x8,%edx
   5c6f3:	mov    %r12,%rdi
   5c6f6:	lea    0x0(,%r14,8),%rsi
   5c6fe:	call   *0x12a30c(%rip)        # 186a10 <_DYNAMIC+0x240>
   5c704:	mov    %rbx,%r12
   5c707:	mov    %r13,%rbx
   5c70a:	mov    0x88(%rsp),%r13d
   5c712:	jmp    5cb00 <probe::main+0x9a0>
   5c717:	mov    %r14,(%rsp)
   5c71b:	mov    %r12,0x8(%rsp)
   5c720:	mov    %r14,0x10(%rsp)
   5c725:	mov    %r14,%rax
   5c728:	test   %r14,%r14
   5c72b:	je     5c829 <probe::main+0x6c9>
   5c731:	incq   (%r12)
   5c735:	je     5c857 <probe::main+0x6f7>
   5c73b:	jmp    5c886 <probe::main+0x726>
   5c740:	movups -0x29a47(%rip),%xmm0        # 32d00 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.16730574089061123067+0x100>
   5c747:	movaps %xmm0,(%rsp)
   5c74b:	movq   $0x0,0x10(%rsp)
   5c754:	mov    %rsp,%rdi
   5c757:	call   *0x12a33b(%rip)        # 186a98 <_DYNAMIC+0x2c8>
   5c75d:	cmp    $0x2,%r14
   5c761:	setae  %al
   5c764:	cmp    $0x1,%r14
   5c768:	sbb    $0x0,%al
   5c76a:	mov    0x8(%rsp),%rcx
   5c76f:	movq   $0x1,(%rcx)
   5c776:	mov    (%rsp),%rsi
   5c77a:	mov    0x8(%rsp),%rdi
   5c77f:	cmp    $0x1,%r14
   5c783:	jne    5c793 <probe::main+0x633>
   5c785:	mov    (%rdi),%rax
   5c788:	cmp    %rax,(%r12)
   5c78c:	je     5c7ee <probe::main+0x68e>
   5c78e:	seta   %al
   5c791:	sbb    $0x0,%al
   5c793:	test   %al,%al
   5c795:	je     5c7ee <probe::main+0x68e>
   5c797:	movzbl %al,%eax
   5c79a:	cmp    $0x1,%eax
   5c79d:	mov    %rsi,0x80(%rsp)
   5c7a5:	mov    %rdi,0xb8(%rsp)
   5c7ad:	jne    5c926 <probe::main+0x7c6>
   5c7b3:	mov    %r14,(%rsp)
   5c7b7:	mov    %r12,0x8(%rsp)
   5c7bc:	mov    %r14,0x10(%rsp)
   5c7c1:	mov    %rsp,%rdi
   5c7c4:	mov    $0x1,%esi
   5c7c9:	call   *0x12a2d1(%rip)        # 186aa0 <_DYNAMIC+0x2d0>
   5c7cf:	mov    (%rsp),%rbx
   5c7d3:	mov    0x8(%rsp),%r12
   5c7d8:	mov    0x10(%rsp),%rbp
   5c7dd:	mov    $0x2,%r13b
   5c7e0:	test   %rbp,%rbp
   5c7e3:	jne    5cadc <probe::main+0x97c>
   5c7e9:	jmp    5ca75 <probe::main+0x915>
   5c7ee:	movl   $0x0,0x7b(%rsp)
   5c7f6:	movl   $0x0,0x78(%rsp)
   5c7fe:	mov    $0x1,%al
   5c800:	mov    %eax,0x88(%rsp)
   5c807:	test   %rsi,%rsi
   5c80a:	je     5c972 <probe::main+0x812>
   5c810:	shl    $0x3,%rsi
   5c814:	mov    $0x8,%ebx
   5c819:	mov    $0x8,%edx
   5c81e:	call   *0x12a1ec(%rip)        # 186a10 <_DYNAMIC+0x240>
   5c824:	jmp    5c977 <probe::main+0x817>
   5c829:	mov    %rsp,%rdi
   5c82c:	call   *0x12a266(%rip)        # 186a98 <_DYNAMIC+0x2c8>
   5c832:	mov    0x8(%rsp),%rax
   5c837:	movq   $0x0,(%rax)
   5c83e:	movq   $0x1,0x10(%rsp)
   5c847:	mov    $0x1,%eax
   5c84c:	mov    0x8(%rsp),%r12
   5c851:	incq   (%r12)
   5c855:	jne    5c886 <probe::main+0x726>
   5c857:	shl    $0x3,%rax
   5c85b:	mov    $0x1,%bl
   5c85d:	mov    $0x8,%ecx
   5c862:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5c870:	cmp    %rcx,%rax
   5c873:	je     5c8b4 <probe::main+0x754>
   5c875:	add    $0xff,%bl
   5c878:	adcq   $0x0,(%r12,%rcx,1)
   5c87d:	lea    0x8(%rcx),%rcx
   5c881:	setb   %bl
   5c884:	jb     5c870 <probe::main+0x710>
   5c886:	mov    0x10(%rsp),%rbp
   5c88b:	mov    (%rsp),%rbx
   5c88f:	mov    0x8(%rsp),%r12
   5c894:	test   %rbp,%rbp
   5c897:	je     5c8eb <probe::main+0x78b>
   5c899:	xor    %r13d,%r13d
   5c89c:	jmp    5cb00 <probe::main+0x9a0>
   5c8a1:	mov    $0x1,%al
   5c8a3:	mov    %eax,0x48(%rsp)
   5c8a7:	movl   $0x0,0x4c(%rsp)
   5c8af:	jmp    5c46a <probe::main+0x30a>
   5c8b4:	mov    0x10(%rsp),%rax
   5c8b9:	mov    %rax,%r12
   5c8bc:	cmp    (%rsp),%rax
   5c8c0:	jne    5c8cb <probe::main+0x76b>
   5c8c2:	mov    %rsp,%rdi
   5c8c5:	call   *0x12a1cd(%rip)        # 186a98 <_DYNAMIC+0x2c8>
   5c8cb:	movzbl %bl,%eax
   5c8ce:	mov    0x8(%rsp),%rcx
   5c8d3:	mov    %r12,%rbp
   5c8d6:	mov    %rax,(%rcx,%r12,8)
   5c8da:	inc    %rbp
   5c8dd:	mov    (%rsp),%rbx
   5c8e1:	mov    0x8(%rsp),%r12
   5c8e6:	test   %rbp,%rbp
   5c8e9:	jne    5c899 <probe::main+0x739>
   5c8eb:	movl   $0x0,0x7b(%rsp)
   5c8f3:	movl   $0x0,0x78(%rsp)
   5c8fb:	mov    $0x1,%r13b
   5c8fe:	test   %rbx,%rbx
   5c901:	je     5c98a <probe::main+0x82a>
   5c907:	shl    $0x3,%rbx
   5c90b:	mov    $0x8,%edx
   5c910:	mov    %r12,%rdi
   5c913:	mov    %rbx,%rsi
   5c916:	call   *0x12a0f4(%rip)        # 186a10 <_DYNAMIC+0x240>
   5c91c:	xor    %ebx,%ebx
   5c91e:	mov    $0x8,%r12d
   5c924:	jmp    5c992 <probe::main+0x832>
   5c926:	mov    %r14,(%rsp)
   5c92a:	mov    %r12,0x8(%rsp)
   5c92f:	mov    %r14,0x10(%rsp)
   5c934:	test   %r14,%r14
   5c937:	je     5c999 <probe::main+0x839>
   5c939:	movq   $0x1,0x50(%rsp)
   5c942:	lea    0x50(%rsp),%rdi
   5c947:	mov    $0x1,%esi
   5c94c:	mov    %r12,%rdx
   5c94f:	mov    %r14,%rcx
   5c952:	call   *0x12a150(%rip)        # 186aa8 <_DYNAMIC+0x2d8>
   5c958:	mov    0x10(%rsp),%rbp
   5c95d:	mov    (%rsp),%rbx
   5c961:	mov    0x8(%rsp),%r12
   5c966:	test   %rbp,%rbp
   5c969:	jne    5c9c5 <probe::main+0x865>
   5c96b:	xor    %ebp,%ebp
   5c96d:	jmp    5ca15 <probe::main+0x8b5>
   5c972:	mov    $0x8,%ebx
   5c977:	xor    %r13d,%r13d
   5c97a:	xor    %ebp,%ebp
   5c97c:	test   %r14,%r14
   5c97f:	jne    5c6ee <probe::main+0x58e>
   5c985:	jmp    5c704 <probe::main+0x5a4>
   5c98a:	mov    $0x8,%r12d
   5c990:	xor    %ebx,%ebx
   5c992:	xor    %ebp,%ebp
   5c994:	jmp    5cb00 <probe::main+0x9a0>
   5c999:	mov    %rsp,%rdi
   5c99c:	call   *0x12a0f6(%rip)        # 186a98 <_DYNAMIC+0x2c8>
   5c9a2:	mov    0x8(%rsp),%rax
   5c9a7:	movq   $0x1,(%rax)
   5c9ae:	movq   $0x1,0x10(%rsp)
   5c9b7:	mov    (%rsp),%rbx
   5c9bb:	mov    0x8(%rsp),%r12
   5c9c0:	mov    $0x1,%ebp
   5c9c5:	cmpq   $0x0,-0x8(%r12,%rbp,8)
   5c9cb:	jne    5ca15 <probe::main+0x8b5>
   5c9cd:	movabs $0x1fffffffffffffff,%rax
   5c9d7:	and    %rbp,%rax
   5c9da:	lea    0x0(,%rbp,8),%rdx
   5c9e2:	xor    %ecx,%ecx
   5c9e4:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5c9f0:	test   %rdx,%rdx
   5c9f3:	je     5c96b <probe::main+0x80b>
   5c9f9:	inc    %rcx
   5c9fc:	cmpq   $0x0,-0x8(%r12,%rdx,1)
   5ca02:	lea    -0x8(%rdx),%rdx
   5ca06:	je     5c9f0 <probe::main+0x890>
   5ca08:	sub    %rcx,%rax
   5ca0b:	inc    %rax
   5ca0e:	cmp    %rbp,%rax
   5ca11:	cmovb  %rax,%rbp
   5ca15:	mov    %rbx,%rax
   5ca18:	shr    $0x2,%rax
   5ca1c:	cmp    %rax,%rbp
   5ca1f:	jae    5ca6b <probe::main+0x90b>
   5ca21:	cmp    %rbx,%rbp
   5ca24:	ja     5dcce <probe::main+0x1b6e>
   5ca2a:	test   %rbx,%rbx
   5ca2d:	je     5ca6b <probe::main+0x90b>
   5ca2f:	shl    $0x3,%rbx
   5ca33:	test   %rbp,%rbp
   5ca36:	je     5caae <probe::main+0x94e>
   5ca38:	mov    %r12,%r13
   5ca3b:	lea    0x0(,%rbp,8),%r12
   5ca43:	mov    $0x8,%edx
   5ca48:	mov    %r13,%rdi
   5ca4b:	mov    %rbx,%rsi
   5ca4e:	mov    %r12,%rcx
   5ca51:	call   *0x12a059(%rip)        # 186ab0 <_DYNAMIC+0x2e0>
   5ca57:	test   %rax,%rax
   5ca5a:	je     5dd81 <probe::main+0x1c21>
   5ca60:	xor    %r13d,%r13d
   5ca63:	mov    %rbp,%rbx
   5ca66:	mov    %rax,%r12
   5ca69:	jmp    5cadc <probe::main+0x97c>
   5ca6b:	test   %rbp,%rbp
   5ca6e:	je     5ca75 <probe::main+0x915>
   5ca70:	xor    %r13d,%r13d
   5ca73:	jmp    5cadc <probe::main+0x97c>
   5ca75:	movl   $0x0,0x7b(%rsp)
   5ca7d:	movl   $0x0,0x78(%rsp)
   5ca85:	mov    $0x1,%r13b
   5ca88:	test   %rbx,%rbx
   5ca8b:	je     5caa4 <probe::main+0x944>
   5ca8d:	shl    $0x3,%rbx
   5ca91:	mov    $0x8,%edx
   5ca96:	mov    %r12,%rdi
   5ca99:	mov    %rbx,%rsi
   5ca9c:	call   *0x129f6e(%rip)        # 186a10 <_DYNAMIC+0x240>
   5caa2:	jmp    5cad2 <probe::main+0x972>
   5caa4:	mov    $0x8,%r12d
   5caaa:	xor    %ebx,%ebx
   5caac:	jmp    5cada <probe::main+0x97a>
   5caae:	mov    $0x8,%edx
   5cab3:	mov    %r12,%rdi
   5cab6:	mov    %rbx,%rsi
   5cab9:	call   *0x129f51(%rip)        # 186a10 <_DYNAMIC+0x240>
   5cabf:	movl   $0x0,0x78(%rsp)
   5cac7:	movl   $0x0,0x7b(%rsp)
   5cacf:	mov    $0x1,%r13b
   5cad2:	xor    %ebx,%ebx
   5cad4:	mov    $0x8,%r12d
   5cada:	xor    %ebp,%ebp
   5cadc:	mov    0x80(%rsp),%rsi
   5cae4:	test   %rsi,%rsi
   5cae7:	je     5cb00 <probe::main+0x9a0>
   5cae9:	shl    $0x3,%rsi
   5caed:	mov    $0x8,%edx
   5caf2:	mov    0xb8(%rsp),%rdi
   5cafa:	call   *0x129f10(%rip)        # 186a10 <_DYNAMIC+0x240>
   5cb00:	mov    $0x2,%al
   5cb02:	sub    %r13b,%al
   5cb05:	movzbl %al,%eax
   5cb08:	movzbl %r13b,%ecx
   5cb0c:	cmpb   $0x0,0x4c(%rsp)
   5cb11:	cmovne %eax,%ecx
   5cb14:	mov    %rbx,(%rsp)
   5cb18:	mov    %r12,0x88(%rsp)
   5cb20:	mov    %r12,0x8(%rsp)
   5cb25:	mov    %rbp,0x10(%rsp)
   5cb2a:	mov    %cl,0x18(%rsp)
   5cb2e:	mov    0x78(%rsp),%eax
   5cb32:	mov    0x7b(%rsp),%ecx
   5cb36:	mov    %eax,0x19(%rsp)
   5cb3a:	mov    %ecx,0x1c(%rsp)
   5cb3e:	mov    $0x8,%r12d
   5cb44:	cmpb   $0x1,0x98(%rsp)
   5cb4c:	je     5cba2 <probe::main+0xa42>
   5cb4e:	mov    %rbx,%r13
   5cb51:	cmpl   $0x2,0xa8(%rsp)
   5cb59:	jne    5dca0 <probe::main+0x1b40>
   5cb5f:	test   %r14,%r14
   5cb62:	je     5cba2 <probe::main+0xa42>
   5cb64:	call   *0x129f16(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5cb6a:	mov    $0x8,%esi
   5cb6f:	lea    0x0(,%r14,8),%rbx
   5cb77:	mov    %rbx,%rdi
   5cb7a:	call   *0x129f08(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5cb80:	test   %rax,%rax
   5cb83:	je     5dd51 <probe::main+0x1bf1>
   5cb89:	mov    %rax,%r12
   5cb8c:	mov    %rax,%rdi
   5cb8f:	mov    0xf8(%rsp),%rsi
   5cb97:	mov    %rbx,%rdx
   5cb9a:	call   *0x129ef0(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5cba0:	jmp    5cba5 <probe::main+0xa45>
   5cba2:	xor    %r14d,%r14d
   5cba5:	mov    %r14,0x50(%rsp)
   5cbaa:	mov    %r12,0x58(%rsp)
   5cbaf:	mov    %r14,0x60(%rsp)
   5cbb4:	lea    0x30(%rsp),%rdi
   5cbb9:	mov    %rsp,%rsi
   5cbbc:	lea    0x50(%rsp),%rdx
   5cbc1:	call   *0x129ef1(%rip)        # 186ab8 <_DYNAMIC+0x2e8>
   5cbc7:	cmpb   $0x1,0x30(%rsp)
   5cbcc:	je     5dbcc <probe::main+0x1a6c>
   5cbd2:	mov    0x38(%rsp),%rax
   5cbd7:	mov    %rax,0xb0(%rsp)
   5cbdf:	lea    0xb0(%rsp),%rcx
   5cbe7:	mov    %rcx,0x148(%rsp)
   5cbef:	lock incq (%rax)
   5cbf3:	jle    5dd92 <probe::main+0x1c32>
   5cbf9:	mov    0xb0(%rsp),%rdi
   5cc01:	call   *0x129e31(%rip)        # 186a38 <_DYNAMIC+0x268>
   5cc07:	mov    %rax,%rdi
   5cc0a:	mov    %rdx,%rsi
   5cc0d:	call   *0x129e2d(%rip)        # 186a40 <_DYNAMIC+0x270>
   5cc13:	mov    %rax,0xe0(%rsp)
   5cc1b:	mov    %rdx,0xe8(%rsp)
   5cc23:	mov    %rsp,%rdi
   5cc26:	lea    0xe0(%rsp),%rsi
   5cc2e:	mov    0x70(%rsp),%edx
   5cc32:	call   *0x129df8(%rip)        # 186a30 <_DYNAMIC+0x260>
   5cc38:	cmpq   $0xffffffffffffffff,(%rsp)
   5cc3d:	je     5dbfc <probe::main+0x1a9c>
   5cc43:	movups (%rsp),%xmm0
   5cc47:	movups 0x10(%rsp),%xmm1
   5cc4c:	movaps %xmm1,0x60(%rsp)
   5cc51:	movaps %xmm0,0x50(%rsp)
   5cc56:	movq   $0x0,0x30(%rsp)
   5cc5f:	movq   $0x1,0x38(%rsp)
   5cc68:	movq   $0x0,0x40(%rsp)
   5cc71:	movq   $0x60000020,0x10(%rsp)
   5cc7a:	lea    0x30(%rsp),%rax
   5cc7f:	mov    %rax,(%rsp)
   5cc83:	lea    0x1256ce(%rip),%rax        # 182358 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x78>
   5cc8a:	mov    %rax,0x8(%rsp)
   5cc8f:	lea    0x50(%rsp),%rdi
   5cc94:	mov    %rsp,%rsi
   5cc97:	call   *0x129e23(%rip)        # 186ac0 <_DYNAMIC+0x2f0>
   5cc9d:	test   %al,%al
   5cc9f:	jne    5dc2f <probe::main+0x1acf>
   5cca5:	mov    0x30(%rsp),%rax
   5ccaa:	mov    %rax,0xb8(%rsp)
   5ccb2:	mov    0x38(%rsp),%rax
   5ccb7:	mov    %rax,0x80(%rsp)
   5ccbf:	mov    0x40(%rsp),%rbx
   5ccc4:	mov    0x50(%rsp),%rsi
   5ccc9:	test   %rsi,%rsi
   5cccc:	je     5cce2 <probe::main+0xb82>
   5ccce:	mov    0x58(%rsp),%rdi
   5ccd3:	shl    $0x3,%rsi
   5ccd7:	mov    $0x8,%edx
   5ccdc:	call   *0x129d2e(%rip)        # 186a10 <_DYNAMIC+0x240>
   5cce2:	test   %r15,%r15
   5cce5:	je     5ce2b <probe::main+0xccb>
   5cceb:	test   %rbx,%rbx
   5ccee:	je     5ce30 <probe::main+0xcd0>
   5ccf4:	mov    0x80(%rsp),%rax
   5ccfc:	add    %rbx,%rax
   5ccff:	mov    %ebx,%ecx
   5cd01:	and    $0x7,%ecx
   5cd04:	xor    %r12d,%r12d
   5cd07:	xor    %edx,%edx
   5cd09:	jmp    5cd2d <probe::main+0xbcd>
   5cd0b:	nopl   0x0(%rax,%rax,1)
   5cd10:	inc    %rdx
   5cd13:	mov    %rdi,%r12
   5cd16:	shl    $0x5,%r12
   5cd1a:	sub    %rdi,%r12
   5cd1d:	add    $0xff,%r12
   5cd24:	cmp    %r15,%rdx
   5cd27:	je     5ce96 <probe::main+0xd36>
   5cd2d:	test   %rcx,%rcx
   5cd30:	je     5cd70 <probe::main+0xc10>
   5cd32:	mov    %rcx,%r8
   5cd35:	mov    0x80(%rsp),%rsi
   5cd3d:	mov    %r12,%rdi
   5cd40:	movzbl (%rsi),%r9d
   5cd44:	inc    %rsi
   5cd47:	shl    $0x5,%rdi
   5cd4b:	sub    %r12,%rdi
   5cd4e:	add    %r9,%rdi
   5cd51:	mov    %rdi,%r12
   5cd54:	dec    %r8
   5cd57:	jne    5cd40 <probe::main+0xbe0>
   5cd59:	cmp    $0x8,%rbx
   5cd5d:	jae    5cd90 <probe::main+0xc30>
   5cd5f:	jmp    5cd10 <probe::main+0xbb0>
   5cd61:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5cd70:	mov    %r12,%rdi
   5cd73:	mov    0x80(%rsp),%rsi
   5cd7b:	cmp    $0x8,%rbx
   5cd7f:	jb     5cd10 <probe::main+0xbb0>
   5cd81:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5cd90:	movzbl (%rsi),%r8d
   5cd94:	mov    %rdi,%r9
   5cd97:	shl    $0x5,%r9
   5cd9b:	sub    %rdi,%r9
   5cd9e:	add    %r8,%r9
   5cda1:	movzbl 0x1(%rsi),%edi
   5cda5:	mov    %r9,%r8
   5cda8:	shl    $0x5,%r8
   5cdac:	sub    %r9,%r8
   5cdaf:	add    %rdi,%r8
   5cdb2:	movzbl 0x2(%rsi),%edi
   5cdb6:	mov    %r8,%r9
   5cdb9:	shl    $0x5,%r9
   5cdbd:	sub    %r8,%r9
   5cdc0:	add    %rdi,%r9
   5cdc3:	movzbl 0x3(%rsi),%edi
   5cdc7:	mov    %r9,%r8
   5cdca:	shl    $0x5,%r8
   5cdce:	sub    %r9,%r8
   5cdd1:	add    %rdi,%r8
   5cdd4:	movzbl 0x4(%rsi),%edi
   5cdd8:	mov    %r8,%r9
   5cddb:	shl    $0x5,%r9
   5cddf:	sub    %r8,%r9
   5cde2:	add    %rdi,%r9
   5cde5:	movzbl 0x5(%rsi),%edi
   5cde9:	mov    %r9,%r8
   5cdec:	shl    $0x5,%r8
   5cdf0:	sub    %r9,%r8
   5cdf3:	add    %rdi,%r8
   5cdf6:	movzbl 0x6(%rsi),%edi
   5cdfa:	mov    %r8,%r9
   5cdfd:	shl    $0x5,%r9
   5ce01:	sub    %r8,%r9
   5ce04:	add    %rdi,%r9
   5ce07:	movzbl 0x7(%rsi),%r8d
   5ce0c:	add    $0x8,%rsi
   5ce10:	mov    %r9,%rdi
   5ce13:	shl    $0x5,%rdi
   5ce17:	sub    %r9,%rdi
   5ce1a:	add    %r8,%rdi
   5ce1d:	cmp    %rax,%rsi
   5ce20:	jne    5cd90 <probe::main+0xc30>
   5ce26:	jmp    5cd10 <probe::main+0xbb0>
   5ce2b:	xor    %r12d,%r12d
   5ce2e:	jmp    5ce96 <probe::main+0xd36>
   5ce30:	mov    %r15d,%eax
   5ce33:	and    $0x7,%eax
   5ce36:	cmp    $0x8,%r15
   5ce3a:	jae    5ce41 <probe::main+0xce1>
   5ce3c:	xor    %r12d,%r12d
   5ce3f:	jmp    5ce72 <probe::main+0xd12>
   5ce41:	mov    %r15,%rcx
   5ce44:	and    $0xfffffffffffffff8,%rcx
   5ce48:	xor    %r12d,%r12d
   5ce4b:	movabs $0xc694446f01,%rdx
   5ce55:	movabs $0x697ec45af80,%rsi
   5ce5f:	nop
   5ce60:	imul   %rdx,%r12
   5ce64:	add    %rsi,%r12
   5ce67:	add    $0xfffffffffffffff8,%rcx
   5ce6b:	jne    5ce60 <probe::main+0xd00>
   5ce6d:	test   %rax,%rax
   5ce70:	je     5ce96 <probe::main+0xd36>
   5ce72:	mov    %r12,%rcx
   5ce75:	data16 cs nopw 0x0(%rax,%rax,1)
   5ce80:	shl    $0x5,%r12
   5ce84:	sub    %rcx,%r12
   5ce87:	add    $0xff,%r12
   5ce8e:	mov    %r12,%rcx
   5ce91:	dec    %rax
   5ce94:	jne    5ce80 <probe::main+0xd20>
   5ce96:	mov    %r12,0x150(%rsp)
   5ce9e:	cmpb   $0x0,0x48(%rsp)
   5cea3:	mov    %rbx,0xa0(%rsp)
   5ceab:	je     5cf04 <probe::main+0xda4>
   5cead:	call   *0x129bcd(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5ceb3:	mov    $0x10,%edi
   5ceb8:	mov    $0x8,%esi
   5cebd:	call   *0x129bc5(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5cec3:	test   %rax,%rax
   5cec6:	je     5dcb2 <probe::main+0x1b52>
   5cecc:	movaps 0xe0(%rsp),%xmm0
   5ced4:	movups %xmm0,(%rax)
   5ced7:	movq   $0x1,0xc8(%rsp)
   5cee3:	mov    %rax,0xd0(%rsp)
   5ceeb:	movq   $0x1,0xd8(%rsp)
   5cef7:	movl   $0x0,0x94(%rsp)
   5cf02:	jmp    5cf37 <probe::main+0xdd7>
   5cf04:	movq   $0x0,0x8(%rsp)
   5cf0d:	mov    %r15,0x10(%rsp)
   5cf12:	lea    0x148(%rsp),%rax
   5cf1a:	mov    %rax,(%rsp)
   5cf1e:	lea    0xc8(%rsp),%rdi
   5cf26:	mov    %rsp,%rsi
   5cf29:	call   60690 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<hyperreal::computable::node::Computable, core::iter::adapters::map::Map<core::ops::range::Range<usize>, probe::main::{closure#2}>>>::from_iter>
   5cf2e:	mov    $0x1,%al
   5cf30:	mov    %eax,0x94(%rsp)
   5cf37:	movq   $0x0,0x138(%rsp)
   5cf43:	movq   $0x0,0x140(%rsp)
   5cf4f:	call   *0x129b73(%rip)        # 186ac8 <_DYNAMIC+0x2f8>
   5cf55:	mov    %rax,0x158(%rsp)
   5cf5d:	mov    %edx,0x160(%rsp)
   5cf64:	test   %r15,%r15
   5cf67:	je     5d1e6 <probe::main+0x1086>
   5cf6d:	mov    0xd0(%rsp),%rax
   5cf75:	mov    %rax,0x88(%rsp)
   5cf7d:	mov    0xd8(%rsp),%rax
   5cf85:	mov    %rax,0xa8(%rsp)
   5cf8d:	xor    %r14d,%r14d
   5cf90:	mov    %rsp,%r13
   5cf93:	lea    0x50(%rsp),%rbp
   5cf98:	movq   $0x0,0x98(%rsp)
   5cfa4:	xor    %ebx,%ebx
   5cfa6:	jmp    5cfbc <probe::main+0xe5c>
   5cfa8:	nopl   0x0(%rax,%rax,1)
   5cfb0:	inc    %rbx
   5cfb3:	cmp    %r15,%rbx
   5cfb6:	je     5d1f5 <probe::main+0x1095>
   5cfbc:	cmpb   $0x0,0x48(%rsp)
   5cfc1:	mov    $0x0,%esi
   5cfc6:	cmove  %rbx,%rsi
   5cfca:	cmp    0xa8(%rsp),%rsi
   5cfd2:	jae    5dcf2 <probe::main+0x1b92>
   5cfd8:	shl    $0x4,%rsi
   5cfdc:	add    0x88(%rsp),%rsi
   5cfe4:	mov    %r13,%rdi
   5cfe7:	mov    0x70(%rsp),%edx
   5cfeb:	call   *0x129a3f(%rip)        # 186a30 <_DYNAMIC+0x260>
   5cff1:	cmpq   $0xffffffffffffffff,(%rsp)
   5cff6:	je     5dad4 <probe::main+0x1974>
   5cffc:	movups (%rsp),%xmm0
   5d000:	movups 0x10(%rsp),%xmm1
   5d005:	movaps %xmm1,0x60(%rsp)
   5d00a:	movaps %xmm0,0x50(%rsp)
   5d00f:	movq   $0x0,0x30(%rsp)
   5d018:	movq   $0x1,0x38(%rsp)
   5d021:	movq   $0x0,0x40(%rsp)
   5d02a:	movq   $0x60000020,0x10(%rsp)
   5d033:	lea    0x30(%rsp),%rax
   5d038:	mov    %rax,(%rsp)
   5d03c:	lea    0x125315(%rip),%rax        # 182358 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x78>
   5d043:	mov    %rax,0x8(%rsp)
   5d048:	mov    %rbp,%rdi
   5d04b:	mov    %r13,%rsi
   5d04e:	call   *0x129a6c(%rip)        # 186ac0 <_DYNAMIC+0x2f0>
   5d054:	test   %al,%al
   5d056:	jne    5db07 <probe::main+0x19a7>
   5d05c:	mov    0x30(%rsp),%rsi
   5d061:	mov    0x38(%rsp),%rdi
   5d066:	mov    0x40(%rsp),%rax
   5d06b:	mov    0x98(%rsp),%rdx
   5d073:	add    %rax,%rdx
   5d076:	mov    %rdx,0x98(%rsp)
   5d07e:	mov    %rdx,0x140(%rsp)
   5d086:	test   %rax,%rax
   5d089:	je     5d0d0 <probe::main+0xf70>
   5d08b:	mov    %rax,%r8
   5d08e:	and    $0x7,%r8
   5d092:	je     5d0e0 <probe::main+0xf80>
   5d094:	mov    %rdi,%rcx
   5d097:	mov    %r14,%rdx
   5d09a:	nopw   0x0(%rax,%rax,1)
   5d0a0:	movzbl (%rcx),%r9d
   5d0a4:	inc    %rcx
   5d0a7:	shl    $0x5,%rdx
   5d0ab:	sub    %r14,%rdx
   5d0ae:	add    %r9,%rdx
   5d0b1:	mov    %rdx,%r14
   5d0b4:	dec    %r8
   5d0b7:	jne    5d0a0 <probe::main+0xf40>
   5d0b9:	cmp    $0x8,%rax
   5d0bd:	jae    5d0f0 <probe::main+0xf90>
   5d0bf:	jmp    5d196 <probe::main+0x1036>
   5d0c4:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5d0d0:	mov    %r14,%rdx
   5d0d3:	jmp    5d196 <probe::main+0x1036>
   5d0d8:	nopl   0x0(%rax,%rax,1)
   5d0e0:	mov    %r14,%rdx
   5d0e3:	mov    %rdi,%rcx
   5d0e6:	cmp    $0x8,%rax
   5d0ea:	jb     5d196 <probe::main+0x1036>
   5d0f0:	add    %rdi,%rax
   5d0f3:	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5d100:	movzbl (%rcx),%r8d
   5d104:	mov    %rdx,%r9
   5d107:	shl    $0x5,%r9
   5d10b:	sub    %rdx,%r9
   5d10e:	add    %r8,%r9
   5d111:	movzbl 0x1(%rcx),%edx
   5d115:	mov    %r9,%r8
   5d118:	shl    $0x5,%r8
   5d11c:	sub    %r9,%r8
   5d11f:	add    %rdx,%r8
   5d122:	movzbl 0x2(%rcx),%edx
   5d126:	mov    %r8,%r9
   5d129:	shl    $0x5,%r9
   5d12d:	sub    %r8,%r9
   5d130:	add    %rdx,%r9
   5d133:	movzbl 0x3(%rcx),%edx
   5d137:	mov    %r9,%r8
   5d13a:	shl    $0x5,%r8
   5d13e:	sub    %r9,%r8
   5d141:	add    %rdx,%r8
   5d144:	movzbl 0x4(%rcx),%edx
   5d148:	mov    %r8,%r9
   5d14b:	shl    $0x5,%r9
   5d14f:	sub    %r8,%r9
   5d152:	add    %rdx,%r9
   5d155:	movzbl 0x5(%rcx),%edx
   5d159:	mov    %r9,%r8
   5d15c:	shl    $0x5,%r8
   5d160:	sub    %r9,%r8
   5d163:	add    %rdx,%r8
   5d166:	movzbl 0x6(%rcx),%edx
   5d16a:	mov    %r8,%r9
   5d16d:	shl    $0x5,%r9
   5d171:	sub    %r8,%r9
   5d174:	add    %rdx,%r9
   5d177:	movzbl 0x7(%rcx),%r8d
   5d17c:	add    $0x8,%rcx
   5d180:	mov    %r9,%rdx
   5d183:	shl    $0x5,%rdx
   5d187:	sub    %r9,%rdx
   5d18a:	add    %r8,%rdx
   5d18d:	cmp    %rax,%rcx
   5d190:	jne    5d100 <probe::main+0xfa0>
   5d196:	mov    %rdx,%r14
   5d199:	shl    $0x5,%r14
   5d19d:	sub    %rdx,%r14
   5d1a0:	add    $0xff,%r14
   5d1a7:	mov    %r14,0x138(%rsp)
   5d1af:	test   %rsi,%rsi
   5d1b2:	je     5d1bf <probe::main+0x105f>
   5d1b4:	mov    $0x1,%edx
   5d1b9:	call   *0x129851(%rip)        # 186a10 <_DYNAMIC+0x240>
   5d1bf:	mov    0x50(%rsp),%rsi
   5d1c4:	test   %rsi,%rsi
   5d1c7:	je     5cfb0 <probe::main+0xe50>
   5d1cd:	mov    0x58(%rsp),%rdi
   5d1d2:	shl    $0x3,%rsi
   5d1d6:	mov    $0x8,%edx
   5d1db:	call   *0x12982f(%rip)        # 186a10 <_DYNAMIC+0x240>
   5d1e1:	jmp    5cfb0 <probe::main+0xe50>
   5d1e6:	movq   $0x0,0x98(%rsp)
   5d1f2:	xor    %r14d,%r14d
   5d1f5:	lea    0x158(%rsp),%rdi
   5d1fd:	call   *0x1298cd(%rip)        # 186ad0 <_DYNAMIC+0x300>
   5d203:	mov    %edx,%ecx
   5d205:	mov    $0x3b9aca00,%edx
   5d20a:	mul    %rdx
   5d20d:	mov    %ecx,%ecx
   5d20f:	add    %rax,%rcx
   5d212:	adc    $0x0,%rdx
   5d216:	cmp    %r12,%r14
   5d219:	jne    5dc59 <probe::main+0x1af9>
   5d21f:	mov    0xa0(%rsp),%rax
   5d227:	imul   %r15,%rax
   5d22b:	mov    %rax,(%rsp)
   5d22f:	cmp    %rax,0x98(%rsp)
   5d237:	mov    0x118(%rsp),%rbx
   5d23f:	jne    5dc7f <probe::main+0x1b1f>
   5d245:	mov    %rcx,0x88(%rsp)
   5d24d:	mov    %rdx,0xa8(%rsp)
   5d255:	movq   $0x0,0x100(%rsp)
   5d261:	movq   $0x0,0x110(%rsp)
   5d26d:	call   *0x12980d(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d273:	mov    $0x4,%ebp
   5d278:	mov    $0x4,%edi
   5d27d:	mov    $0x1,%esi
   5d282:	call   *0x129800(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d288:	test   %rax,%rax
   5d28b:	je     5dd0f <probe::main+0x1baf>
   5d291:	mov    %rax,%r13
   5d294:	movl   $0x65736163,(%rax)
   5d29a:	movq   $0x4,0x30(%rsp)
   5d2a3:	mov    %rax,0x38(%rsp)
   5d2a8:	movq   $0x4,0x40(%rsp)
   5d2b1:	mov    0x10(%rbx),%rbp
   5d2b5:	test   %rbp,%rbp
   5d2b8:	jns    5d2ca <probe::main+0x116a>
   5d2ba:	xor    %edi,%edi
   5d2bc:	mov    %rbp,%rsi
   5d2bf:	call   *0x129813(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5d2c5:	jmp    5dd92 <probe::main+0x1c32>
   5d2ca:	je     5d30f <probe::main+0x11af>
   5d2cc:	mov    0x8(%rbx),%rax
   5d2d0:	mov    %rax,0x118(%rsp)
   5d2d8:	call   *0x1297a2(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d2de:	mov    $0x1,%esi
   5d2e3:	mov    %rbp,%rdi
   5d2e6:	call   *0x12979c(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d2ec:	mov    $0x1,%edi
   5d2f1:	test   %rax,%rax
   5d2f4:	je     5d2bc <probe::main+0x115c>
   5d2f6:	mov    %rax,%rbx
   5d2f9:	mov    %rax,%rdi
   5d2fc:	mov    0x118(%rsp),%rsi
   5d304:	mov    %rbp,%rdx
   5d307:	call   *0x129783(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5d30d:	jmp    5d314 <probe::main+0x11b4>
   5d30f:	mov    $0x1,%ebx
   5d314:	movb   $0x3,(%rsp)
   5d318:	mov    %rbp,0x8(%rsp)
   5d31d:	mov    %rbx,0x10(%rsp)
   5d322:	mov    %rbp,0x18(%rsp)
   5d327:	lea    0x50(%rsp),%rdi
   5d32c:	lea    0x100(%rsp),%rsi
   5d334:	lea    0x30(%rsp),%rdx
   5d339:	mov    %rsp,%rcx
   5d33c:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d341:	cmpb   $0xff,0x50(%rsp)
   5d346:	je     5d352 <probe::main+0x11f2>
   5d348:	lea    0x50(%rsp),%rdi
   5d34d:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d352:	call   *0x129728(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d358:	mov    $0xa,%ebp
   5d35d:	mov    $0xa,%edi
   5d362:	mov    $0x1,%esi
   5d367:	call   *0x12971b(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d36d:	test   %rax,%rax
   5d370:	je     5dd0f <probe::main+0x1baf>
   5d376:	movabs $0x6f69746172657469,%rcx
   5d380:	mov    %rcx,(%rax)
   5d383:	movw   $0x736e,0x8(%rax)
   5d389:	movq   $0xa,0x30(%rsp)
   5d392:	mov    %rax,0x38(%rsp)
   5d397:	movq   $0xa,0x40(%rsp)
   5d3a0:	movb   $0x2,(%rsp)
   5d3a4:	movq   $0x0,0x8(%rsp)
   5d3ad:	mov    %r15,0x10(%rsp)
   5d3b2:	lea    0x50(%rsp),%rdi
   5d3b7:	lea    0x100(%rsp),%rsi
   5d3bf:	lea    0x30(%rsp),%rdx
   5d3c4:	mov    %rsp,%rcx
   5d3c7:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d3cc:	cmpb   $0xff,0x50(%rsp)
   5d3d1:	je     5d3dd <probe::main+0x127d>
   5d3d3:	lea    0x50(%rsp),%rdi
   5d3d8:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d3dd:	call   *0x12969d(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d3e3:	mov    $0x4,%ebp
   5d3e8:	mov    $0x4,%edi
   5d3ed:	mov    $0x1,%esi
   5d3f2:	call   *0x129690(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d3f8:	test   %rax,%rax
   5d3fb:	je     5dd0f <probe::main+0x1baf>
   5d401:	movl   $0x73746962,(%rax)
   5d407:	movq   $0x4,0x30(%rsp)
   5d410:	mov    %rax,0x38(%rsp)
   5d415:	movq   $0x4,0x40(%rsp)
   5d41e:	movb   $0x2,(%rsp)
   5d422:	movq   $0x0,0x8(%rsp)
   5d42b:	mov    0xc0(%rsp),%rax
   5d433:	mov    %rax,0x10(%rsp)
   5d438:	lea    0x50(%rsp),%rdi
   5d43d:	lea    0x100(%rsp),%rsi
   5d445:	lea    0x30(%rsp),%rdx
   5d44a:	mov    %rsp,%rcx
   5d44d:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d452:	cmpb   $0xff,0x50(%rsp)
   5d457:	je     5d463 <probe::main+0x1303>
   5d459:	lea    0x50(%rsp),%rdi
   5d45e:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d463:	call   *0x129617(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d469:	mov    $0x9,%ebp
   5d46e:	mov    $0x9,%edi
   5d473:	mov    $0x1,%esi
   5d478:	call   *0x12960a(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d47e:	test   %rax,%rax
   5d481:	je     5dd0f <probe::main+0x1baf>
   5d487:	movabs $0x6f69736963657270,%rcx
   5d491:	mov    %rcx,(%rax)
   5d494:	movb   $0x6e,0x8(%rax)
   5d498:	movq   $0x9,0x30(%rsp)
   5d4a1:	mov    %rax,0x38(%rsp)
   5d4a6:	movq   $0x9,0x40(%rsp)
   5d4af:	movslq 0x70(%rsp),%rax
   5d4b4:	mov    %rax,%rcx
   5d4b7:	shr    $0x3f,%rcx
   5d4bb:	movb   $0x2,(%rsp)
   5d4bf:	mov    %rcx,0x8(%rsp)
   5d4c4:	mov    %rax,0x10(%rsp)
   5d4c9:	lea    0x50(%rsp),%rdi
   5d4ce:	lea    0x100(%rsp),%rsi
   5d4d6:	lea    0x30(%rsp),%rdx
   5d4db:	mov    %rsp,%rcx
   5d4de:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d4e3:	cmpb   $0xff,0x50(%rsp)
   5d4e8:	je     5d4f4 <probe::main+0x1394>
   5d4ea:	lea    0x50(%rsp),%rdi
   5d4ef:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d4f4:	call   *0x129586(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d4fa:	mov    $0x8,%ebp
   5d4ff:	mov    $0x8,%edi
   5d504:	mov    $0x1,%esi
   5d509:	call   *0x129579(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d50f:	test   %rax,%rax
   5d512:	je     5dd0f <probe::main+0x1baf>
   5d518:	movabs $0x657669746167656e,%rcx
   5d522:	mov    %rcx,(%rax)
   5d525:	movq   $0x8,0x30(%rsp)
   5d52e:	mov    %rax,0x38(%rsp)
   5d533:	movq   $0x8,0x40(%rsp)
   5d53c:	movb   $0x1,(%rsp)
   5d540:	mov    0x4c(%rsp),%eax
   5d544:	mov    %al,0x1(%rsp)
   5d548:	lea    0x50(%rsp),%rdi
   5d54d:	lea    0x100(%rsp),%rsi
   5d555:	lea    0x30(%rsp),%rdx
   5d55a:	mov    %rsp,%rcx
   5d55d:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d562:	cmpb   $0xff,0x50(%rsp)
   5d567:	je     5d573 <probe::main+0x1413>
   5d569:	lea    0x50(%rsp),%rdi
   5d56e:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d573:	call   *0x129507(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d579:	mov    $0x6,%ebp
   5d57e:	mov    $0x6,%edi
   5d583:	mov    $0x1,%esi
   5d588:	call   *0x1294fa(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d58e:	test   %rax,%rax
   5d591:	je     5dd0f <probe::main+0x1baf>
   5d597:	movw   $0x6465,0x4(%rax)
   5d59d:	movl   $0x68636163,(%rax)
   5d5a3:	movq   $0x6,0x30(%rsp)
   5d5ac:	mov    %rax,0x38(%rsp)
   5d5b1:	movq   $0x6,0x40(%rsp)
   5d5ba:	movb   $0x1,(%rsp)
   5d5be:	mov    0x48(%rsp),%eax
   5d5c2:	mov    %al,0x1(%rsp)
   5d5c6:	lea    0x50(%rsp),%rdi
   5d5cb:	lea    0x100(%rsp),%rsi
   5d5d3:	lea    0x30(%rsp),%rdx
   5d5d8:	mov    %rsp,%rcx
   5d5db:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d5e0:	cmpb   $0xff,0x50(%rsp)
   5d5e5:	je     5d5f1 <probe::main+0x1491>
   5d5e7:	lea    0x50(%rsp),%rdi
   5d5ec:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d5f1:	call   *0x129489(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d5f7:	mov    $0xa,%ebp
   5d5fc:	mov    $0xa,%edi
   5d601:	mov    $0x1,%esi
   5d606:	call   *0x12947c(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d60c:	test   %rax,%rax
   5d60f:	je     5dd0f <probe::main+0x1baf>
   5d615:	mov    %rax,%r13
   5d618:	movabs $0x5f64657370616c65,%rax
   5d622:	mov    %rax,0x0(%r13)
   5d626:	movw   $0x736e,0x8(%r13)
   5d62d:	movq   $0xa,0x30(%rsp)
   5d636:	mov    %r13,0x38(%rsp)
   5d63b:	movq   $0xa,0x40(%rsp)
   5d644:	mov    %rsp,%rdx
   5d647:	mov    $0x27,%ecx
   5d64c:	mov    0x88(%rsp),%rdi
   5d654:	mov    0xa8(%rsp),%rsi
   5d65c:	call   *0x12947e(%rip)        # 186ae0 <_DYNAMIC+0x310>
   5d662:	mov    %rdx,%rbx
   5d665:	test   %rdx,%rdx
   5d668:	jns    5d67d <probe::main+0x151d>
   5d66a:	xor    %ebp,%ebp
   5d66c:	mov    %rbp,%rdi
   5d66f:	mov    %rbx,%rsi
   5d672:	call   *0x129460(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5d678:	jmp    5dd92 <probe::main+0x1c32>
   5d67d:	test   %rbx,%rbx
   5d680:	je     5d6e9 <probe::main+0x1589>
   5d682:	mov    %rax,%r15
   5d685:	call   *0x1293f5(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d68b:	mov    $0x1,%ebp
   5d690:	mov    $0x1,%esi
   5d695:	mov    %rbx,%rdi
   5d698:	call   *0x1293ea(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d69e:	test   %rax,%rax
   5d6a1:	je     5d66c <probe::main+0x150c>
   5d6a3:	mov    %rax,%r14
   5d6a6:	mov    %rax,%rdi
   5d6a9:	mov    %r15,%rsi
   5d6ac:	mov    %rbx,%rdx
   5d6af:	call   *0x1293db(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5d6b5:	call   *0x1293c5(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d6bb:	mov    $0x1,%esi
   5d6c0:	mov    %rbx,%rdi
   5d6c3:	call   *0x1293bf(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d6c9:	test   %rax,%rax
   5d6cc:	mov    %r14,%rbp
   5d6cf:	je     5dd61 <probe::main+0x1c01>
   5d6d5:	mov    %rax,%r15
   5d6d8:	mov    %rax,%rdi
   5d6db:	mov    %rbp,%rsi
   5d6de:	mov    %rbx,%rdx
   5d6e1:	call   *0x1293a9(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5d6e7:	jmp    5d6f4 <probe::main+0x1594>
   5d6e9:	mov    $0x1,%r15d
   5d6ef:	mov    $0x1,%ebp
   5d6f4:	mov    0xa0(%rsp),%r14
   5d6fc:	movb   $0x3,(%rsp)
   5d700:	mov    %rbx,0x8(%rsp)
   5d705:	mov    %r15,0x10(%rsp)
   5d70a:	mov    %rbx,0x18(%rsp)
   5d70f:	lea    0x50(%rsp),%rdi
   5d714:	lea    0x100(%rsp),%rsi
   5d71c:	lea    0x30(%rsp),%rdx
   5d721:	mov    %rsp,%rcx
   5d724:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d729:	cmpb   $0xff,0x50(%rsp)
   5d72e:	je     5d73a <probe::main+0x15da>
   5d730:	lea    0x50(%rsp),%rdi
   5d735:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d73a:	test   %rbx,%rbx
   5d73d:	je     5d750 <probe::main+0x15f0>
   5d73f:	mov    $0x1,%edx
   5d744:	mov    %rbp,%rdi
   5d747:	mov    %rbx,%rsi
   5d74a:	call   *0x1292c0(%rip)        # 186a10 <_DYNAMIC+0x240>
   5d750:	call   *0x12932a(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d756:	mov    $0x8,%ebp
   5d75b:	mov    $0x8,%edi
   5d760:	mov    $0x1,%esi
   5d765:	call   *0x12931d(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d76b:	test   %rax,%rax
   5d76e:	je     5dd0f <probe::main+0x1baf>
   5d774:	movabs $0x6d75736b63656863,%rcx
   5d77e:	mov    %rcx,(%rax)
   5d781:	movq   $0x8,0x30(%rsp)
   5d78a:	mov    %rax,0x38(%rsp)
   5d78f:	movq   $0x8,0x40(%rsp)
   5d798:	movb   $0x2,(%rsp)
   5d79c:	movq   $0x0,0x8(%rsp)
   5d7a5:	mov    %r12,0x10(%rsp)
   5d7aa:	lea    0x50(%rsp),%rdi
   5d7af:	lea    0x100(%rsp),%rsi
   5d7b7:	lea    0x30(%rsp),%rdx
   5d7bc:	mov    %rsp,%rcx
   5d7bf:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d7c4:	cmpb   $0xff,0x50(%rsp)
   5d7c9:	je     5d7d5 <probe::main+0x1675>
   5d7cb:	lea    0x50(%rsp),%rdi
   5d7d0:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d7d5:	call   *0x1292a5(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d7db:	mov    $0xc,%ebp
   5d7e0:	mov    $0xc,%edi
   5d7e5:	mov    $0x1,%esi
   5d7ea:	call   *0x129298(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d7f0:	test   %rax,%rax
   5d7f3:	je     5dd0f <probe::main+0x1baf>
   5d7f9:	movabs $0x625f74757074756f,%rcx
   5d803:	mov    %rcx,(%rax)
   5d806:	movl   $0x73657479,0x8(%rax)
   5d80d:	movq   $0xc,0x30(%rsp)
   5d816:	mov    %rax,0x38(%rsp)
   5d81b:	movq   $0xc,0x40(%rsp)
   5d824:	movb   $0x2,(%rsp)
   5d828:	movq   $0x0,0x8(%rsp)
   5d831:	mov    0x98(%rsp),%rax
   5d839:	mov    %rax,0x10(%rsp)
   5d83e:	lea    0x50(%rsp),%rdi
   5d843:	lea    0x100(%rsp),%rsi
   5d84b:	lea    0x30(%rsp),%rdx
   5d850:	mov    %rsp,%rcx
   5d853:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d858:	cmpb   $0xff,0x50(%rsp)
   5d85d:	je     5d869 <probe::main+0x1709>
   5d85f:	lea    0x50(%rsp),%rdi
   5d864:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d869:	call   *0x129211(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d86f:	mov    $0x9,%ebp
   5d874:	mov    $0x1,%ebx
   5d879:	mov    $0x9,%edi
   5d87e:	mov    $0x1,%esi
   5d883:	call   *0x1291ff(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d889:	test   %rax,%rax
   5d88c:	je     5dd0f <probe::main+0x1baf>
   5d892:	movabs $0x636e657265666572,%rcx
   5d89c:	mov    %rcx,(%rax)
   5d89f:	movb   $0x65,0x8(%rax)
   5d8a3:	movq   $0x9,0x30(%rsp)
   5d8ac:	mov    %rax,0x38(%rsp)
   5d8b1:	movq   $0x9,0x40(%rsp)
   5d8ba:	test   %r14,%r14
   5d8bd:	je     5d8f6 <probe::main+0x1796>
   5d8bf:	mov    %rax,%r12
   5d8c2:	call   *0x1291b8(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   5d8c8:	mov    $0x1,%esi
   5d8cd:	mov    %r14,%rdi
   5d8d0:	call   *0x1291b2(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   5d8d6:	test   %rax,%rax
   5d8d9:	je     5dd71 <probe::main+0x1c11>
   5d8df:	mov    %rax,%rbx
   5d8e2:	mov    %rax,%rdi
   5d8e5:	mov    0x80(%rsp),%rsi
   5d8ed:	mov    %r14,%rdx
   5d8f0:	call   *0x12919a(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   5d8f6:	movb   $0x3,(%rsp)
   5d8fa:	mov    %r14,0x8(%rsp)
   5d8ff:	mov    %rbx,0x10(%rsp)
   5d904:	mov    %r14,0x18(%rsp)
   5d909:	lea    0x50(%rsp),%rdi
   5d90e:	lea    0x100(%rsp),%rsi
   5d916:	lea    0x30(%rsp),%rdx
   5d91b:	mov    %rsp,%rbx
   5d91e:	mov    %rbx,%rcx
   5d921:	call   5e240 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d926:	cmpb   $0xff,0x50(%rsp)
   5d92b:	je     5d937 <probe::main+0x17d7>
   5d92d:	lea    0x50(%rsp),%rdi
   5d932:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d937:	movups 0x100(%rsp),%xmm0
   5d93f:	movups %xmm0,0x8(%rsp)
   5d944:	mov    0x110(%rsp),%rax
   5d94c:	mov    %rax,0x18(%rsp)
   5d951:	movb   $0x5,(%rsp)
   5d955:	mov    %rbx,0x50(%rsp)
   5d95a:	mov    0x129187(%rip),%rax        # 186ae8 <_DYNAMIC+0x318>
   5d961:	mov    %rax,0x58(%rsp)
   5d966:	lea    -0x2da45(%rip),%rdi        # 2ff28 <anon.c3a302fb892fb1604dd2767d56b20e73.4.llvm.15839081146404610231+0x1dc>
   5d96d:	lea    0x50(%rsp),%rsi
   5d972:	call   *0x129178(%rip)        # 186af0 <_DYNAMIC+0x320>
   5d978:	mov    %rsp,%rdi
   5d97b:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d980:	lea    0xc8(%rsp),%rdi
   5d988:	call   60790 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   5d98d:	mov    0xc8(%rsp),%rsi
   5d995:	test   %rsi,%rsi
   5d998:	je     5d9b1 <probe::main+0x1851>
   5d99a:	mov    0xd0(%rsp),%rdi
   5d9a2:	shl    $0x4,%rsi
   5d9a6:	mov    $0x8,%edx
   5d9ab:	call   *0x12905f(%rip)        # 186a10 <_DYNAMIC+0x240>
   5d9b1:	mov    0xb8(%rsp),%rsi
   5d9b9:	test   %rsi,%rsi
   5d9bc:	je     5d9d1 <probe::main+0x1871>
   5d9be:	mov    $0x1,%edx
   5d9c3:	mov    0x80(%rsp),%rdi
   5d9cb:	call   *0x12903f(%rip)        # 186a10 <_DYNAMIC+0x240>
   5d9d1:	cmpb   $0x0,0x94(%rsp)
   5d9d9:	je     5da1b <probe::main+0x18bb>
   5d9db:	lea    0xe8(%rsp),%rbx
   5d9e3:	mov    0xe0(%rsp),%rax
   5d9eb:	lock decq (%rax)
   5d9ef:	jne    5d9ff <probe::main+0x189f>
   5d9f1:	lea    0xe0(%rsp),%rdi
   5d9f9:	call   *0x129021(%rip)        # 186a20 <_DYNAMIC+0x250>
   5d9ff:	mov    0xe8(%rsp),%rax
   5da07:	test   %rax,%rax
   5da0a:	je     5da1b <probe::main+0x18bb>
   5da0c:	lock decq (%rax)
   5da10:	jne    5da1b <probe::main+0x18bb>
   5da12:	mov    %rbx,%rdi
   5da15:	call   *0x12900d(%rip)        # 186a28 <_DYNAMIC+0x258>
   5da1b:	mov    0xb0(%rsp),%rax
   5da23:	lock decq (%rax)
   5da27:	jne    5da37 <probe::main+0x18d7>
   5da29:	lea    0xb0(%rsp),%rdi
   5da31:	call   *0x1290c1(%rip)        # 186af8 <_DYNAMIC+0x328>
   5da37:	mov    0xf0(%rsp),%rsi
   5da3f:	test   %rsi,%rsi
   5da42:	je     5da5b <probe::main+0x18fb>
   5da44:	shl    $0x3,%rsi
   5da48:	mov    $0x8,%edx
   5da4d:	mov    0xf8(%rsp),%rdi
   5da55:	call   *0x128fb5(%rip)        # 186a10 <_DYNAMIC+0x240>
   5da5b:	mov    0x128(%rsp),%rbx
   5da63:	mov    0x130(%rsp),%r14
   5da6b:	test   %r14,%r14
   5da6e:	je     5da9f <probe::main+0x193f>
   5da70:	lea    0x8(%rbx),%r15
   5da74:	mov    0x128f95(%rip),%r12        # 186a10 <_DYNAMIC+0x240>
   5da7b:	jmp    5da89 <probe::main+0x1929>
   5da7d:	nopl   (%rax)
   5da80:	add    $0x18,%r15
   5da84:	dec    %r14
   5da87:	je     5da9f <probe::main+0x193f>
   5da89:	mov    -0x8(%r15),%rsi
   5da8d:	test   %rsi,%rsi
   5da90:	je     5da80 <probe::main+0x1920>
   5da92:	mov    (%r15),%rdi
   5da95:	mov    $0x1,%edx
   5da9a:	call   *%r12
   5da9d:	jmp    5da80 <probe::main+0x1920>
   5da9f:	mov    0x120(%rsp),%rax
   5daa7:	test   %rax,%rax
   5daaa:	je     5dac2 <probe::main+0x1962>
   5daac:	shl    $0x3,%rax
   5dab0:	lea    (%rax,%rax,2),%rsi
   5dab4:	mov    $0x8,%edx
   5dab9:	mov    %rbx,%rdi
   5dabc:	call   *0x128f4e(%rip)        # 186a10 <_DYNAMIC+0x240>
   5dac2:	add    $0x168,%rsp
   5dac9:	pop    %rbx
   5daca:	pop    %r12
   5dacc:	pop    %r13
   5dace:	pop    %r14
   5dad0:	pop    %r15
   5dad2:	pop    %rbp
   5dad3:	ret
   5dad4:	movzbl 0x8(%rsp),%eax
   5dad9:	mov    %al,0x50(%rsp)
   5dadd:	lea    -0x2adcc(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5dae4:	lea    0x1247f5(%rip),%rcx        # 1822e0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231>
   5daeb:	lea    0x1247d6(%rip),%r8        # 1822c8 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x138>
   5daf2:	lea    0x50(%rsp),%rdx
   5daf7:	mov    $0x2b,%esi
   5dafc:	call   *0x128f46(%rip)        # 186a48 <_DYNAMIC+0x278>
   5db02:	jmp    5dd92 <probe::main+0x1c32>
   5db07:	lea    -0x2ac3d(%rip),%rdi        # 32ed1 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231+0x1b9>
   5db0e:	lea    0x12480b(%rip),%rcx        # 182320 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x40>
   5db15:	lea    0x12486c(%rip),%r8        # 182388 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0xa8>
   5db1c:	lea    0x77(%rsp),%rdx
   5db21:	mov    $0x37,%esi
   5db26:	call   *0x128f1c(%rip)        # 186a48 <_DYNAMIC+0x278>
   5db2c:	jmp    5dd92 <probe::main+0x1c32>
   5db31:	add    $0xd0,%dl
   5db34:	xor    %eax,%eax
   5db36:	cmp    $0xa,%dl
   5db39:	adc    $0x1,%al
   5db3b:	mov    %al,(%rsp)
   5db3e:	lea    -0x2ae2d(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5db45:	lea    0x1247b4(%rip),%rcx        # 182300 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x20>
   5db4c:	lea    0x124685(%rip),%r8        # 1821d8 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x48>
   5db53:	mov    %rsp,%rdx
   5db56:	mov    $0x2b,%esi
   5db5b:	call   *0x128ee7(%rip)        # 186a48 <_DYNAMIC+0x278>
   5db61:	jmp    5dd92 <probe::main+0x1c32>
   5db66:	movzbl 0x8(%rsp),%eax
   5db6b:	mov    %al,0x30(%rsp)
   5db6f:	lea    -0x2ae5e(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5db76:	lea    0x124763(%rip),%rcx        # 1822e0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231>
   5db7d:	lea    0x124684(%rip),%r8        # 182208 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x78>
   5db84:	lea    0x30(%rsp),%rdx
   5db89:	mov    $0x2b,%esi
   5db8e:	call   *0x128eb4(%rip)        # 186a48 <_DYNAMIC+0x278>
   5db94:	jmp    5dd92 <probe::main+0x1c32>
   5db99:	movzbl 0x8(%rsp),%eax
   5db9e:	mov    %al,0x50(%rsp)
   5dba2:	lea    -0x2ae91(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5dba9:	lea    0x124730(%rip),%rcx        # 1822e0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231>
   5dbb0:	lea    0x124669(%rip),%r8        # 182220 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x90>
   5dbb7:	lea    0x50(%rsp),%rdx
   5dbbc:	mov    $0x2b,%esi
   5dbc1:	call   *0x128e81(%rip)        # 186a48 <_DYNAMIC+0x278>
   5dbc7:	jmp    5dd92 <probe::main+0x1c32>
   5dbcc:	movzbl 0x31(%rsp),%eax
   5dbd1:	mov    %al,(%rsp)
   5dbd4:	lea    -0x2aec3(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5dbdb:	lea    0x1246fe(%rip),%rcx        # 1822e0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231>
   5dbe2:	lea    0x124667(%rip),%r8        # 182250 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0xc0>
   5dbe9:	mov    %rsp,%rdx
   5dbec:	mov    $0x2b,%esi
   5dbf1:	call   *0x128e51(%rip)        # 186a48 <_DYNAMIC+0x278>
   5dbf7:	jmp    5dd92 <probe::main+0x1c32>
   5dbfc:	movzbl 0x8(%rsp),%eax
   5dc01:	mov    %al,0x50(%rsp)
   5dc05:	lea    -0x2aef4(%rip),%rdi        # 32d18 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231>
   5dc0c:	lea    0x1246cd(%rip),%rcx        # 1822e0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231>
   5dc13:	lea    0x12464e(%rip),%r8        # 182268 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0xd8>
   5dc1a:	lea    0x50(%rsp),%rdx
   5dc1f:	mov    $0x2b,%esi
   5dc24:	call   *0x128e1e(%rip)        # 186a48 <_DYNAMIC+0x278>
   5dc2a:	jmp    5dd92 <probe::main+0x1c32>
   5dc2f:	lea    -0x2ad65(%rip),%rdi        # 32ed1 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231+0x1b9>
   5dc36:	lea    0x1246e3(%rip),%rcx        # 182320 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x40>
   5dc3d:	lea    0x124744(%rip),%r8        # 182388 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0xa8>
   5dc44:	lea    0x77(%rsp),%rdx
   5dc49:	mov    $0x37,%esi
   5dc4e:	call   *0x128df4(%rip)        # 186a48 <_DYNAMIC+0x278>
   5dc54:	jmp    5dd92 <probe::main+0x1c32>
   5dc59:	lea    0x124620(%rip),%r9        # 182280 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0xf0>
   5dc60:	lea    0x138(%rsp),%rsi
   5dc68:	lea    0x150(%rsp),%rdx
   5dc70:	xor    %edi,%edi
   5dc72:	xor    %ecx,%ecx
   5dc74:	call   *0x128e86(%rip)        # 186b00 <_DYNAMIC+0x330>
   5dc7a:	jmp    5dd92 <probe::main+0x1c32>
   5dc7f:	lea    0x124612(%rip),%r9        # 182298 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x108>
   5dc86:	lea    0x140(%rsp),%rsi
   5dc8e:	mov    %rsp,%rdx
   5dc91:	xor    %edi,%edi
   5dc93:	xor    %ecx,%ecx
   5dc95:	call   *0x128e6d(%rip)        # 186b08 <_DYNAMIC+0x338>
   5dc9b:	jmp    5dd92 <probe::main+0x1c32>
   5dca0:	lea    0x124591(%rip),%rdi        # 182238 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0xa8>
   5dca7:	call   *0x128e63(%rip)        # 186b10 <_DYNAMIC+0x340>
   5dcad:	jmp    5dd92 <probe::main+0x1c32>
   5dcb2:	mov    $0x8,%edi
   5dcb7:	mov    $0x10,%esi
   5dcbc:	call   *0x128e56(%rip)        # 186b18 <_DYNAMIC+0x348>
   5dcc2:	jmp    5dd92 <probe::main+0x1c32>
   5dcc7:	mov    $0x1,%al
   5dcc9:	jmp    5db3b <probe::main+0x19db>
   5dcce:	mov    %rbx,%rbp
   5dcd1:	mov    %r12,%r13
   5dcd4:	lea    -0x2af98(%rip),%rdi        # 32d43 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231+0x2b>
   5dcdb:	lea    0x12465e(%rip),%rdx        # 182340 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.15839081146404610231+0x60>
   5dce2:	mov    $0x49,%esi
   5dce7:	call   *0x128d73(%rip)        # 186a60 <_DYNAMIC+0x290>
   5dced:	jmp    5dd92 <probe::main+0x1c32>
   5dcf2:	lea    0x1245b7(%rip),%rdx        # 1822b0 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x120>
   5dcf9:	mov    %rsi,%rdi
   5dcfc:	mov    0xa8(%rsp),%rsi
   5dd04:	call   *0x128d46(%rip)        # 186a50 <_DYNAMIC+0x280>
   5dd0a:	jmp    5dd92 <probe::main+0x1c32>
   5dd0f:	mov    $0x1,%edi
   5dd14:	mov    %rbp,%rsi
   5dd17:	call   *0x128dbb(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd1d:	jmp    5dd92 <probe::main+0x1c32>
   5dd1f:	lea    0x12449a(%rip),%rdx        # 1821c0 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x30>
   5dd26:	jmp    5dd2f <probe::main+0x1bcf>
   5dd28:	lea    0x124479(%rip),%rdx        # 1821a8 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.15839081146404610231+0x18>
   5dd2f:	mov    %rdi,%rsi
   5dd32:	call   *0x128d18(%rip)        # 186a50 <_DYNAMIC+0x280>
   5dd38:	jmp    5dd92 <probe::main+0x1c32>
   5dd3a:	xor    %eax,%eax
   5dd3c:	jmp    5db3b <probe::main+0x19db>
   5dd41:	mov    $0x8,%edi
   5dd46:	mov    %r13,%rsi
   5dd49:	call   *0x128d89(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd4f:	jmp    5dd92 <probe::main+0x1c32>
   5dd51:	mov    $0x8,%edi
   5dd56:	mov    %rbx,%rsi
   5dd59:	call   *0x128d79(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd5f:	jmp    5dd92 <probe::main+0x1c32>
   5dd61:	mov    $0x1,%edi
   5dd66:	mov    %rbx,%rsi
   5dd69:	call   *0x128d69(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd6f:	jmp    5dd92 <probe::main+0x1c32>
   5dd71:	mov    $0x1,%edi
   5dd76:	mov    %r14,%rsi
   5dd79:	call   *0x128d59(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd7f:	jmp    5dd92 <probe::main+0x1c32>
   5dd81:	mov    %rbx,%rbp
   5dd84:	mov    $0x8,%edi
   5dd89:	mov    %r12,%rsi
   5dd8c:	call   *0x128d46(%rip)        # 186ad8 <_DYNAMIC+0x308>
   5dd92:	ud2
   5dd94:	mov    %rax,%r15
   5dd97:	jmp    5df72 <probe::main+0x1e12>
   5dd9c:	mov    %rax,%r15
   5dd9f:	mov    0xe8(%rsp),%rax
   5dda7:	test   %rax,%rax
   5ddaa:	je     5e1bc <probe::main+0x205c>
   5ddb0:	lock decq (%rax)
   5ddb4:	jne    5e1bc <probe::main+0x205c>
   5ddba:	mov    %rbx,%rdi
   5ddbd:	call   *0x128c65(%rip)        # 186a28 <_DYNAMIC+0x258>
   5ddc3:	jmp    5e1bc <probe::main+0x205c>
   5ddc8:	call   *0x128c4a(%rip)        # 186a18 <_DYNAMIC+0x248>
   5ddce:	jmp    5ddd0 <probe::main+0x1c70>
   5ddd0:	mov    %rax,%r15
   5ddd3:	mov    (%rsp),%rsi
   5ddd7:	test   %rsi,%rsi
   5ddda:	je     5df83 <probe::main+0x1e23>
   5dde0:	mov    0x8(%rsp),%rdi
   5dde5:	shl    $0x3,%rsi
   5dde9:	mov    $0x8,%edx
   5ddee:	jmp    5df7d <probe::main+0x1e1d>
   5ddf3:	mov    %rax,%r15
   5ddf6:	mov    $0x9,%esi
   5ddfb:	mov    $0x1,%edx
   5de00:	mov    %r12,%rdi
   5de03:	jmp    5e141 <probe::main+0x1fe1>
   5de08:	mov    %rax,%r15
   5de0b:	mov    $0xa,%esi
   5de10:	mov    $0x1,%edx
   5de15:	mov    %r13,%rdi
   5de18:	call   *0x128bf2(%rip)        # 186a10 <_DYNAMIC+0x240>
   5de1e:	jmp    5de7e <probe::main+0x1d1e>
   5de20:	jmp    5de72 <probe::main+0x1d12>
   5de22:	mov    %rax,%r15
   5de25:	mov    (%rsp),%rsi
   5de29:	test   %rsi,%rsi
   5de2c:	je     5e1d8 <probe::main+0x2078>
   5de32:	mov    0x8(%rsp),%rdi
   5de37:	shl    $0x3,%rsi
   5de3b:	mov    $0x8,%edx
   5de40:	call   *0x128bca(%rip)        # 186a10 <_DYNAMIC+0x240>
   5de46:	jmp    5e1d8 <probe::main+0x2078>
   5de4b:	mov    %rax,%r15
   5de4e:	jmp    5e161 <probe::main+0x2001>
   5de53:	mov    %rax,%r15
   5de56:	mov    %rsp,%rdi
   5de59:	call   5be70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5de5e:	jmp    5e154 <probe::main+0x1ff4>
   5de63:	jmp    5df54 <probe::main+0x1df4>
   5de68:	jmp    5df54 <probe::main+0x1df4>
   5de6d:	jmp    5df54 <probe::main+0x1df4>
   5de72:	mov    %rax,%r15
   5de75:	test   %rbx,%rbx
   5de78:	je     5e147 <probe::main+0x1fe7>
   5de7e:	mov    $0x1,%edx
   5de83:	mov    %rbp,%rdi
   5de86:	mov    %rbx,%rsi
   5de89:	jmp    5e141 <probe::main+0x1fe1>
   5de8e:	mov    %r13,0xa0(%rsp)
   5de96:	jmp    5de98 <probe::main+0x1d38>
   5de98:	mov    %rax,%r15
   5de9b:	mov    (%rsp),%rsi
   5de9f:	test   %rsi,%rsi
   5dea2:	je     5deb6 <probe::main+0x1d56>
   5dea4:	mov    (%rbx),%rdi
   5dea7:	shl    $0x3,%rsi
   5deab:	mov    $0x8,%edx
   5deb0:	call   *0x128b5a(%rip)        # 186a10 <_DYNAMIC+0x240>
   5deb6:	test   %r14,%r14
   5deb9:	je     5e1d8 <probe::main+0x2078>
   5debf:	mov    $0x8,%edx
   5dec4:	mov    %r12,%rdi
   5dec7:	mov    0xa0(%rsp),%rsi
   5decf:	call   *0x128b3b(%rip)        # 186a10 <_DYNAMIC+0x240>
   5ded5:	jmp    5e1d8 <probe::main+0x2078>
   5deda:	jmp    5df54 <probe::main+0x1df4>
   5dedc:	jmp    5df54 <probe::main+0x1df4>
   5dede:	jmp    5df54 <probe::main+0x1df4>
   5dee0:	jmp    5df54 <probe::main+0x1df4>
   5dee2:	jmp    5df54 <probe::main+0x1df4>
   5dee4:	jmp    5df54 <probe::main+0x1df4>
   5dee6:	mov    %rax,%r15
   5dee9:	mov    0x38(%rsp),%rax
   5deee:	test   %rax,%rax
   5def1:	je     5e1d8 <probe::main+0x2078>
   5def7:	lock decq (%rax)
   5defb:	jne    5e1d8 <probe::main+0x2078>
   5df01:	mov    %rbx,%rdi
   5df04:	call   *0x128b1e(%rip)        # 186a28 <_DYNAMIC+0x258>
   5df0a:	jmp    5e1d8 <probe::main+0x2078>
   5df0f:	call   *0x128b03(%rip)        # 186a18 <_DYNAMIC+0x248>
   5df15:	mov    %rax,%r15
   5df18:	mov    0x58(%rsp),%rax
   5df1d:	test   %rax,%rax
   5df20:	je     5e0c0 <probe::main+0x1f60>
   5df26:	lock decq (%rax)
   5df2a:	jne    5e0c0 <probe::main+0x1f60>
   5df30:	mov    %rbx,%rdi
   5df33:	call   *0x128aef(%rip)        # 186a28 <_DYNAMIC+0x258>
   5df39:	jmp    5e0c0 <probe::main+0x1f60>
   5df3e:	call   *0x128ad4(%rip)        # 186a18 <_DYNAMIC+0x248>
   5df44:	mov    %rax,%r15
   5df47:	jmp    5e1d8 <probe::main+0x2078>
   5df4c:	mov    %rax,%r15
   5df4f:	jmp    5e1bc <probe::main+0x205c>
   5df54:	mov    %rax,%r15
   5df57:	jmp    5e147 <probe::main+0x1fe7>
   5df5c:	jmp    5e064 <probe::main+0x1f04>
   5df61:	jmp    5e0a8 <probe::main+0x1f48>
   5df66:	mov    %rax,%r15
   5df69:	test   %rbp,%rbp
   5df6c:	je     5df83 <probe::main+0x1e23>
   5df6e:	shl    $0x3,%rbp
   5df72:	mov    $0x8,%edx
   5df77:	mov    %r13,%rdi
   5df7a:	mov    %rbp,%rsi
   5df7d:	call   *0x128a8d(%rip)        # 186a10 <_DYNAMIC+0x240>
   5df83:	cmpq   $0x0,0x80(%rsp)
   5df8c:	je     5e1d8 <probe::main+0x2078>
   5df92:	mov    0x80(%rsp),%rsi
   5df9a:	shl    $0x3,%rsi
   5df9e:	mov    $0x8,%edx
   5dfa3:	mov    0xb8(%rsp),%rdi
   5dfab:	call   *0x128a5f(%rip)        # 186a10 <_DYNAMIC+0x240>
   5dfb1:	jmp    5e1d8 <probe::main+0x2078>
   5dfb6:	mov    %rax,%r15
   5dfb9:	mov    $0x1,%al
   5dfbb:	mov    %eax,0x94(%rsp)
   5dfc2:	jmp    5e185 <probe::main+0x2025>
   5dfc7:	mov    %rax,%r15
   5dfca:	test   %r13,%r13
   5dfcd:	je     5e1d8 <probe::main+0x2078>
   5dfd3:	shl    $0x3,%r13
   5dfd7:	mov    $0x8,%edx
   5dfdc:	mov    0x88(%rsp),%rdi
   5dfe4:	mov    %r13,%rsi
   5dfe7:	call   *0x128a23(%rip)        # 186a10 <_DYNAMIC+0x240>
   5dfed:	jmp    5e1d8 <probe::main+0x2078>
   5dff2:	mov    %rax,%r15
   5dff5:	mov    0x30(%rsp),%rsi
   5dffa:	test   %rsi,%rsi
   5dffd:	je     5e00f <probe::main+0x1eaf>
   5dfff:	mov    0x38(%rsp),%rdi
   5e004:	mov    $0x1,%edx
   5e009:	call   *0x128a01(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e00f:	mov    0x50(%rsp),%rsi
   5e014:	test   %rsi,%rsi
   5e017:	je     5e1af <probe::main+0x204f>
   5e01d:	mov    0x58(%rsp),%rdi
   5e022:	shl    $0x3,%rsi
   5e026:	mov    $0x8,%edx
   5e02b:	call   *0x1289df(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e031:	jmp    5e1af <probe::main+0x204f>
   5e036:	mov    %rax,%r15
   5e039:	jmp    5e1af <probe::main+0x204f>
   5e03e:	mov    %rax,%r15
   5e041:	jmp    5e1d8 <probe::main+0x2078>
   5e046:	mov    %rax,%r15
   5e049:	lea    0x50(%rsp),%rdi
   5e04e:	call   5bf60 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e053:	jmp    5e0c0 <probe::main+0x1f60>
   5e055:	mov    %rax,%r15
   5e058:	lea    0x30(%rsp),%rdi
   5e05d:	call   5bf60 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e062:	jmp    5e0c0 <probe::main+0x1f60>
   5e064:	mov    %rax,%r15
   5e067:	mov    0x30(%rsp),%rsi
   5e06c:	test   %rsi,%rsi
   5e06f:	je     5e081 <probe::main+0x1f21>
   5e071:	mov    0x38(%rsp),%rdi
   5e076:	mov    $0x1,%edx
   5e07b:	call   *0x12898f(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e081:	mov    0x50(%rsp),%rsi
   5e086:	test   %rsi,%rsi
   5e089:	je     5e154 <probe::main+0x1ff4>
   5e08f:	mov    0x58(%rsp),%rdi
   5e094:	shl    $0x3,%rsi
   5e098:	mov    $0x8,%edx
   5e09d:	call   *0x12896d(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e0a3:	jmp    5e154 <probe::main+0x1ff4>
   5e0a8:	mov    %rax,%r15
   5e0ab:	jmp    5e154 <probe::main+0x1ff4>
   5e0b0:	mov    %rax,%r15
   5e0b3:	mov    $0xa,%esi
   5e0b8:	jmp    5e139 <probe::main+0x1fd9>
   5e0bd:	mov    %rax,%r15
   5e0c0:	mov    0x128(%rsp),%rbx
   5e0c8:	mov    0x130(%rsp),%r14
   5e0d0:	test   %r14,%r14
   5e0d3:	jne    5e100 <probe::main+0x1fa0>
   5e0d5:	mov    0x120(%rsp),%rax
   5e0dd:	test   %rax,%rax
   5e0e0:	je     5e0f8 <probe::main+0x1f98>
   5e0e2:	shl    $0x3,%rax
   5e0e6:	lea    (%rax,%rax,2),%rsi
   5e0ea:	mov    $0x8,%edx
   5e0ef:	mov    %rbx,%rdi
   5e0f2:	call   *0x128918(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e0f8:	mov    %r15,%rdi
   5e0fb:	call   180f40 <_Unwind_Resume@plt>
   5e100:	lea    0x8(%rbx),%r12
   5e104:	mov    0x128905(%rip),%r13        # 186a10 <_DYNAMIC+0x240>
   5e10b:	jmp    5e119 <probe::main+0x1fb9>
   5e10d:	nopl   (%rax)
   5e110:	add    $0x18,%r12
   5e114:	dec    %r14
   5e117:	je     5e0d5 <probe::main+0x1f75>
   5e119:	mov    -0x8(%r12),%rsi
   5e11e:	test   %rsi,%rsi
   5e121:	je     5e110 <probe::main+0x1fb0>
   5e123:	mov    (%r12),%rdi
   5e127:	mov    $0x1,%edx
   5e12c:	call   *%r13
   5e12f:	jmp    5e110 <probe::main+0x1fb0>
   5e131:	mov    %rax,%r15
   5e134:	mov    $0x4,%esi
   5e139:	mov    $0x1,%edx
   5e13e:	mov    %r13,%rdi
   5e141:	call   *0x1288c9(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e147:	lea    0x100(%rsp),%rdi
   5e14f:	call   5e8f0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value> as core::ops::drop::Drop>::drop>
   5e154:	lea    0xc8(%rsp),%rdi
   5e15c:	call   60790 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   5e161:	mov    0xc8(%rsp),%rsi
   5e169:	test   %rsi,%rsi
   5e16c:	je     5e185 <probe::main+0x2025>
   5e16e:	mov    0xd0(%rsp),%rdi
   5e176:	shl    $0x4,%rsi
   5e17a:	mov    $0x8,%edx
   5e17f:	call   *0x12888b(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e185:	mov    0xb8(%rsp),%rsi
   5e18d:	test   %rsi,%rsi
   5e190:	je     5e1a5 <probe::main+0x2045>
   5e192:	mov    $0x1,%edx
   5e197:	mov    0x80(%rsp),%rdi
   5e19f:	call   *0x12886b(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e1a5:	cmpb   $0x0,0x94(%rsp)
   5e1ad:	je     5e1bc <probe::main+0x205c>
   5e1af:	lea    0xe0(%rsp),%rdi
   5e1b7:	call   5bf60 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e1bc:	mov    0xb0(%rsp),%rax
   5e1c4:	lock decq (%rax)
   5e1c8:	jne    5e1d8 <probe::main+0x2078>
   5e1ca:	lea    0xb0(%rsp),%rdi
   5e1d2:	call   *0x128920(%rip)        # 186af8 <_DYNAMIC+0x328>
   5e1d8:	mov    0xf0(%rsp),%rsi
   5e1e0:	test   %rsi,%rsi
   5e1e3:	je     5e0c0 <probe::main+0x1f60>
   5e1e9:	shl    $0x3,%rsi
   5e1ed:	mov    $0x8,%edx
   5e1f2:	mov    0xf8(%rsp),%rdi
   5e1fa:	call   *0x128810(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e200:	jmp    5e0c0 <probe::main+0x1f60>
   5e205:	mov    0xc8(%rsp),%rsi
   5e20d:	test   %rsi,%rsi
   5e210:	je     5e22f <probe::main+0x20cf>
   5e212:	mov    0xd0(%rsp),%rdi
   5e21a:	shl    $0x4,%rsi
   5e21e:	mov    $0x8,%edx
   5e223:	call   *0x1287e7(%rip)        # 186a10 <_DYNAMIC+0x240>
   5e229:	call   *0x1287e9(%rip)        # 186a18 <_DYNAMIC+0x248>
   5e22f:	call   *0x1287e3(%rip)        # 186a18 <_DYNAMIC+0x248>
