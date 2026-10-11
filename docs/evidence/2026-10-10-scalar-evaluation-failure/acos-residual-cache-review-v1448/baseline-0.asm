
target/typed-api-prototype-v1418-acos-warm-baseline-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

000000000005c0e0 <probe::main>:
   5c0e0:	push   %rbp
   5c0e1:	push   %r15
   5c0e3:	push   %r14
   5c0e5:	push   %r13
   5c0e7:	push   %r12
   5c0e9:	push   %rbx
   5c0ea:	sub    $0x168,%rsp
   5c0f1:	mov    %rsp,%rbx
   5c0f4:	mov    %rbx,%rdi
   5c0f7:	call   *0x12a513(%rip)        # 186610 <_DYNAMIC+0x288>
   5c0fd:	movq   $0x1,0x20(%rsp)
   5c106:	lea    0x120(%rsp),%rdi
   5c10e:	mov    %rbx,%rsi
   5c111:	call   601d0 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::skip::Skip<std::env::Args>>>::from_iter>
   5c116:	mov    0x130(%rsp),%rdi
   5c11e:	cmp    $0x1,%rdi
   5c122:	je     5dc9f <probe::main+0x1bbf>
   5c128:	test   %rdi,%rdi
   5c12b:	je     5dca8 <probe::main+0x1bc8>
   5c131:	mov    0x128(%rsp),%r9
   5c139:	mov    0x20(%r9),%rcx
   5c13d:	mov    0x28(%r9),%rsi
   5c141:	cmp    $0x1,%rsi
   5c145:	je     5c155 <probe::main+0x75>
   5c147:	test   %rsi,%rsi
   5c14a:	je     5dcba <probe::main+0x1bda>
   5c150:	movzbl (%rcx),%edx
   5c153:	jmp    5c16c <probe::main+0x8c>
   5c155:	movzbl (%rcx),%edx
   5c158:	mov    $0x1,%al
   5c15a:	cmp    $0x2b,%edx
   5c15d:	je     5dabb <probe::main+0x19db>
   5c163:	cmp    $0x2d,%edx
   5c166:	je     5dabb <probe::main+0x19db>
   5c16c:	xor    %eax,%eax
   5c16e:	cmp    $0x2b,%dl
   5c171:	sete   %al
   5c174:	mov    %rsi,%rdx
   5c177:	sub    %rax,%rdx
   5c17a:	add    %rax,%rcx
   5c17d:	neg    %rax
   5c180:	cmp    $0x11,%rdx
   5c184:	jae    5c1c7 <probe::main+0xe7>
   5c186:	test   %rdx,%rdx
   5c189:	je     5c219 <probe::main+0x139>
   5c18f:	add    %rax,%rsi
   5c192:	neg    %rsi
   5c195:	xor    %r15d,%r15d
   5c198:	xor    %eax,%eax
   5c19a:	nopw   0x0(%rax,%rax,1)
   5c1a0:	movzbl (%rcx,%rax,1),%edx
   5c1a4:	add    $0xffffffd0,%edx
   5c1a7:	cmp    $0x9,%edx
   5c1aa:	ja     5dc47 <probe::main+0x1b67>
   5c1b0:	lea    (%r15,%r15,4),%rdi
   5c1b4:	mov    %edx,%edx
   5c1b6:	lea    (%rdx,%rdi,2),%r15
   5c1ba:	inc    %rax
   5c1bd:	mov    %rsi,%rdx
   5c1c0:	add    %rax,%rdx
   5c1c3:	jne    5c1a0 <probe::main+0xc0>
   5c1c5:	jmp    5c21c <probe::main+0x13c>
   5c1c7:	add    %rax,%rsi
   5c1ca:	neg    %rsi
   5c1cd:	xor    %r15d,%r15d
   5c1d0:	mov    $0xa,%edi
   5c1d5:	xor    %r8d,%r8d
   5c1d8:	nopl   0x0(%rax,%rax,1)
   5c1e0:	mov    %rsi,%rax
   5c1e3:	add    %r8,%rax
   5c1e6:	je     5c21c <probe::main+0x13c>
   5c1e8:	mov    %r15,%rax
   5c1eb:	mul    %rdi
   5c1ee:	movzbl (%rcx,%r8,1),%edx
   5c1f3:	jo     5dab1 <probe::main+0x19d1>
   5c1f9:	add    $0xffffffd0,%edx
   5c1fc:	cmp    $0xa,%edx
   5c1ff:	jae    5dc47 <probe::main+0x1b67>
   5c205:	mov    %rax,%r15
   5c208:	mov    %edx,%eax
   5c20a:	inc    %r8
   5c20d:	add    %rax,%r15
   5c210:	jae    5c1e0 <probe::main+0x100>
   5c212:	mov    $0x2,%al
   5c214:	jmp    5dabb <probe::main+0x19db>
   5c219:	xor    %r15d,%r15d
   5c21c:	mov    0x10(%r9),%rcx
   5c220:	add    $0xfffffffffffffff4,%rcx
   5c224:	cmp    $0x3,%rcx
   5c228:	ja     5c35a <probe::main+0x27a>
   5c22e:	mov    0x8(%r9),%rax
   5c232:	lea    -0x2c739(%rip),%rdx        # 2fb00 <GCC_except_table0+0x2c>
   5c239:	movslq (%rdx,%rcx,4),%rcx
   5c23d:	add    %rdx,%rcx
   5c240:	jmp    *%rcx
   5c242:	movabs $0x69736f705f64696d,%rcx
   5c24c:	xor    (%rax),%rcx
   5c24f:	mov    0x8(%rax),%edx
   5c252:	xor    $0x65766974,%rdx
   5c259:	mov    $0x1001,%esi
   5c25e:	mov    %rsi,0xc0(%rsp)
   5c266:	or     %rcx,%rdx
   5c269:	je     5c3d2 <probe::main+0x2f2>
   5c26f:	movabs $0x6167656e5f64696d,%rcx
   5c279:	xor    (%rax),%rcx
   5c27c:	mov    0x8(%rax),%eax
   5c27f:	xor    $0x65766974,%rax
   5c285:	or     %rcx,%rax
   5c288:	je     5c3c2 <probe::main+0x2e2>
   5c28e:	jmp    5c35a <probe::main+0x27a>
   5c293:	movabs $0x6f705f6c6c616d73,%rdx
   5c29d:	xor    (%rax),%rdx
   5c2a0:	movabs $0x657669746167656e,%rcx
   5c2aa:	lea    0x80c0a02(%rcx),%rsi
   5c2b1:	xor    0x6(%rax),%rsi
   5c2b5:	movl   $0xffffffc0,0x70(%rsp)
   5c2bd:	mov    $0x8,%edi
   5c2c2:	mov    %rdi,0xc0(%rsp)
   5c2ca:	or     %rdx,%rsi
   5c2cd:	je     5c3da <probe::main+0x2fa>
   5c2d3:	movabs $0x656e5f6c6c616d73,%rdx
   5c2dd:	xor    (%rax),%rdx
   5c2e0:	xor    0x6(%rax),%rcx
   5c2e4:	or     %rdx,%rcx
   5c2e7:	jne    5c35a <probe::main+0x27a>
   5c2e9:	mov    $0x1,%al
   5c2eb:	mov    %eax,0x4c(%rsp)
   5c2ef:	jmp    5c3e2 <probe::main+0x302>
   5c2f4:	movabs $0x705f646568636163,%rdx
   5c2fe:	xor    (%rax),%rdx
   5c301:	movabs $0x657669746167656e,%rcx
   5c30b:	lea    0x80c0a02(%rcx),%rsi
   5c312:	xor    0x7(%rax),%rsi
   5c316:	movl   $0x0,0x70(%rsp)
   5c31e:	mov    $0xf4241,%edi
   5c323:	mov    %rdi,0xc0(%rsp)
   5c32b:	or     %rdx,%rsi
   5c32e:	je     5c821 <probe::main+0x741>
   5c334:	movabs $0x6e5f646568636163,%rdx
   5c33e:	xor    (%rax),%rdx
   5c341:	xor    0x7(%rax),%rcx
   5c345:	mov    $0x1,%al
   5c347:	mov    %eax,0x4c(%rsp)
   5c34b:	or     %rdx,%rcx
   5c34e:	mov    $0x1,%al
   5c350:	mov    %eax,0x48(%rsp)
   5c354:	je     5c3ea <probe::main+0x30a>
   5c35a:	lea    -0x296c1(%rip),%rdi        # 32ca0 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.17861709745643738061+0xf0>
   5c361:	lea    0x125a58(%rip),%rdx        # 181dc0 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x60>
   5c368:	mov    $0x21,%esi
   5c36d:	call   *0x12a2a5(%rip)        # 186618 <_DYNAMIC+0x290>
   5c373:	jmp    5dd12 <probe::main+0x1c32>
   5c378:	movabs $0x657669746167656e,%rcx
   5c382:	movabs $0x736f705f65646977,%rdx
   5c38c:	xor    (%rax),%rdx
   5c38f:	lea    0x80c0a02(%rcx),%rsi
   5c396:	xor    0x5(%rax),%rsi
   5c39a:	mov    $0xf4241,%edi
   5c39f:	mov    %rdi,0xc0(%rsp)
   5c3a7:	or     %rdx,%rsi
   5c3aa:	je     5c3d2 <probe::main+0x2f2>
   5c3ac:	movabs $0x67656e5f65646977,%rdx
   5c3b6:	xor    (%rax),%rdx
   5c3b9:	xor    0x5(%rax),%rcx
   5c3bd:	or     %rdx,%rcx
   5c3c0:	jne    5c35a <probe::main+0x27a>
   5c3c2:	mov    $0x1,%al
   5c3c4:	mov    %eax,0x4c(%rsp)
   5c3c8:	movl   $0x0,0x70(%rsp)
   5c3d0:	jmp    5c3e2 <probe::main+0x302>
   5c3d2:	movl   $0x0,0x70(%rsp)
   5c3da:	movl   $0x0,0x4c(%rsp)
   5c3e2:	movl   $0x0,0x48(%rsp)
   5c3ea:	mov    %r9,0x118(%rsp)
   5c3f2:	call   *0x12a228(%rip)        # 186620 <_DYNAMIC+0x298>
   5c3f8:	mov    %rax,0x50(%rsp)
   5c3fd:	mov    %rdx,0x58(%rsp)
   5c402:	mov    %rsp,%rdi
   5c405:	lea    0x50(%rsp),%rsi
   5c40a:	mov    $0xffffffa0,%edx
   5c40f:	call   *0x12a1d3(%rip)        # 1865e8 <_DYNAMIC+0x260>
   5c415:	mov    (%rsp),%rsi
   5c419:	cmp    $0xffffffffffffffff,%rsi
   5c41d:	je     5dae6 <probe::main+0x1a06>
   5c423:	test   %rsi,%rsi
   5c426:	je     5c43c <probe::main+0x35c>
   5c428:	mov    0x8(%rsp),%rdi
   5c42d:	shl    $0x3,%rsi
   5c431:	mov    $0x8,%edx
   5c436:	call   *0x12a18c(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5c43c:	lea    0x58(%rsp),%rbx
   5c441:	mov    0x50(%rsp),%rax
   5c446:	lock decq (%rax)
   5c44a:	jne    5c457 <probe::main+0x377>
   5c44c:	lea    0x50(%rsp),%rdi
   5c451:	call   *0x12a181(%rip)        # 1865d8 <_DYNAMIC+0x250>
   5c457:	mov    0x58(%rsp),%rax
   5c45c:	test   %rax,%rax
   5c45f:	je     5c470 <probe::main+0x390>
   5c461:	lock decq (%rax)
   5c465:	jne    5c470 <probe::main+0x390>
   5c467:	mov    %rbx,%rdi
   5c46a:	call   *0x12a170(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5c470:	call   *0x12a1b2(%rip)        # 186628 <_DYNAMIC+0x2a0>
   5c476:	mov    %rax,0x30(%rsp)
   5c47b:	mov    %rdx,0x38(%rsp)
   5c480:	mov    %rsp,%rdi
   5c483:	lea    0x30(%rsp),%rsi
   5c488:	xor    %edx,%edx
   5c48a:	call   *0x12a158(%rip)        # 1865e8 <_DYNAMIC+0x260>
   5c490:	mov    (%rsp),%rax
   5c494:	cmp    $0xffffffffffffffff,%rax
   5c498:	je     5db19 <probe::main+0x1a39>
   5c49e:	mov    0x8(%rsp),%rcx
   5c4a3:	mov    0x10(%rsp),%rsi
   5c4a8:	movzbl 0x18(%rsp),%ebx
   5c4ad:	test   %rsi,%rsi
   5c4b0:	je     5c507 <probe::main+0x427>
   5c4b2:	mov    0xc0(%rsp),%rdx
   5c4ba:	dec    %rdx
   5c4bd:	mov    %edx,%edi
   5c4bf:	and    $0x3f,%dil
   5c4c3:	shr    $0x6,%rdx
   5c4c7:	mov    %rax,(%rsp)
   5c4cb:	mov    %rcx,0x8(%rsp)
   5c4d0:	mov    %rsi,0x10(%rsp)
   5c4d5:	movzbl %dil,%ecx
   5c4d9:	lea    0x50(%rsp),%rdi
   5c4de:	mov    %rsp,%rsi
   5c4e1:	call   *0x12a149(%rip)        # 186630 <_DYNAMIC+0x2a8>
   5c4e7:	cmp    $0x1,%bl
   5c4ea:	jne    5c51f <probe::main+0x43f>
   5c4ec:	mov    0x50(%rsp),%rsi
   5c4f1:	mov    $0x1,%bpl
   5c4f4:	cmp    $0x4,%rsi
   5c4f8:	jae    5c541 <probe::main+0x461>
   5c4fa:	mov    %rsi,0xf0(%rsp)
   5c502:	xor    %r14d,%r14d
   5c505:	jmp    5c56d <probe::main+0x48d>
   5c507:	mov    %rax,0x50(%rsp)
   5c50c:	mov    %rcx,0x58(%rsp)
   5c511:	movq   $0x0,0x60(%rsp)
   5c51a:	cmp    $0x1,%bl
   5c51d:	je     5c4ec <probe::main+0x40c>
   5c51f:	mov    0x50(%rsp),%rax
   5c524:	mov    %rax,0xf0(%rsp)
   5c52c:	mov    0x60(%rsp),%r14
   5c531:	test   %r14,%r14
   5c534:	movzbl %bl,%eax
   5c537:	mov    $0x1,%ebp
   5c53c:	cmovne %eax,%ebp
   5c53f:	jmp    5c56d <probe::main+0x48d>
   5c541:	mov    0x58(%rsp),%rdi
   5c546:	shl    $0x3,%rsi
   5c54a:	mov    $0x8,%edx
   5c54f:	call   *0x12a073(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5c555:	movq   $0x8,0x58(%rsp)
   5c55e:	xor    %r14d,%r14d
   5c561:	movq   $0x0,0xf0(%rsp)
   5c56d:	lea    0x38(%rsp),%rbx
   5c572:	mov    0x58(%rsp),%rax
   5c577:	mov    %rax,0xf8(%rsp)
   5c57f:	mov    0x30(%rsp),%rax
   5c584:	lock decq (%rax)
   5c588:	jne    5c595 <probe::main+0x4b5>
   5c58a:	lea    0x30(%rsp),%rdi
   5c58f:	call   *0x12a043(%rip)        # 1865d8 <_DYNAMIC+0x250>
   5c595:	mov    0x38(%rsp),%rax
   5c59a:	test   %rax,%rax
   5c59d:	je     5c5ae <probe::main+0x4ce>
   5c59f:	lock decq (%rax)
   5c5a3:	jne    5c5ae <probe::main+0x4ce>
   5c5a5:	mov    %rbx,%rdi
   5c5a8:	call   *0x12a032(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5c5ae:	lea    0x0(,%r14,8),%r13
   5c5b6:	test   %r14,%r14
   5c5b9:	je     5c5f1 <probe::main+0x511>
   5c5bb:	call   *0x12a077(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5c5c1:	mov    $0x8,%esi
   5c5c6:	mov    %r13,%rdi
   5c5c9:	call   *0x12a071(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5c5cf:	test   %rax,%rax
   5c5d2:	je     5dcc1 <probe::main+0x1be1>
   5c5d8:	mov    %rax,%r12
   5c5db:	mov    %rax,%rdi
   5c5de:	mov    0xf8(%rsp),%rsi
   5c5e6:	mov    %r13,%rdx
   5c5e9:	call   *0x12a059(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5c5ef:	jmp    5c5f7 <probe::main+0x517>
   5c5f1:	mov    $0x8,%r12d
   5c5f7:	movzbl %bpl,%eax
   5c5fb:	test   %al,%al
   5c5fd:	mov    %ebp,0x98(%rsp)
   5c604:	mov    %eax,0xa8(%rsp)
   5c60b:	je     5c697 <probe::main+0x5b7>
   5c611:	lea    0x8(%rsp),%rbx
   5c616:	cmp    $0x1,%eax
   5c619:	jne    5c6c0 <probe::main+0x5e0>
   5c61f:	mov    %r13,0xa0(%rsp)
   5c627:	movups -0x2997e(%rip),%xmm0        # 32cb0 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.17861709745643738061+0x100>
   5c62e:	movaps %xmm0,(%rsp)
   5c632:	movq   $0x0,0x10(%rsp)
   5c63b:	mov    %rsp,%rdi
   5c63e:	call   *0x12a00c(%rip)        # 186650 <_DYNAMIC+0x2c8>
   5c644:	mov    0x8(%rsp),%rax
   5c649:	movq   $0x1,(%rax)
   5c650:	mov    (%rsp),%r13
   5c654:	mov    0x8(%rsp),%rbx
   5c659:	mov    $0x1,%ebp
   5c65e:	movl   $0x0,0x88(%rsp)
   5c669:	test   %r14,%r14
   5c66c:	je     5c684 <probe::main+0x5a4>
   5c66e:	mov    $0x8,%edx
   5c673:	mov    %r12,%rdi
   5c676:	lea    0x0(,%r14,8),%rsi
   5c67e:	call   *0x129f44(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5c684:	mov    %rbx,%r12
   5c687:	mov    %r13,%rbx
   5c68a:	mov    0x88(%rsp),%r13d
   5c692:	jmp    5ca80 <probe::main+0x9a0>
   5c697:	mov    %r14,(%rsp)
   5c69b:	mov    %r12,0x8(%rsp)
   5c6a0:	mov    %r14,0x10(%rsp)
   5c6a5:	mov    %r14,%rax
   5c6a8:	test   %r14,%r14
   5c6ab:	je     5c7a9 <probe::main+0x6c9>
   5c6b1:	incq   (%r12)
   5c6b5:	je     5c7d7 <probe::main+0x6f7>
   5c6bb:	jmp    5c806 <probe::main+0x726>
   5c6c0:	movups -0x29a17(%rip),%xmm0        # 32cb0 <anon.7b75c566c6a805bace2b01974d435009.343.llvm.17861709745643738061+0x100>
   5c6c7:	movaps %xmm0,(%rsp)
   5c6cb:	movq   $0x0,0x10(%rsp)
   5c6d4:	mov    %rsp,%rdi
   5c6d7:	call   *0x129f73(%rip)        # 186650 <_DYNAMIC+0x2c8>
   5c6dd:	cmp    $0x2,%r14
   5c6e1:	setae  %al
   5c6e4:	cmp    $0x1,%r14
   5c6e8:	sbb    $0x0,%al
   5c6ea:	mov    0x8(%rsp),%rcx
   5c6ef:	movq   $0x1,(%rcx)
   5c6f6:	mov    (%rsp),%rsi
   5c6fa:	mov    0x8(%rsp),%rdi
   5c6ff:	cmp    $0x1,%r14
   5c703:	jne    5c713 <probe::main+0x633>
   5c705:	mov    (%rdi),%rax
   5c708:	cmp    %rax,(%r12)
   5c70c:	je     5c76e <probe::main+0x68e>
   5c70e:	seta   %al
   5c711:	sbb    $0x0,%al
   5c713:	test   %al,%al
   5c715:	je     5c76e <probe::main+0x68e>
   5c717:	movzbl %al,%eax
   5c71a:	cmp    $0x1,%eax
   5c71d:	mov    %rsi,0x80(%rsp)
   5c725:	mov    %rdi,0xb8(%rsp)
   5c72d:	jne    5c8a6 <probe::main+0x7c6>
   5c733:	mov    %r14,(%rsp)
   5c737:	mov    %r12,0x8(%rsp)
   5c73c:	mov    %r14,0x10(%rsp)
   5c741:	mov    %rsp,%rdi
   5c744:	mov    $0x1,%esi
   5c749:	call   *0x129f09(%rip)        # 186658 <_DYNAMIC+0x2d0>
   5c74f:	mov    (%rsp),%rbx
   5c753:	mov    0x8(%rsp),%r12
   5c758:	mov    0x10(%rsp),%rbp
   5c75d:	mov    $0x2,%r13b
   5c760:	test   %rbp,%rbp
   5c763:	jne    5ca5c <probe::main+0x97c>
   5c769:	jmp    5c9f5 <probe::main+0x915>
   5c76e:	movl   $0x0,0x7b(%rsp)
   5c776:	movl   $0x0,0x78(%rsp)
   5c77e:	mov    $0x1,%al
   5c780:	mov    %eax,0x88(%rsp)
   5c787:	test   %rsi,%rsi
   5c78a:	je     5c8f2 <probe::main+0x812>
   5c790:	shl    $0x3,%rsi
   5c794:	mov    $0x8,%ebx
   5c799:	mov    $0x8,%edx
   5c79e:	call   *0x129e24(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5c7a4:	jmp    5c8f7 <probe::main+0x817>
   5c7a9:	mov    %rsp,%rdi
   5c7ac:	call   *0x129e9e(%rip)        # 186650 <_DYNAMIC+0x2c8>
   5c7b2:	mov    0x8(%rsp),%rax
   5c7b7:	movq   $0x0,(%rax)
   5c7be:	movq   $0x1,0x10(%rsp)
   5c7c7:	mov    $0x1,%eax
   5c7cc:	mov    0x8(%rsp),%r12
   5c7d1:	incq   (%r12)
   5c7d5:	jne    5c806 <probe::main+0x726>
   5c7d7:	shl    $0x3,%rax
   5c7db:	mov    $0x1,%bl
   5c7dd:	mov    $0x8,%ecx
   5c7e2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5c7f0:	cmp    %rcx,%rax
   5c7f3:	je     5c834 <probe::main+0x754>
   5c7f5:	add    $0xff,%bl
   5c7f8:	adcq   $0x0,(%r12,%rcx,1)
   5c7fd:	lea    0x8(%rcx),%rcx
   5c801:	setb   %bl
   5c804:	jb     5c7f0 <probe::main+0x710>
   5c806:	mov    0x10(%rsp),%rbp
   5c80b:	mov    (%rsp),%rbx
   5c80f:	mov    0x8(%rsp),%r12
   5c814:	test   %rbp,%rbp
   5c817:	je     5c86b <probe::main+0x78b>
   5c819:	xor    %r13d,%r13d
   5c81c:	jmp    5ca80 <probe::main+0x9a0>
   5c821:	mov    $0x1,%al
   5c823:	mov    %eax,0x48(%rsp)
   5c827:	movl   $0x0,0x4c(%rsp)
   5c82f:	jmp    5c3ea <probe::main+0x30a>
   5c834:	mov    0x10(%rsp),%rax
   5c839:	mov    %rax,%r12
   5c83c:	cmp    (%rsp),%rax
   5c840:	jne    5c84b <probe::main+0x76b>
   5c842:	mov    %rsp,%rdi
   5c845:	call   *0x129e05(%rip)        # 186650 <_DYNAMIC+0x2c8>
   5c84b:	movzbl %bl,%eax
   5c84e:	mov    0x8(%rsp),%rcx
   5c853:	mov    %r12,%rbp
   5c856:	mov    %rax,(%rcx,%r12,8)
   5c85a:	inc    %rbp
   5c85d:	mov    (%rsp),%rbx
   5c861:	mov    0x8(%rsp),%r12
   5c866:	test   %rbp,%rbp
   5c869:	jne    5c819 <probe::main+0x739>
   5c86b:	movl   $0x0,0x7b(%rsp)
   5c873:	movl   $0x0,0x78(%rsp)
   5c87b:	mov    $0x1,%r13b
   5c87e:	test   %rbx,%rbx
   5c881:	je     5c90a <probe::main+0x82a>
   5c887:	shl    $0x3,%rbx
   5c88b:	mov    $0x8,%edx
   5c890:	mov    %r12,%rdi
   5c893:	mov    %rbx,%rsi
   5c896:	call   *0x129d2c(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5c89c:	xor    %ebx,%ebx
   5c89e:	mov    $0x8,%r12d
   5c8a4:	jmp    5c912 <probe::main+0x832>
   5c8a6:	mov    %r14,(%rsp)
   5c8aa:	mov    %r12,0x8(%rsp)
   5c8af:	mov    %r14,0x10(%rsp)
   5c8b4:	test   %r14,%r14
   5c8b7:	je     5c919 <probe::main+0x839>
   5c8b9:	movq   $0x1,0x50(%rsp)
   5c8c2:	lea    0x50(%rsp),%rdi
   5c8c7:	mov    $0x1,%esi
   5c8cc:	mov    %r12,%rdx
   5c8cf:	mov    %r14,%rcx
   5c8d2:	call   *0x129d88(%rip)        # 186660 <_DYNAMIC+0x2d8>
   5c8d8:	mov    0x10(%rsp),%rbp
   5c8dd:	mov    (%rsp),%rbx
   5c8e1:	mov    0x8(%rsp),%r12
   5c8e6:	test   %rbp,%rbp
   5c8e9:	jne    5c945 <probe::main+0x865>
   5c8eb:	xor    %ebp,%ebp
   5c8ed:	jmp    5c995 <probe::main+0x8b5>
   5c8f2:	mov    $0x8,%ebx
   5c8f7:	xor    %r13d,%r13d
   5c8fa:	xor    %ebp,%ebp
   5c8fc:	test   %r14,%r14
   5c8ff:	jne    5c66e <probe::main+0x58e>
   5c905:	jmp    5c684 <probe::main+0x5a4>
   5c90a:	mov    $0x8,%r12d
   5c910:	xor    %ebx,%ebx
   5c912:	xor    %ebp,%ebp
   5c914:	jmp    5ca80 <probe::main+0x9a0>
   5c919:	mov    %rsp,%rdi
   5c91c:	call   *0x129d2e(%rip)        # 186650 <_DYNAMIC+0x2c8>
   5c922:	mov    0x8(%rsp),%rax
   5c927:	movq   $0x1,(%rax)
   5c92e:	movq   $0x1,0x10(%rsp)
   5c937:	mov    (%rsp),%rbx
   5c93b:	mov    0x8(%rsp),%r12
   5c940:	mov    $0x1,%ebp
   5c945:	cmpq   $0x0,-0x8(%r12,%rbp,8)
   5c94b:	jne    5c995 <probe::main+0x8b5>
   5c94d:	movabs $0x1fffffffffffffff,%rax
   5c957:	and    %rbp,%rax
   5c95a:	lea    0x0(,%rbp,8),%rdx
   5c962:	xor    %ecx,%ecx
   5c964:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5c970:	test   %rdx,%rdx
   5c973:	je     5c8eb <probe::main+0x80b>
   5c979:	inc    %rcx
   5c97c:	cmpq   $0x0,-0x8(%r12,%rdx,1)
   5c982:	lea    -0x8(%rdx),%rdx
   5c986:	je     5c970 <probe::main+0x890>
   5c988:	sub    %rcx,%rax
   5c98b:	inc    %rax
   5c98e:	cmp    %rbp,%rax
   5c991:	cmovb  %rax,%rbp
   5c995:	mov    %rbx,%rax
   5c998:	shr    $0x2,%rax
   5c99c:	cmp    %rax,%rbp
   5c99f:	jae    5c9eb <probe::main+0x90b>
   5c9a1:	cmp    %rbx,%rbp
   5c9a4:	ja     5dc4e <probe::main+0x1b6e>
   5c9aa:	test   %rbx,%rbx
   5c9ad:	je     5c9eb <probe::main+0x90b>
   5c9af:	shl    $0x3,%rbx
   5c9b3:	test   %rbp,%rbp
   5c9b6:	je     5ca2e <probe::main+0x94e>
   5c9b8:	mov    %r12,%r13
   5c9bb:	lea    0x0(,%rbp,8),%r12
   5c9c3:	mov    $0x8,%edx
   5c9c8:	mov    %r13,%rdi
   5c9cb:	mov    %rbx,%rsi
   5c9ce:	mov    %r12,%rcx
   5c9d1:	call   *0x129c91(%rip)        # 186668 <_DYNAMIC+0x2e0>
   5c9d7:	test   %rax,%rax
   5c9da:	je     5dd01 <probe::main+0x1c21>
   5c9e0:	xor    %r13d,%r13d
   5c9e3:	mov    %rbp,%rbx
   5c9e6:	mov    %rax,%r12
   5c9e9:	jmp    5ca5c <probe::main+0x97c>
   5c9eb:	test   %rbp,%rbp
   5c9ee:	je     5c9f5 <probe::main+0x915>
   5c9f0:	xor    %r13d,%r13d
   5c9f3:	jmp    5ca5c <probe::main+0x97c>
   5c9f5:	movl   $0x0,0x7b(%rsp)
   5c9fd:	movl   $0x0,0x78(%rsp)
   5ca05:	mov    $0x1,%r13b
   5ca08:	test   %rbx,%rbx
   5ca0b:	je     5ca24 <probe::main+0x944>
   5ca0d:	shl    $0x3,%rbx
   5ca11:	mov    $0x8,%edx
   5ca16:	mov    %r12,%rdi
   5ca19:	mov    %rbx,%rsi
   5ca1c:	call   *0x129ba6(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5ca22:	jmp    5ca52 <probe::main+0x972>
   5ca24:	mov    $0x8,%r12d
   5ca2a:	xor    %ebx,%ebx
   5ca2c:	jmp    5ca5a <probe::main+0x97a>
   5ca2e:	mov    $0x8,%edx
   5ca33:	mov    %r12,%rdi
   5ca36:	mov    %rbx,%rsi
   5ca39:	call   *0x129b89(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5ca3f:	movl   $0x0,0x78(%rsp)
   5ca47:	movl   $0x0,0x7b(%rsp)
   5ca4f:	mov    $0x1,%r13b
   5ca52:	xor    %ebx,%ebx
   5ca54:	mov    $0x8,%r12d
   5ca5a:	xor    %ebp,%ebp
   5ca5c:	mov    0x80(%rsp),%rsi
   5ca64:	test   %rsi,%rsi
   5ca67:	je     5ca80 <probe::main+0x9a0>
   5ca69:	shl    $0x3,%rsi
   5ca6d:	mov    $0x8,%edx
   5ca72:	mov    0xb8(%rsp),%rdi
   5ca7a:	call   *0x129b48(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5ca80:	mov    $0x2,%al
   5ca82:	sub    %r13b,%al
   5ca85:	movzbl %al,%eax
   5ca88:	movzbl %r13b,%ecx
   5ca8c:	cmpb   $0x0,0x4c(%rsp)
   5ca91:	cmovne %eax,%ecx
   5ca94:	mov    %rbx,(%rsp)
   5ca98:	mov    %r12,0x88(%rsp)
   5caa0:	mov    %r12,0x8(%rsp)
   5caa5:	mov    %rbp,0x10(%rsp)
   5caaa:	mov    %cl,0x18(%rsp)
   5caae:	mov    0x78(%rsp),%eax
   5cab2:	mov    0x7b(%rsp),%ecx
   5cab6:	mov    %eax,0x19(%rsp)
   5caba:	mov    %ecx,0x1c(%rsp)
   5cabe:	mov    $0x8,%r12d
   5cac4:	cmpb   $0x1,0x98(%rsp)
   5cacc:	je     5cb22 <probe::main+0xa42>
   5cace:	mov    %rbx,%r13
   5cad1:	cmpl   $0x2,0xa8(%rsp)
   5cad9:	jne    5dc20 <probe::main+0x1b40>
   5cadf:	test   %r14,%r14
   5cae2:	je     5cb22 <probe::main+0xa42>
   5cae4:	call   *0x129b4e(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5caea:	mov    $0x8,%esi
   5caef:	lea    0x0(,%r14,8),%rbx
   5caf7:	mov    %rbx,%rdi
   5cafa:	call   *0x129b40(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5cb00:	test   %rax,%rax
   5cb03:	je     5dcd1 <probe::main+0x1bf1>
   5cb09:	mov    %rax,%r12
   5cb0c:	mov    %rax,%rdi
   5cb0f:	mov    0xf8(%rsp),%rsi
   5cb17:	mov    %rbx,%rdx
   5cb1a:	call   *0x129b28(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5cb20:	jmp    5cb25 <probe::main+0xa45>
   5cb22:	xor    %r14d,%r14d
   5cb25:	mov    %r14,0x50(%rsp)
   5cb2a:	mov    %r12,0x58(%rsp)
   5cb2f:	mov    %r14,0x60(%rsp)
   5cb34:	lea    0x30(%rsp),%rdi
   5cb39:	mov    %rsp,%rsi
   5cb3c:	lea    0x50(%rsp),%rdx
   5cb41:	call   *0x129b29(%rip)        # 186670 <_DYNAMIC+0x2e8>
   5cb47:	cmpb   $0x1,0x30(%rsp)
   5cb4c:	je     5db4c <probe::main+0x1a6c>
   5cb52:	mov    0x38(%rsp),%rax
   5cb57:	mov    %rax,0xb0(%rsp)
   5cb5f:	lea    0xb0(%rsp),%rcx
   5cb67:	mov    %rcx,0x148(%rsp)
   5cb6f:	lock incq (%rax)
   5cb73:	jle    5dd12 <probe::main+0x1c32>
   5cb79:	mov    0xb0(%rsp),%rdi
   5cb81:	call   *0x129a69(%rip)        # 1865f0 <_DYNAMIC+0x268>
   5cb87:	mov    %rax,%rdi
   5cb8a:	mov    %rdx,%rsi
   5cb8d:	call   *0x129a65(%rip)        # 1865f8 <_DYNAMIC+0x270>
   5cb93:	mov    %rax,0xe0(%rsp)
   5cb9b:	mov    %rdx,0xe8(%rsp)
   5cba3:	mov    %rsp,%rdi
   5cba6:	lea    0xe0(%rsp),%rsi
   5cbae:	mov    0x70(%rsp),%edx
   5cbb2:	call   *0x129a30(%rip)        # 1865e8 <_DYNAMIC+0x260>
   5cbb8:	cmpq   $0xffffffffffffffff,(%rsp)
   5cbbd:	je     5db7c <probe::main+0x1a9c>
   5cbc3:	movups (%rsp),%xmm0
   5cbc7:	movups 0x10(%rsp),%xmm1
   5cbcc:	movaps %xmm1,0x60(%rsp)
   5cbd1:	movaps %xmm0,0x50(%rsp)
   5cbd6:	movq   $0x0,0x30(%rsp)
   5cbdf:	movq   $0x1,0x38(%rsp)
   5cbe8:	movq   $0x0,0x40(%rsp)
   5cbf1:	movq   $0x60000020,0x10(%rsp)
   5cbfa:	lea    0x30(%rsp),%rax
   5cbff:	mov    %rax,(%rsp)
   5cc03:	lea    0x12531e(%rip),%rax        # 181f28 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x78>
   5cc0a:	mov    %rax,0x8(%rsp)
   5cc0f:	lea    0x50(%rsp),%rdi
   5cc14:	mov    %rsp,%rsi
   5cc17:	call   *0x129a5b(%rip)        # 186678 <_DYNAMIC+0x2f0>
   5cc1d:	test   %al,%al
   5cc1f:	jne    5dbaf <probe::main+0x1acf>
   5cc25:	mov    0x30(%rsp),%rax
   5cc2a:	mov    %rax,0xb8(%rsp)
   5cc32:	mov    0x38(%rsp),%rax
   5cc37:	mov    %rax,0x80(%rsp)
   5cc3f:	mov    0x40(%rsp),%rbx
   5cc44:	mov    0x50(%rsp),%rsi
   5cc49:	test   %rsi,%rsi
   5cc4c:	je     5cc62 <probe::main+0xb82>
   5cc4e:	mov    0x58(%rsp),%rdi
   5cc53:	shl    $0x3,%rsi
   5cc57:	mov    $0x8,%edx
   5cc5c:	call   *0x129966(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5cc62:	test   %r15,%r15
   5cc65:	je     5cdab <probe::main+0xccb>
   5cc6b:	test   %rbx,%rbx
   5cc6e:	je     5cdb0 <probe::main+0xcd0>
   5cc74:	mov    0x80(%rsp),%rax
   5cc7c:	add    %rbx,%rax
   5cc7f:	mov    %ebx,%ecx
   5cc81:	and    $0x7,%ecx
   5cc84:	xor    %r12d,%r12d
   5cc87:	xor    %edx,%edx
   5cc89:	jmp    5ccad <probe::main+0xbcd>
   5cc8b:	nopl   0x0(%rax,%rax,1)
   5cc90:	inc    %rdx
   5cc93:	mov    %rdi,%r12
   5cc96:	shl    $0x5,%r12
   5cc9a:	sub    %rdi,%r12
   5cc9d:	add    $0xff,%r12
   5cca4:	cmp    %r15,%rdx
   5cca7:	je     5ce16 <probe::main+0xd36>
   5ccad:	test   %rcx,%rcx
   5ccb0:	je     5ccf0 <probe::main+0xc10>
   5ccb2:	mov    %rcx,%r8
   5ccb5:	mov    0x80(%rsp),%rsi
   5ccbd:	mov    %r12,%rdi
   5ccc0:	movzbl (%rsi),%r9d
   5ccc4:	inc    %rsi
   5ccc7:	shl    $0x5,%rdi
   5cccb:	sub    %r12,%rdi
   5ccce:	add    %r9,%rdi
   5ccd1:	mov    %rdi,%r12
   5ccd4:	dec    %r8
   5ccd7:	jne    5ccc0 <probe::main+0xbe0>
   5ccd9:	cmp    $0x8,%rbx
   5ccdd:	jae    5cd10 <probe::main+0xc30>
   5ccdf:	jmp    5cc90 <probe::main+0xbb0>
   5cce1:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5ccf0:	mov    %r12,%rdi
   5ccf3:	mov    0x80(%rsp),%rsi
   5ccfb:	cmp    $0x8,%rbx
   5ccff:	jb     5cc90 <probe::main+0xbb0>
   5cd01:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5cd10:	movzbl (%rsi),%r8d
   5cd14:	mov    %rdi,%r9
   5cd17:	shl    $0x5,%r9
   5cd1b:	sub    %rdi,%r9
   5cd1e:	add    %r8,%r9
   5cd21:	movzbl 0x1(%rsi),%edi
   5cd25:	mov    %r9,%r8
   5cd28:	shl    $0x5,%r8
   5cd2c:	sub    %r9,%r8
   5cd2f:	add    %rdi,%r8
   5cd32:	movzbl 0x2(%rsi),%edi
   5cd36:	mov    %r8,%r9
   5cd39:	shl    $0x5,%r9
   5cd3d:	sub    %r8,%r9
   5cd40:	add    %rdi,%r9
   5cd43:	movzbl 0x3(%rsi),%edi
   5cd47:	mov    %r9,%r8
   5cd4a:	shl    $0x5,%r8
   5cd4e:	sub    %r9,%r8
   5cd51:	add    %rdi,%r8
   5cd54:	movzbl 0x4(%rsi),%edi
   5cd58:	mov    %r8,%r9
   5cd5b:	shl    $0x5,%r9
   5cd5f:	sub    %r8,%r9
   5cd62:	add    %rdi,%r9
   5cd65:	movzbl 0x5(%rsi),%edi
   5cd69:	mov    %r9,%r8
   5cd6c:	shl    $0x5,%r8
   5cd70:	sub    %r9,%r8
   5cd73:	add    %rdi,%r8
   5cd76:	movzbl 0x6(%rsi),%edi
   5cd7a:	mov    %r8,%r9
   5cd7d:	shl    $0x5,%r9
   5cd81:	sub    %r8,%r9
   5cd84:	add    %rdi,%r9
   5cd87:	movzbl 0x7(%rsi),%r8d
   5cd8c:	add    $0x8,%rsi
   5cd90:	mov    %r9,%rdi
   5cd93:	shl    $0x5,%rdi
   5cd97:	sub    %r9,%rdi
   5cd9a:	add    %r8,%rdi
   5cd9d:	cmp    %rax,%rsi
   5cda0:	jne    5cd10 <probe::main+0xc30>
   5cda6:	jmp    5cc90 <probe::main+0xbb0>
   5cdab:	xor    %r12d,%r12d
   5cdae:	jmp    5ce16 <probe::main+0xd36>
   5cdb0:	mov    %r15d,%eax
   5cdb3:	and    $0x7,%eax
   5cdb6:	cmp    $0x8,%r15
   5cdba:	jae    5cdc1 <probe::main+0xce1>
   5cdbc:	xor    %r12d,%r12d
   5cdbf:	jmp    5cdf2 <probe::main+0xd12>
   5cdc1:	mov    %r15,%rcx
   5cdc4:	and    $0xfffffffffffffff8,%rcx
   5cdc8:	xor    %r12d,%r12d
   5cdcb:	movabs $0xc694446f01,%rdx
   5cdd5:	movabs $0x697ec45af80,%rsi
   5cddf:	nop
   5cde0:	imul   %rdx,%r12
   5cde4:	add    %rsi,%r12
   5cde7:	add    $0xfffffffffffffff8,%rcx
   5cdeb:	jne    5cde0 <probe::main+0xd00>
   5cded:	test   %rax,%rax
   5cdf0:	je     5ce16 <probe::main+0xd36>
   5cdf2:	mov    %r12,%rcx
   5cdf5:	data16 cs nopw 0x0(%rax,%rax,1)
   5ce00:	shl    $0x5,%r12
   5ce04:	sub    %rcx,%r12
   5ce07:	add    $0xff,%r12
   5ce0e:	mov    %r12,%rcx
   5ce11:	dec    %rax
   5ce14:	jne    5ce00 <probe::main+0xd20>
   5ce16:	mov    %r12,0x150(%rsp)
   5ce1e:	cmpb   $0x0,0x48(%rsp)
   5ce23:	mov    %rbx,0xa0(%rsp)
   5ce2b:	je     5ce84 <probe::main+0xda4>
   5ce2d:	call   *0x129805(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5ce33:	mov    $0x10,%edi
   5ce38:	mov    $0x8,%esi
   5ce3d:	call   *0x1297fd(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5ce43:	test   %rax,%rax
   5ce46:	je     5dc32 <probe::main+0x1b52>
   5ce4c:	movaps 0xe0(%rsp),%xmm0
   5ce54:	movups %xmm0,(%rax)
   5ce57:	movq   $0x1,0xc8(%rsp)
   5ce63:	mov    %rax,0xd0(%rsp)
   5ce6b:	movq   $0x1,0xd8(%rsp)
   5ce77:	movl   $0x0,0x94(%rsp)
   5ce82:	jmp    5ceb7 <probe::main+0xdd7>
   5ce84:	movq   $0x0,0x8(%rsp)
   5ce8d:	mov    %r15,0x10(%rsp)
   5ce92:	lea    0x148(%rsp),%rax
   5ce9a:	mov    %rax,(%rsp)
   5ce9e:	lea    0xc8(%rsp),%rdi
   5cea6:	mov    %rsp,%rsi
   5cea9:	call   60610 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<hyperreal::computable::node::Computable, core::iter::adapters::map::Map<core::ops::range::Range<usize>, probe::main::{closure#2}>>>::from_iter>
   5ceae:	mov    $0x1,%al
   5ceb0:	mov    %eax,0x94(%rsp)
   5ceb7:	movq   $0x0,0x138(%rsp)
   5cec3:	movq   $0x0,0x140(%rsp)
   5cecf:	call   *0x1297ab(%rip)        # 186680 <_DYNAMIC+0x2f8>
   5ced5:	mov    %rax,0x158(%rsp)
   5cedd:	mov    %edx,0x160(%rsp)
   5cee4:	test   %r15,%r15
   5cee7:	je     5d166 <probe::main+0x1086>
   5ceed:	mov    0xd0(%rsp),%rax
   5cef5:	mov    %rax,0x88(%rsp)
   5cefd:	mov    0xd8(%rsp),%rax
   5cf05:	mov    %rax,0xa8(%rsp)
   5cf0d:	xor    %r14d,%r14d
   5cf10:	mov    %rsp,%r13
   5cf13:	lea    0x50(%rsp),%rbp
   5cf18:	movq   $0x0,0x98(%rsp)
   5cf24:	xor    %ebx,%ebx
   5cf26:	jmp    5cf3c <probe::main+0xe5c>
   5cf28:	nopl   0x0(%rax,%rax,1)
   5cf30:	inc    %rbx
   5cf33:	cmp    %r15,%rbx
   5cf36:	je     5d175 <probe::main+0x1095>
   5cf3c:	cmpb   $0x0,0x48(%rsp)
   5cf41:	mov    $0x0,%esi
   5cf46:	cmove  %rbx,%rsi
   5cf4a:	cmp    0xa8(%rsp),%rsi
   5cf52:	jae    5dc72 <probe::main+0x1b92>
   5cf58:	shl    $0x4,%rsi
   5cf5c:	add    0x88(%rsp),%rsi
   5cf64:	mov    %r13,%rdi
   5cf67:	mov    0x70(%rsp),%edx
   5cf6b:	call   *0x129677(%rip)        # 1865e8 <_DYNAMIC+0x260>
   5cf71:	cmpq   $0xffffffffffffffff,(%rsp)
   5cf76:	je     5da54 <probe::main+0x1974>
   5cf7c:	movups (%rsp),%xmm0
   5cf80:	movups 0x10(%rsp),%xmm1
   5cf85:	movaps %xmm1,0x60(%rsp)
   5cf8a:	movaps %xmm0,0x50(%rsp)
   5cf8f:	movq   $0x0,0x30(%rsp)
   5cf98:	movq   $0x1,0x38(%rsp)
   5cfa1:	movq   $0x0,0x40(%rsp)
   5cfaa:	movq   $0x60000020,0x10(%rsp)
   5cfb3:	lea    0x30(%rsp),%rax
   5cfb8:	mov    %rax,(%rsp)
   5cfbc:	lea    0x124f65(%rip),%rax        # 181f28 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x78>
   5cfc3:	mov    %rax,0x8(%rsp)
   5cfc8:	mov    %rbp,%rdi
   5cfcb:	mov    %r13,%rsi
   5cfce:	call   *0x1296a4(%rip)        # 186678 <_DYNAMIC+0x2f0>
   5cfd4:	test   %al,%al
   5cfd6:	jne    5da87 <probe::main+0x19a7>
   5cfdc:	mov    0x30(%rsp),%rsi
   5cfe1:	mov    0x38(%rsp),%rdi
   5cfe6:	mov    0x40(%rsp),%rax
   5cfeb:	mov    0x98(%rsp),%rdx
   5cff3:	add    %rax,%rdx
   5cff6:	mov    %rdx,0x98(%rsp)
   5cffe:	mov    %rdx,0x140(%rsp)
   5d006:	test   %rax,%rax
   5d009:	je     5d050 <probe::main+0xf70>
   5d00b:	mov    %rax,%r8
   5d00e:	and    $0x7,%r8
   5d012:	je     5d060 <probe::main+0xf80>
   5d014:	mov    %rdi,%rcx
   5d017:	mov    %r14,%rdx
   5d01a:	nopw   0x0(%rax,%rax,1)
   5d020:	movzbl (%rcx),%r9d
   5d024:	inc    %rcx
   5d027:	shl    $0x5,%rdx
   5d02b:	sub    %r14,%rdx
   5d02e:	add    %r9,%rdx
   5d031:	mov    %rdx,%r14
   5d034:	dec    %r8
   5d037:	jne    5d020 <probe::main+0xf40>
   5d039:	cmp    $0x8,%rax
   5d03d:	jae    5d070 <probe::main+0xf90>
   5d03f:	jmp    5d116 <probe::main+0x1036>
   5d044:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5d050:	mov    %r14,%rdx
   5d053:	jmp    5d116 <probe::main+0x1036>
   5d058:	nopl   0x0(%rax,%rax,1)
   5d060:	mov    %r14,%rdx
   5d063:	mov    %rdi,%rcx
   5d066:	cmp    $0x8,%rax
   5d06a:	jb     5d116 <probe::main+0x1036>
   5d070:	add    %rdi,%rax
   5d073:	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5d080:	movzbl (%rcx),%r8d
   5d084:	mov    %rdx,%r9
   5d087:	shl    $0x5,%r9
   5d08b:	sub    %rdx,%r9
   5d08e:	add    %r8,%r9
   5d091:	movzbl 0x1(%rcx),%edx
   5d095:	mov    %r9,%r8
   5d098:	shl    $0x5,%r8
   5d09c:	sub    %r9,%r8
   5d09f:	add    %rdx,%r8
   5d0a2:	movzbl 0x2(%rcx),%edx
   5d0a6:	mov    %r8,%r9
   5d0a9:	shl    $0x5,%r9
   5d0ad:	sub    %r8,%r9
   5d0b0:	add    %rdx,%r9
   5d0b3:	movzbl 0x3(%rcx),%edx
   5d0b7:	mov    %r9,%r8
   5d0ba:	shl    $0x5,%r8
   5d0be:	sub    %r9,%r8
   5d0c1:	add    %rdx,%r8
   5d0c4:	movzbl 0x4(%rcx),%edx
   5d0c8:	mov    %r8,%r9
   5d0cb:	shl    $0x5,%r9
   5d0cf:	sub    %r8,%r9
   5d0d2:	add    %rdx,%r9
   5d0d5:	movzbl 0x5(%rcx),%edx
   5d0d9:	mov    %r9,%r8
   5d0dc:	shl    $0x5,%r8
   5d0e0:	sub    %r9,%r8
   5d0e3:	add    %rdx,%r8
   5d0e6:	movzbl 0x6(%rcx),%edx
   5d0ea:	mov    %r8,%r9
   5d0ed:	shl    $0x5,%r9
   5d0f1:	sub    %r8,%r9
   5d0f4:	add    %rdx,%r9
   5d0f7:	movzbl 0x7(%rcx),%r8d
   5d0fc:	add    $0x8,%rcx
   5d100:	mov    %r9,%rdx
   5d103:	shl    $0x5,%rdx
   5d107:	sub    %r9,%rdx
   5d10a:	add    %r8,%rdx
   5d10d:	cmp    %rax,%rcx
   5d110:	jne    5d080 <probe::main+0xfa0>
   5d116:	mov    %rdx,%r14
   5d119:	shl    $0x5,%r14
   5d11d:	sub    %rdx,%r14
   5d120:	add    $0xff,%r14
   5d127:	mov    %r14,0x138(%rsp)
   5d12f:	test   %rsi,%rsi
   5d132:	je     5d13f <probe::main+0x105f>
   5d134:	mov    $0x1,%edx
   5d139:	call   *0x129489(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d13f:	mov    0x50(%rsp),%rsi
   5d144:	test   %rsi,%rsi
   5d147:	je     5cf30 <probe::main+0xe50>
   5d14d:	mov    0x58(%rsp),%rdi
   5d152:	shl    $0x3,%rsi
   5d156:	mov    $0x8,%edx
   5d15b:	call   *0x129467(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d161:	jmp    5cf30 <probe::main+0xe50>
   5d166:	movq   $0x0,0x98(%rsp)
   5d172:	xor    %r14d,%r14d
   5d175:	lea    0x158(%rsp),%rdi
   5d17d:	call   *0x129505(%rip)        # 186688 <_DYNAMIC+0x300>
   5d183:	mov    %edx,%ecx
   5d185:	mov    $0x3b9aca00,%edx
   5d18a:	mul    %rdx
   5d18d:	mov    %ecx,%ecx
   5d18f:	add    %rax,%rcx
   5d192:	adc    $0x0,%rdx
   5d196:	cmp    %r12,%r14
   5d199:	jne    5dbd9 <probe::main+0x1af9>
   5d19f:	mov    0xa0(%rsp),%rax
   5d1a7:	imul   %r15,%rax
   5d1ab:	mov    %rax,(%rsp)
   5d1af:	cmp    %rax,0x98(%rsp)
   5d1b7:	mov    0x118(%rsp),%rbx
   5d1bf:	jne    5dbff <probe::main+0x1b1f>
   5d1c5:	mov    %rcx,0x88(%rsp)
   5d1cd:	mov    %rdx,0xa8(%rsp)
   5d1d5:	movq   $0x0,0x100(%rsp)
   5d1e1:	movq   $0x0,0x110(%rsp)
   5d1ed:	call   *0x129445(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d1f3:	mov    $0x4,%ebp
   5d1f8:	mov    $0x4,%edi
   5d1fd:	mov    $0x1,%esi
   5d202:	call   *0x129438(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d208:	test   %rax,%rax
   5d20b:	je     5dc8f <probe::main+0x1baf>
   5d211:	mov    %rax,%r13
   5d214:	movl   $0x65736163,(%rax)
   5d21a:	movq   $0x4,0x30(%rsp)
   5d223:	mov    %rax,0x38(%rsp)
   5d228:	movq   $0x4,0x40(%rsp)
   5d231:	mov    0x10(%rbx),%rbp
   5d235:	test   %rbp,%rbp
   5d238:	jns    5d24a <probe::main+0x116a>
   5d23a:	xor    %edi,%edi
   5d23c:	mov    %rbp,%rsi
   5d23f:	call   *0x12944b(%rip)        # 186690 <_DYNAMIC+0x308>
   5d245:	jmp    5dd12 <probe::main+0x1c32>
   5d24a:	je     5d28f <probe::main+0x11af>
   5d24c:	mov    0x8(%rbx),%rax
   5d250:	mov    %rax,0x118(%rsp)
   5d258:	call   *0x1293da(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d25e:	mov    $0x1,%esi
   5d263:	mov    %rbp,%rdi
   5d266:	call   *0x1293d4(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d26c:	mov    $0x1,%edi
   5d271:	test   %rax,%rax
   5d274:	je     5d23c <probe::main+0x115c>
   5d276:	mov    %rax,%rbx
   5d279:	mov    %rax,%rdi
   5d27c:	mov    0x118(%rsp),%rsi
   5d284:	mov    %rbp,%rdx
   5d287:	call   *0x1293bb(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5d28d:	jmp    5d294 <probe::main+0x11b4>
   5d28f:	mov    $0x1,%ebx
   5d294:	movb   $0x3,(%rsp)
   5d298:	mov    %rbp,0x8(%rsp)
   5d29d:	mov    %rbx,0x10(%rsp)
   5d2a2:	mov    %rbp,0x18(%rsp)
   5d2a7:	lea    0x50(%rsp),%rdi
   5d2ac:	lea    0x100(%rsp),%rsi
   5d2b4:	lea    0x30(%rsp),%rdx
   5d2b9:	mov    %rsp,%rcx
   5d2bc:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d2c1:	cmpb   $0xff,0x50(%rsp)
   5d2c6:	je     5d2d2 <probe::main+0x11f2>
   5d2c8:	lea    0x50(%rsp),%rdi
   5d2cd:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d2d2:	call   *0x129360(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d2d8:	mov    $0xa,%ebp
   5d2dd:	mov    $0xa,%edi
   5d2e2:	mov    $0x1,%esi
   5d2e7:	call   *0x129353(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d2ed:	test   %rax,%rax
   5d2f0:	je     5dc8f <probe::main+0x1baf>
   5d2f6:	movabs $0x6f69746172657469,%rcx
   5d300:	mov    %rcx,(%rax)
   5d303:	movw   $0x736e,0x8(%rax)
   5d309:	movq   $0xa,0x30(%rsp)
   5d312:	mov    %rax,0x38(%rsp)
   5d317:	movq   $0xa,0x40(%rsp)
   5d320:	movb   $0x2,(%rsp)
   5d324:	movq   $0x0,0x8(%rsp)
   5d32d:	mov    %r15,0x10(%rsp)
   5d332:	lea    0x50(%rsp),%rdi
   5d337:	lea    0x100(%rsp),%rsi
   5d33f:	lea    0x30(%rsp),%rdx
   5d344:	mov    %rsp,%rcx
   5d347:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d34c:	cmpb   $0xff,0x50(%rsp)
   5d351:	je     5d35d <probe::main+0x127d>
   5d353:	lea    0x50(%rsp),%rdi
   5d358:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d35d:	call   *0x1292d5(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d363:	mov    $0x4,%ebp
   5d368:	mov    $0x4,%edi
   5d36d:	mov    $0x1,%esi
   5d372:	call   *0x1292c8(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d378:	test   %rax,%rax
   5d37b:	je     5dc8f <probe::main+0x1baf>
   5d381:	movl   $0x73746962,(%rax)
   5d387:	movq   $0x4,0x30(%rsp)
   5d390:	mov    %rax,0x38(%rsp)
   5d395:	movq   $0x4,0x40(%rsp)
   5d39e:	movb   $0x2,(%rsp)
   5d3a2:	movq   $0x0,0x8(%rsp)
   5d3ab:	mov    0xc0(%rsp),%rax
   5d3b3:	mov    %rax,0x10(%rsp)
   5d3b8:	lea    0x50(%rsp),%rdi
   5d3bd:	lea    0x100(%rsp),%rsi
   5d3c5:	lea    0x30(%rsp),%rdx
   5d3ca:	mov    %rsp,%rcx
   5d3cd:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d3d2:	cmpb   $0xff,0x50(%rsp)
   5d3d7:	je     5d3e3 <probe::main+0x1303>
   5d3d9:	lea    0x50(%rsp),%rdi
   5d3de:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d3e3:	call   *0x12924f(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d3e9:	mov    $0x9,%ebp
   5d3ee:	mov    $0x9,%edi
   5d3f3:	mov    $0x1,%esi
   5d3f8:	call   *0x129242(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d3fe:	test   %rax,%rax
   5d401:	je     5dc8f <probe::main+0x1baf>
   5d407:	movabs $0x6f69736963657270,%rcx
   5d411:	mov    %rcx,(%rax)
   5d414:	movb   $0x6e,0x8(%rax)
   5d418:	movq   $0x9,0x30(%rsp)
   5d421:	mov    %rax,0x38(%rsp)
   5d426:	movq   $0x9,0x40(%rsp)
   5d42f:	movslq 0x70(%rsp),%rax
   5d434:	mov    %rax,%rcx
   5d437:	shr    $0x3f,%rcx
   5d43b:	movb   $0x2,(%rsp)
   5d43f:	mov    %rcx,0x8(%rsp)
   5d444:	mov    %rax,0x10(%rsp)
   5d449:	lea    0x50(%rsp),%rdi
   5d44e:	lea    0x100(%rsp),%rsi
   5d456:	lea    0x30(%rsp),%rdx
   5d45b:	mov    %rsp,%rcx
   5d45e:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d463:	cmpb   $0xff,0x50(%rsp)
   5d468:	je     5d474 <probe::main+0x1394>
   5d46a:	lea    0x50(%rsp),%rdi
   5d46f:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d474:	call   *0x1291be(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d47a:	mov    $0x8,%ebp
   5d47f:	mov    $0x8,%edi
   5d484:	mov    $0x1,%esi
   5d489:	call   *0x1291b1(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d48f:	test   %rax,%rax
   5d492:	je     5dc8f <probe::main+0x1baf>
   5d498:	movabs $0x657669746167656e,%rcx
   5d4a2:	mov    %rcx,(%rax)
   5d4a5:	movq   $0x8,0x30(%rsp)
   5d4ae:	mov    %rax,0x38(%rsp)
   5d4b3:	movq   $0x8,0x40(%rsp)
   5d4bc:	movb   $0x1,(%rsp)
   5d4c0:	mov    0x4c(%rsp),%eax
   5d4c4:	mov    %al,0x1(%rsp)
   5d4c8:	lea    0x50(%rsp),%rdi
   5d4cd:	lea    0x100(%rsp),%rsi
   5d4d5:	lea    0x30(%rsp),%rdx
   5d4da:	mov    %rsp,%rcx
   5d4dd:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d4e2:	cmpb   $0xff,0x50(%rsp)
   5d4e7:	je     5d4f3 <probe::main+0x1413>
   5d4e9:	lea    0x50(%rsp),%rdi
   5d4ee:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d4f3:	call   *0x12913f(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d4f9:	mov    $0x6,%ebp
   5d4fe:	mov    $0x6,%edi
   5d503:	mov    $0x1,%esi
   5d508:	call   *0x129132(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d50e:	test   %rax,%rax
   5d511:	je     5dc8f <probe::main+0x1baf>
   5d517:	movw   $0x6465,0x4(%rax)
   5d51d:	movl   $0x68636163,(%rax)
   5d523:	movq   $0x6,0x30(%rsp)
   5d52c:	mov    %rax,0x38(%rsp)
   5d531:	movq   $0x6,0x40(%rsp)
   5d53a:	movb   $0x1,(%rsp)
   5d53e:	mov    0x48(%rsp),%eax
   5d542:	mov    %al,0x1(%rsp)
   5d546:	lea    0x50(%rsp),%rdi
   5d54b:	lea    0x100(%rsp),%rsi
   5d553:	lea    0x30(%rsp),%rdx
   5d558:	mov    %rsp,%rcx
   5d55b:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d560:	cmpb   $0xff,0x50(%rsp)
   5d565:	je     5d571 <probe::main+0x1491>
   5d567:	lea    0x50(%rsp),%rdi
   5d56c:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d571:	call   *0x1290c1(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d577:	mov    $0xa,%ebp
   5d57c:	mov    $0xa,%edi
   5d581:	mov    $0x1,%esi
   5d586:	call   *0x1290b4(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d58c:	test   %rax,%rax
   5d58f:	je     5dc8f <probe::main+0x1baf>
   5d595:	mov    %rax,%r13
   5d598:	movabs $0x5f64657370616c65,%rax
   5d5a2:	mov    %rax,0x0(%r13)
   5d5a6:	movw   $0x736e,0x8(%r13)
   5d5ad:	movq   $0xa,0x30(%rsp)
   5d5b6:	mov    %r13,0x38(%rsp)
   5d5bb:	movq   $0xa,0x40(%rsp)
   5d5c4:	mov    %rsp,%rdx
   5d5c7:	mov    $0x27,%ecx
   5d5cc:	mov    0x88(%rsp),%rdi
   5d5d4:	mov    0xa8(%rsp),%rsi
   5d5dc:	call   *0x1290b6(%rip)        # 186698 <_DYNAMIC+0x310>
   5d5e2:	mov    %rdx,%rbx
   5d5e5:	test   %rdx,%rdx
   5d5e8:	jns    5d5fd <probe::main+0x151d>
   5d5ea:	xor    %ebp,%ebp
   5d5ec:	mov    %rbp,%rdi
   5d5ef:	mov    %rbx,%rsi
   5d5f2:	call   *0x129098(%rip)        # 186690 <_DYNAMIC+0x308>
   5d5f8:	jmp    5dd12 <probe::main+0x1c32>
   5d5fd:	test   %rbx,%rbx
   5d600:	je     5d669 <probe::main+0x1589>
   5d602:	mov    %rax,%r15
   5d605:	call   *0x12902d(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d60b:	mov    $0x1,%ebp
   5d610:	mov    $0x1,%esi
   5d615:	mov    %rbx,%rdi
   5d618:	call   *0x129022(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d61e:	test   %rax,%rax
   5d621:	je     5d5ec <probe::main+0x150c>
   5d623:	mov    %rax,%r14
   5d626:	mov    %rax,%rdi
   5d629:	mov    %r15,%rsi
   5d62c:	mov    %rbx,%rdx
   5d62f:	call   *0x129013(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5d635:	call   *0x128ffd(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d63b:	mov    $0x1,%esi
   5d640:	mov    %rbx,%rdi
   5d643:	call   *0x128ff7(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d649:	test   %rax,%rax
   5d64c:	mov    %r14,%rbp
   5d64f:	je     5dce1 <probe::main+0x1c01>
   5d655:	mov    %rax,%r15
   5d658:	mov    %rax,%rdi
   5d65b:	mov    %rbp,%rsi
   5d65e:	mov    %rbx,%rdx
   5d661:	call   *0x128fe1(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5d667:	jmp    5d674 <probe::main+0x1594>
   5d669:	mov    $0x1,%r15d
   5d66f:	mov    $0x1,%ebp
   5d674:	mov    0xa0(%rsp),%r14
   5d67c:	movb   $0x3,(%rsp)
   5d680:	mov    %rbx,0x8(%rsp)
   5d685:	mov    %r15,0x10(%rsp)
   5d68a:	mov    %rbx,0x18(%rsp)
   5d68f:	lea    0x50(%rsp),%rdi
   5d694:	lea    0x100(%rsp),%rsi
   5d69c:	lea    0x30(%rsp),%rdx
   5d6a1:	mov    %rsp,%rcx
   5d6a4:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d6a9:	cmpb   $0xff,0x50(%rsp)
   5d6ae:	je     5d6ba <probe::main+0x15da>
   5d6b0:	lea    0x50(%rsp),%rdi
   5d6b5:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d6ba:	test   %rbx,%rbx
   5d6bd:	je     5d6d0 <probe::main+0x15f0>
   5d6bf:	mov    $0x1,%edx
   5d6c4:	mov    %rbp,%rdi
   5d6c7:	mov    %rbx,%rsi
   5d6ca:	call   *0x128ef8(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d6d0:	call   *0x128f62(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d6d6:	mov    $0x8,%ebp
   5d6db:	mov    $0x8,%edi
   5d6e0:	mov    $0x1,%esi
   5d6e5:	call   *0x128f55(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d6eb:	test   %rax,%rax
   5d6ee:	je     5dc8f <probe::main+0x1baf>
   5d6f4:	movabs $0x6d75736b63656863,%rcx
   5d6fe:	mov    %rcx,(%rax)
   5d701:	movq   $0x8,0x30(%rsp)
   5d70a:	mov    %rax,0x38(%rsp)
   5d70f:	movq   $0x8,0x40(%rsp)
   5d718:	movb   $0x2,(%rsp)
   5d71c:	movq   $0x0,0x8(%rsp)
   5d725:	mov    %r12,0x10(%rsp)
   5d72a:	lea    0x50(%rsp),%rdi
   5d72f:	lea    0x100(%rsp),%rsi
   5d737:	lea    0x30(%rsp),%rdx
   5d73c:	mov    %rsp,%rcx
   5d73f:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d744:	cmpb   $0xff,0x50(%rsp)
   5d749:	je     5d755 <probe::main+0x1675>
   5d74b:	lea    0x50(%rsp),%rdi
   5d750:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d755:	call   *0x128edd(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d75b:	mov    $0xc,%ebp
   5d760:	mov    $0xc,%edi
   5d765:	mov    $0x1,%esi
   5d76a:	call   *0x128ed0(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d770:	test   %rax,%rax
   5d773:	je     5dc8f <probe::main+0x1baf>
   5d779:	movabs $0x625f74757074756f,%rcx
   5d783:	mov    %rcx,(%rax)
   5d786:	movl   $0x73657479,0x8(%rax)
   5d78d:	movq   $0xc,0x30(%rsp)
   5d796:	mov    %rax,0x38(%rsp)
   5d79b:	movq   $0xc,0x40(%rsp)
   5d7a4:	movb   $0x2,(%rsp)
   5d7a8:	movq   $0x0,0x8(%rsp)
   5d7b1:	mov    0x98(%rsp),%rax
   5d7b9:	mov    %rax,0x10(%rsp)
   5d7be:	lea    0x50(%rsp),%rdi
   5d7c3:	lea    0x100(%rsp),%rsi
   5d7cb:	lea    0x30(%rsp),%rdx
   5d7d0:	mov    %rsp,%rcx
   5d7d3:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d7d8:	cmpb   $0xff,0x50(%rsp)
   5d7dd:	je     5d7e9 <probe::main+0x1709>
   5d7df:	lea    0x50(%rsp),%rdi
   5d7e4:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d7e9:	call   *0x128e49(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d7ef:	mov    $0x9,%ebp
   5d7f4:	mov    $0x1,%ebx
   5d7f9:	mov    $0x9,%edi
   5d7fe:	mov    $0x1,%esi
   5d803:	call   *0x128e37(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d809:	test   %rax,%rax
   5d80c:	je     5dc8f <probe::main+0x1baf>
   5d812:	movabs $0x636e657265666572,%rcx
   5d81c:	mov    %rcx,(%rax)
   5d81f:	movb   $0x65,0x8(%rax)
   5d823:	movq   $0x9,0x30(%rsp)
   5d82c:	mov    %rax,0x38(%rsp)
   5d831:	movq   $0x9,0x40(%rsp)
   5d83a:	test   %r14,%r14
   5d83d:	je     5d876 <probe::main+0x1796>
   5d83f:	mov    %rax,%r12
   5d842:	call   *0x128df0(%rip)        # 186638 <_DYNAMIC+0x2b0>
   5d848:	mov    $0x1,%esi
   5d84d:	mov    %r14,%rdi
   5d850:	call   *0x128dea(%rip)        # 186640 <_DYNAMIC+0x2b8>
   5d856:	test   %rax,%rax
   5d859:	je     5dcf1 <probe::main+0x1c11>
   5d85f:	mov    %rax,%rbx
   5d862:	mov    %rax,%rdi
   5d865:	mov    0x80(%rsp),%rsi
   5d86d:	mov    %r14,%rdx
   5d870:	call   *0x128dd2(%rip)        # 186648 <memcpy@GLIBC_2.14>
   5d876:	movb   $0x3,(%rsp)
   5d87a:	mov    %r14,0x8(%rsp)
   5d87f:	mov    %rbx,0x10(%rsp)
   5d884:	mov    %r14,0x18(%rsp)
   5d889:	lea    0x50(%rsp),%rdi
   5d88e:	lea    0x100(%rsp),%rsi
   5d896:	lea    0x30(%rsp),%rdx
   5d89b:	mov    %rsp,%rbx
   5d89e:	mov    %rbx,%rcx
   5d8a1:	call   5e1c0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5d8a6:	cmpb   $0xff,0x50(%rsp)
   5d8ab:	je     5d8b7 <probe::main+0x17d7>
   5d8ad:	lea    0x50(%rsp),%rdi
   5d8b2:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d8b7:	movups 0x100(%rsp),%xmm0
   5d8bf:	movups %xmm0,0x8(%rsp)
   5d8c4:	mov    0x110(%rsp),%rax
   5d8cc:	mov    %rax,0x18(%rsp)
   5d8d1:	movb   $0x5,(%rsp)
   5d8d5:	mov    %rbx,0x50(%rsp)
   5d8da:	mov    0x128dbf(%rip),%rax        # 1866a0 <_DYNAMIC+0x318>
   5d8e1:	mov    %rax,0x58(%rsp)
   5d8e6:	lea    -0x2da09(%rip),%rdi        # 2fee4 <anon.c3a302fb892fb1604dd2767d56b20e73.4.llvm.8018543666394721898+0x1d8>
   5d8ed:	lea    0x50(%rsp),%rsi
   5d8f2:	call   *0x128db0(%rip)        # 1866a8 <_DYNAMIC+0x320>
   5d8f8:	mov    %rsp,%rdi
   5d8fb:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5d900:	lea    0xc8(%rsp),%rdi
   5d908:	call   60710 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   5d90d:	mov    0xc8(%rsp),%rsi
   5d915:	test   %rsi,%rsi
   5d918:	je     5d931 <probe::main+0x1851>
   5d91a:	mov    0xd0(%rsp),%rdi
   5d922:	shl    $0x4,%rsi
   5d926:	mov    $0x8,%edx
   5d92b:	call   *0x128c97(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d931:	mov    0xb8(%rsp),%rsi
   5d939:	test   %rsi,%rsi
   5d93c:	je     5d951 <probe::main+0x1871>
   5d93e:	mov    $0x1,%edx
   5d943:	mov    0x80(%rsp),%rdi
   5d94b:	call   *0x128c77(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d951:	cmpb   $0x0,0x94(%rsp)
   5d959:	je     5d99b <probe::main+0x18bb>
   5d95b:	lea    0xe8(%rsp),%rbx
   5d963:	mov    0xe0(%rsp),%rax
   5d96b:	lock decq (%rax)
   5d96f:	jne    5d97f <probe::main+0x189f>
   5d971:	lea    0xe0(%rsp),%rdi
   5d979:	call   *0x128c59(%rip)        # 1865d8 <_DYNAMIC+0x250>
   5d97f:	mov    0xe8(%rsp),%rax
   5d987:	test   %rax,%rax
   5d98a:	je     5d99b <probe::main+0x18bb>
   5d98c:	lock decq (%rax)
   5d990:	jne    5d99b <probe::main+0x18bb>
   5d992:	mov    %rbx,%rdi
   5d995:	call   *0x128c45(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5d99b:	mov    0xb0(%rsp),%rax
   5d9a3:	lock decq (%rax)
   5d9a7:	jne    5d9b7 <probe::main+0x18d7>
   5d9a9:	lea    0xb0(%rsp),%rdi
   5d9b1:	call   *0x128cf9(%rip)        # 1866b0 <_DYNAMIC+0x328>
   5d9b7:	mov    0xf0(%rsp),%rsi
   5d9bf:	test   %rsi,%rsi
   5d9c2:	je     5d9db <probe::main+0x18fb>
   5d9c4:	shl    $0x3,%rsi
   5d9c8:	mov    $0x8,%edx
   5d9cd:	mov    0xf8(%rsp),%rdi
   5d9d5:	call   *0x128bed(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5d9db:	mov    0x128(%rsp),%rbx
   5d9e3:	mov    0x130(%rsp),%r14
   5d9eb:	test   %r14,%r14
   5d9ee:	je     5da1f <probe::main+0x193f>
   5d9f0:	lea    0x8(%rbx),%r15
   5d9f4:	mov    0x128bcd(%rip),%r12        # 1865c8 <_DYNAMIC+0x240>
   5d9fb:	jmp    5da09 <probe::main+0x1929>
   5d9fd:	nopl   (%rax)
   5da00:	add    $0x18,%r15
   5da04:	dec    %r14
   5da07:	je     5da1f <probe::main+0x193f>
   5da09:	mov    -0x8(%r15),%rsi
   5da0d:	test   %rsi,%rsi
   5da10:	je     5da00 <probe::main+0x1920>
   5da12:	mov    (%r15),%rdi
   5da15:	mov    $0x1,%edx
   5da1a:	call   *%r12
   5da1d:	jmp    5da00 <probe::main+0x1920>
   5da1f:	mov    0x120(%rsp),%rax
   5da27:	test   %rax,%rax
   5da2a:	je     5da42 <probe::main+0x1962>
   5da2c:	shl    $0x3,%rax
   5da30:	lea    (%rax,%rax,2),%rsi
   5da34:	mov    $0x8,%edx
   5da39:	mov    %rbx,%rdi
   5da3c:	call   *0x128b86(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5da42:	add    $0x168,%rsp
   5da49:	pop    %rbx
   5da4a:	pop    %r12
   5da4c:	pop    %r13
   5da4e:	pop    %r14
   5da50:	pop    %r15
   5da52:	pop    %rbp
   5da53:	ret
   5da54:	movzbl 0x8(%rsp),%eax
   5da59:	mov    %al,0x50(%rsp)
   5da5d:	lea    -0x2ad9c(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5da64:	lea    0x124445(%rip),%rcx        # 181eb0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898>
   5da6b:	lea    0x124426(%rip),%r8        # 181e98 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x138>
   5da72:	lea    0x50(%rsp),%rdx
   5da77:	mov    $0x2b,%esi
   5da7c:	call   *0x128b7e(%rip)        # 186600 <_DYNAMIC+0x278>
   5da82:	jmp    5dd12 <probe::main+0x1c32>
   5da87:	lea    -0x2ac0d(%rip),%rdi        # 32e81 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898+0x1b9>
   5da8e:	lea    0x12445b(%rip),%rcx        # 181ef0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x40>
   5da95:	lea    0x1244bc(%rip),%r8        # 181f58 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0xa8>
   5da9c:	lea    0x77(%rsp),%rdx
   5daa1:	mov    $0x37,%esi
   5daa6:	call   *0x128b54(%rip)        # 186600 <_DYNAMIC+0x278>
   5daac:	jmp    5dd12 <probe::main+0x1c32>
   5dab1:	add    $0xd0,%dl
   5dab4:	xor    %eax,%eax
   5dab6:	cmp    $0xa,%dl
   5dab9:	adc    $0x1,%al
   5dabb:	mov    %al,(%rsp)
   5dabe:	lea    -0x2adfd(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5dac5:	lea    0x124404(%rip),%rcx        # 181ed0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x20>
   5dacc:	lea    0x1242d5(%rip),%r8        # 181da8 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x48>
   5dad3:	mov    %rsp,%rdx
   5dad6:	mov    $0x2b,%esi
   5dadb:	call   *0x128b1f(%rip)        # 186600 <_DYNAMIC+0x278>
   5dae1:	jmp    5dd12 <probe::main+0x1c32>
   5dae6:	movzbl 0x8(%rsp),%eax
   5daeb:	mov    %al,0x30(%rsp)
   5daef:	lea    -0x2ae2e(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5daf6:	lea    0x1243b3(%rip),%rcx        # 181eb0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898>
   5dafd:	lea    0x1242d4(%rip),%r8        # 181dd8 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x78>
   5db04:	lea    0x30(%rsp),%rdx
   5db09:	mov    $0x2b,%esi
   5db0e:	call   *0x128aec(%rip)        # 186600 <_DYNAMIC+0x278>
   5db14:	jmp    5dd12 <probe::main+0x1c32>
   5db19:	movzbl 0x8(%rsp),%eax
   5db1e:	mov    %al,0x50(%rsp)
   5db22:	lea    -0x2ae61(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5db29:	lea    0x124380(%rip),%rcx        # 181eb0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898>
   5db30:	lea    0x1242b9(%rip),%r8        # 181df0 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x90>
   5db37:	lea    0x50(%rsp),%rdx
   5db3c:	mov    $0x2b,%esi
   5db41:	call   *0x128ab9(%rip)        # 186600 <_DYNAMIC+0x278>
   5db47:	jmp    5dd12 <probe::main+0x1c32>
   5db4c:	movzbl 0x31(%rsp),%eax
   5db51:	mov    %al,(%rsp)
   5db54:	lea    -0x2ae93(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5db5b:	lea    0x12434e(%rip),%rcx        # 181eb0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898>
   5db62:	lea    0x1242b7(%rip),%r8        # 181e20 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0xc0>
   5db69:	mov    %rsp,%rdx
   5db6c:	mov    $0x2b,%esi
   5db71:	call   *0x128a89(%rip)        # 186600 <_DYNAMIC+0x278>
   5db77:	jmp    5dd12 <probe::main+0x1c32>
   5db7c:	movzbl 0x8(%rsp),%eax
   5db81:	mov    %al,0x50(%rsp)
   5db85:	lea    -0x2aec4(%rip),%rdi        # 32cc8 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898>
   5db8c:	lea    0x12431d(%rip),%rcx        # 181eb0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898>
   5db93:	lea    0x12429e(%rip),%r8        # 181e38 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0xd8>
   5db9a:	lea    0x50(%rsp),%rdx
   5db9f:	mov    $0x2b,%esi
   5dba4:	call   *0x128a56(%rip)        # 186600 <_DYNAMIC+0x278>
   5dbaa:	jmp    5dd12 <probe::main+0x1c32>
   5dbaf:	lea    -0x2ad35(%rip),%rdi        # 32e81 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898+0x1b9>
   5dbb6:	lea    0x124333(%rip),%rcx        # 181ef0 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x40>
   5dbbd:	lea    0x124394(%rip),%r8        # 181f58 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0xa8>
   5dbc4:	lea    0x77(%rsp),%rdx
   5dbc9:	mov    $0x37,%esi
   5dbce:	call   *0x128a2c(%rip)        # 186600 <_DYNAMIC+0x278>
   5dbd4:	jmp    5dd12 <probe::main+0x1c32>
   5dbd9:	lea    0x124270(%rip),%r9        # 181e50 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0xf0>
   5dbe0:	lea    0x138(%rsp),%rsi
   5dbe8:	lea    0x150(%rsp),%rdx
   5dbf0:	xor    %edi,%edi
   5dbf2:	xor    %ecx,%ecx
   5dbf4:	call   *0x128abe(%rip)        # 1866b8 <_DYNAMIC+0x330>
   5dbfa:	jmp    5dd12 <probe::main+0x1c32>
   5dbff:	lea    0x124262(%rip),%r9        # 181e68 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x108>
   5dc06:	lea    0x140(%rsp),%rsi
   5dc0e:	mov    %rsp,%rdx
   5dc11:	xor    %edi,%edi
   5dc13:	xor    %ecx,%ecx
   5dc15:	call   *0x128aa5(%rip)        # 1866c0 <_DYNAMIC+0x338>
   5dc1b:	jmp    5dd12 <probe::main+0x1c32>
   5dc20:	lea    0x1241e1(%rip),%rdi        # 181e08 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0xa8>
   5dc27:	call   *0x128a9b(%rip)        # 1866c8 <_DYNAMIC+0x340>
   5dc2d:	jmp    5dd12 <probe::main+0x1c32>
   5dc32:	mov    $0x8,%edi
   5dc37:	mov    $0x10,%esi
   5dc3c:	call   *0x128a8e(%rip)        # 1866d0 <_DYNAMIC+0x348>
   5dc42:	jmp    5dd12 <probe::main+0x1c32>
   5dc47:	mov    $0x1,%al
   5dc49:	jmp    5dabb <probe::main+0x19db>
   5dc4e:	mov    %rbx,%rbp
   5dc51:	mov    %r12,%r13
   5dc54:	lea    -0x2af68(%rip),%rdi        # 32cf3 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898+0x2b>
   5dc5b:	lea    0x1242ae(%rip),%rdx        # 181f10 <anon.c3a302fb892fb1604dd2767d56b20e73.36.llvm.8018543666394721898+0x60>
   5dc62:	mov    $0x49,%esi
   5dc67:	call   *0x1289ab(%rip)        # 186618 <_DYNAMIC+0x290>
   5dc6d:	jmp    5dd12 <probe::main+0x1c32>
   5dc72:	lea    0x124207(%rip),%rdx        # 181e80 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x120>
   5dc79:	mov    %rsi,%rdi
   5dc7c:	mov    0xa8(%rsp),%rsi
   5dc84:	call   *0x12897e(%rip)        # 186608 <_DYNAMIC+0x280>
   5dc8a:	jmp    5dd12 <probe::main+0x1c32>
   5dc8f:	mov    $0x1,%edi
   5dc94:	mov    %rbp,%rsi
   5dc97:	call   *0x1289f3(%rip)        # 186690 <_DYNAMIC+0x308>
   5dc9d:	jmp    5dd12 <probe::main+0x1c32>
   5dc9f:	lea    0x1240ea(%rip),%rdx        # 181d90 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x30>
   5dca6:	jmp    5dcaf <probe::main+0x1bcf>
   5dca8:	lea    0x1240c9(%rip),%rdx        # 181d78 <anon.c3a302fb892fb1604dd2767d56b20e73.5.llvm.8018543666394721898+0x18>
   5dcaf:	mov    %rdi,%rsi
   5dcb2:	call   *0x128950(%rip)        # 186608 <_DYNAMIC+0x280>
   5dcb8:	jmp    5dd12 <probe::main+0x1c32>
   5dcba:	xor    %eax,%eax
   5dcbc:	jmp    5dabb <probe::main+0x19db>
   5dcc1:	mov    $0x8,%edi
   5dcc6:	mov    %r13,%rsi
   5dcc9:	call   *0x1289c1(%rip)        # 186690 <_DYNAMIC+0x308>
   5dccf:	jmp    5dd12 <probe::main+0x1c32>
   5dcd1:	mov    $0x8,%edi
   5dcd6:	mov    %rbx,%rsi
   5dcd9:	call   *0x1289b1(%rip)        # 186690 <_DYNAMIC+0x308>
   5dcdf:	jmp    5dd12 <probe::main+0x1c32>
   5dce1:	mov    $0x1,%edi
   5dce6:	mov    %rbx,%rsi
   5dce9:	call   *0x1289a1(%rip)        # 186690 <_DYNAMIC+0x308>
   5dcef:	jmp    5dd12 <probe::main+0x1c32>
   5dcf1:	mov    $0x1,%edi
   5dcf6:	mov    %r14,%rsi
   5dcf9:	call   *0x128991(%rip)        # 186690 <_DYNAMIC+0x308>
   5dcff:	jmp    5dd12 <probe::main+0x1c32>
   5dd01:	mov    %rbx,%rbp
   5dd04:	mov    $0x8,%edi
   5dd09:	mov    %r12,%rsi
   5dd0c:	call   *0x12897e(%rip)        # 186690 <_DYNAMIC+0x308>
   5dd12:	ud2
   5dd14:	mov    %rax,%r15
   5dd17:	jmp    5def2 <probe::main+0x1e12>
   5dd1c:	mov    %rax,%r15
   5dd1f:	mov    0xe8(%rsp),%rax
   5dd27:	test   %rax,%rax
   5dd2a:	je     5e13c <probe::main+0x205c>
   5dd30:	lock decq (%rax)
   5dd34:	jne    5e13c <probe::main+0x205c>
   5dd3a:	mov    %rbx,%rdi
   5dd3d:	call   *0x12889d(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5dd43:	jmp    5e13c <probe::main+0x205c>
   5dd48:	call   *0x128882(%rip)        # 1865d0 <_DYNAMIC+0x248>
   5dd4e:	jmp    5dd50 <probe::main+0x1c70>
   5dd50:	mov    %rax,%r15
   5dd53:	mov    (%rsp),%rsi
   5dd57:	test   %rsi,%rsi
   5dd5a:	je     5df03 <probe::main+0x1e23>
   5dd60:	mov    0x8(%rsp),%rdi
   5dd65:	shl    $0x3,%rsi
   5dd69:	mov    $0x8,%edx
   5dd6e:	jmp    5defd <probe::main+0x1e1d>
   5dd73:	mov    %rax,%r15
   5dd76:	mov    $0x9,%esi
   5dd7b:	mov    $0x1,%edx
   5dd80:	mov    %r12,%rdi
   5dd83:	jmp    5e0c1 <probe::main+0x1fe1>
   5dd88:	mov    %rax,%r15
   5dd8b:	mov    $0xa,%esi
   5dd90:	mov    $0x1,%edx
   5dd95:	mov    %r13,%rdi
   5dd98:	call   *0x12882a(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5dd9e:	jmp    5ddfe <probe::main+0x1d1e>
   5dda0:	jmp    5ddf2 <probe::main+0x1d12>
   5dda2:	mov    %rax,%r15
   5dda5:	mov    (%rsp),%rsi
   5dda9:	test   %rsi,%rsi
   5ddac:	je     5e158 <probe::main+0x2078>
   5ddb2:	mov    0x8(%rsp),%rdi
   5ddb7:	shl    $0x3,%rsi
   5ddbb:	mov    $0x8,%edx
   5ddc0:	call   *0x128802(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5ddc6:	jmp    5e158 <probe::main+0x2078>
   5ddcb:	mov    %rax,%r15
   5ddce:	jmp    5e0e1 <probe::main+0x2001>
   5ddd3:	mov    %rax,%r15
   5ddd6:	mov    %rsp,%rdi
   5ddd9:	call   5bdf0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5ddde:	jmp    5e0d4 <probe::main+0x1ff4>
   5dde3:	jmp    5ded4 <probe::main+0x1df4>
   5dde8:	jmp    5ded4 <probe::main+0x1df4>
   5dded:	jmp    5ded4 <probe::main+0x1df4>
   5ddf2:	mov    %rax,%r15
   5ddf5:	test   %rbx,%rbx
   5ddf8:	je     5e0c7 <probe::main+0x1fe7>
   5ddfe:	mov    $0x1,%edx
   5de03:	mov    %rbp,%rdi
   5de06:	mov    %rbx,%rsi
   5de09:	jmp    5e0c1 <probe::main+0x1fe1>
   5de0e:	mov    %r13,0xa0(%rsp)
   5de16:	jmp    5de18 <probe::main+0x1d38>
   5de18:	mov    %rax,%r15
   5de1b:	mov    (%rsp),%rsi
   5de1f:	test   %rsi,%rsi
   5de22:	je     5de36 <probe::main+0x1d56>
   5de24:	mov    (%rbx),%rdi
   5de27:	shl    $0x3,%rsi
   5de2b:	mov    $0x8,%edx
   5de30:	call   *0x128792(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5de36:	test   %r14,%r14
   5de39:	je     5e158 <probe::main+0x2078>
   5de3f:	mov    $0x8,%edx
   5de44:	mov    %r12,%rdi
   5de47:	mov    0xa0(%rsp),%rsi
   5de4f:	call   *0x128773(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5de55:	jmp    5e158 <probe::main+0x2078>
   5de5a:	jmp    5ded4 <probe::main+0x1df4>
   5de5c:	jmp    5ded4 <probe::main+0x1df4>
   5de5e:	jmp    5ded4 <probe::main+0x1df4>
   5de60:	jmp    5ded4 <probe::main+0x1df4>
   5de62:	jmp    5ded4 <probe::main+0x1df4>
   5de64:	jmp    5ded4 <probe::main+0x1df4>
   5de66:	mov    %rax,%r15
   5de69:	mov    0x38(%rsp),%rax
   5de6e:	test   %rax,%rax
   5de71:	je     5e158 <probe::main+0x2078>
   5de77:	lock decq (%rax)
   5de7b:	jne    5e158 <probe::main+0x2078>
   5de81:	mov    %rbx,%rdi
   5de84:	call   *0x128756(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5de8a:	jmp    5e158 <probe::main+0x2078>
   5de8f:	call   *0x12873b(%rip)        # 1865d0 <_DYNAMIC+0x248>
   5de95:	mov    %rax,%r15
   5de98:	mov    0x58(%rsp),%rax
   5de9d:	test   %rax,%rax
   5dea0:	je     5e040 <probe::main+0x1f60>
   5dea6:	lock decq (%rax)
   5deaa:	jne    5e040 <probe::main+0x1f60>
   5deb0:	mov    %rbx,%rdi
   5deb3:	call   *0x128727(%rip)        # 1865e0 <_DYNAMIC+0x258>
   5deb9:	jmp    5e040 <probe::main+0x1f60>
   5debe:	call   *0x12870c(%rip)        # 1865d0 <_DYNAMIC+0x248>
   5dec4:	mov    %rax,%r15
   5dec7:	jmp    5e158 <probe::main+0x2078>
   5decc:	mov    %rax,%r15
   5decf:	jmp    5e13c <probe::main+0x205c>
   5ded4:	mov    %rax,%r15
   5ded7:	jmp    5e0c7 <probe::main+0x1fe7>
   5dedc:	jmp    5dfe4 <probe::main+0x1f04>
   5dee1:	jmp    5e028 <probe::main+0x1f48>
   5dee6:	mov    %rax,%r15
   5dee9:	test   %rbp,%rbp
   5deec:	je     5df03 <probe::main+0x1e23>
   5deee:	shl    $0x3,%rbp
   5def2:	mov    $0x8,%edx
   5def7:	mov    %r13,%rdi
   5defa:	mov    %rbp,%rsi
   5defd:	call   *0x1286c5(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5df03:	cmpq   $0x0,0x80(%rsp)
   5df0c:	je     5e158 <probe::main+0x2078>
   5df12:	mov    0x80(%rsp),%rsi
   5df1a:	shl    $0x3,%rsi
   5df1e:	mov    $0x8,%edx
   5df23:	mov    0xb8(%rsp),%rdi
   5df2b:	call   *0x128697(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5df31:	jmp    5e158 <probe::main+0x2078>
   5df36:	mov    %rax,%r15
   5df39:	mov    $0x1,%al
   5df3b:	mov    %eax,0x94(%rsp)
   5df42:	jmp    5e105 <probe::main+0x2025>
   5df47:	mov    %rax,%r15
   5df4a:	test   %r13,%r13
   5df4d:	je     5e158 <probe::main+0x2078>
   5df53:	shl    $0x3,%r13
   5df57:	mov    $0x8,%edx
   5df5c:	mov    0x88(%rsp),%rdi
   5df64:	mov    %r13,%rsi
   5df67:	call   *0x12865b(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5df6d:	jmp    5e158 <probe::main+0x2078>
   5df72:	mov    %rax,%r15
   5df75:	mov    0x30(%rsp),%rsi
   5df7a:	test   %rsi,%rsi
   5df7d:	je     5df8f <probe::main+0x1eaf>
   5df7f:	mov    0x38(%rsp),%rdi
   5df84:	mov    $0x1,%edx
   5df89:	call   *0x128639(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5df8f:	mov    0x50(%rsp),%rsi
   5df94:	test   %rsi,%rsi
   5df97:	je     5e12f <probe::main+0x204f>
   5df9d:	mov    0x58(%rsp),%rdi
   5dfa2:	shl    $0x3,%rsi
   5dfa6:	mov    $0x8,%edx
   5dfab:	call   *0x128617(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5dfb1:	jmp    5e12f <probe::main+0x204f>
   5dfb6:	mov    %rax,%r15
   5dfb9:	jmp    5e12f <probe::main+0x204f>
   5dfbe:	mov    %rax,%r15
   5dfc1:	jmp    5e158 <probe::main+0x2078>
   5dfc6:	mov    %rax,%r15
   5dfc9:	lea    0x50(%rsp),%rdi
   5dfce:	call   5bee0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5dfd3:	jmp    5e040 <probe::main+0x1f60>
   5dfd5:	mov    %rax,%r15
   5dfd8:	lea    0x30(%rsp),%rdi
   5dfdd:	call   5bee0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5dfe2:	jmp    5e040 <probe::main+0x1f60>
   5dfe4:	mov    %rax,%r15
   5dfe7:	mov    0x30(%rsp),%rsi
   5dfec:	test   %rsi,%rsi
   5dfef:	je     5e001 <probe::main+0x1f21>
   5dff1:	mov    0x38(%rsp),%rdi
   5dff6:	mov    $0x1,%edx
   5dffb:	call   *0x1285c7(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e001:	mov    0x50(%rsp),%rsi
   5e006:	test   %rsi,%rsi
   5e009:	je     5e0d4 <probe::main+0x1ff4>
   5e00f:	mov    0x58(%rsp),%rdi
   5e014:	shl    $0x3,%rsi
   5e018:	mov    $0x8,%edx
   5e01d:	call   *0x1285a5(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e023:	jmp    5e0d4 <probe::main+0x1ff4>
   5e028:	mov    %rax,%r15
   5e02b:	jmp    5e0d4 <probe::main+0x1ff4>
   5e030:	mov    %rax,%r15
   5e033:	mov    $0xa,%esi
   5e038:	jmp    5e0b9 <probe::main+0x1fd9>
   5e03d:	mov    %rax,%r15
   5e040:	mov    0x128(%rsp),%rbx
   5e048:	mov    0x130(%rsp),%r14
   5e050:	test   %r14,%r14
   5e053:	jne    5e080 <probe::main+0x1fa0>
   5e055:	mov    0x120(%rsp),%rax
   5e05d:	test   %rax,%rax
   5e060:	je     5e078 <probe::main+0x1f98>
   5e062:	shl    $0x3,%rax
   5e066:	lea    (%rax,%rax,2),%rsi
   5e06a:	mov    $0x8,%edx
   5e06f:	mov    %rbx,%rdi
   5e072:	call   *0x128550(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e078:	mov    %r15,%rdi
   5e07b:	call   180b10 <_Unwind_Resume@plt>
   5e080:	lea    0x8(%rbx),%r12
   5e084:	mov    0x12853d(%rip),%r13        # 1865c8 <_DYNAMIC+0x240>
   5e08b:	jmp    5e099 <probe::main+0x1fb9>
   5e08d:	nopl   (%rax)
   5e090:	add    $0x18,%r12
   5e094:	dec    %r14
   5e097:	je     5e055 <probe::main+0x1f75>
   5e099:	mov    -0x8(%r12),%rsi
   5e09e:	test   %rsi,%rsi
   5e0a1:	je     5e090 <probe::main+0x1fb0>
   5e0a3:	mov    (%r12),%rdi
   5e0a7:	mov    $0x1,%edx
   5e0ac:	call   *%r13
   5e0af:	jmp    5e090 <probe::main+0x1fb0>
   5e0b1:	mov    %rax,%r15
   5e0b4:	mov    $0x4,%esi
   5e0b9:	mov    $0x1,%edx
   5e0be:	mov    %r13,%rdi
   5e0c1:	call   *0x128501(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e0c7:	lea    0x100(%rsp),%rdi
   5e0cf:	call   5e870 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value> as core::ops::drop::Drop>::drop>
   5e0d4:	lea    0xc8(%rsp),%rdi
   5e0dc:	call   60710 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   5e0e1:	mov    0xc8(%rsp),%rsi
   5e0e9:	test   %rsi,%rsi
   5e0ec:	je     5e105 <probe::main+0x2025>
   5e0ee:	mov    0xd0(%rsp),%rdi
   5e0f6:	shl    $0x4,%rsi
   5e0fa:	mov    $0x8,%edx
   5e0ff:	call   *0x1284c3(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e105:	mov    0xb8(%rsp),%rsi
   5e10d:	test   %rsi,%rsi
   5e110:	je     5e125 <probe::main+0x2045>
   5e112:	mov    $0x1,%edx
   5e117:	mov    0x80(%rsp),%rdi
   5e11f:	call   *0x1284a3(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e125:	cmpb   $0x0,0x94(%rsp)
   5e12d:	je     5e13c <probe::main+0x205c>
   5e12f:	lea    0xe0(%rsp),%rdi
   5e137:	call   5bee0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e13c:	mov    0xb0(%rsp),%rax
   5e144:	lock decq (%rax)
   5e148:	jne    5e158 <probe::main+0x2078>
   5e14a:	lea    0xb0(%rsp),%rdi
   5e152:	call   *0x128558(%rip)        # 1866b0 <_DYNAMIC+0x328>
   5e158:	mov    0xf0(%rsp),%rsi
   5e160:	test   %rsi,%rsi
   5e163:	je     5e040 <probe::main+0x1f60>
   5e169:	shl    $0x3,%rsi
   5e16d:	mov    $0x8,%edx
   5e172:	mov    0xf8(%rsp),%rdi
   5e17a:	call   *0x128448(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e180:	jmp    5e040 <probe::main+0x1f60>
   5e185:	mov    0xc8(%rsp),%rsi
   5e18d:	test   %rsi,%rsi
   5e190:	je     5e1af <probe::main+0x20cf>
   5e192:	mov    0xd0(%rsp),%rdi
   5e19a:	shl    $0x4,%rsi
   5e19e:	mov    $0x8,%edx
   5e1a3:	call   *0x12841f(%rip)        # 1865c8 <_DYNAMIC+0x240>
   5e1a9:	call   *0x128421(%rip)        # 1865d0 <_DYNAMIC+0x248>
   5e1af:	call   *0x12841b(%rip)        # 1865d0 <_DYNAMIC+0x248>
