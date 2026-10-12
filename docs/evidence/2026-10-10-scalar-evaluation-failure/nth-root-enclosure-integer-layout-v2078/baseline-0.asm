
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1774-domain-predicate-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

000000000005ced0 <probe::main>:
   5ced0:	push   %rbp
   5ced1:	push   %r15
   5ced3:	push   %r14
   5ced5:	push   %r13
   5ced7:	push   %r12
   5ced9:	push   %rbx
   5ceda:	sub    $0x138,%rsp
   5cee1:	mov    %rsp,%rbx
   5cee4:	mov    %rbx,%rdi
   5cee7:	call   *0x1284eb(%rip)        # 1853d8 <_DYNAMIC+0x2a8>
   5ceed:	movq   $0x1,0x20(%rsp)
   5cef6:	lea    0xd0(%rsp),%rdi
   5cefe:	mov    %rbx,%rsi
   5cf01:	call   5f580 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::skip::Skip<std::env::Args>>>::from_iter>
   5cf06:	mov    0xe0(%rsp),%rdi
   5cf0e:	test   %rdi,%rdi
   5cf11:	je     5e43c <probe::main+0x156c>
   5cf17:	cmp    $0x1,%rdi
   5cf1b:	je     5e445 <probe::main+0x1575>
   5cf21:	mov    0xd8(%rsp),%r9
   5cf29:	mov    0x20(%r9),%rcx
   5cf2d:	mov    0x28(%r9),%rsi
   5cf31:	cmp    $0x1,%rsi
   5cf35:	je     5cf45 <probe::main+0x75>
   5cf37:	test   %rsi,%rsi
   5cf3a:	je     5e467 <probe::main+0x1597>
   5cf40:	movzbl (%rcx),%edx
   5cf43:	jmp    5cf5c <probe::main+0x8c>
   5cf45:	movzbl (%rcx),%edx
   5cf48:	mov    $0x1,%al
   5cf4a:	cmp    $0x2b,%edx
   5cf4d:	je     5e2d3 <probe::main+0x1403>
   5cf53:	cmp    $0x2d,%edx
   5cf56:	je     5e2d3 <probe::main+0x1403>
   5cf5c:	xor    %eax,%eax
   5cf5e:	cmp    $0x2b,%dl
   5cf61:	sete   %al
   5cf64:	mov    %rsi,%rdx
   5cf67:	sub    %rax,%rdx
   5cf6a:	add    %rax,%rcx
   5cf6d:	neg    %rax
   5cf70:	cmp    $0x11,%rdx
   5cf74:	jae    5cfb7 <probe::main+0xe7>
   5cf76:	test   %rdx,%rdx
   5cf79:	je     5d009 <probe::main+0x139>
   5cf7f:	add    %rax,%rsi
   5cf82:	neg    %rsi
   5cf85:	xor    %r15d,%r15d
   5cf88:	xor    %eax,%eax
   5cf8a:	nopw   0x0(%rax,%rax,1)
   5cf90:	movzbl (%rcx,%rax,1),%edx
   5cf94:	add    $0xffffffd0,%edx
   5cf97:	cmp    $0x9,%edx
   5cf9a:	ja     5e3f0 <probe::main+0x1520>
   5cfa0:	lea    (%r15,%r15,4),%rdi
   5cfa4:	mov    %edx,%edx
   5cfa6:	lea    (%rdx,%rdi,2),%r15
   5cfaa:	inc    %rax
   5cfad:	mov    %rsi,%rdx
   5cfb0:	add    %rax,%rdx
   5cfb3:	jne    5cf90 <probe::main+0xc0>
   5cfb5:	jmp    5d00c <probe::main+0x13c>
   5cfb7:	add    %rax,%rsi
   5cfba:	neg    %rsi
   5cfbd:	xor    %r15d,%r15d
   5cfc0:	mov    $0xa,%edi
   5cfc5:	xor    %r8d,%r8d
   5cfc8:	nopl   0x0(%rax,%rax,1)
   5cfd0:	mov    %rsi,%rax
   5cfd3:	add    %r8,%rax
   5cfd6:	je     5d00c <probe::main+0x13c>
   5cfd8:	mov    %r15,%rax
   5cfdb:	mul    %rdi
   5cfde:	movzbl (%rcx,%r8,1),%edx
   5cfe3:	jo     5e2c9 <probe::main+0x13f9>
   5cfe9:	add    $0xffffffd0,%edx
   5cfec:	cmp    $0xa,%edx
   5cfef:	jae    5e3f0 <probe::main+0x1520>
   5cff5:	mov    %rax,%r15
   5cff8:	mov    %edx,%eax
   5cffa:	inc    %r8
   5cffd:	add    %rax,%r15
   5d000:	jae    5cfd0 <probe::main+0x100>
   5d002:	mov    $0x2,%al
   5d004:	jmp    5e2d3 <probe::main+0x1403>
   5d009:	xor    %r15d,%r15d
   5d00c:	mov    0x10(%r9),%rcx
   5d010:	add    $0xfffffffffffffff8,%rcx
   5d014:	cmp    $0x6,%rcx
   5d018:	ja     5e3d2 <probe::main+0x1502>
   5d01e:	mov    0x8(%r9),%rax
   5d022:	lea    -0x2d6e9(%rip),%rdx        # 2f940 <GCC_except_table0+0x10>
   5d029:	movslq (%rdx,%rcx,4),%rcx
   5d02d:	add    %rdx,%rcx
   5d030:	mov    %r9,0x108(%rsp)
   5d038:	jmp    *%rcx
   5d03a:	movabs $0x7061675f65646977,%rcx
   5d044:	cmp    %rcx,(%rax)
   5d047:	jne    5e3d2 <probe::main+0x1502>
   5d04d:	mov    $0x7fffffff,%eax
   5d052:	mov    $0xf4241,%ecx
   5d057:	jmp    5d19d <probe::main+0x2cd>
   5d05c:	movabs $0x6f635f6c6c616d73,%rcx
   5d066:	xor    (%rax),%rcx
   5d069:	mov    0x8(%rax),%edx
   5d06c:	xor    $0x65737261,%rdx
   5d073:	or     %rcx,%rdx
   5d076:	je     5d193 <probe::main+0x2c3>
   5d07c:	movabs $0x6c61685f65646977,%rcx
   5d086:	xor    (%rax),%rcx
   5d089:	mov    0x8(%rax),%eax
   5d08c:	xor    $0x79617766,%rax
   5d092:	or     %rcx,%rax
   5d095:	jne    5e3d2 <probe::main+0x1502>
   5d09b:	mov    $0x1,%r12b
   5d09e:	mov    $0xf4241,%eax
   5d0a3:	mov    %rax,0x50(%rsp)
   5d0a8:	mov    $0xf4241,%eax
   5d0ad:	jmp    5d1a5 <probe::main+0x2d5>
   5d0b2:	movabs $0x69665f6c6c616d73,%rcx
   5d0bc:	xor    (%rax),%rcx
   5d0bf:	movzwl 0x8(%rax),%eax
   5d0c3:	xor    $0x656e,%rax
   5d0c9:	or     %rcx,%rax
   5d0cc:	jne    5e3d2 <probe::main+0x1502>
   5d0d2:	mov    $0xffffff80,%eax
   5d0d7:	jmp    5d198 <probe::main+0x2c8>
   5d0dc:	movabs $0x64695f6c6c616d73,%rcx
   5d0e6:	xor    (%rax),%rcx
   5d0e9:	movabs $0x797469746e656469,%rdx
   5d0f3:	xor    0x6(%rax),%rdx
   5d0f7:	or     %rcx,%rdx
   5d0fa:	jne    5e3d2 <probe::main+0x1502>
   5d100:	xor    %eax,%eax
   5d102:	mov    $0x41,%ecx
   5d107:	jmp    5d19d <probe::main+0x2cd>
   5d10c:	movabs $0x616f635f65646977,%rcx
   5d116:	xor    (%rax),%rcx
   5d119:	movabs $0x657372616f635f65,%rdx
   5d123:	xor    0x3(%rax),%rdx
   5d127:	or     %rcx,%rdx
   5d12a:	jne    5e3d2 <probe::main+0x1502>
   5d130:	mov    $0xf4240,%eax
   5d135:	mov    $0xf4241,%ecx
   5d13a:	jmp    5d19d <probe::main+0x2cd>
   5d13c:	movabs $0x6e69665f65646977,%rcx
   5d146:	xor    (%rax),%rcx
   5d149:	movzbl 0x8(%rax),%eax
   5d14d:	xor    $0x65,%rax
   5d151:	or     %rcx,%rax
   5d154:	jne    5e3d2 <probe::main+0x1502>
   5d15a:	mov    $0xffffff80,%eax
   5d15f:	mov    $0x10001,%ecx
   5d164:	jmp    5d19d <probe::main+0x2cd>
   5d166:	movabs $0x6564695f65646977,%rcx
   5d170:	xor    (%rax),%rcx
   5d173:	movabs $0x797469746e656469,%rdx
   5d17d:	xor    0x5(%rax),%rdx
   5d181:	or     %rcx,%rdx
   5d184:	jne    5e3d2 <probe::main+0x1502>
   5d18a:	xor    %eax,%eax
   5d18c:	mov    $0x10001,%ecx
   5d191:	jmp    5d19d <probe::main+0x2cd>
   5d193:	mov    $0x1,%eax
   5d198:	mov    $0x8,%ecx
   5d19d:	mov    %rcx,0x50(%rsp)
   5d1a2:	xor    %r12d,%r12d
   5d1a5:	mov    %eax,0x74(%rsp)
   5d1a9:	call   *0x128231(%rip)        # 1853e0 <_DYNAMIC+0x2b0>
   5d1af:	mov    %rax,0x30(%rsp)
   5d1b4:	mov    %rdx,0x38(%rsp)
   5d1b9:	mov    %rsp,%rdi
   5d1bc:	lea    0x30(%rsp),%rsi
   5d1c1:	xor    %edx,%edx
   5d1c3:	call   *0x12821f(%rip)        # 1853e8 <_DYNAMIC+0x2b8>
   5d1c9:	mov    (%rsp),%rax
   5d1cd:	mov    %rax,0xc8(%rsp)
   5d1d5:	cmp    $0xffffffffffffffff,%rax
   5d1d9:	je     5e32b <probe::main+0x145b>
   5d1df:	lea    0x38(%rsp),%rbx
   5d1e4:	mov    0x8(%rsp),%rax
   5d1e9:	mov    %rax,0x78(%rsp)
   5d1ee:	mov    0x10(%rsp),%rax
   5d1f3:	mov    %rax,0x80(%rsp)
   5d1fb:	movzbl 0x18(%rsp),%ebp
   5d200:	mov    0x30(%rsp),%rax
   5d205:	lock decq (%rax)
   5d209:	jne    5d216 <probe::main+0x346>
   5d20b:	lea    0x30(%rsp),%rdi
   5d210:	call   *0x1281b2(%rip)        # 1853c8 <_DYNAMIC+0x298>
   5d216:	mov    0x38(%rsp),%rax
   5d21b:	test   %rax,%rax
   5d21e:	je     5d22f <probe::main+0x35f>
   5d220:	lock decq (%rax)
   5d224:	jne    5d22f <probe::main+0x35f>
   5d226:	mov    %rbx,%rdi
   5d229:	call   *0x1281a1(%rip)        # 1853d0 <_DYNAMIC+0x2a0>
   5d22f:	call   *0x1281bb(%rip)        # 1853f0 <_DYNAMIC+0x2c0>
   5d235:	mov    %rax,0x90(%rsp)
   5d23d:	mov    %rdx,0x98(%rsp)
   5d245:	mov    %rsp,%rdi
   5d248:	lea    0x90(%rsp),%rsi
   5d250:	xor    %edx,%edx
   5d252:	call   *0x128190(%rip)        # 1853e8 <_DYNAMIC+0x2b8>
   5d258:	mov    (%rsp),%rax
   5d25c:	cmp    $0xffffffffffffffff,%rax
   5d260:	je     5e364 <probe::main+0x1494>
   5d266:	mov    0x8(%rsp),%rcx
   5d26b:	mov    0x10(%rsp),%rsi
   5d270:	movzbl 0x18(%rsp),%r14d
   5d276:	test   %rsi,%rsi
   5d279:	je     5d2e7 <probe::main+0x417>
   5d27b:	mov    0x50(%rsp),%rdx
   5d280:	lea    -0x1(%rdx),%edi
   5d283:	mov    %edi,%edx
   5d285:	shr    $0x6,%edx
   5d288:	and    $0x3f,%dil
   5d28c:	mov    %rax,(%rsp)
   5d290:	mov    %rcx,0x8(%rsp)
   5d295:	mov    %rsi,0x10(%rsp)
   5d29a:	movzbl %dil,%ecx
   5d29e:	lea    0x30(%rsp),%rdi
   5d2a3:	mov    %rsp,%rsi
   5d2a6:	call   *0x12814c(%rip)        # 1853f8 <_DYNAMIC+0x2c8>
   5d2ac:	cmp    $0x1,%r14b
   5d2b0:	jne    5d300 <probe::main+0x430>
   5d2b2:	mov    0x30(%rsp),%r13
   5d2b7:	mov    $0x1,%r14b
   5d2ba:	cmp    $0x4,%r13
   5d2be:	jb     5d2e3 <probe::main+0x413>
   5d2c0:	mov    0x38(%rsp),%rdi
   5d2c5:	shl    $0x3,%r13
   5d2c9:	mov    $0x8,%edx
   5d2ce:	mov    %r13,%rsi
   5d2d1:	call   *0x1280e1(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5d2d7:	movq   $0x8,0x38(%rsp)
   5d2e0:	xor    %r13d,%r13d
   5d2e3:	xor    %ebx,%ebx
   5d2e5:	jmp    5d31b <probe::main+0x44b>
   5d2e7:	mov    %rax,0x30(%rsp)
   5d2ec:	mov    %rcx,0x38(%rsp)
   5d2f1:	movq   $0x0,0x40(%rsp)
   5d2fa:	cmp    $0x1,%r14b
   5d2fe:	je     5d2b2 <probe::main+0x3e2>
   5d300:	mov    0x30(%rsp),%r13
   5d305:	mov    0x40(%rsp),%rbx
   5d30a:	test   %rbx,%rbx
   5d30d:	movzbl %r14b,%eax
   5d311:	mov    $0x1,%r14d
   5d317:	cmovne %eax,%r14d
   5d31b:	mov    0x38(%rsp),%rax
   5d320:	mov    %rax,0x28(%rsp)
   5d325:	mov    0x90(%rsp),%rax
   5d32d:	lock decq (%rax)
   5d331:	jne    5d341 <probe::main+0x471>
   5d333:	lea    0x90(%rsp),%rdi
   5d33b:	call   *0x128087(%rip)        # 1853c8 <_DYNAMIC+0x298>
   5d341:	mov    0x98(%rsp),%rax
   5d349:	test   %rax,%rax
   5d34c:	je     5d362 <probe::main+0x492>
   5d34e:	lock decq (%rax)
   5d352:	jne    5d362 <probe::main+0x492>
   5d354:	lea    0x98(%rsp),%rdi
   5d35c:	call   *0x12806e(%rip)        # 1853d0 <_DYNAMIC+0x2a0>
   5d362:	test   %r12b,%r12b
   5d365:	je     5d384 <probe::main+0x4b4>
   5d367:	mov    %r13,(%rsp)
   5d36b:	mov    0x28(%rsp),%rax
   5d370:	mov    %rax,0x8(%rsp)
   5d375:	mov    %rbx,0x10(%rsp)
   5d37a:	mov    %r14b,0x18(%rsp)
   5d37f:	jmp    5d7ea <probe::main+0x91a>
   5d384:	test   %r14b,%r14b
   5d387:	je     5d3e3 <probe::main+0x513>
   5d389:	movzbl %r14b,%eax
   5d38d:	cmp    $0x1,%eax
   5d390:	jne    5d4a8 <probe::main+0x5d8>
   5d396:	lea    0x38(%rsp),%r14
   5d39b:	movups -0x2a552(%rip),%xmm0        # 32e50 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x90>
   5d3a2:	movaps %xmm0,0x30(%rsp)
   5d3a7:	movq   $0x0,0x40(%rsp)
   5d3b0:	lea    0x30(%rsp),%rdi
   5d3b5:	call   *0x128045(%rip)        # 185400 <_DYNAMIC+0x2d0>
   5d3bb:	mov    0x38(%rsp),%rax
   5d3c0:	movq   $0x1,(%rax)
   5d3c7:	movaps 0x30(%rsp),%xmm0
   5d3cc:	movaps %xmm0,(%rsp)
   5d3d0:	movq   $0x1,0x10(%rsp)
   5d3d9:	movb   $0x2,0x18(%rsp)
   5d3de:	jmp    5d4ff <probe::main+0x62f>
   5d3e3:	lea    0x8(%rsp),%r14
   5d3e8:	movups -0x2a59f(%rip),%xmm0        # 32e50 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x90>
   5d3ef:	movaps %xmm0,(%rsp)
   5d3f3:	movq   $0x0,0x10(%rsp)
   5d3fc:	mov    %rsp,%rdi
   5d3ff:	call   *0x127ffb(%rip)        # 185400 <_DYNAMIC+0x2d0>
   5d405:	cmp    $0x2,%rbx
   5d409:	setae  %al
   5d40c:	cmp    $0x1,%rbx
   5d410:	sbb    $0x0,%al
   5d412:	mov    0x8(%rsp),%rcx
   5d417:	movq   $0x1,(%rcx)
   5d41e:	mov    (%rsp),%r12
   5d422:	mov    0x8(%rsp),%r14
   5d427:	cmp    $0x1,%rbx
   5d42b:	jne    5d443 <probe::main+0x573>
   5d42d:	mov    (%r14),%rax
   5d430:	mov    0x28(%rsp),%rcx
   5d435:	cmp    %rax,(%rcx)
   5d438:	je     5d4ce <probe::main+0x5fe>
   5d43e:	seta   %al
   5d441:	sbb    $0x0,%al
   5d443:	test   %al,%al
   5d445:	je     5d4ce <probe::main+0x5fe>
   5d44b:	movzbl %al,%eax
   5d44e:	cmp    $0x1,%eax
   5d451:	jne    5d610 <probe::main+0x740>
   5d457:	mov    %r13,(%rsp)
   5d45b:	mov    0x28(%rsp),%rax
   5d460:	mov    %rax,0x8(%rsp)
   5d465:	mov    %rbx,0x10(%rsp)
   5d46a:	mov    %rsp,%rdi
   5d46d:	mov    $0x1,%esi
   5d472:	call   *0x127f90(%rip)        # 185408 <_DYNAMIC+0x2d8>
   5d478:	mov    (%rsp),%rsi
   5d47c:	mov    0x8(%rsp),%rdi
   5d481:	mov    0x10(%rsp),%rax
   5d486:	test   %rax,%rax
   5d489:	je     5d760 <probe::main+0x890>
   5d48f:	mov    %rsi,(%rsp)
   5d493:	mov    %rdi,0x8(%rsp)
   5d498:	mov    %rax,0x10(%rsp)
   5d49d:	xor    %ebx,%ebx
   5d49f:	mov    %bl,0x18(%rsp)
   5d4a3:	jmp    5d7d0 <probe::main+0x900>
   5d4a8:	mov    %r13,(%rsp)
   5d4ac:	mov    0x28(%rsp),%rax
   5d4b1:	mov    %rax,0x8(%rsp)
   5d4b6:	mov    %rbx,0x10(%rsp)
   5d4bb:	test   %rbx,%rbx
   5d4be:	je     5d51e <probe::main+0x64e>
   5d4c0:	incq   (%rax)
   5d4c3:	je     5d554 <probe::main+0x684>
   5d4c9:	jmp    5d5e5 <probe::main+0x715>
   5d4ce:	movups -0x2a705(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   5d4d5:	movaps %xmm0,0x10(%rsp)
   5d4da:	movups -0x2a721(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   5d4e1:	movaps %xmm0,(%rsp)
   5d4e5:	test   %r12,%r12
   5d4e8:	je     5d4ff <probe::main+0x62f>
   5d4ea:	shl    $0x3,%r12
   5d4ee:	mov    $0x8,%edx
   5d4f3:	mov    %r14,%rdi
   5d4f6:	mov    %r12,%rsi
   5d4f9:	call   *0x127eb9(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5d4ff:	test   %r13,%r13
   5d502:	je     5d7ea <probe::main+0x91a>
   5d508:	shl    $0x3,%r13
   5d50c:	mov    $0x8,%edx
   5d511:	mov    0x28(%rsp),%rdi
   5d516:	mov    %r13,%rsi
   5d519:	jmp    5d7e4 <probe::main+0x914>
   5d51e:	test   %r13,%r13
   5d521:	jne    5d531 <probe::main+0x661>
   5d523:	mov    %rsp,%rdi
   5d526:	call   *0x127ed4(%rip)        # 185400 <_DYNAMIC+0x2d0>
   5d52c:	mov    0x8(%rsp),%rax
   5d531:	movq   $0x0,(%rax)
   5d538:	movq   $0x1,0x10(%rsp)
   5d541:	mov    $0x1,%ebx
   5d546:	mov    0x8(%rsp),%rax
   5d54b:	incq   (%rax)
   5d54e:	jne    5d5e5 <probe::main+0x715>
   5d554:	mov    $0x1,%r14d
   5d55a:	cmp    $0x1,%rbx
   5d55e:	je     5d589 <probe::main+0x6b9>
   5d560:	mov    %rax,%rdx
   5d563:	shl    $0x3,%rbx
   5d567:	mov    $0x1,%al
   5d569:	mov    $0x8,%ecx
   5d56e:	xchg   %ax,%ax
   5d570:	add    $0xff,%al
   5d572:	adcq   $0x0,(%rdx,%rcx,1)
   5d577:	setb   %al
   5d57a:	jae    5d5e5 <probe::main+0x715>
   5d57c:	add    $0x8,%rcx
   5d580:	cmp    %rcx,%rbx
   5d583:	jne    5d570 <probe::main+0x6a0>
   5d585:	movzbl %al,%r14d
   5d589:	mov    0x10(%rsp),%rbx
   5d58e:	cmp    (%rsp),%rbx
   5d592:	jne    5d59d <probe::main+0x6cd>
   5d594:	mov    %rsp,%rdi
   5d597:	call   *0x127e63(%rip)        # 185400 <_DYNAMIC+0x2d0>
   5d59d:	mov    0x8(%rsp),%rax
   5d5a2:	mov    %r14,(%rax,%rbx,8)
   5d5a6:	inc    %rbx
   5d5a9:	mov    (%rsp),%rsi
   5d5ad:	mov    0x8(%rsp),%rdi
   5d5b2:	test   %rbx,%rbx
   5d5b5:	jne    5d5f8 <probe::main+0x728>
   5d5b7:	movups -0x2a7ee(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   5d5be:	movaps %xmm0,0x10(%rsp)
   5d5c3:	movups -0x2a80a(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   5d5ca:	movaps %xmm0,(%rsp)
   5d5ce:	test   %rsi,%rsi
   5d5d1:	je     5d7ea <probe::main+0x91a>
   5d5d7:	shl    $0x3,%rsi
   5d5db:	mov    $0x8,%edx
   5d5e0:	jmp    5d7e4 <probe::main+0x914>
   5d5e5:	mov    0x10(%rsp),%rbx
   5d5ea:	mov    (%rsp),%rsi
   5d5ee:	mov    0x8(%rsp),%rdi
   5d5f3:	test   %rbx,%rbx
   5d5f6:	je     5d5b7 <probe::main+0x6e7>
   5d5f8:	mov    %rsi,(%rsp)
   5d5fc:	mov    %rdi,0x8(%rsp)
   5d601:	mov    %rbx,0x10(%rsp)
   5d606:	movb   $0x2,0x18(%rsp)
   5d60b:	jmp    5d7ea <probe::main+0x91a>
   5d610:	mov    %r13,(%rsp)
   5d614:	mov    0x28(%rsp),%rdx
   5d619:	mov    %rdx,0x8(%rsp)
   5d61e:	mov    %rbx,0x10(%rsp)
   5d623:	test   %rbx,%rbx
   5d626:	mov    %r14,0x88(%rsp)
   5d62e:	je     5d664 <probe::main+0x794>
   5d630:	movq   $0x1,0x30(%rsp)
   5d639:	lea    0x30(%rsp),%rdi
   5d63e:	mov    $0x1,%esi
   5d643:	mov    %rbx,%rcx
   5d646:	call   *0x127dc4(%rip)        # 185410 <_DYNAMIC+0x2e0>
   5d64c:	mov    0x10(%rsp),%r14
   5d651:	mov    (%rsp),%r13
   5d655:	mov    0x8(%rsp),%rbx
   5d65a:	test   %r14,%r14
   5d65d:	jne    5d6a0 <probe::main+0x7d0>
   5d65f:	xor    %r14d,%r14d
   5d662:	jmp    5d6e0 <probe::main+0x810>
   5d664:	test   %r13,%r13
   5d667:	jne    5d67c <probe::main+0x7ac>
   5d669:	mov    %rsp,%rdi
   5d66c:	call   *0x127d8e(%rip)        # 185400 <_DYNAMIC+0x2d0>
   5d672:	mov    0x8(%rsp),%rax
   5d677:	mov    %rax,0x28(%rsp)
   5d67c:	mov    0x28(%rsp),%rax
   5d681:	movq   $0x1,(%rax)
   5d688:	movq   $0x1,0x10(%rsp)
   5d691:	mov    (%rsp),%r13
   5d695:	mov    0x8(%rsp),%rbx
   5d69a:	mov    $0x1,%r14d
   5d6a0:	cmpq   $0x0,-0x8(%rbx,%r14,8)
   5d6a6:	jne    5d6e0 <probe::main+0x810>
   5d6a8:	movabs $0x1fffffffffffffff,%rax
   5d6b2:	and    %r14,%rax
   5d6b5:	lea    0x0(,%r14,8),%rdx
   5d6bd:	xor    %ecx,%ecx
   5d6bf:	test   %rdx,%rdx
   5d6c2:	je     5d65f <probe::main+0x78f>
   5d6c4:	inc    %rcx
   5d6c7:	cmpq   $0x0,-0x8(%rbx,%rdx,1)
   5d6cd:	lea    -0x8(%rdx),%rdx
   5d6d1:	je     5d6bf <probe::main+0x7ef>
   5d6d3:	sub    %rcx,%rax
   5d6d6:	inc    %rax
   5d6d9:	cmp    %r14,%rax
   5d6dc:	cmovb  %rax,%r14
   5d6e0:	mov    %r13,%rax
   5d6e3:	shr    $0x2,%rax
   5d6e7:	cmp    %rax,%r14
   5d6ea:	jae    5d73e <probe::main+0x86e>
   5d6ec:	cmp    %r13,%r14
   5d6ef:	ja     5e409 <probe::main+0x1539>
   5d6f5:	test   %r13,%r13
   5d6f8:	je     5d73e <probe::main+0x86e>
   5d6fa:	shl    $0x3,%r13
   5d6fe:	test   %r14,%r14
   5d701:	je     5e2fe <probe::main+0x142e>
   5d707:	mov    %r12,0x28(%rsp)
   5d70c:	lea    0x0(,%r14,8),%r12
   5d714:	mov    $0x8,%edx
   5d719:	mov    %rbx,%rdi
   5d71c:	mov    %r13,%rsi
   5d71f:	mov    %r12,%rcx
   5d722:	call   *0x127cf0(%rip)        # 185418 <_DYNAMIC+0x2e8>
   5d728:	test   %rax,%rax
   5d72b:	je     5e4aa <probe::main+0x15da>
   5d731:	mov    %rax,%rbx
   5d734:	mov    %r14,%r13
   5d737:	mov    0x28(%rsp),%r12
   5d73c:	jmp    5d743 <probe::main+0x873>
   5d73e:	test   %r14,%r14
   5d741:	je     5d797 <probe::main+0x8c7>
   5d743:	mov    %r13,(%rsp)
   5d747:	mov    %rbx,0x8(%rsp)
   5d74c:	mov    %r14,0x10(%rsp)
   5d751:	movb   $0x2,0x18(%rsp)
   5d756:	mov    0x88(%rsp),%r14
   5d75e:	jmp    5d7d0 <probe::main+0x900>
   5d760:	movups -0x2a997(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   5d767:	movaps %xmm0,0x10(%rsp)
   5d76c:	movups -0x2a9b3(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   5d773:	movaps %xmm0,(%rsp)
   5d777:	mov    $0x1,%bl
   5d779:	test   %rsi,%rsi
   5d77c:	je     5d49f <probe::main+0x5cf>
   5d782:	shl    $0x3,%rsi
   5d786:	mov    $0x8,%edx
   5d78b:	call   *0x127c27(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5d791:	mov    %bl,0x18(%rsp)
   5d795:	jmp    5d7d0 <probe::main+0x900>
   5d797:	movups -0x2a9ce(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   5d79e:	movaps %xmm0,0x10(%rsp)
   5d7a3:	movups -0x2a9ea(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   5d7aa:	movaps %xmm0,(%rsp)
   5d7ae:	test   %r13,%r13
   5d7b1:	mov    0x88(%rsp),%r14
   5d7b9:	je     5d7d0 <probe::main+0x900>
   5d7bb:	shl    $0x3,%r13
   5d7bf:	mov    $0x8,%edx
   5d7c4:	mov    %rbx,%rdi
   5d7c7:	mov    %r13,%rsi
   5d7ca:	call   *0x127be8(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5d7d0:	test   %r12,%r12
   5d7d3:	je     5d7ea <probe::main+0x91a>
   5d7d5:	shl    $0x3,%r12
   5d7d9:	mov    $0x8,%edx
   5d7de:	mov    %r14,%rdi
   5d7e1:	mov    %r12,%rsi
   5d7e4:	call   *0x127bce(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5d7ea:	mov    %rsp,%rdi
   5d7ed:	call   *0x127c2d(%rip)        # 185420 <_DYNAMIC+0x2f0>
   5d7f3:	mov    %rax,%rdi
   5d7f6:	call   *0x127c2c(%rip)        # 185428 <_DYNAMIC+0x2f8>
   5d7fc:	mov    %rax,%r12
   5d7ff:	mov    %rdx,%rbx
   5d802:	mov    %rax,0x118(%rsp)
   5d80a:	mov    %rdx,0x120(%rsp)
   5d812:	lock incq (%rax)
   5d816:	jle    5e4b8 <probe::main+0x15e8>
   5d81c:	test   %rbx,%rbx
   5d81f:	je     5d82b <probe::main+0x95b>
   5d821:	lock incq (%rbx)
   5d825:	jle    5e4b8 <probe::main+0x15e8>
   5d82b:	mov    %r12,%rdi
   5d82e:	mov    %rbx,%rsi
   5d831:	call   *0x127bf9(%rip)        # 185430 <_DYNAMIC+0x300>
   5d837:	mov    %r12,0x90(%rsp)
   5d83f:	mov    %rbx,0x98(%rsp)
   5d847:	mov    %rax,0xa0(%rsp)
   5d84f:	mov    %rdx,0xa8(%rsp)
   5d857:	lea    0xb0(%rsp),%rax
   5d85f:	lea    0x90(%rsp),%rcx
   5d867:	mov    %rcx,(%rsp)
   5d86b:	mov    %rax,0x8(%rsp)
   5d870:	lea    0x74(%rsp),%rax
   5d875:	mov    %rax,0x10(%rsp)
   5d87a:	lea    0xe8(%rsp),%rdi
   5d882:	mov    %rsp,%rsi
   5d885:	call   5fb30 <<alloc::vec::Vec<num_bigint::bigint::BigInt> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<num_bigint::bigint::BigInt, core::iter::adapters::map::Map<core::slice::iter::Iter<hyperreal::computable::node::Computable>, probe::main::{closure#0}>>>::from_iter>
   5d88a:	test   %r15,%r15
   5d88d:	je     5d9b2 <probe::main+0xae2>
   5d893:	mov    0xf0(%rsp),%rax
   5d89b:	mov    0xf8(%rsp),%rsi
   5d8a3:	xor    %r12d,%r12d
   5d8a6:	xor    %ecx,%ecx
   5d8a8:	jmp    5d8cd <probe::main+0x9fd>
   5d8aa:	nopw   0x0(%rax,%rax,1)
   5d8b0:	sub    %r10b,%dil
   5d8b3:	inc    %rcx
   5d8b6:	shr    $0x7,%dil
   5d8ba:	movzbl %dil,%edi
   5d8be:	add    %rdx,%r12
   5d8c1:	add    %rdi,%r12
   5d8c4:	cmp    %r15,%rcx
   5d8c7:	je     5d9b5 <probe::main+0xae5>
   5d8cd:	mov    %ecx,%edi
   5d8cf:	and    $0x1,%edi
   5d8d2:	cmp    %rsi,%rdi
   5d8d5:	jae    5e3f7 <probe::main+0x1527>
   5d8db:	shl    $0x5,%edi
   5d8de:	mov    0x8(%rax,%rdi,1),%r8
   5d8e3:	mov    0x10(%rax,%rdi,1),%r9
   5d8e8:	test   %r9,%r9
   5d8eb:	je     5d910 <probe::main+0xa40>
   5d8ed:	mov    $0x7f,%edx
   5d8f2:	bsr    -0x8(%r8,%r9,8),%rdx
   5d8f8:	xor    $0x3f,%rdx
   5d8fc:	mov    %r9,%r10
   5d8ff:	shl    $0x6,%r10
   5d903:	sub    %rdx,%r10
   5d906:	lea    (%r10,%r10,2),%rdx
   5d90a:	jmp    5d912 <probe::main+0xa42>
   5d90c:	nopl   0x0(%rax)
   5d910:	xor    %edx,%edx
   5d912:	add    %rax,%rdi
   5d915:	movzbl 0x18(%rdi),%r11d
   5d91a:	cmp    %rbp,%r11
   5d91d:	setl   %r10b
   5d921:	setg   %dil
   5d925:	cmp    %bpl,%r11b
   5d928:	jne    5d8b0 <probe::main+0x9e0>
   5d92a:	cmp    $0x2,%rbp
   5d92e:	je     5d93c <probe::main+0xa6c>
   5d930:	cmp    $0x1,%ebp
   5d933:	jne    5d962 <probe::main+0xa92>
   5d935:	xor    %edi,%edi
   5d937:	jmp    5d8b3 <probe::main+0x9e3>
   5d93c:	mov    0x80(%rsp),%r10
   5d944:	cmp    %r10,%r9
   5d947:	seta   %dil
   5d94b:	sbb    $0x0,%dil
   5d94f:	cmp    %r10,%r9
   5d952:	jne    5d8b3 <probe::main+0x9e3>
   5d958:	mov    %r8,%rdi
   5d95b:	mov    0x78(%rsp),%r8
   5d960:	jmp    5d983 <probe::main+0xab3>
   5d962:	mov    0x80(%rsp),%r10
   5d96a:	cmp    %r9,%r10
   5d96d:	seta   %dil
   5d971:	sbb    $0x0,%dil
   5d975:	cmp    %r9,%r10
   5d978:	jne    5d8b3 <probe::main+0x9e3>
   5d97e:	mov    0x78(%rsp),%rdi
   5d983:	shl    $0x3,%r9
   5d987:	nopw   0x0(%rax,%rax,1)
   5d990:	test   %r9,%r9
   5d993:	je     5d935 <probe::main+0xa65>
   5d995:	mov    -0x8(%r8,%r9,1),%r10
   5d99a:	cmp    %r10,-0x8(%rdi,%r9,1)
   5d99f:	lea    -0x8(%r9),%r9
   5d9a3:	je     5d990 <probe::main+0xac0>
   5d9a5:	seta   %dil
   5d9a9:	sbb    $0x0,%dil
   5d9ad:	jmp    5d8b3 <probe::main+0x9e3>
   5d9b2:	xor    %r12d,%r12d
   5d9b5:	mov    %r12,0x110(%rsp)
   5d9bd:	movq   $0x0,0x100(%rsp)
   5d9c9:	call   *0x127a69(%rip)        # 185438 <_DYNAMIC+0x308>
   5d9cf:	mov    %rax,0x128(%rsp)
   5d9d7:	mov    %edx,0x130(%rsp)
   5d9de:	xor    %r13d,%r13d
   5d9e1:	test   %r15,%r15
   5d9e4:	je     5db54 <probe::main+0xc84>
   5d9ea:	mov    %rsp,%rbx
   5d9ed:	xor    %r14d,%r14d
   5d9f0:	jmp    5da0c <probe::main+0xb3c>
   5d9f2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5da00:	inc    %r14
   5da03:	cmp    %r15,%r14
   5da06:	je     5db54 <probe::main+0xc84>
   5da0c:	mov    %r14d,%eax
   5da0f:	and    $0x1,%eax
   5da12:	shl    $0x4,%eax
   5da15:	lea    (%rsp,%rax,1),%rsi
   5da19:	add    $0x90,%rsi
   5da20:	mov    0x74(%rsp),%edx
   5da24:	mov    %rbx,%rdi
   5da27:	call   *0x1279bb(%rip)        # 1853e8 <_DYNAMIC+0x2b8>
   5da2d:	cmpq   $0xffffffffffffffff,(%rsp)
   5da32:	je     5e296 <probe::main+0x13c6>
   5da38:	movups (%rsp),%xmm0
   5da3c:	movups 0x10(%rsp),%xmm1
   5da41:	movaps %xmm1,0x40(%rsp)
   5da46:	movaps %xmm0,0x30(%rsp)
   5da4b:	lea    0x30(%rsp),%rax
   5da50:	mov    0x38(%rsp),%rdi
   5da55:	mov    0x40(%rsp),%rcx
   5da5a:	test   %rcx,%rcx
   5da5d:	je     5da80 <probe::main+0xbb0>
   5da5f:	mov    $0x7f,%eax
   5da64:	bsr    -0x8(%rdi,%rcx,8),%rax
   5da6a:	xor    $0x3f,%rax
   5da6e:	mov    %rcx,%rdx
   5da71:	shl    $0x6,%rdx
   5da75:	sub    %rax,%rdx
   5da78:	lea    (%rdx,%rdx,2),%rax
   5da7c:	jmp    5da82 <probe::main+0xbb2>
   5da7e:	xchg   %ax,%ax
   5da80:	xor    %eax,%eax
   5da82:	movzbl 0x48(%rsp),%r8d
   5da88:	cmp    %rbp,%r8
   5da8b:	setl   %sil
   5da8f:	setg   %dl
   5da92:	cmp    %bpl,%r8b
   5da95:	jne    5dab0 <probe::main+0xbe0>
   5da97:	cmp    $0x2,%rbp
   5da9b:	je     5dae9 <probe::main+0xc19>
   5da9d:	cmp    $0x1,%ebp
   5daa0:	jne    5db09 <probe::main+0xc39>
   5daa2:	xor    %edx,%edx
   5daa4:	jmp    5dab3 <probe::main+0xbe3>
   5daa6:	cs nopw 0x0(%rax,%rax,1)
   5dab0:	sub    %sil,%dl
   5dab3:	shr    $0x7,%dl
   5dab6:	movzbl %dl,%ecx
   5dab9:	add    %rax,%r13
   5dabc:	add    %rcx,%r13
   5dabf:	mov    %r13,0x100(%rsp)
   5dac7:	mov    0x30(%rsp),%rsi
   5dacc:	test   %rsi,%rsi
   5dacf:	je     5da00 <probe::main+0xb30>
   5dad5:	shl    $0x3,%rsi
   5dad9:	mov    $0x8,%edx
   5dade:	call   *0x1278d4(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5dae4:	jmp    5da00 <probe::main+0xb30>
   5dae9:	mov    0x80(%rsp),%rsi
   5daf1:	cmp    %rsi,%rcx
   5daf4:	seta   %dl
   5daf7:	sbb    $0x0,%dl
   5dafa:	cmp    %rsi,%rcx
   5dafd:	jne    5dab3 <probe::main+0xbe3>
   5daff:	mov    %rdi,%rdx
   5db02:	mov    0x78(%rsp),%rsi
   5db07:	jmp    5db27 <probe::main+0xc57>
   5db09:	mov    0x80(%rsp),%rsi
   5db11:	cmp    %rcx,%rsi
   5db14:	seta   %dl
   5db17:	sbb    $0x0,%dl
   5db1a:	cmp    %rcx,%rsi
   5db1d:	jne    5dab3 <probe::main+0xbe3>
   5db1f:	mov    0x78(%rsp),%rdx
   5db24:	mov    %rdi,%rsi
   5db27:	shl    $0x3,%rcx
   5db2b:	nopl   0x0(%rax,%rax,1)
   5db30:	test   %rcx,%rcx
   5db33:	je     5daa2 <probe::main+0xbd2>
   5db39:	mov    -0x8(%rsi,%rcx,1),%r8
   5db3e:	cmp    %r8,-0x8(%rdx,%rcx,1)
   5db43:	lea    -0x8(%rcx),%rcx
   5db47:	je     5db30 <probe::main+0xc60>
   5db49:	seta   %dl
   5db4c:	sbb    $0x0,%dl
   5db4f:	jmp    5dab3 <probe::main+0xbe3>
   5db54:	lea    0x128(%rsp),%rdi
   5db5c:	call   *0x1278de(%rip)        # 185440 <_DYNAMIC+0x310>
   5db62:	mov    %edx,%ecx
   5db64:	mov    $0x3b9aca00,%edx
   5db69:	mul    %rdx
   5db6c:	mov    %ecx,%ecx
   5db6e:	add    %rax,%rcx
   5db71:	adc    $0x0,%rdx
   5db75:	cmp    %r12,%r13
   5db78:	jne    5e397 <probe::main+0x14c7>
   5db7e:	mov    %rcx,%r13
   5db81:	mov    %rdx,0x80(%rsp)
   5db89:	movq   $0x0,0xb0(%rsp)
   5db95:	movq   $0x0,0xc0(%rsp)
   5dba1:	call   *0x1277c9(%rip)        # 185370 <_DYNAMIC+0x240>
   5dba7:	mov    $0x4,%ebx
   5dbac:	mov    $0x1,%ebp
   5dbb1:	mov    $0x4,%edi
   5dbb6:	mov    $0x1,%esi
   5dbbb:	call   *0x1277b7(%rip)        # 185378 <_DYNAMIC+0x248>
   5dbc1:	test   %rax,%rax
   5dbc4:	je     5e42c <probe::main+0x155c>
   5dbca:	movl   $0x65736163,(%rax)
   5dbd0:	movq   $0x4,0x58(%rsp)
   5dbd9:	mov    %rax,0x60(%rsp)
   5dbde:	movq   $0x4,0x68(%rsp)
   5dbe7:	mov    0x108(%rsp),%rcx
   5dbef:	mov    0x10(%rcx),%rbx
   5dbf3:	test   %rbx,%rbx
   5dbf6:	je     5dc30 <probe::main+0xd60>
   5dbf8:	mov    %rax,0x28(%rsp)
   5dbfd:	mov    0x8(%rcx),%r14
   5dc01:	call   *0x127769(%rip)        # 185370 <_DYNAMIC+0x240>
   5dc07:	mov    $0x1,%esi
   5dc0c:	mov    %rbx,%rdi
   5dc0f:	call   *0x127763(%rip)        # 185378 <_DYNAMIC+0x248>
   5dc15:	test   %rax,%rax
   5dc18:	je     5e457 <probe::main+0x1587>
   5dc1e:	mov    %rax,%rbp
   5dc21:	mov    %rax,%rdi
   5dc24:	mov    %r14,%rsi
   5dc27:	mov    %rbx,%rdx
   5dc2a:	call   *0x127760(%rip)        # 185390 <memcpy@GLIBC_2.14>
   5dc30:	movb   $0x3,(%rsp)
   5dc34:	mov    %rbx,0x8(%rsp)
   5dc39:	mov    %rbp,0x10(%rsp)
   5dc3e:	mov    %rbx,0x18(%rsp)
   5dc43:	lea    0x30(%rsp),%rdi
   5dc48:	lea    0xb0(%rsp),%rsi
   5dc50:	lea    0x58(%rsp),%rdx
   5dc55:	mov    %rsp,%rcx
   5dc58:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dc5d:	cmpb   $0xff,0x30(%rsp)
   5dc62:	je     5dc6e <probe::main+0xd9e>
   5dc64:	lea    0x30(%rsp),%rdi
   5dc69:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dc6e:	call   *0x1276fc(%rip)        # 185370 <_DYNAMIC+0x240>
   5dc74:	mov    $0xa,%ebx
   5dc79:	mov    $0xa,%edi
   5dc7e:	mov    $0x1,%esi
   5dc83:	call   *0x1276ef(%rip)        # 185378 <_DYNAMIC+0x248>
   5dc89:	test   %rax,%rax
   5dc8c:	je     5e42c <probe::main+0x155c>
   5dc92:	movabs $0x6f69746172657469,%rcx
   5dc9c:	mov    %rcx,(%rax)
   5dc9f:	movw   $0x736e,0x8(%rax)
   5dca5:	movq   $0xa,0x58(%rsp)
   5dcae:	mov    %rax,0x60(%rsp)
   5dcb3:	movq   $0xa,0x68(%rsp)
   5dcbc:	movb   $0x2,(%rsp)
   5dcc0:	movq   $0x0,0x8(%rsp)
   5dcc9:	mov    %r15,0x10(%rsp)
   5dcce:	lea    0x30(%rsp),%rdi
   5dcd3:	lea    0xb0(%rsp),%rsi
   5dcdb:	lea    0x58(%rsp),%rdx
   5dce0:	mov    %rsp,%rcx
   5dce3:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dce8:	cmpb   $0xff,0x30(%rsp)
   5dced:	je     5dcf9 <probe::main+0xe29>
   5dcef:	lea    0x30(%rsp),%rdi
   5dcf4:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dcf9:	call   *0x127671(%rip)        # 185370 <_DYNAMIC+0x240>
   5dcff:	mov    $0x4,%ebx
   5dd04:	mov    $0x4,%edi
   5dd09:	mov    $0x1,%esi
   5dd0e:	call   *0x127664(%rip)        # 185378 <_DYNAMIC+0x248>
   5dd14:	test   %rax,%rax
   5dd17:	je     5e42c <probe::main+0x155c>
   5dd1d:	movl   $0x73746962,(%rax)
   5dd23:	movq   $0x4,0x58(%rsp)
   5dd2c:	mov    %rax,0x60(%rsp)
   5dd31:	movq   $0x4,0x68(%rsp)
   5dd3a:	mov    0x50(%rsp),%eax
   5dd3e:	movb   $0x2,(%rsp)
   5dd42:	movq   $0x0,0x8(%rsp)
   5dd4b:	mov    %rax,0x10(%rsp)
   5dd50:	lea    0x30(%rsp),%rdi
   5dd55:	lea    0xb0(%rsp),%rsi
   5dd5d:	lea    0x58(%rsp),%rdx
   5dd62:	mov    %rsp,%rcx
   5dd65:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dd6a:	cmpb   $0xff,0x30(%rsp)
   5dd6f:	je     5dd7b <probe::main+0xeab>
   5dd71:	lea    0x30(%rsp),%rdi
   5dd76:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dd7b:	call   *0x1275ef(%rip)        # 185370 <_DYNAMIC+0x240>
   5dd81:	mov    $0x9,%ebx
   5dd86:	mov    $0x9,%edi
   5dd8b:	mov    $0x1,%esi
   5dd90:	call   *0x1275e2(%rip)        # 185378 <_DYNAMIC+0x248>
   5dd96:	test   %rax,%rax
   5dd99:	je     5e42c <probe::main+0x155c>
   5dd9f:	movabs $0x6f69736963657270,%rcx
   5dda9:	mov    %rcx,(%rax)
   5ddac:	movb   $0x6e,0x8(%rax)
   5ddb0:	movq   $0x9,0x58(%rsp)
   5ddb9:	mov    %rax,0x60(%rsp)
   5ddbe:	movq   $0x9,0x68(%rsp)
   5ddc7:	movslq 0x74(%rsp),%rax
   5ddcc:	mov    %rax,%rcx
   5ddcf:	shr    $0x3f,%rcx
   5ddd3:	movb   $0x2,(%rsp)
   5ddd7:	mov    %rcx,0x8(%rsp)
   5dddc:	mov    %rax,0x10(%rsp)
   5dde1:	lea    0x30(%rsp),%rdi
   5dde6:	lea    0xb0(%rsp),%rsi
   5ddee:	lea    0x58(%rsp),%rdx
   5ddf3:	mov    %rsp,%rcx
   5ddf6:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5ddfb:	cmpb   $0xff,0x30(%rsp)
   5de00:	je     5de0c <probe::main+0xf3c>
   5de02:	lea    0x30(%rsp),%rdi
   5de07:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5de0c:	call   *0x12755e(%rip)        # 185370 <_DYNAMIC+0x240>
   5de12:	mov    $0xa,%ebx
   5de17:	mov    $0xa,%edi
   5de1c:	mov    $0x1,%esi
   5de21:	call   *0x127551(%rip)        # 185378 <_DYNAMIC+0x248>
   5de27:	test   %rax,%rax
   5de2a:	je     5e42c <probe::main+0x155c>
   5de30:	mov    %rax,%rbp
   5de33:	movabs $0x5f64657370616c65,%rax
   5de3d:	mov    %rax,0x0(%rbp)
   5de41:	movw   $0x736e,0x8(%rbp)
   5de47:	movq   $0xa,0x58(%rsp)
   5de50:	mov    %rbp,0x60(%rsp)
   5de55:	movq   $0xa,0x68(%rsp)
   5de5e:	mov    %rsp,%rdx
   5de61:	mov    $0x27,%ecx
   5de66:	mov    %r13,%rdi
   5de69:	mov    0x80(%rsp),%rsi
   5de71:	call   *0x1275d1(%rip)        # 185448 <_DYNAMIC+0x318>
   5de77:	mov    %rdx,%rbx
   5de7a:	test   %rdx,%rdx
   5de7d:	jns    5de93 <probe::main+0xfc3>
   5de7f:	xor    %r15d,%r15d
   5de82:	mov    %r15,%rdi
   5de85:	mov    %rbx,%rsi
   5de88:	call   *0x1275c2(%rip)        # 185450 <_DYNAMIC+0x320>
   5de8e:	jmp    5e4b8 <probe::main+0x15e8>
   5de93:	test   %rbx,%rbx
   5de96:	je     5defd <probe::main+0x102d>
   5de98:	mov    %rax,%r14
   5de9b:	call   *0x1274cf(%rip)        # 185370 <_DYNAMIC+0x240>
   5dea1:	mov    $0x1,%r15d
   5dea7:	mov    $0x1,%esi
   5deac:	mov    %rbx,%rdi
   5deaf:	call   *0x1274c3(%rip)        # 185378 <_DYNAMIC+0x248>
   5deb5:	test   %rax,%rax
   5deb8:	je     5de82 <probe::main+0xfb2>
   5deba:	mov    %rax,%r13
   5debd:	mov    %rax,%rdi
   5dec0:	mov    %r14,%rsi
   5dec3:	mov    %rbx,%rdx
   5dec6:	call   *0x1274c4(%rip)        # 185390 <memcpy@GLIBC_2.14>
   5decc:	call   *0x12749e(%rip)        # 185370 <_DYNAMIC+0x240>
   5ded2:	mov    $0x1,%esi
   5ded7:	mov    %rbx,%rdi
   5deda:	call   *0x127498(%rip)        # 185378 <_DYNAMIC+0x248>
   5dee0:	test   %rax,%rax
   5dee3:	je     5e49a <probe::main+0x15ca>
   5dee9:	mov    %rax,%r14
   5deec:	mov    %rax,%rdi
   5deef:	mov    %r13,%rsi
   5def2:	mov    %rbx,%rdx
   5def5:	call   *0x127495(%rip)        # 185390 <memcpy@GLIBC_2.14>
   5defb:	jmp    5df09 <probe::main+0x1039>
   5defd:	mov    $0x1,%r13d
   5df03:	mov    $0x1,%r14d
   5df09:	movb   $0x3,(%rsp)
   5df0d:	mov    %rbx,0x8(%rsp)
   5df12:	mov    %r14,0x10(%rsp)
   5df17:	mov    %rbx,0x18(%rsp)
   5df1c:	lea    0x30(%rsp),%rdi
   5df21:	lea    0xb0(%rsp),%rsi
   5df29:	lea    0x58(%rsp),%rdx
   5df2e:	mov    %rsp,%rcx
   5df31:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5df36:	cmpb   $0xff,0x30(%rsp)
   5df3b:	je     5df47 <probe::main+0x1077>
   5df3d:	lea    0x30(%rsp),%rdi
   5df42:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5df47:	test   %rbx,%rbx
   5df4a:	je     5df5d <probe::main+0x108d>
   5df4c:	mov    $0x1,%edx
   5df51:	mov    %r13,%rdi
   5df54:	mov    %rbx,%rsi
   5df57:	call   *0x12745b(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5df5d:	call   *0x12740d(%rip)        # 185370 <_DYNAMIC+0x240>
   5df63:	mov    $0x8,%ebx
   5df68:	mov    $0x8,%edi
   5df6d:	mov    $0x1,%esi
   5df72:	call   *0x127400(%rip)        # 185378 <_DYNAMIC+0x248>
   5df78:	test   %rax,%rax
   5df7b:	je     5e42c <probe::main+0x155c>
   5df81:	movabs $0x6d75736b63656863,%rcx
   5df8b:	mov    %rcx,(%rax)
   5df8e:	movq   $0x8,0x58(%rsp)
   5df97:	mov    %rax,0x60(%rsp)
   5df9c:	movq   $0x8,0x68(%rsp)
   5dfa5:	movb   $0x2,(%rsp)
   5dfa9:	movq   $0x0,0x8(%rsp)
   5dfb2:	mov    %r12,0x10(%rsp)
   5dfb7:	lea    0x30(%rsp),%rdi
   5dfbc:	lea    0xb0(%rsp),%rsi
   5dfc4:	lea    0x58(%rsp),%rdx
   5dfc9:	mov    %rsp,%rcx
   5dfcc:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dfd1:	cmpb   $0xff,0x30(%rsp)
   5dfd6:	je     5dfe2 <probe::main+0x1112>
   5dfd8:	lea    0x30(%rsp),%rdi
   5dfdd:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dfe2:	call   *0x127388(%rip)        # 185370 <_DYNAMIC+0x240>
   5dfe8:	mov    $0xd,%ebx
   5dfed:	mov    $0xd,%edi
   5dff2:	mov    $0x1,%esi
   5dff7:	call   *0x12737b(%rip)        # 185378 <_DYNAMIC+0x248>
   5dffd:	test   %rax,%rax
   5e000:	je     5e42c <probe::main+0x155c>
   5e006:	mov    %rax,%r12
   5e009:	movabs $0x737469625f646574,%rax
   5e013:	mov    %rax,0x5(%r12)
   5e018:	movabs $0x6465746365707865,%rax
   5e022:	mov    %rax,(%r12)
   5e026:	movq   $0xd,0x58(%rsp)
   5e02f:	mov    %r12,0x60(%rsp)
   5e034:	movq   $0xd,0x68(%rsp)
   5e03d:	call   *0x12732d(%rip)        # 185370 <_DYNAMIC+0x240>
   5e043:	mov    $0x40,%edi
   5e048:	mov    $0x8,%esi
   5e04d:	call   *0x127325(%rip)        # 185378 <_DYNAMIC+0x248>
   5e053:	test   %rax,%rax
   5e056:	je     5e3bd <probe::main+0x14ed>
   5e05c:	mov    %rax,%r13
   5e05f:	mov    0xf8(%rsp),%r15
   5e067:	test   %r15,%r15
   5e06a:	je     5e46e <probe::main+0x159e>
   5e070:	mov    0xf0(%rsp),%rbx
   5e078:	mov    0x10(%rbx),%rax
   5e07c:	test   %rax,%rax
   5e07f:	je     5e09d <probe::main+0x11cd>
   5e081:	mov    0x8(%rbx),%rcx
   5e085:	mov    $0x7f,%edx
   5e08a:	bsr    -0x8(%rcx,%rax,8),%rdx
   5e090:	xor    $0x3f,%rdx
   5e094:	shl    $0x6,%rax
   5e098:	sub    %rdx,%rax
   5e09b:	jmp    5e09f <probe::main+0x11cf>
   5e09d:	xor    %eax,%eax
   5e09f:	movb   $0x2,(%rsp)
   5e0a3:	movq   $0x0,0x8(%rsp)
   5e0ac:	mov    %rax,0x10(%rsp)
   5e0b1:	cmp    $0x1,%r15
   5e0b5:	je     5e481 <probe::main+0x15b1>
   5e0bb:	mov    0x30(%rbx),%rax
   5e0bf:	test   %rax,%rax
   5e0c2:	je     5e0e0 <probe::main+0x1210>
   5e0c4:	mov    0x28(%rbx),%rcx
   5e0c8:	mov    $0x7f,%edx
   5e0cd:	bsr    -0x8(%rcx,%rax,8),%rdx
   5e0d3:	xor    $0x3f,%rdx
   5e0d7:	shl    $0x6,%rax
   5e0db:	sub    %rdx,%rax
   5e0de:	jmp    5e0e2 <probe::main+0x1212>
   5e0e0:	xor    %eax,%eax
   5e0e2:	movb   $0x2,0x20(%r13)
   5e0e7:	movq   $0x0,0x28(%r13)
   5e0ef:	mov    %rax,0x30(%r13)
   5e0f3:	movups (%rsp),%xmm0
   5e0f7:	movups 0x10(%rsp),%xmm1
   5e0fc:	movups %xmm1,0x10(%r13)
   5e101:	movups %xmm0,0x0(%r13)
   5e106:	movq   $0x2,0x8(%rsp)
   5e10f:	mov    %r13,0x10(%rsp)
   5e114:	movq   $0x2,0x18(%rsp)
   5e11d:	movb   $0x4,(%rsp)
   5e121:	lea    0x30(%rsp),%rdi
   5e126:	lea    0xb0(%rsp),%rsi
   5e12e:	lea    0x58(%rsp),%rdx
   5e133:	mov    %rsp,%r14
   5e136:	mov    %r14,%rcx
   5e139:	call   5eed0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5e13e:	cmpb   $0xff,0x30(%rsp)
   5e143:	je     5e14f <probe::main+0x127f>
   5e145:	lea    0x30(%rsp),%rdi
   5e14a:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e14f:	movups 0xb0(%rsp),%xmm0
   5e157:	movups %xmm0,0x8(%rsp)
   5e15c:	mov    0xc0(%rsp),%rax
   5e164:	mov    %rax,0x18(%rsp)
   5e169:	movb   $0x5,(%rsp)
   5e16d:	mov    %r14,0x30(%rsp)
   5e172:	mov    0x1272df(%rip),%rax        # 185458 <_DYNAMIC+0x328>
   5e179:	mov    %rax,0x38(%rsp)
   5e17e:	lea    -0x2e55c(%rip),%rdi        # 2fc29 <GCC_except_table0+0x2f9>
   5e185:	lea    0x30(%rsp),%rsi
   5e18a:	call   *0x1272d0(%rip)        # 185460 <_DYNAMIC+0x330>
   5e190:	mov    %rsp,%rdi
   5e193:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e198:	lea    0x8(%rbx),%r12
   5e19c:	mov    0x127215(%rip),%r14        # 1853b8 <_DYNAMIC+0x288>
   5e1a3:	jmp    5e1b9 <probe::main+0x12e9>
   5e1a5:	data16 cs nopw 0x0(%rax,%rax,1)
   5e1b0:	add    $0x20,%r12
   5e1b4:	dec    %r15
   5e1b7:	je     5e1d5 <probe::main+0x1305>
   5e1b9:	mov    -0x8(%r12),%rsi
   5e1be:	test   %rsi,%rsi
   5e1c1:	je     5e1b0 <probe::main+0x12e0>
   5e1c3:	mov    (%r12),%rdi
   5e1c7:	shl    $0x3,%rsi
   5e1cb:	mov    $0x8,%edx
   5e1d0:	call   *%r14
   5e1d3:	jmp    5e1b0 <probe::main+0x12e0>
   5e1d5:	mov    0xe8(%rsp),%rsi
   5e1dd:	test   %rsi,%rsi
   5e1e0:	je     5e1f4 <probe::main+0x1324>
   5e1e2:	shl    $0x5,%rsi
   5e1e6:	mov    $0x8,%edx
   5e1eb:	mov    %rbx,%rdi
   5e1ee:	call   *0x1271c4(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e1f4:	lea    0x90(%rsp),%rdi
   5e1fc:	call   5cc50 <core::ptr::drop_glue::<[hyperreal::computable::node::Computable; 2]>>
   5e201:	mov    0xc8(%rsp),%rsi
   5e209:	test   %rsi,%rsi
   5e20c:	je     5e222 <probe::main+0x1352>
   5e20e:	shl    $0x3,%rsi
   5e212:	mov    $0x8,%edx
   5e217:	mov    0x78(%rsp),%rdi
   5e21c:	call   *0x127196(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e222:	mov    0xd8(%rsp),%rbx
   5e22a:	mov    0xe0(%rsp),%r15
   5e232:	test   %r15,%r15
   5e235:	je     5e261 <probe::main+0x1391>
   5e237:	lea    0x8(%rbx),%r12
   5e23b:	jmp    5e249 <probe::main+0x1379>
   5e23d:	nopl   (%rax)
   5e240:	add    $0x18,%r12
   5e244:	dec    %r15
   5e247:	je     5e261 <probe::main+0x1391>
   5e249:	mov    -0x8(%r12),%rsi
   5e24e:	test   %rsi,%rsi
   5e251:	je     5e240 <probe::main+0x1370>
   5e253:	mov    (%r12),%rdi
   5e257:	mov    $0x1,%edx
   5e25c:	call   *%r14
   5e25f:	jmp    5e240 <probe::main+0x1370>
   5e261:	mov    0xd0(%rsp),%rax
   5e269:	test   %rax,%rax
   5e26c:	je     5e284 <probe::main+0x13b4>
   5e26e:	shl    $0x3,%rax
   5e272:	lea    (%rax,%rax,2),%rsi
   5e276:	mov    $0x8,%edx
   5e27b:	mov    %rbx,%rdi
   5e27e:	call   *0x127134(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e284:	add    $0x138,%rsp
   5e28b:	pop    %rbx
   5e28c:	pop    %r12
   5e28e:	pop    %r13
   5e290:	pop    %r14
   5e292:	pop    %r15
   5e294:	pop    %rbp
   5e295:	ret
   5e296:	movzbl 0x8(%rsp),%eax
   5e29b:	mov    %al,0x30(%rsp)
   5e29f:	lea    -0x2b6d6(%rip),%rdi        # 32bd0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x100>
   5e2a6:	lea    0x122a13(%rip),%rcx        # 180cc0 <__dso_handle+0x188>
   5e2ad:	lea    0x1229f4(%rip),%r8        # 180ca8 <__dso_handle+0x170>
   5e2b4:	lea    0x30(%rsp),%rdx
   5e2b9:	mov    $0x2b,%esi
   5e2be:	call   *0x1271a4(%rip)        # 185468 <_DYNAMIC+0x338>
   5e2c4:	jmp    5e4b8 <probe::main+0x15e8>
   5e2c9:	add    $0xd0,%dl
   5e2cc:	xor    %eax,%eax
   5e2ce:	cmp    $0xa,%dl
   5e2d1:	adc    $0x1,%al
   5e2d3:	mov    %al,(%rsp)
   5e2d6:	lea    -0x2b70d(%rip),%rdi        # 32bd0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x100>
   5e2dd:	lea    0x1229fc(%rip),%rcx        # 180ce0 <__dso_handle+0x1a8>
   5e2e4:	lea    0x122915(%rip),%r8        # 180c00 <__dso_handle+0xc8>
   5e2eb:	mov    %rsp,%rdx
   5e2ee:	mov    $0x2b,%esi
   5e2f3:	call   *0x12716f(%rip)        # 185468 <_DYNAMIC+0x338>
   5e2f9:	jmp    5e4b8 <probe::main+0x15e8>
   5e2fe:	mov    $0x8,%edx
   5e303:	mov    %rbx,%rdi
   5e306:	mov    %r13,%rsi
   5e309:	call   *0x1270a9(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e30f:	movups -0x2b556(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   5e316:	movaps %xmm0,(%rsp)
   5e31a:	movups -0x2b551(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   5e321:	movaps %xmm0,0x10(%rsp)
   5e326:	jmp    5d756 <probe::main+0x886>
   5e32b:	movzbl 0x8(%rsp),%eax
   5e330:	mov    %al,0x90(%rsp)
   5e337:	lea    -0x2b76e(%rip),%rdi        # 32bd0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x100>
   5e33e:	lea    0x12297b(%rip),%rcx        # 180cc0 <__dso_handle+0x188>
   5e345:	lea    0x1228e4(%rip),%r8        # 180c30 <__dso_handle+0xf8>
   5e34c:	lea    0x90(%rsp),%rdx
   5e354:	mov    $0x2b,%esi
   5e359:	call   *0x127109(%rip)        # 185468 <_DYNAMIC+0x338>
   5e35f:	jmp    5e4b8 <probe::main+0x15e8>
   5e364:	movzbl 0x8(%rsp),%eax
   5e369:	mov    %al,0x30(%rsp)
   5e36d:	lea    -0x2b7a4(%rip),%rdi        # 32bd0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x100>
   5e374:	lea    0x122945(%rip),%rcx        # 180cc0 <__dso_handle+0x188>
   5e37b:	lea    0x1228c6(%rip),%r8        # 180c48 <__dso_handle+0x110>
   5e382:	lea    0x30(%rsp),%rdx
   5e387:	mov    $0x2b,%esi
   5e38c:	call   *0x1270d6(%rip)        # 185468 <_DYNAMIC+0x338>
   5e392:	jmp    5e4b8 <probe::main+0x15e8>
   5e397:	lea    0x1228c2(%rip),%r9        # 180c60 <__dso_handle+0x128>
   5e39e:	lea    0x100(%rsp),%rsi
   5e3a6:	lea    0x110(%rsp),%rdx
   5e3ae:	xor    %edi,%edi
   5e3b0:	xor    %ecx,%ecx
   5e3b2:	call   *0x1270b8(%rip)        # 185470 <_DYNAMIC+0x340>
   5e3b8:	jmp    5e4b8 <probe::main+0x15e8>
   5e3bd:	mov    $0x8,%edi
   5e3c2:	mov    $0x40,%esi
   5e3c7:	call   *0x126fb3(%rip)        # 185380 <_DYNAMIC+0x250>
   5e3cd:	jmp    5e4b8 <probe::main+0x15e8>
   5e3d2:	lea    -0x2b819(%rip),%rdi        # 32bc0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0xf0>
   5e3d9:	lea    0x122838(%rip),%rdx        # 180c18 <__dso_handle+0xe0>
   5e3e0:	mov    $0x21,%esi
   5e3e5:	call   *0x12708d(%rip)        # 185478 <_DYNAMIC+0x348>
   5e3eb:	jmp    5e4b8 <probe::main+0x15e8>
   5e3f0:	mov    $0x1,%al
   5e3f2:	jmp    5e2d3 <probe::main+0x1403>
   5e3f7:	lea    0x1227a2(%rip),%rdx        # 180ba0 <__dso_handle+0x68>
   5e3fe:	call   *0x12707c(%rip)        # 185480 <_DYNAMIC+0x350>
   5e404:	jmp    5e4b8 <probe::main+0x15e8>
   5e409:	mov    %r12,0x28(%rsp)
   5e40e:	lea    -0x2b81a(%rip),%rdi        # 32bfb <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.1768662447242434102+0x12b>
   5e415:	lea    0x1228e4(%rip),%rdx        # 180d00 <__dso_handle+0x1c8>
   5e41c:	mov    $0x49,%esi
   5e421:	call   *0x127051(%rip)        # 185478 <_DYNAMIC+0x348>
   5e427:	jmp    5e4b8 <probe::main+0x15e8>
   5e42c:	mov    $0x1,%edi
   5e431:	mov    %rbx,%rsi
   5e434:	call   *0x127016(%rip)        # 185450 <_DYNAMIC+0x320>
   5e43a:	jmp    5e4b8 <probe::main+0x15e8>
   5e43c:	lea    0x12278d(%rip),%rdx        # 180bd0 <__dso_handle+0x98>
   5e443:	jmp    5e44c <probe::main+0x157c>
   5e445:	lea    0x12279c(%rip),%rdx        # 180be8 <__dso_handle+0xb0>
   5e44c:	mov    %rdi,%rsi
   5e44f:	call   *0x12702b(%rip)        # 185480 <_DYNAMIC+0x350>
   5e455:	jmp    5e4b8 <probe::main+0x15e8>
   5e457:	mov    $0x1,%edi
   5e45c:	mov    %rbx,%rsi
   5e45f:	call   *0x126feb(%rip)        # 185450 <_DYNAMIC+0x320>
   5e465:	jmp    5e4b8 <probe::main+0x15e8>
   5e467:	xor    %eax,%eax
   5e469:	jmp    5e2d3 <probe::main+0x1403>
   5e46e:	lea    0x122803(%rip),%rdx        # 180c78 <__dso_handle+0x140>
   5e475:	xor    %edi,%edi
   5e477:	xor    %esi,%esi
   5e479:	call   *0x127001(%rip)        # 185480 <_DYNAMIC+0x350>
   5e47f:	jmp    5e4b8 <probe::main+0x15e8>
   5e481:	lea    0x122808(%rip),%rdx        # 180c90 <__dso_handle+0x158>
   5e488:	mov    $0x1,%edi
   5e48d:	mov    $0x1,%esi
   5e492:	call   *0x126fe8(%rip)        # 185480 <_DYNAMIC+0x350>
   5e498:	jmp    5e4b8 <probe::main+0x15e8>
   5e49a:	mov    $0x1,%edi
   5e49f:	mov    %rbx,%rsi
   5e4a2:	call   *0x126fa8(%rip)        # 185450 <_DYNAMIC+0x320>
   5e4a8:	jmp    5e4b8 <probe::main+0x15e8>
   5e4aa:	mov    $0x8,%edi
   5e4af:	mov    %r12,%rsi
   5e4b2:	call   *0x126f98(%rip)        # 185450 <_DYNAMIC+0x320>
   5e4b8:	ud2
   5e4ba:	mov    %rax,%r15
   5e4bd:	jmp    5e6d2 <probe::main+0x1802>
   5e4c2:	jmp    5e4cc <probe::main+0x15fc>
   5e4c4:	mov    %r14,0x88(%rsp)
   5e4cc:	mov    %r12,0x28(%rsp)
   5e4d1:	mov    %rax,%r15
   5e4d4:	mov    (%rsp),%rsi
   5e4d8:	test   %rsi,%rsi
   5e4db:	je     5e6e3 <probe::main+0x1813>
   5e4e1:	mov    0x8(%rsp),%rdi
   5e4e6:	shl    $0x3,%rsi
   5e4ea:	mov    $0x8,%edx
   5e4ef:	jmp    5e6dd <probe::main+0x180d>
   5e4f4:	mov    %rax,%r15
   5e4f7:	mov    (%rsp),%rsi
   5e4fb:	test   %rsi,%rsi
   5e4fe:	je     5e7b6 <probe::main+0x18e6>
   5e504:	mov    0x8(%rsp),%rdi
   5e509:	shl    $0x3,%rsi
   5e50d:	mov    $0x8,%edx
   5e512:	call   *0x126ea0(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e518:	jmp    5e7b6 <probe::main+0x18e6>
   5e51d:	mov    %rax,%r15
   5e520:	mov    (%rsp),%rsi
   5e524:	jmp    5e52e <probe::main+0x165e>
   5e526:	mov    %rax,%r15
   5e529:	mov    0x30(%rsp),%rsi
   5e52e:	test   %rsi,%rsi
   5e531:	je     5e600 <probe::main+0x1730>
   5e537:	mov    (%r14),%rdi
   5e53a:	shl    $0x3,%rsi
   5e53e:	mov    $0x8,%edx
   5e543:	call   *0x126e6f(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e549:	jmp    5e600 <probe::main+0x1730>
   5e54e:	mov    %rax,%r15
   5e551:	mov    $0xa,%esi
   5e556:	mov    $0x1,%edx
   5e55b:	mov    %rbp,%rdi
   5e55e:	call   *0x126e54(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e564:	jmp    5e5cc <probe::main+0x16fc>
   5e566:	jmp    5e5c0 <probe::main+0x16f0>
   5e568:	mov    %rax,%r15
   5e56b:	jmp    5e600 <probe::main+0x1730>
   5e570:	mov    %rax,%r15
   5e573:	mov    %rsp,%rdi
   5e576:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e57b:	jmp    5e580 <probe::main+0x16b0>
   5e57d:	mov    %rax,%r15
   5e580:	mov    $0x40,%esi
   5e585:	mov    $0x8,%edx
   5e58a:	mov    %r13,%rdi
   5e58d:	call   *0x126e25(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e593:	jmp    5e715 <probe::main+0x1845>
   5e598:	mov    %rax,%r15
   5e59b:	jmp    5e7b6 <probe::main+0x18e6>
   5e5a0:	mov    %rax,%r15
   5e5a3:	mov    %rsp,%rdi
   5e5a6:	call   5cd30 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e5ab:	jmp    5e778 <probe::main+0x18a8>
   5e5b0:	mov    %rax,%r15
   5e5b3:	jmp    5e76b <probe::main+0x189b>
   5e5b8:	mov    %rax,%r15
   5e5bb:	jmp    5e76b <probe::main+0x189b>
   5e5c0:	mov    %rax,%r15
   5e5c3:	test   %rbx,%rbx
   5e5c6:	je     5e76b <probe::main+0x189b>
   5e5cc:	mov    $0x1,%edx
   5e5d1:	mov    %r13,%rdi
   5e5d4:	mov    %rbx,%rsi
   5e5d7:	jmp    5e765 <probe::main+0x1895>
   5e5dc:	mov    %rax,%r15
   5e5df:	mov    0x98(%rsp),%rax
   5e5e7:	test   %rax,%rax
   5e5ea:	je     5e600 <probe::main+0x1730>
   5e5ec:	lock decq (%rax)
   5e5f0:	jne    5e600 <probe::main+0x1730>
   5e5f2:	lea    0x98(%rsp),%rdi
   5e5fa:	call   *0x126dd0(%rip)        # 1853d0 <_DYNAMIC+0x2a0>
   5e600:	test   %r13,%r13
   5e603:	je     5e7b6 <probe::main+0x18e6>
   5e609:	shl    $0x3,%r13
   5e60d:	mov    $0x8,%edx
   5e612:	mov    0x28(%rsp),%rdi
   5e617:	mov    %r13,%rsi
   5e61a:	call   *0x126d98(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e620:	jmp    5e7b6 <probe::main+0x18e6>
   5e625:	call   *0x126d85(%rip)        # 1853b0 <_DYNAMIC+0x280>
   5e62b:	mov    %rax,%r15
   5e62e:	mov    0x38(%rsp),%rax
   5e633:	test   %rax,%rax
   5e636:	je     5e7b6 <probe::main+0x18e6>
   5e63c:	lock decq (%rax)
   5e640:	jne    5e7b6 <probe::main+0x18e6>
   5e646:	mov    %rbx,%rdi
   5e649:	call   *0x126d81(%rip)        # 1853d0 <_DYNAMIC+0x2a0>
   5e64f:	jmp    5e7b6 <probe::main+0x18e6>
   5e654:	call   *0x126d56(%rip)        # 1853b0 <_DYNAMIC+0x280>
   5e65a:	mov    %rax,%r15
   5e65d:	mov    $0x4,%esi
   5e662:	mov    $0x1,%edx
   5e667:	mov    0x28(%rsp),%rdi
   5e66c:	jmp    5e765 <probe::main+0x1895>
   5e671:	mov    %rax,%r15
   5e674:	jmp    5e76b <probe::main+0x189b>
   5e679:	mov    %rax,%r15
   5e67c:	jmp    5e76b <probe::main+0x189b>
   5e681:	mov    %rax,%r15
   5e684:	jmp    5e76b <probe::main+0x189b>
   5e689:	mov    %rax,%r15
   5e68c:	jmp    5e76b <probe::main+0x189b>
   5e691:	mov    %rax,%r15
   5e694:	jmp    5e7a9 <probe::main+0x18d9>
   5e699:	mov    %rax,%r15
   5e69c:	lea    0x118(%rsp),%rdi
   5e6a4:	call   5ce20 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e6a9:	jmp    5e7b6 <probe::main+0x18e6>
   5e6ae:	mov    %rax,%r15
   5e6b1:	jmp    5e7b6 <probe::main+0x18e6>
   5e6b6:	mov    %rax,%r15
   5e6b9:	jmp    5e7b6 <probe::main+0x18e6>
   5e6be:	mov    %rax,%r15
   5e6c1:	jmp    5e76b <probe::main+0x189b>
   5e6c6:	mov    %rax,%r15
   5e6c9:	test   %r13,%r13
   5e6cc:	je     5e6e3 <probe::main+0x1813>
   5e6ce:	shl    $0x3,%r13
   5e6d2:	mov    $0x8,%edx
   5e6d7:	mov    %rbx,%rdi
   5e6da:	mov    %r13,%rsi
   5e6dd:	call   *0x126cd5(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e6e3:	cmpq   $0x0,0x28(%rsp)
   5e6e9:	je     5e7b6 <probe::main+0x18e6>
   5e6ef:	mov    0x28(%rsp),%rsi
   5e6f4:	shl    $0x3,%rsi
   5e6f8:	mov    $0x8,%edx
   5e6fd:	mov    0x88(%rsp),%rdi
   5e705:	call   *0x126cad(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e70b:	jmp    5e7b6 <probe::main+0x18e6>
   5e710:	jmp    5e748 <probe::main+0x1878>
   5e712:	mov    %rax,%r15
   5e715:	mov    $0xd,%esi
   5e71a:	mov    $0x1,%edx
   5e71f:	mov    %r12,%rdi
   5e722:	jmp    5e765 <probe::main+0x1895>
   5e724:	mov    %rax,%r15
   5e727:	lea    0x30(%rsp),%rdi
   5e72c:	call   5ce20 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e731:	jmp    5e7d7 <probe::main+0x1907>
   5e736:	mov    %rax,%r15
   5e739:	lea    0x90(%rsp),%rdi
   5e741:	call   5ce20 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e746:	jmp    5e7b6 <probe::main+0x18e6>
   5e748:	mov    %rax,%r15
   5e74b:	jmp    5e778 <probe::main+0x18a8>
   5e74d:	mov    %rax,%r15
   5e750:	jmp    5e7d7 <probe::main+0x1907>
   5e755:	mov    %rax,%r15
   5e758:	mov    $0xa,%esi
   5e75d:	mov    $0x1,%edx
   5e762:	mov    %rbp,%rdi
   5e765:	call   *0x126c4d(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e76b:	lea    0xb0(%rsp),%rdi
   5e773:	call   5f9b0 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value> as core::ops::drop::Drop>::drop>
   5e778:	lea    0xe8(%rsp),%rdi
   5e780:	call   5fcb0 <<alloc::vec::Vec<num_bigint::bigint::BigInt> as core::ops::drop::Drop>::drop>
   5e785:	mov    0xe8(%rsp),%rsi
   5e78d:	test   %rsi,%rsi
   5e790:	je     5e7a9 <probe::main+0x18d9>
   5e792:	mov    0xf0(%rsp),%rdi
   5e79a:	shl    $0x5,%rsi
   5e79e:	mov    $0x8,%edx
   5e7a3:	call   *0x126c0f(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e7a9:	lea    0x90(%rsp),%rdi
   5e7b1:	call   5cc50 <core::ptr::drop_glue::<[hyperreal::computable::node::Computable; 2]>>
   5e7b6:	mov    0xc8(%rsp),%rsi
   5e7be:	test   %rsi,%rsi
   5e7c1:	je     5e7d7 <probe::main+0x1907>
   5e7c3:	shl    $0x3,%rsi
   5e7c7:	mov    $0x8,%edx
   5e7cc:	mov    0x78(%rsp),%rdi
   5e7d1:	call   *0x126be1(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e7d7:	mov    0xd8(%rsp),%rbx
   5e7df:	mov    0xe0(%rsp),%r14
   5e7e7:	test   %r14,%r14
   5e7ea:	jne    5e817 <probe::main+0x1947>
   5e7ec:	mov    0xd0(%rsp),%rax
   5e7f4:	test   %rax,%rax
   5e7f7:	je     5e80f <probe::main+0x193f>
   5e7f9:	shl    $0x3,%rax
   5e7fd:	lea    (%rax,%rax,2),%rsi
   5e801:	mov    $0x8,%edx
   5e806:	mov    %rbx,%rdi
   5e809:	call   *0x126ba9(%rip)        # 1853b8 <_DYNAMIC+0x288>
   5e80f:	mov    %r15,%rdi
   5e812:	call   17f950 <_Unwind_Resume@plt>
   5e817:	lea    0x8(%rbx),%r12
   5e81b:	mov    0x126b96(%rip),%r13        # 1853b8 <_DYNAMIC+0x288>
   5e822:	jmp    5e839 <probe::main+0x1969>
   5e824:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5e830:	add    $0x18,%r12
   5e834:	dec    %r14
   5e837:	je     5e7ec <probe::main+0x191c>
   5e839:	mov    -0x8(%r12),%rsi
   5e83e:	test   %rsi,%rsi
   5e841:	je     5e830 <probe::main+0x1960>
   5e843:	mov    (%r12),%rdi
   5e847:	mov    $0x1,%edx
   5e84c:	call   *%r13
   5e84f:	jmp    5e830 <probe::main+0x1960>
   5e851:	call   *0x126b59(%rip)        # 1853b0 <_DYNAMIC+0x280>
