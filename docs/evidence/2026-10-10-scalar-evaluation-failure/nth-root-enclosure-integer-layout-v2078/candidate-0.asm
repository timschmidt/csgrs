
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v2015-nth-root-enclosure-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

000000000005ce60 <probe::main>:
   5ce60:	push   %rbp
   5ce61:	push   %r15
   5ce63:	push   %r14
   5ce65:	push   %r13
   5ce67:	push   %r12
   5ce69:	push   %rbx
   5ce6a:	sub    $0x138,%rsp
   5ce71:	mov    %rsp,%rbx
   5ce74:	mov    %rbx,%rdi
   5ce77:	call   *0x12821b(%rip)        # 185098 <_DYNAMIC+0x2a8>
   5ce7d:	movq   $0x1,0x20(%rsp)
   5ce86:	lea    0xd0(%rsp),%rdi
   5ce8e:	mov    %rbx,%rsi
   5ce91:	call   5f510 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::skip::Skip<std::env::Args>>>::from_iter>
   5ce96:	mov    0xe0(%rsp),%rdi
   5ce9e:	test   %rdi,%rdi
   5cea1:	je     5e3cc <probe::main+0x156c>
   5cea7:	cmp    $0x1,%rdi
   5ceab:	je     5e3d5 <probe::main+0x1575>
   5ceb1:	mov    0xd8(%rsp),%r9
   5ceb9:	mov    0x20(%r9),%rcx
   5cebd:	mov    0x28(%r9),%rsi
   5cec1:	cmp    $0x1,%rsi
   5cec5:	je     5ced5 <probe::main+0x75>
   5cec7:	test   %rsi,%rsi
   5ceca:	je     5e3f7 <probe::main+0x1597>
   5ced0:	movzbl (%rcx),%edx
   5ced3:	jmp    5ceec <probe::main+0x8c>
   5ced5:	movzbl (%rcx),%edx
   5ced8:	mov    $0x1,%al
   5ceda:	cmp    $0x2b,%edx
   5cedd:	je     5e263 <probe::main+0x1403>
   5cee3:	cmp    $0x2d,%edx
   5cee6:	je     5e263 <probe::main+0x1403>
   5ceec:	xor    %eax,%eax
   5ceee:	cmp    $0x2b,%dl
   5cef1:	sete   %al
   5cef4:	mov    %rsi,%rdx
   5cef7:	sub    %rax,%rdx
   5cefa:	add    %rax,%rcx
   5cefd:	neg    %rax
   5cf00:	cmp    $0x11,%rdx
   5cf04:	jae    5cf47 <probe::main+0xe7>
   5cf06:	test   %rdx,%rdx
   5cf09:	je     5cf99 <probe::main+0x139>
   5cf0f:	add    %rax,%rsi
   5cf12:	neg    %rsi
   5cf15:	xor    %r15d,%r15d
   5cf18:	xor    %eax,%eax
   5cf1a:	nopw   0x0(%rax,%rax,1)
   5cf20:	movzbl (%rcx,%rax,1),%edx
   5cf24:	add    $0xffffffd0,%edx
   5cf27:	cmp    $0x9,%edx
   5cf2a:	ja     5e380 <probe::main+0x1520>
   5cf30:	lea    (%r15,%r15,4),%rdi
   5cf34:	mov    %edx,%edx
   5cf36:	lea    (%rdx,%rdi,2),%r15
   5cf3a:	inc    %rax
   5cf3d:	mov    %rsi,%rdx
   5cf40:	add    %rax,%rdx
   5cf43:	jne    5cf20 <probe::main+0xc0>
   5cf45:	jmp    5cf9c <probe::main+0x13c>
   5cf47:	add    %rax,%rsi
   5cf4a:	neg    %rsi
   5cf4d:	xor    %r15d,%r15d
   5cf50:	mov    $0xa,%edi
   5cf55:	xor    %r8d,%r8d
   5cf58:	nopl   0x0(%rax,%rax,1)
   5cf60:	mov    %rsi,%rax
   5cf63:	add    %r8,%rax
   5cf66:	je     5cf9c <probe::main+0x13c>
   5cf68:	mov    %r15,%rax
   5cf6b:	mul    %rdi
   5cf6e:	movzbl (%rcx,%r8,1),%edx
   5cf73:	jo     5e259 <probe::main+0x13f9>
   5cf79:	add    $0xffffffd0,%edx
   5cf7c:	cmp    $0xa,%edx
   5cf7f:	jae    5e380 <probe::main+0x1520>
   5cf85:	mov    %rax,%r15
   5cf88:	mov    %edx,%eax
   5cf8a:	inc    %r8
   5cf8d:	add    %rax,%r15
   5cf90:	jae    5cf60 <probe::main+0x100>
   5cf92:	mov    $0x2,%al
   5cf94:	jmp    5e263 <probe::main+0x1403>
   5cf99:	xor    %r15d,%r15d
   5cf9c:	mov    0x10(%r9),%rcx
   5cfa0:	add    $0xfffffffffffffff8,%rcx
   5cfa4:	cmp    $0x6,%rcx
   5cfa8:	ja     5e362 <probe::main+0x1502>
   5cfae:	mov    0x8(%r9),%rax
   5cfb2:	lea    -0x2d6b9(%rip),%rdx        # 2f900 <GCC_except_table0+0x10>
   5cfb9:	movslq (%rdx,%rcx,4),%rcx
   5cfbd:	add    %rdx,%rcx
   5cfc0:	mov    %r9,0x108(%rsp)
   5cfc8:	jmp    *%rcx
   5cfca:	movabs $0x7061675f65646977,%rcx
   5cfd4:	cmp    %rcx,(%rax)
   5cfd7:	jne    5e362 <probe::main+0x1502>
   5cfdd:	mov    $0x7fffffff,%eax
   5cfe2:	mov    $0xf4241,%ecx
   5cfe7:	jmp    5d12d <probe::main+0x2cd>
   5cfec:	movabs $0x6f635f6c6c616d73,%rcx
   5cff6:	xor    (%rax),%rcx
   5cff9:	mov    0x8(%rax),%edx
   5cffc:	xor    $0x65737261,%rdx
   5d003:	or     %rcx,%rdx
   5d006:	je     5d123 <probe::main+0x2c3>
   5d00c:	movabs $0x6c61685f65646977,%rcx
   5d016:	xor    (%rax),%rcx
   5d019:	mov    0x8(%rax),%eax
   5d01c:	xor    $0x79617766,%rax
   5d022:	or     %rcx,%rax
   5d025:	jne    5e362 <probe::main+0x1502>
   5d02b:	mov    $0x1,%r12b
   5d02e:	mov    $0xf4241,%eax
   5d033:	mov    %rax,0x50(%rsp)
   5d038:	mov    $0xf4241,%eax
   5d03d:	jmp    5d135 <probe::main+0x2d5>
   5d042:	movabs $0x69665f6c6c616d73,%rcx
   5d04c:	xor    (%rax),%rcx
   5d04f:	movzwl 0x8(%rax),%eax
   5d053:	xor    $0x656e,%rax
   5d059:	or     %rcx,%rax
   5d05c:	jne    5e362 <probe::main+0x1502>
   5d062:	mov    $0xffffff80,%eax
   5d067:	jmp    5d128 <probe::main+0x2c8>
   5d06c:	movabs $0x64695f6c6c616d73,%rcx
   5d076:	xor    (%rax),%rcx
   5d079:	movabs $0x797469746e656469,%rdx
   5d083:	xor    0x6(%rax),%rdx
   5d087:	or     %rcx,%rdx
   5d08a:	jne    5e362 <probe::main+0x1502>
   5d090:	xor    %eax,%eax
   5d092:	mov    $0x41,%ecx
   5d097:	jmp    5d12d <probe::main+0x2cd>
   5d09c:	movabs $0x616f635f65646977,%rcx
   5d0a6:	xor    (%rax),%rcx
   5d0a9:	movabs $0x657372616f635f65,%rdx
   5d0b3:	xor    0x3(%rax),%rdx
   5d0b7:	or     %rcx,%rdx
   5d0ba:	jne    5e362 <probe::main+0x1502>
   5d0c0:	mov    $0xf4240,%eax
   5d0c5:	mov    $0xf4241,%ecx
   5d0ca:	jmp    5d12d <probe::main+0x2cd>
   5d0cc:	movabs $0x6e69665f65646977,%rcx
   5d0d6:	xor    (%rax),%rcx
   5d0d9:	movzbl 0x8(%rax),%eax
   5d0dd:	xor    $0x65,%rax
   5d0e1:	or     %rcx,%rax
   5d0e4:	jne    5e362 <probe::main+0x1502>
   5d0ea:	mov    $0xffffff80,%eax
   5d0ef:	mov    $0x10001,%ecx
   5d0f4:	jmp    5d12d <probe::main+0x2cd>
   5d0f6:	movabs $0x6564695f65646977,%rcx
   5d100:	xor    (%rax),%rcx
   5d103:	movabs $0x797469746e656469,%rdx
   5d10d:	xor    0x5(%rax),%rdx
   5d111:	or     %rcx,%rdx
   5d114:	jne    5e362 <probe::main+0x1502>
   5d11a:	xor    %eax,%eax
   5d11c:	mov    $0x10001,%ecx
   5d121:	jmp    5d12d <probe::main+0x2cd>
   5d123:	mov    $0x1,%eax
   5d128:	mov    $0x8,%ecx
   5d12d:	mov    %rcx,0x50(%rsp)
   5d132:	xor    %r12d,%r12d
   5d135:	mov    %eax,0x74(%rsp)
   5d139:	call   *0x127f61(%rip)        # 1850a0 <_DYNAMIC+0x2b0>
   5d13f:	mov    %rax,0x30(%rsp)
   5d144:	mov    %rdx,0x38(%rsp)
   5d149:	mov    %rsp,%rdi
   5d14c:	lea    0x30(%rsp),%rsi
   5d151:	xor    %edx,%edx
   5d153:	call   *0x127f4f(%rip)        # 1850a8 <_DYNAMIC+0x2b8>
   5d159:	mov    (%rsp),%rax
   5d15d:	mov    %rax,0xc8(%rsp)
   5d165:	cmp    $0xffffffffffffffff,%rax
   5d169:	je     5e2bb <probe::main+0x145b>
   5d16f:	lea    0x38(%rsp),%rbx
   5d174:	mov    0x8(%rsp),%rax
   5d179:	mov    %rax,0x78(%rsp)
   5d17e:	mov    0x10(%rsp),%rax
   5d183:	mov    %rax,0x80(%rsp)
   5d18b:	movzbl 0x18(%rsp),%ebp
   5d190:	mov    0x30(%rsp),%rax
   5d195:	lock decq (%rax)
   5d199:	jne    5d1a6 <probe::main+0x346>
   5d19b:	lea    0x30(%rsp),%rdi
   5d1a0:	call   *0x127ee2(%rip)        # 185088 <_DYNAMIC+0x298>
   5d1a6:	mov    0x38(%rsp),%rax
   5d1ab:	test   %rax,%rax
   5d1ae:	je     5d1bf <probe::main+0x35f>
   5d1b0:	lock decq (%rax)
   5d1b4:	jne    5d1bf <probe::main+0x35f>
   5d1b6:	mov    %rbx,%rdi
   5d1b9:	call   *0x127ed1(%rip)        # 185090 <_DYNAMIC+0x2a0>
   5d1bf:	call   *0x127eeb(%rip)        # 1850b0 <_DYNAMIC+0x2c0>
   5d1c5:	mov    %rax,0x90(%rsp)
   5d1cd:	mov    %rdx,0x98(%rsp)
   5d1d5:	mov    %rsp,%rdi
   5d1d8:	lea    0x90(%rsp),%rsi
   5d1e0:	xor    %edx,%edx
   5d1e2:	call   *0x127ec0(%rip)        # 1850a8 <_DYNAMIC+0x2b8>
   5d1e8:	mov    (%rsp),%rax
   5d1ec:	cmp    $0xffffffffffffffff,%rax
   5d1f0:	je     5e2f4 <probe::main+0x1494>
   5d1f6:	mov    0x8(%rsp),%rcx
   5d1fb:	mov    0x10(%rsp),%rsi
   5d200:	movzbl 0x18(%rsp),%r14d
   5d206:	test   %rsi,%rsi
   5d209:	je     5d277 <probe::main+0x417>
   5d20b:	mov    0x50(%rsp),%rdx
   5d210:	lea    -0x1(%rdx),%edi
   5d213:	mov    %edi,%edx
   5d215:	shr    $0x6,%edx
   5d218:	and    $0x3f,%dil
   5d21c:	mov    %rax,(%rsp)
   5d220:	mov    %rcx,0x8(%rsp)
   5d225:	mov    %rsi,0x10(%rsp)
   5d22a:	movzbl %dil,%ecx
   5d22e:	lea    0x30(%rsp),%rdi
   5d233:	mov    %rsp,%rsi
   5d236:	call   *0x127e7c(%rip)        # 1850b8 <_DYNAMIC+0x2c8>
   5d23c:	cmp    $0x1,%r14b
   5d240:	jne    5d290 <probe::main+0x430>
   5d242:	mov    0x30(%rsp),%r13
   5d247:	mov    $0x1,%r14b
   5d24a:	cmp    $0x4,%r13
   5d24e:	jb     5d273 <probe::main+0x413>
   5d250:	mov    0x38(%rsp),%rdi
   5d255:	shl    $0x3,%r13
   5d259:	mov    $0x8,%edx
   5d25e:	mov    %r13,%rsi
   5d261:	call   *0x127e11(%rip)        # 185078 <_DYNAMIC+0x288>
   5d267:	movq   $0x8,0x38(%rsp)
   5d270:	xor    %r13d,%r13d
   5d273:	xor    %ebx,%ebx
   5d275:	jmp    5d2ab <probe::main+0x44b>
   5d277:	mov    %rax,0x30(%rsp)
   5d27c:	mov    %rcx,0x38(%rsp)
   5d281:	movq   $0x0,0x40(%rsp)
   5d28a:	cmp    $0x1,%r14b
   5d28e:	je     5d242 <probe::main+0x3e2>
   5d290:	mov    0x30(%rsp),%r13
   5d295:	mov    0x40(%rsp),%rbx
   5d29a:	test   %rbx,%rbx
   5d29d:	movzbl %r14b,%eax
   5d2a1:	mov    $0x1,%r14d
   5d2a7:	cmovne %eax,%r14d
   5d2ab:	mov    0x38(%rsp),%rax
   5d2b0:	mov    %rax,0x28(%rsp)
   5d2b5:	mov    0x90(%rsp),%rax
   5d2bd:	lock decq (%rax)
   5d2c1:	jne    5d2d1 <probe::main+0x471>
   5d2c3:	lea    0x90(%rsp),%rdi
   5d2cb:	call   *0x127db7(%rip)        # 185088 <_DYNAMIC+0x298>
   5d2d1:	mov    0x98(%rsp),%rax
   5d2d9:	test   %rax,%rax
   5d2dc:	je     5d2f2 <probe::main+0x492>
   5d2de:	lock decq (%rax)
   5d2e2:	jne    5d2f2 <probe::main+0x492>
   5d2e4:	lea    0x98(%rsp),%rdi
   5d2ec:	call   *0x127d9e(%rip)        # 185090 <_DYNAMIC+0x2a0>
   5d2f2:	test   %r12b,%r12b
   5d2f5:	je     5d314 <probe::main+0x4b4>
   5d2f7:	mov    %r13,(%rsp)
   5d2fb:	mov    0x28(%rsp),%rax
   5d300:	mov    %rax,0x8(%rsp)
   5d305:	mov    %rbx,0x10(%rsp)
   5d30a:	mov    %r14b,0x18(%rsp)
   5d30f:	jmp    5d77a <probe::main+0x91a>
   5d314:	test   %r14b,%r14b
   5d317:	je     5d373 <probe::main+0x513>
   5d319:	movzbl %r14b,%eax
   5d31d:	cmp    $0x1,%eax
   5d320:	jne    5d438 <probe::main+0x5d8>
   5d326:	lea    0x38(%rsp),%r14
   5d32b:	movups -0x2a522(%rip),%xmm0        # 32e10 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x90>
   5d332:	movaps %xmm0,0x30(%rsp)
   5d337:	movq   $0x0,0x40(%rsp)
   5d340:	lea    0x30(%rsp),%rdi
   5d345:	call   *0x127d75(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   5d34b:	mov    0x38(%rsp),%rax
   5d350:	movq   $0x1,(%rax)
   5d357:	movaps 0x30(%rsp),%xmm0
   5d35c:	movaps %xmm0,(%rsp)
   5d360:	movq   $0x1,0x10(%rsp)
   5d369:	movb   $0x2,0x18(%rsp)
   5d36e:	jmp    5d48f <probe::main+0x62f>
   5d373:	lea    0x8(%rsp),%r14
   5d378:	movups -0x2a56f(%rip),%xmm0        # 32e10 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x90>
   5d37f:	movaps %xmm0,(%rsp)
   5d383:	movq   $0x0,0x10(%rsp)
   5d38c:	mov    %rsp,%rdi
   5d38f:	call   *0x127d2b(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   5d395:	cmp    $0x2,%rbx
   5d399:	setae  %al
   5d39c:	cmp    $0x1,%rbx
   5d3a0:	sbb    $0x0,%al
   5d3a2:	mov    0x8(%rsp),%rcx
   5d3a7:	movq   $0x1,(%rcx)
   5d3ae:	mov    (%rsp),%r12
   5d3b2:	mov    0x8(%rsp),%r14
   5d3b7:	cmp    $0x1,%rbx
   5d3bb:	jne    5d3d3 <probe::main+0x573>
   5d3bd:	mov    (%r14),%rax
   5d3c0:	mov    0x28(%rsp),%rcx
   5d3c5:	cmp    %rax,(%rcx)
   5d3c8:	je     5d45e <probe::main+0x5fe>
   5d3ce:	seta   %al
   5d3d1:	sbb    $0x0,%al
   5d3d3:	test   %al,%al
   5d3d5:	je     5d45e <probe::main+0x5fe>
   5d3db:	movzbl %al,%eax
   5d3de:	cmp    $0x1,%eax
   5d3e1:	jne    5d5a0 <probe::main+0x740>
   5d3e7:	mov    %r13,(%rsp)
   5d3eb:	mov    0x28(%rsp),%rax
   5d3f0:	mov    %rax,0x8(%rsp)
   5d3f5:	mov    %rbx,0x10(%rsp)
   5d3fa:	mov    %rsp,%rdi
   5d3fd:	mov    $0x1,%esi
   5d402:	call   *0x127cc0(%rip)        # 1850c8 <_DYNAMIC+0x2d8>
   5d408:	mov    (%rsp),%rsi
   5d40c:	mov    0x8(%rsp),%rdi
   5d411:	mov    0x10(%rsp),%rax
   5d416:	test   %rax,%rax
   5d419:	je     5d6f0 <probe::main+0x890>
   5d41f:	mov    %rsi,(%rsp)
   5d423:	mov    %rdi,0x8(%rsp)
   5d428:	mov    %rax,0x10(%rsp)
   5d42d:	xor    %ebx,%ebx
   5d42f:	mov    %bl,0x18(%rsp)
   5d433:	jmp    5d760 <probe::main+0x900>
   5d438:	mov    %r13,(%rsp)
   5d43c:	mov    0x28(%rsp),%rax
   5d441:	mov    %rax,0x8(%rsp)
   5d446:	mov    %rbx,0x10(%rsp)
   5d44b:	test   %rbx,%rbx
   5d44e:	je     5d4ae <probe::main+0x64e>
   5d450:	incq   (%rax)
   5d453:	je     5d4e4 <probe::main+0x684>
   5d459:	jmp    5d575 <probe::main+0x715>
   5d45e:	movups -0x2a6d5(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   5d465:	movaps %xmm0,0x10(%rsp)
   5d46a:	movups -0x2a6f1(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   5d471:	movaps %xmm0,(%rsp)
   5d475:	test   %r12,%r12
   5d478:	je     5d48f <probe::main+0x62f>
   5d47a:	shl    $0x3,%r12
   5d47e:	mov    $0x8,%edx
   5d483:	mov    %r14,%rdi
   5d486:	mov    %r12,%rsi
   5d489:	call   *0x127be9(%rip)        # 185078 <_DYNAMIC+0x288>
   5d48f:	test   %r13,%r13
   5d492:	je     5d77a <probe::main+0x91a>
   5d498:	shl    $0x3,%r13
   5d49c:	mov    $0x8,%edx
   5d4a1:	mov    0x28(%rsp),%rdi
   5d4a6:	mov    %r13,%rsi
   5d4a9:	jmp    5d774 <probe::main+0x914>
   5d4ae:	test   %r13,%r13
   5d4b1:	jne    5d4c1 <probe::main+0x661>
   5d4b3:	mov    %rsp,%rdi
   5d4b6:	call   *0x127c04(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   5d4bc:	mov    0x8(%rsp),%rax
   5d4c1:	movq   $0x0,(%rax)
   5d4c8:	movq   $0x1,0x10(%rsp)
   5d4d1:	mov    $0x1,%ebx
   5d4d6:	mov    0x8(%rsp),%rax
   5d4db:	incq   (%rax)
   5d4de:	jne    5d575 <probe::main+0x715>
   5d4e4:	mov    $0x1,%r14d
   5d4ea:	cmp    $0x1,%rbx
   5d4ee:	je     5d519 <probe::main+0x6b9>
   5d4f0:	mov    %rax,%rdx
   5d4f3:	shl    $0x3,%rbx
   5d4f7:	mov    $0x1,%al
   5d4f9:	mov    $0x8,%ecx
   5d4fe:	xchg   %ax,%ax
   5d500:	add    $0xff,%al
   5d502:	adcq   $0x0,(%rdx,%rcx,1)
   5d507:	setb   %al
   5d50a:	jae    5d575 <probe::main+0x715>
   5d50c:	add    $0x8,%rcx
   5d510:	cmp    %rcx,%rbx
   5d513:	jne    5d500 <probe::main+0x6a0>
   5d515:	movzbl %al,%r14d
   5d519:	mov    0x10(%rsp),%rbx
   5d51e:	cmp    (%rsp),%rbx
   5d522:	jne    5d52d <probe::main+0x6cd>
   5d524:	mov    %rsp,%rdi
   5d527:	call   *0x127b93(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   5d52d:	mov    0x8(%rsp),%rax
   5d532:	mov    %r14,(%rax,%rbx,8)
   5d536:	inc    %rbx
   5d539:	mov    (%rsp),%rsi
   5d53d:	mov    0x8(%rsp),%rdi
   5d542:	test   %rbx,%rbx
   5d545:	jne    5d588 <probe::main+0x728>
   5d547:	movups -0x2a7be(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   5d54e:	movaps %xmm0,0x10(%rsp)
   5d553:	movups -0x2a7da(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   5d55a:	movaps %xmm0,(%rsp)
   5d55e:	test   %rsi,%rsi
   5d561:	je     5d77a <probe::main+0x91a>
   5d567:	shl    $0x3,%rsi
   5d56b:	mov    $0x8,%edx
   5d570:	jmp    5d774 <probe::main+0x914>
   5d575:	mov    0x10(%rsp),%rbx
   5d57a:	mov    (%rsp),%rsi
   5d57e:	mov    0x8(%rsp),%rdi
   5d583:	test   %rbx,%rbx
   5d586:	je     5d547 <probe::main+0x6e7>
   5d588:	mov    %rsi,(%rsp)
   5d58c:	mov    %rdi,0x8(%rsp)
   5d591:	mov    %rbx,0x10(%rsp)
   5d596:	movb   $0x2,0x18(%rsp)
   5d59b:	jmp    5d77a <probe::main+0x91a>
   5d5a0:	mov    %r13,(%rsp)
   5d5a4:	mov    0x28(%rsp),%rdx
   5d5a9:	mov    %rdx,0x8(%rsp)
   5d5ae:	mov    %rbx,0x10(%rsp)
   5d5b3:	test   %rbx,%rbx
   5d5b6:	mov    %r14,0x88(%rsp)
   5d5be:	je     5d5f4 <probe::main+0x794>
   5d5c0:	movq   $0x1,0x30(%rsp)
   5d5c9:	lea    0x30(%rsp),%rdi
   5d5ce:	mov    $0x1,%esi
   5d5d3:	mov    %rbx,%rcx
   5d5d6:	call   *0x127af4(%rip)        # 1850d0 <_DYNAMIC+0x2e0>
   5d5dc:	mov    0x10(%rsp),%r14
   5d5e1:	mov    (%rsp),%r13
   5d5e5:	mov    0x8(%rsp),%rbx
   5d5ea:	test   %r14,%r14
   5d5ed:	jne    5d630 <probe::main+0x7d0>
   5d5ef:	xor    %r14d,%r14d
   5d5f2:	jmp    5d670 <probe::main+0x810>
   5d5f4:	test   %r13,%r13
   5d5f7:	jne    5d60c <probe::main+0x7ac>
   5d5f9:	mov    %rsp,%rdi
   5d5fc:	call   *0x127abe(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   5d602:	mov    0x8(%rsp),%rax
   5d607:	mov    %rax,0x28(%rsp)
   5d60c:	mov    0x28(%rsp),%rax
   5d611:	movq   $0x1,(%rax)
   5d618:	movq   $0x1,0x10(%rsp)
   5d621:	mov    (%rsp),%r13
   5d625:	mov    0x8(%rsp),%rbx
   5d62a:	mov    $0x1,%r14d
   5d630:	cmpq   $0x0,-0x8(%rbx,%r14,8)
   5d636:	jne    5d670 <probe::main+0x810>
   5d638:	movabs $0x1fffffffffffffff,%rax
   5d642:	and    %r14,%rax
   5d645:	lea    0x0(,%r14,8),%rdx
   5d64d:	xor    %ecx,%ecx
   5d64f:	test   %rdx,%rdx
   5d652:	je     5d5ef <probe::main+0x78f>
   5d654:	inc    %rcx
   5d657:	cmpq   $0x0,-0x8(%rbx,%rdx,1)
   5d65d:	lea    -0x8(%rdx),%rdx
   5d661:	je     5d64f <probe::main+0x7ef>
   5d663:	sub    %rcx,%rax
   5d666:	inc    %rax
   5d669:	cmp    %r14,%rax
   5d66c:	cmovb  %rax,%r14
   5d670:	mov    %r13,%rax
   5d673:	shr    $0x2,%rax
   5d677:	cmp    %rax,%r14
   5d67a:	jae    5d6ce <probe::main+0x86e>
   5d67c:	cmp    %r13,%r14
   5d67f:	ja     5e399 <probe::main+0x1539>
   5d685:	test   %r13,%r13
   5d688:	je     5d6ce <probe::main+0x86e>
   5d68a:	shl    $0x3,%r13
   5d68e:	test   %r14,%r14
   5d691:	je     5e28e <probe::main+0x142e>
   5d697:	mov    %r12,0x28(%rsp)
   5d69c:	lea    0x0(,%r14,8),%r12
   5d6a4:	mov    $0x8,%edx
   5d6a9:	mov    %rbx,%rdi
   5d6ac:	mov    %r13,%rsi
   5d6af:	mov    %r12,%rcx
   5d6b2:	call   *0x127a20(%rip)        # 1850d8 <_DYNAMIC+0x2e8>
   5d6b8:	test   %rax,%rax
   5d6bb:	je     5e43a <probe::main+0x15da>
   5d6c1:	mov    %rax,%rbx
   5d6c4:	mov    %r14,%r13
   5d6c7:	mov    0x28(%rsp),%r12
   5d6cc:	jmp    5d6d3 <probe::main+0x873>
   5d6ce:	test   %r14,%r14
   5d6d1:	je     5d727 <probe::main+0x8c7>
   5d6d3:	mov    %r13,(%rsp)
   5d6d7:	mov    %rbx,0x8(%rsp)
   5d6dc:	mov    %r14,0x10(%rsp)
   5d6e1:	movb   $0x2,0x18(%rsp)
   5d6e6:	mov    0x88(%rsp),%r14
   5d6ee:	jmp    5d760 <probe::main+0x900>
   5d6f0:	movups -0x2a967(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   5d6f7:	movaps %xmm0,0x10(%rsp)
   5d6fc:	movups -0x2a983(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   5d703:	movaps %xmm0,(%rsp)
   5d707:	mov    $0x1,%bl
   5d709:	test   %rsi,%rsi
   5d70c:	je     5d42f <probe::main+0x5cf>
   5d712:	shl    $0x3,%rsi
   5d716:	mov    $0x8,%edx
   5d71b:	call   *0x127957(%rip)        # 185078 <_DYNAMIC+0x288>
   5d721:	mov    %bl,0x18(%rsp)
   5d725:	jmp    5d760 <probe::main+0x900>
   5d727:	movups -0x2a99e(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   5d72e:	movaps %xmm0,0x10(%rsp)
   5d733:	movups -0x2a9ba(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   5d73a:	movaps %xmm0,(%rsp)
   5d73e:	test   %r13,%r13
   5d741:	mov    0x88(%rsp),%r14
   5d749:	je     5d760 <probe::main+0x900>
   5d74b:	shl    $0x3,%r13
   5d74f:	mov    $0x8,%edx
   5d754:	mov    %rbx,%rdi
   5d757:	mov    %r13,%rsi
   5d75a:	call   *0x127918(%rip)        # 185078 <_DYNAMIC+0x288>
   5d760:	test   %r12,%r12
   5d763:	je     5d77a <probe::main+0x91a>
   5d765:	shl    $0x3,%r12
   5d769:	mov    $0x8,%edx
   5d76e:	mov    %r14,%rdi
   5d771:	mov    %r12,%rsi
   5d774:	call   *0x1278fe(%rip)        # 185078 <_DYNAMIC+0x288>
   5d77a:	mov    %rsp,%rdi
   5d77d:	call   *0x12795d(%rip)        # 1850e0 <_DYNAMIC+0x2f0>
   5d783:	mov    %rax,%rdi
   5d786:	call   *0x12795c(%rip)        # 1850e8 <_DYNAMIC+0x2f8>
   5d78c:	mov    %rax,%r12
   5d78f:	mov    %rdx,%rbx
   5d792:	mov    %rax,0x118(%rsp)
   5d79a:	mov    %rdx,0x120(%rsp)
   5d7a2:	lock incq (%rax)
   5d7a6:	jle    5e448 <probe::main+0x15e8>
   5d7ac:	test   %rbx,%rbx
   5d7af:	je     5d7bb <probe::main+0x95b>
   5d7b1:	lock incq (%rbx)
   5d7b5:	jle    5e448 <probe::main+0x15e8>
   5d7bb:	mov    %r12,%rdi
   5d7be:	mov    %rbx,%rsi
   5d7c1:	call   *0x127929(%rip)        # 1850f0 <_DYNAMIC+0x300>
   5d7c7:	mov    %r12,0x90(%rsp)
   5d7cf:	mov    %rbx,0x98(%rsp)
   5d7d7:	mov    %rax,0xa0(%rsp)
   5d7df:	mov    %rdx,0xa8(%rsp)
   5d7e7:	lea    0xb0(%rsp),%rax
   5d7ef:	lea    0x90(%rsp),%rcx
   5d7f7:	mov    %rcx,(%rsp)
   5d7fb:	mov    %rax,0x8(%rsp)
   5d800:	lea    0x74(%rsp),%rax
   5d805:	mov    %rax,0x10(%rsp)
   5d80a:	lea    0xe8(%rsp),%rdi
   5d812:	mov    %rsp,%rsi
   5d815:	call   5fac0 <<alloc::vec::Vec<num_bigint::bigint::BigInt> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<num_bigint::bigint::BigInt, core::iter::adapters::map::Map<core::slice::iter::Iter<hyperreal::computable::node::Computable>, probe::main::{closure#0}>>>::from_iter>
   5d81a:	test   %r15,%r15
   5d81d:	je     5d942 <probe::main+0xae2>
   5d823:	mov    0xf0(%rsp),%rax
   5d82b:	mov    0xf8(%rsp),%rsi
   5d833:	xor    %r12d,%r12d
   5d836:	xor    %ecx,%ecx
   5d838:	jmp    5d85d <probe::main+0x9fd>
   5d83a:	nopw   0x0(%rax,%rax,1)
   5d840:	sub    %r10b,%dil
   5d843:	inc    %rcx
   5d846:	shr    $0x7,%dil
   5d84a:	movzbl %dil,%edi
   5d84e:	add    %rdx,%r12
   5d851:	add    %rdi,%r12
   5d854:	cmp    %r15,%rcx
   5d857:	je     5d945 <probe::main+0xae5>
   5d85d:	mov    %ecx,%edi
   5d85f:	and    $0x1,%edi
   5d862:	cmp    %rsi,%rdi
   5d865:	jae    5e387 <probe::main+0x1527>
   5d86b:	shl    $0x5,%edi
   5d86e:	mov    0x8(%rax,%rdi,1),%r8
   5d873:	mov    0x10(%rax,%rdi,1),%r9
   5d878:	test   %r9,%r9
   5d87b:	je     5d8a0 <probe::main+0xa40>
   5d87d:	mov    $0x7f,%edx
   5d882:	bsr    -0x8(%r8,%r9,8),%rdx
   5d888:	xor    $0x3f,%rdx
   5d88c:	mov    %r9,%r10
   5d88f:	shl    $0x6,%r10
   5d893:	sub    %rdx,%r10
   5d896:	lea    (%r10,%r10,2),%rdx
   5d89a:	jmp    5d8a2 <probe::main+0xa42>
   5d89c:	nopl   0x0(%rax)
   5d8a0:	xor    %edx,%edx
   5d8a2:	add    %rax,%rdi
   5d8a5:	movzbl 0x18(%rdi),%r11d
   5d8aa:	cmp    %rbp,%r11
   5d8ad:	setl   %r10b
   5d8b1:	setg   %dil
   5d8b5:	cmp    %bpl,%r11b
   5d8b8:	jne    5d840 <probe::main+0x9e0>
   5d8ba:	cmp    $0x2,%rbp
   5d8be:	je     5d8cc <probe::main+0xa6c>
   5d8c0:	cmp    $0x1,%ebp
   5d8c3:	jne    5d8f2 <probe::main+0xa92>
   5d8c5:	xor    %edi,%edi
   5d8c7:	jmp    5d843 <probe::main+0x9e3>
   5d8cc:	mov    0x80(%rsp),%r10
   5d8d4:	cmp    %r10,%r9
   5d8d7:	seta   %dil
   5d8db:	sbb    $0x0,%dil
   5d8df:	cmp    %r10,%r9
   5d8e2:	jne    5d843 <probe::main+0x9e3>
   5d8e8:	mov    %r8,%rdi
   5d8eb:	mov    0x78(%rsp),%r8
   5d8f0:	jmp    5d913 <probe::main+0xab3>
   5d8f2:	mov    0x80(%rsp),%r10
   5d8fa:	cmp    %r9,%r10
   5d8fd:	seta   %dil
   5d901:	sbb    $0x0,%dil
   5d905:	cmp    %r9,%r10
   5d908:	jne    5d843 <probe::main+0x9e3>
   5d90e:	mov    0x78(%rsp),%rdi
   5d913:	shl    $0x3,%r9
   5d917:	nopw   0x0(%rax,%rax,1)
   5d920:	test   %r9,%r9
   5d923:	je     5d8c5 <probe::main+0xa65>
   5d925:	mov    -0x8(%r8,%r9,1),%r10
   5d92a:	cmp    %r10,-0x8(%rdi,%r9,1)
   5d92f:	lea    -0x8(%r9),%r9
   5d933:	je     5d920 <probe::main+0xac0>
   5d935:	seta   %dil
   5d939:	sbb    $0x0,%dil
   5d93d:	jmp    5d843 <probe::main+0x9e3>
   5d942:	xor    %r12d,%r12d
   5d945:	mov    %r12,0x110(%rsp)
   5d94d:	movq   $0x0,0x100(%rsp)
   5d959:	call   *0x127799(%rip)        # 1850f8 <_DYNAMIC+0x308>
   5d95f:	mov    %rax,0x128(%rsp)
   5d967:	mov    %edx,0x130(%rsp)
   5d96e:	xor    %r13d,%r13d
   5d971:	test   %r15,%r15
   5d974:	je     5dae4 <probe::main+0xc84>
   5d97a:	mov    %rsp,%rbx
   5d97d:	xor    %r14d,%r14d
   5d980:	jmp    5d99c <probe::main+0xb3c>
   5d982:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5d990:	inc    %r14
   5d993:	cmp    %r15,%r14
   5d996:	je     5dae4 <probe::main+0xc84>
   5d99c:	mov    %r14d,%eax
   5d99f:	and    $0x1,%eax
   5d9a2:	shl    $0x4,%eax
   5d9a5:	lea    (%rsp,%rax,1),%rsi
   5d9a9:	add    $0x90,%rsi
   5d9b0:	mov    0x74(%rsp),%edx
   5d9b4:	mov    %rbx,%rdi
   5d9b7:	call   *0x1276eb(%rip)        # 1850a8 <_DYNAMIC+0x2b8>
   5d9bd:	cmpq   $0xffffffffffffffff,(%rsp)
   5d9c2:	je     5e226 <probe::main+0x13c6>
   5d9c8:	movups (%rsp),%xmm0
   5d9cc:	movups 0x10(%rsp),%xmm1
   5d9d1:	movaps %xmm1,0x40(%rsp)
   5d9d6:	movaps %xmm0,0x30(%rsp)
   5d9db:	lea    0x30(%rsp),%rax
   5d9e0:	mov    0x38(%rsp),%rdi
   5d9e5:	mov    0x40(%rsp),%rcx
   5d9ea:	test   %rcx,%rcx
   5d9ed:	je     5da10 <probe::main+0xbb0>
   5d9ef:	mov    $0x7f,%eax
   5d9f4:	bsr    -0x8(%rdi,%rcx,8),%rax
   5d9fa:	xor    $0x3f,%rax
   5d9fe:	mov    %rcx,%rdx
   5da01:	shl    $0x6,%rdx
   5da05:	sub    %rax,%rdx
   5da08:	lea    (%rdx,%rdx,2),%rax
   5da0c:	jmp    5da12 <probe::main+0xbb2>
   5da0e:	xchg   %ax,%ax
   5da10:	xor    %eax,%eax
   5da12:	movzbl 0x48(%rsp),%r8d
   5da18:	cmp    %rbp,%r8
   5da1b:	setl   %sil
   5da1f:	setg   %dl
   5da22:	cmp    %bpl,%r8b
   5da25:	jne    5da40 <probe::main+0xbe0>
   5da27:	cmp    $0x2,%rbp
   5da2b:	je     5da79 <probe::main+0xc19>
   5da2d:	cmp    $0x1,%ebp
   5da30:	jne    5da99 <probe::main+0xc39>
   5da32:	xor    %edx,%edx
   5da34:	jmp    5da43 <probe::main+0xbe3>
   5da36:	cs nopw 0x0(%rax,%rax,1)
   5da40:	sub    %sil,%dl
   5da43:	shr    $0x7,%dl
   5da46:	movzbl %dl,%ecx
   5da49:	add    %rax,%r13
   5da4c:	add    %rcx,%r13
   5da4f:	mov    %r13,0x100(%rsp)
   5da57:	mov    0x30(%rsp),%rsi
   5da5c:	test   %rsi,%rsi
   5da5f:	je     5d990 <probe::main+0xb30>
   5da65:	shl    $0x3,%rsi
   5da69:	mov    $0x8,%edx
   5da6e:	call   *0x127604(%rip)        # 185078 <_DYNAMIC+0x288>
   5da74:	jmp    5d990 <probe::main+0xb30>
   5da79:	mov    0x80(%rsp),%rsi
   5da81:	cmp    %rsi,%rcx
   5da84:	seta   %dl
   5da87:	sbb    $0x0,%dl
   5da8a:	cmp    %rsi,%rcx
   5da8d:	jne    5da43 <probe::main+0xbe3>
   5da8f:	mov    %rdi,%rdx
   5da92:	mov    0x78(%rsp),%rsi
   5da97:	jmp    5dab7 <probe::main+0xc57>
   5da99:	mov    0x80(%rsp),%rsi
   5daa1:	cmp    %rcx,%rsi
   5daa4:	seta   %dl
   5daa7:	sbb    $0x0,%dl
   5daaa:	cmp    %rcx,%rsi
   5daad:	jne    5da43 <probe::main+0xbe3>
   5daaf:	mov    0x78(%rsp),%rdx
   5dab4:	mov    %rdi,%rsi
   5dab7:	shl    $0x3,%rcx
   5dabb:	nopl   0x0(%rax,%rax,1)
   5dac0:	test   %rcx,%rcx
   5dac3:	je     5da32 <probe::main+0xbd2>
   5dac9:	mov    -0x8(%rsi,%rcx,1),%r8
   5dace:	cmp    %r8,-0x8(%rdx,%rcx,1)
   5dad3:	lea    -0x8(%rcx),%rcx
   5dad7:	je     5dac0 <probe::main+0xc60>
   5dad9:	seta   %dl
   5dadc:	sbb    $0x0,%dl
   5dadf:	jmp    5da43 <probe::main+0xbe3>
   5dae4:	lea    0x128(%rsp),%rdi
   5daec:	call   *0x12760e(%rip)        # 185100 <_DYNAMIC+0x310>
   5daf2:	mov    %edx,%ecx
   5daf4:	mov    $0x3b9aca00,%edx
   5daf9:	mul    %rdx
   5dafc:	mov    %ecx,%ecx
   5dafe:	add    %rax,%rcx
   5db01:	adc    $0x0,%rdx
   5db05:	cmp    %r12,%r13
   5db08:	jne    5e327 <probe::main+0x14c7>
   5db0e:	mov    %rcx,%r13
   5db11:	mov    %rdx,0x80(%rsp)
   5db19:	movq   $0x0,0xb0(%rsp)
   5db25:	movq   $0x0,0xc0(%rsp)
   5db31:	call   *0x1274f9(%rip)        # 185030 <_DYNAMIC+0x240>
   5db37:	mov    $0x4,%ebx
   5db3c:	mov    $0x1,%ebp
   5db41:	mov    $0x4,%edi
   5db46:	mov    $0x1,%esi
   5db4b:	call   *0x1274e7(%rip)        # 185038 <_DYNAMIC+0x248>
   5db51:	test   %rax,%rax
   5db54:	je     5e3bc <probe::main+0x155c>
   5db5a:	movl   $0x65736163,(%rax)
   5db60:	movq   $0x4,0x58(%rsp)
   5db69:	mov    %rax,0x60(%rsp)
   5db6e:	movq   $0x4,0x68(%rsp)
   5db77:	mov    0x108(%rsp),%rcx
   5db7f:	mov    0x10(%rcx),%rbx
   5db83:	test   %rbx,%rbx
   5db86:	je     5dbc0 <probe::main+0xd60>
   5db88:	mov    %rax,0x28(%rsp)
   5db8d:	mov    0x8(%rcx),%r14
   5db91:	call   *0x127499(%rip)        # 185030 <_DYNAMIC+0x240>
   5db97:	mov    $0x1,%esi
   5db9c:	mov    %rbx,%rdi
   5db9f:	call   *0x127493(%rip)        # 185038 <_DYNAMIC+0x248>
   5dba5:	test   %rax,%rax
   5dba8:	je     5e3e7 <probe::main+0x1587>
   5dbae:	mov    %rax,%rbp
   5dbb1:	mov    %rax,%rdi
   5dbb4:	mov    %r14,%rsi
   5dbb7:	mov    %rbx,%rdx
   5dbba:	call   *0x127490(%rip)        # 185050 <memcpy@GLIBC_2.14>
   5dbc0:	movb   $0x3,(%rsp)
   5dbc4:	mov    %rbx,0x8(%rsp)
   5dbc9:	mov    %rbp,0x10(%rsp)
   5dbce:	mov    %rbx,0x18(%rsp)
   5dbd3:	lea    0x30(%rsp),%rdi
   5dbd8:	lea    0xb0(%rsp),%rsi
   5dbe0:	lea    0x58(%rsp),%rdx
   5dbe5:	mov    %rsp,%rcx
   5dbe8:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dbed:	cmpb   $0xff,0x30(%rsp)
   5dbf2:	je     5dbfe <probe::main+0xd9e>
   5dbf4:	lea    0x30(%rsp),%rdi
   5dbf9:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dbfe:	call   *0x12742c(%rip)        # 185030 <_DYNAMIC+0x240>
   5dc04:	mov    $0xa,%ebx
   5dc09:	mov    $0xa,%edi
   5dc0e:	mov    $0x1,%esi
   5dc13:	call   *0x12741f(%rip)        # 185038 <_DYNAMIC+0x248>
   5dc19:	test   %rax,%rax
   5dc1c:	je     5e3bc <probe::main+0x155c>
   5dc22:	movabs $0x6f69746172657469,%rcx
   5dc2c:	mov    %rcx,(%rax)
   5dc2f:	movw   $0x736e,0x8(%rax)
   5dc35:	movq   $0xa,0x58(%rsp)
   5dc3e:	mov    %rax,0x60(%rsp)
   5dc43:	movq   $0xa,0x68(%rsp)
   5dc4c:	movb   $0x2,(%rsp)
   5dc50:	movq   $0x0,0x8(%rsp)
   5dc59:	mov    %r15,0x10(%rsp)
   5dc5e:	lea    0x30(%rsp),%rdi
   5dc63:	lea    0xb0(%rsp),%rsi
   5dc6b:	lea    0x58(%rsp),%rdx
   5dc70:	mov    %rsp,%rcx
   5dc73:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dc78:	cmpb   $0xff,0x30(%rsp)
   5dc7d:	je     5dc89 <probe::main+0xe29>
   5dc7f:	lea    0x30(%rsp),%rdi
   5dc84:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dc89:	call   *0x1273a1(%rip)        # 185030 <_DYNAMIC+0x240>
   5dc8f:	mov    $0x4,%ebx
   5dc94:	mov    $0x4,%edi
   5dc99:	mov    $0x1,%esi
   5dc9e:	call   *0x127394(%rip)        # 185038 <_DYNAMIC+0x248>
   5dca4:	test   %rax,%rax
   5dca7:	je     5e3bc <probe::main+0x155c>
   5dcad:	movl   $0x73746962,(%rax)
   5dcb3:	movq   $0x4,0x58(%rsp)
   5dcbc:	mov    %rax,0x60(%rsp)
   5dcc1:	movq   $0x4,0x68(%rsp)
   5dcca:	mov    0x50(%rsp),%eax
   5dcce:	movb   $0x2,(%rsp)
   5dcd2:	movq   $0x0,0x8(%rsp)
   5dcdb:	mov    %rax,0x10(%rsp)
   5dce0:	lea    0x30(%rsp),%rdi
   5dce5:	lea    0xb0(%rsp),%rsi
   5dced:	lea    0x58(%rsp),%rdx
   5dcf2:	mov    %rsp,%rcx
   5dcf5:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dcfa:	cmpb   $0xff,0x30(%rsp)
   5dcff:	je     5dd0b <probe::main+0xeab>
   5dd01:	lea    0x30(%rsp),%rdi
   5dd06:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dd0b:	call   *0x12731f(%rip)        # 185030 <_DYNAMIC+0x240>
   5dd11:	mov    $0x9,%ebx
   5dd16:	mov    $0x9,%edi
   5dd1b:	mov    $0x1,%esi
   5dd20:	call   *0x127312(%rip)        # 185038 <_DYNAMIC+0x248>
   5dd26:	test   %rax,%rax
   5dd29:	je     5e3bc <probe::main+0x155c>
   5dd2f:	movabs $0x6f69736963657270,%rcx
   5dd39:	mov    %rcx,(%rax)
   5dd3c:	movb   $0x6e,0x8(%rax)
   5dd40:	movq   $0x9,0x58(%rsp)
   5dd49:	mov    %rax,0x60(%rsp)
   5dd4e:	movq   $0x9,0x68(%rsp)
   5dd57:	movslq 0x74(%rsp),%rax
   5dd5c:	mov    %rax,%rcx
   5dd5f:	shr    $0x3f,%rcx
   5dd63:	movb   $0x2,(%rsp)
   5dd67:	mov    %rcx,0x8(%rsp)
   5dd6c:	mov    %rax,0x10(%rsp)
   5dd71:	lea    0x30(%rsp),%rdi
   5dd76:	lea    0xb0(%rsp),%rsi
   5dd7e:	lea    0x58(%rsp),%rdx
   5dd83:	mov    %rsp,%rcx
   5dd86:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dd8b:	cmpb   $0xff,0x30(%rsp)
   5dd90:	je     5dd9c <probe::main+0xf3c>
   5dd92:	lea    0x30(%rsp),%rdi
   5dd97:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5dd9c:	call   *0x12728e(%rip)        # 185030 <_DYNAMIC+0x240>
   5dda2:	mov    $0xa,%ebx
   5dda7:	mov    $0xa,%edi
   5ddac:	mov    $0x1,%esi
   5ddb1:	call   *0x127281(%rip)        # 185038 <_DYNAMIC+0x248>
   5ddb7:	test   %rax,%rax
   5ddba:	je     5e3bc <probe::main+0x155c>
   5ddc0:	mov    %rax,%rbp
   5ddc3:	movabs $0x5f64657370616c65,%rax
   5ddcd:	mov    %rax,0x0(%rbp)
   5ddd1:	movw   $0x736e,0x8(%rbp)
   5ddd7:	movq   $0xa,0x58(%rsp)
   5dde0:	mov    %rbp,0x60(%rsp)
   5dde5:	movq   $0xa,0x68(%rsp)
   5ddee:	mov    %rsp,%rdx
   5ddf1:	mov    $0x27,%ecx
   5ddf6:	mov    %r13,%rdi
   5ddf9:	mov    0x80(%rsp),%rsi
   5de01:	call   *0x127301(%rip)        # 185108 <_DYNAMIC+0x318>
   5de07:	mov    %rdx,%rbx
   5de0a:	test   %rdx,%rdx
   5de0d:	jns    5de23 <probe::main+0xfc3>
   5de0f:	xor    %r15d,%r15d
   5de12:	mov    %r15,%rdi
   5de15:	mov    %rbx,%rsi
   5de18:	call   *0x1272f2(%rip)        # 185110 <_DYNAMIC+0x320>
   5de1e:	jmp    5e448 <probe::main+0x15e8>
   5de23:	test   %rbx,%rbx
   5de26:	je     5de8d <probe::main+0x102d>
   5de28:	mov    %rax,%r14
   5de2b:	call   *0x1271ff(%rip)        # 185030 <_DYNAMIC+0x240>
   5de31:	mov    $0x1,%r15d
   5de37:	mov    $0x1,%esi
   5de3c:	mov    %rbx,%rdi
   5de3f:	call   *0x1271f3(%rip)        # 185038 <_DYNAMIC+0x248>
   5de45:	test   %rax,%rax
   5de48:	je     5de12 <probe::main+0xfb2>
   5de4a:	mov    %rax,%r13
   5de4d:	mov    %rax,%rdi
   5de50:	mov    %r14,%rsi
   5de53:	mov    %rbx,%rdx
   5de56:	call   *0x1271f4(%rip)        # 185050 <memcpy@GLIBC_2.14>
   5de5c:	call   *0x1271ce(%rip)        # 185030 <_DYNAMIC+0x240>
   5de62:	mov    $0x1,%esi
   5de67:	mov    %rbx,%rdi
   5de6a:	call   *0x1271c8(%rip)        # 185038 <_DYNAMIC+0x248>
   5de70:	test   %rax,%rax
   5de73:	je     5e42a <probe::main+0x15ca>
   5de79:	mov    %rax,%r14
   5de7c:	mov    %rax,%rdi
   5de7f:	mov    %r13,%rsi
   5de82:	mov    %rbx,%rdx
   5de85:	call   *0x1271c5(%rip)        # 185050 <memcpy@GLIBC_2.14>
   5de8b:	jmp    5de99 <probe::main+0x1039>
   5de8d:	mov    $0x1,%r13d
   5de93:	mov    $0x1,%r14d
   5de99:	movb   $0x3,(%rsp)
   5de9d:	mov    %rbx,0x8(%rsp)
   5dea2:	mov    %r14,0x10(%rsp)
   5dea7:	mov    %rbx,0x18(%rsp)
   5deac:	lea    0x30(%rsp),%rdi
   5deb1:	lea    0xb0(%rsp),%rsi
   5deb9:	lea    0x58(%rsp),%rdx
   5debe:	mov    %rsp,%rcx
   5dec1:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5dec6:	cmpb   $0xff,0x30(%rsp)
   5decb:	je     5ded7 <probe::main+0x1077>
   5decd:	lea    0x30(%rsp),%rdi
   5ded2:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5ded7:	test   %rbx,%rbx
   5deda:	je     5deed <probe::main+0x108d>
   5dedc:	mov    $0x1,%edx
   5dee1:	mov    %r13,%rdi
   5dee4:	mov    %rbx,%rsi
   5dee7:	call   *0x12718b(%rip)        # 185078 <_DYNAMIC+0x288>
   5deed:	call   *0x12713d(%rip)        # 185030 <_DYNAMIC+0x240>
   5def3:	mov    $0x8,%ebx
   5def8:	mov    $0x8,%edi
   5defd:	mov    $0x1,%esi
   5df02:	call   *0x127130(%rip)        # 185038 <_DYNAMIC+0x248>
   5df08:	test   %rax,%rax
   5df0b:	je     5e3bc <probe::main+0x155c>
   5df11:	movabs $0x6d75736b63656863,%rcx
   5df1b:	mov    %rcx,(%rax)
   5df1e:	movq   $0x8,0x58(%rsp)
   5df27:	mov    %rax,0x60(%rsp)
   5df2c:	movq   $0x8,0x68(%rsp)
   5df35:	movb   $0x2,(%rsp)
   5df39:	movq   $0x0,0x8(%rsp)
   5df42:	mov    %r12,0x10(%rsp)
   5df47:	lea    0x30(%rsp),%rdi
   5df4c:	lea    0xb0(%rsp),%rsi
   5df54:	lea    0x58(%rsp),%rdx
   5df59:	mov    %rsp,%rcx
   5df5c:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5df61:	cmpb   $0xff,0x30(%rsp)
   5df66:	je     5df72 <probe::main+0x1112>
   5df68:	lea    0x30(%rsp),%rdi
   5df6d:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5df72:	call   *0x1270b8(%rip)        # 185030 <_DYNAMIC+0x240>
   5df78:	mov    $0xd,%ebx
   5df7d:	mov    $0xd,%edi
   5df82:	mov    $0x1,%esi
   5df87:	call   *0x1270ab(%rip)        # 185038 <_DYNAMIC+0x248>
   5df8d:	test   %rax,%rax
   5df90:	je     5e3bc <probe::main+0x155c>
   5df96:	mov    %rax,%r12
   5df99:	movabs $0x737469625f646574,%rax
   5dfa3:	mov    %rax,0x5(%r12)
   5dfa8:	movabs $0x6465746365707865,%rax
   5dfb2:	mov    %rax,(%r12)
   5dfb6:	movq   $0xd,0x58(%rsp)
   5dfbf:	mov    %r12,0x60(%rsp)
   5dfc4:	movq   $0xd,0x68(%rsp)
   5dfcd:	call   *0x12705d(%rip)        # 185030 <_DYNAMIC+0x240>
   5dfd3:	mov    $0x40,%edi
   5dfd8:	mov    $0x8,%esi
   5dfdd:	call   *0x127055(%rip)        # 185038 <_DYNAMIC+0x248>
   5dfe3:	test   %rax,%rax
   5dfe6:	je     5e34d <probe::main+0x14ed>
   5dfec:	mov    %rax,%r13
   5dfef:	mov    0xf8(%rsp),%r15
   5dff7:	test   %r15,%r15
   5dffa:	je     5e3fe <probe::main+0x159e>
   5e000:	mov    0xf0(%rsp),%rbx
   5e008:	mov    0x10(%rbx),%rax
   5e00c:	test   %rax,%rax
   5e00f:	je     5e02d <probe::main+0x11cd>
   5e011:	mov    0x8(%rbx),%rcx
   5e015:	mov    $0x7f,%edx
   5e01a:	bsr    -0x8(%rcx,%rax,8),%rdx
   5e020:	xor    $0x3f,%rdx
   5e024:	shl    $0x6,%rax
   5e028:	sub    %rdx,%rax
   5e02b:	jmp    5e02f <probe::main+0x11cf>
   5e02d:	xor    %eax,%eax
   5e02f:	movb   $0x2,(%rsp)
   5e033:	movq   $0x0,0x8(%rsp)
   5e03c:	mov    %rax,0x10(%rsp)
   5e041:	cmp    $0x1,%r15
   5e045:	je     5e411 <probe::main+0x15b1>
   5e04b:	mov    0x30(%rbx),%rax
   5e04f:	test   %rax,%rax
   5e052:	je     5e070 <probe::main+0x1210>
   5e054:	mov    0x28(%rbx),%rcx
   5e058:	mov    $0x7f,%edx
   5e05d:	bsr    -0x8(%rcx,%rax,8),%rdx
   5e063:	xor    $0x3f,%rdx
   5e067:	shl    $0x6,%rax
   5e06b:	sub    %rdx,%rax
   5e06e:	jmp    5e072 <probe::main+0x1212>
   5e070:	xor    %eax,%eax
   5e072:	movb   $0x2,0x20(%r13)
   5e077:	movq   $0x0,0x28(%r13)
   5e07f:	mov    %rax,0x30(%r13)
   5e083:	movups (%rsp),%xmm0
   5e087:	movups 0x10(%rsp),%xmm1
   5e08c:	movups %xmm1,0x10(%r13)
   5e091:	movups %xmm0,0x0(%r13)
   5e096:	movq   $0x2,0x8(%rsp)
   5e09f:	mov    %r13,0x10(%rsp)
   5e0a4:	movq   $0x2,0x18(%rsp)
   5e0ad:	movb   $0x4,(%rsp)
   5e0b1:	lea    0x30(%rsp),%rdi
   5e0b6:	lea    0xb0(%rsp),%rsi
   5e0be:	lea    0x58(%rsp),%rdx
   5e0c3:	mov    %rsp,%r14
   5e0c6:	mov    %r14,%rcx
   5e0c9:	call   5ee60 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5e0ce:	cmpb   $0xff,0x30(%rsp)
   5e0d3:	je     5e0df <probe::main+0x127f>
   5e0d5:	lea    0x30(%rsp),%rdi
   5e0da:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e0df:	movups 0xb0(%rsp),%xmm0
   5e0e7:	movups %xmm0,0x8(%rsp)
   5e0ec:	mov    0xc0(%rsp),%rax
   5e0f4:	mov    %rax,0x18(%rsp)
   5e0f9:	movb   $0x5,(%rsp)
   5e0fd:	mov    %r14,0x30(%rsp)
   5e102:	mov    0x12700f(%rip),%rax        # 185118 <_DYNAMIC+0x328>
   5e109:	mov    %rax,0x38(%rsp)
   5e10e:	lea    -0x2e52c(%rip),%rdi        # 2fbe9 <GCC_except_table0+0x2f9>
   5e115:	lea    0x30(%rsp),%rsi
   5e11a:	call   *0x127000(%rip)        # 185120 <_DYNAMIC+0x330>
   5e120:	mov    %rsp,%rdi
   5e123:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e128:	lea    0x8(%rbx),%r12
   5e12c:	mov    0x126f45(%rip),%r14        # 185078 <_DYNAMIC+0x288>
   5e133:	jmp    5e149 <probe::main+0x12e9>
   5e135:	data16 cs nopw 0x0(%rax,%rax,1)
   5e140:	add    $0x20,%r12
   5e144:	dec    %r15
   5e147:	je     5e165 <probe::main+0x1305>
   5e149:	mov    -0x8(%r12),%rsi
   5e14e:	test   %rsi,%rsi
   5e151:	je     5e140 <probe::main+0x12e0>
   5e153:	mov    (%r12),%rdi
   5e157:	shl    $0x3,%rsi
   5e15b:	mov    $0x8,%edx
   5e160:	call   *%r14
   5e163:	jmp    5e140 <probe::main+0x12e0>
   5e165:	mov    0xe8(%rsp),%rsi
   5e16d:	test   %rsi,%rsi
   5e170:	je     5e184 <probe::main+0x1324>
   5e172:	shl    $0x5,%rsi
   5e176:	mov    $0x8,%edx
   5e17b:	mov    %rbx,%rdi
   5e17e:	call   *0x126ef4(%rip)        # 185078 <_DYNAMIC+0x288>
   5e184:	lea    0x90(%rsp),%rdi
   5e18c:	call   5cbe0 <core::ptr::drop_glue::<[hyperreal::computable::node::Computable; 2]>>
   5e191:	mov    0xc8(%rsp),%rsi
   5e199:	test   %rsi,%rsi
   5e19c:	je     5e1b2 <probe::main+0x1352>
   5e19e:	shl    $0x3,%rsi
   5e1a2:	mov    $0x8,%edx
   5e1a7:	mov    0x78(%rsp),%rdi
   5e1ac:	call   *0x126ec6(%rip)        # 185078 <_DYNAMIC+0x288>
   5e1b2:	mov    0xd8(%rsp),%rbx
   5e1ba:	mov    0xe0(%rsp),%r15
   5e1c2:	test   %r15,%r15
   5e1c5:	je     5e1f1 <probe::main+0x1391>
   5e1c7:	lea    0x8(%rbx),%r12
   5e1cb:	jmp    5e1d9 <probe::main+0x1379>
   5e1cd:	nopl   (%rax)
   5e1d0:	add    $0x18,%r12
   5e1d4:	dec    %r15
   5e1d7:	je     5e1f1 <probe::main+0x1391>
   5e1d9:	mov    -0x8(%r12),%rsi
   5e1de:	test   %rsi,%rsi
   5e1e1:	je     5e1d0 <probe::main+0x1370>
   5e1e3:	mov    (%r12),%rdi
   5e1e7:	mov    $0x1,%edx
   5e1ec:	call   *%r14
   5e1ef:	jmp    5e1d0 <probe::main+0x1370>
   5e1f1:	mov    0xd0(%rsp),%rax
   5e1f9:	test   %rax,%rax
   5e1fc:	je     5e214 <probe::main+0x13b4>
   5e1fe:	shl    $0x3,%rax
   5e202:	lea    (%rax,%rax,2),%rsi
   5e206:	mov    $0x8,%edx
   5e20b:	mov    %rbx,%rdi
   5e20e:	call   *0x126e64(%rip)        # 185078 <_DYNAMIC+0x288>
   5e214:	add    $0x138,%rsp
   5e21b:	pop    %rbx
   5e21c:	pop    %r12
   5e21e:	pop    %r13
   5e220:	pop    %r14
   5e222:	pop    %r15
   5e224:	pop    %rbp
   5e225:	ret
   5e226:	movzbl 0x8(%rsp),%eax
   5e22b:	mov    %al,0x30(%rsp)
   5e22f:	lea    -0x2b6a6(%rip),%rdi        # 32b90 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x100>
   5e236:	lea    0x122773(%rip),%rcx        # 1809b0 <__dso_handle+0x188>
   5e23d:	lea    0x122754(%rip),%r8        # 180998 <__dso_handle+0x170>
   5e244:	lea    0x30(%rsp),%rdx
   5e249:	mov    $0x2b,%esi
   5e24e:	call   *0x126ed4(%rip)        # 185128 <_DYNAMIC+0x338>
   5e254:	jmp    5e448 <probe::main+0x15e8>
   5e259:	add    $0xd0,%dl
   5e25c:	xor    %eax,%eax
   5e25e:	cmp    $0xa,%dl
   5e261:	adc    $0x1,%al
   5e263:	mov    %al,(%rsp)
   5e266:	lea    -0x2b6dd(%rip),%rdi        # 32b90 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x100>
   5e26d:	lea    0x12275c(%rip),%rcx        # 1809d0 <__dso_handle+0x1a8>
   5e274:	lea    0x122675(%rip),%r8        # 1808f0 <__dso_handle+0xc8>
   5e27b:	mov    %rsp,%rdx
   5e27e:	mov    $0x2b,%esi
   5e283:	call   *0x126e9f(%rip)        # 185128 <_DYNAMIC+0x338>
   5e289:	jmp    5e448 <probe::main+0x15e8>
   5e28e:	mov    $0x8,%edx
   5e293:	mov    %rbx,%rdi
   5e296:	mov    %r13,%rsi
   5e299:	call   *0x126dd9(%rip)        # 185078 <_DYNAMIC+0x288>
   5e29f:	movups -0x2b526(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   5e2a6:	movaps %xmm0,(%rsp)
   5e2aa:	movups -0x2b521(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   5e2b1:	movaps %xmm0,0x10(%rsp)
   5e2b6:	jmp    5d6e6 <probe::main+0x886>
   5e2bb:	movzbl 0x8(%rsp),%eax
   5e2c0:	mov    %al,0x90(%rsp)
   5e2c7:	lea    -0x2b73e(%rip),%rdi        # 32b90 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x100>
   5e2ce:	lea    0x1226db(%rip),%rcx        # 1809b0 <__dso_handle+0x188>
   5e2d5:	lea    0x122644(%rip),%r8        # 180920 <__dso_handle+0xf8>
   5e2dc:	lea    0x90(%rsp),%rdx
   5e2e4:	mov    $0x2b,%esi
   5e2e9:	call   *0x126e39(%rip)        # 185128 <_DYNAMIC+0x338>
   5e2ef:	jmp    5e448 <probe::main+0x15e8>
   5e2f4:	movzbl 0x8(%rsp),%eax
   5e2f9:	mov    %al,0x30(%rsp)
   5e2fd:	lea    -0x2b774(%rip),%rdi        # 32b90 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x100>
   5e304:	lea    0x1226a5(%rip),%rcx        # 1809b0 <__dso_handle+0x188>
   5e30b:	lea    0x122626(%rip),%r8        # 180938 <__dso_handle+0x110>
   5e312:	lea    0x30(%rsp),%rdx
   5e317:	mov    $0x2b,%esi
   5e31c:	call   *0x126e06(%rip)        # 185128 <_DYNAMIC+0x338>
   5e322:	jmp    5e448 <probe::main+0x15e8>
   5e327:	lea    0x122622(%rip),%r9        # 180950 <__dso_handle+0x128>
   5e32e:	lea    0x100(%rsp),%rsi
   5e336:	lea    0x110(%rsp),%rdx
   5e33e:	xor    %edi,%edi
   5e340:	xor    %ecx,%ecx
   5e342:	call   *0x126de8(%rip)        # 185130 <_DYNAMIC+0x340>
   5e348:	jmp    5e448 <probe::main+0x15e8>
   5e34d:	mov    $0x8,%edi
   5e352:	mov    $0x40,%esi
   5e357:	call   *0x126ce3(%rip)        # 185040 <_DYNAMIC+0x250>
   5e35d:	jmp    5e448 <probe::main+0x15e8>
   5e362:	lea    -0x2b7e9(%rip),%rdi        # 32b80 <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0xf0>
   5e369:	lea    0x122598(%rip),%rdx        # 180908 <__dso_handle+0xe0>
   5e370:	mov    $0x21,%esi
   5e375:	call   *0x126dbd(%rip)        # 185138 <_DYNAMIC+0x348>
   5e37b:	jmp    5e448 <probe::main+0x15e8>
   5e380:	mov    $0x1,%al
   5e382:	jmp    5e263 <probe::main+0x1403>
   5e387:	lea    0x122502(%rip),%rdx        # 180890 <__dso_handle+0x68>
   5e38e:	call   *0x126dac(%rip)        # 185140 <_DYNAMIC+0x350>
   5e394:	jmp    5e448 <probe::main+0x15e8>
   5e399:	mov    %r12,0x28(%rsp)
   5e39e:	lea    -0x2b7ea(%rip),%rdi        # 32bbb <anon.f651f877c5f1b0c83bd435f3f6e617b1.345.llvm.9350274297799828109+0x12b>
   5e3a5:	lea    0x122644(%rip),%rdx        # 1809f0 <__dso_handle+0x1c8>
   5e3ac:	mov    $0x49,%esi
   5e3b1:	call   *0x126d81(%rip)        # 185138 <_DYNAMIC+0x348>
   5e3b7:	jmp    5e448 <probe::main+0x15e8>
   5e3bc:	mov    $0x1,%edi
   5e3c1:	mov    %rbx,%rsi
   5e3c4:	call   *0x126d46(%rip)        # 185110 <_DYNAMIC+0x320>
   5e3ca:	jmp    5e448 <probe::main+0x15e8>
   5e3cc:	lea    0x1224ed(%rip),%rdx        # 1808c0 <__dso_handle+0x98>
   5e3d3:	jmp    5e3dc <probe::main+0x157c>
   5e3d5:	lea    0x1224fc(%rip),%rdx        # 1808d8 <__dso_handle+0xb0>
   5e3dc:	mov    %rdi,%rsi
   5e3df:	call   *0x126d5b(%rip)        # 185140 <_DYNAMIC+0x350>
   5e3e5:	jmp    5e448 <probe::main+0x15e8>
   5e3e7:	mov    $0x1,%edi
   5e3ec:	mov    %rbx,%rsi
   5e3ef:	call   *0x126d1b(%rip)        # 185110 <_DYNAMIC+0x320>
   5e3f5:	jmp    5e448 <probe::main+0x15e8>
   5e3f7:	xor    %eax,%eax
   5e3f9:	jmp    5e263 <probe::main+0x1403>
   5e3fe:	lea    0x122563(%rip),%rdx        # 180968 <__dso_handle+0x140>
   5e405:	xor    %edi,%edi
   5e407:	xor    %esi,%esi
   5e409:	call   *0x126d31(%rip)        # 185140 <_DYNAMIC+0x350>
   5e40f:	jmp    5e448 <probe::main+0x15e8>
   5e411:	lea    0x122568(%rip),%rdx        # 180980 <__dso_handle+0x158>
   5e418:	mov    $0x1,%edi
   5e41d:	mov    $0x1,%esi
   5e422:	call   *0x126d18(%rip)        # 185140 <_DYNAMIC+0x350>
   5e428:	jmp    5e448 <probe::main+0x15e8>
   5e42a:	mov    $0x1,%edi
   5e42f:	mov    %rbx,%rsi
   5e432:	call   *0x126cd8(%rip)        # 185110 <_DYNAMIC+0x320>
   5e438:	jmp    5e448 <probe::main+0x15e8>
   5e43a:	mov    $0x8,%edi
   5e43f:	mov    %r12,%rsi
   5e442:	call   *0x126cc8(%rip)        # 185110 <_DYNAMIC+0x320>
   5e448:	ud2
   5e44a:	mov    %rax,%r15
   5e44d:	jmp    5e662 <probe::main+0x1802>
   5e452:	jmp    5e45c <probe::main+0x15fc>
   5e454:	mov    %r14,0x88(%rsp)
   5e45c:	mov    %r12,0x28(%rsp)
   5e461:	mov    %rax,%r15
   5e464:	mov    (%rsp),%rsi
   5e468:	test   %rsi,%rsi
   5e46b:	je     5e673 <probe::main+0x1813>
   5e471:	mov    0x8(%rsp),%rdi
   5e476:	shl    $0x3,%rsi
   5e47a:	mov    $0x8,%edx
   5e47f:	jmp    5e66d <probe::main+0x180d>
   5e484:	mov    %rax,%r15
   5e487:	mov    (%rsp),%rsi
   5e48b:	test   %rsi,%rsi
   5e48e:	je     5e746 <probe::main+0x18e6>
   5e494:	mov    0x8(%rsp),%rdi
   5e499:	shl    $0x3,%rsi
   5e49d:	mov    $0x8,%edx
   5e4a2:	call   *0x126bd0(%rip)        # 185078 <_DYNAMIC+0x288>
   5e4a8:	jmp    5e746 <probe::main+0x18e6>
   5e4ad:	mov    %rax,%r15
   5e4b0:	mov    (%rsp),%rsi
   5e4b4:	jmp    5e4be <probe::main+0x165e>
   5e4b6:	mov    %rax,%r15
   5e4b9:	mov    0x30(%rsp),%rsi
   5e4be:	test   %rsi,%rsi
   5e4c1:	je     5e590 <probe::main+0x1730>
   5e4c7:	mov    (%r14),%rdi
   5e4ca:	shl    $0x3,%rsi
   5e4ce:	mov    $0x8,%edx
   5e4d3:	call   *0x126b9f(%rip)        # 185078 <_DYNAMIC+0x288>
   5e4d9:	jmp    5e590 <probe::main+0x1730>
   5e4de:	mov    %rax,%r15
   5e4e1:	mov    $0xa,%esi
   5e4e6:	mov    $0x1,%edx
   5e4eb:	mov    %rbp,%rdi
   5e4ee:	call   *0x126b84(%rip)        # 185078 <_DYNAMIC+0x288>
   5e4f4:	jmp    5e55c <probe::main+0x16fc>
   5e4f6:	jmp    5e550 <probe::main+0x16f0>
   5e4f8:	mov    %rax,%r15
   5e4fb:	jmp    5e590 <probe::main+0x1730>
   5e500:	mov    %rax,%r15
   5e503:	mov    %rsp,%rdi
   5e506:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e50b:	jmp    5e510 <probe::main+0x16b0>
   5e50d:	mov    %rax,%r15
   5e510:	mov    $0x40,%esi
   5e515:	mov    $0x8,%edx
   5e51a:	mov    %r13,%rdi
   5e51d:	call   *0x126b55(%rip)        # 185078 <_DYNAMIC+0x288>
   5e523:	jmp    5e6a5 <probe::main+0x1845>
   5e528:	mov    %rax,%r15
   5e52b:	jmp    5e746 <probe::main+0x18e6>
   5e530:	mov    %rax,%r15
   5e533:	mov    %rsp,%rdi
   5e536:	call   5ccc0 <core::ptr::drop_glue::<serde_json::value::Value>>
   5e53b:	jmp    5e708 <probe::main+0x18a8>
   5e540:	mov    %rax,%r15
   5e543:	jmp    5e6fb <probe::main+0x189b>
   5e548:	mov    %rax,%r15
   5e54b:	jmp    5e6fb <probe::main+0x189b>
   5e550:	mov    %rax,%r15
   5e553:	test   %rbx,%rbx
   5e556:	je     5e6fb <probe::main+0x189b>
   5e55c:	mov    $0x1,%edx
   5e561:	mov    %r13,%rdi
   5e564:	mov    %rbx,%rsi
   5e567:	jmp    5e6f5 <probe::main+0x1895>
   5e56c:	mov    %rax,%r15
   5e56f:	mov    0x98(%rsp),%rax
   5e577:	test   %rax,%rax
   5e57a:	je     5e590 <probe::main+0x1730>
   5e57c:	lock decq (%rax)
   5e580:	jne    5e590 <probe::main+0x1730>
   5e582:	lea    0x98(%rsp),%rdi
   5e58a:	call   *0x126b00(%rip)        # 185090 <_DYNAMIC+0x2a0>
   5e590:	test   %r13,%r13
   5e593:	je     5e746 <probe::main+0x18e6>
   5e599:	shl    $0x3,%r13
   5e59d:	mov    $0x8,%edx
   5e5a2:	mov    0x28(%rsp),%rdi
   5e5a7:	mov    %r13,%rsi
   5e5aa:	call   *0x126ac8(%rip)        # 185078 <_DYNAMIC+0x288>
   5e5b0:	jmp    5e746 <probe::main+0x18e6>
   5e5b5:	call   *0x126ab5(%rip)        # 185070 <_DYNAMIC+0x280>
   5e5bb:	mov    %rax,%r15
   5e5be:	mov    0x38(%rsp),%rax
   5e5c3:	test   %rax,%rax
   5e5c6:	je     5e746 <probe::main+0x18e6>
   5e5cc:	lock decq (%rax)
   5e5d0:	jne    5e746 <probe::main+0x18e6>
   5e5d6:	mov    %rbx,%rdi
   5e5d9:	call   *0x126ab1(%rip)        # 185090 <_DYNAMIC+0x2a0>
   5e5df:	jmp    5e746 <probe::main+0x18e6>
   5e5e4:	call   *0x126a86(%rip)        # 185070 <_DYNAMIC+0x280>
   5e5ea:	mov    %rax,%r15
   5e5ed:	mov    $0x4,%esi
   5e5f2:	mov    $0x1,%edx
   5e5f7:	mov    0x28(%rsp),%rdi
   5e5fc:	jmp    5e6f5 <probe::main+0x1895>
   5e601:	mov    %rax,%r15
   5e604:	jmp    5e6fb <probe::main+0x189b>
   5e609:	mov    %rax,%r15
   5e60c:	jmp    5e6fb <probe::main+0x189b>
   5e611:	mov    %rax,%r15
   5e614:	jmp    5e6fb <probe::main+0x189b>
   5e619:	mov    %rax,%r15
   5e61c:	jmp    5e6fb <probe::main+0x189b>
   5e621:	mov    %rax,%r15
   5e624:	jmp    5e739 <probe::main+0x18d9>
   5e629:	mov    %rax,%r15
   5e62c:	lea    0x118(%rsp),%rdi
   5e634:	call   5cdb0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e639:	jmp    5e746 <probe::main+0x18e6>
   5e63e:	mov    %rax,%r15
   5e641:	jmp    5e746 <probe::main+0x18e6>
   5e646:	mov    %rax,%r15
   5e649:	jmp    5e746 <probe::main+0x18e6>
   5e64e:	mov    %rax,%r15
   5e651:	jmp    5e6fb <probe::main+0x189b>
   5e656:	mov    %rax,%r15
   5e659:	test   %r13,%r13
   5e65c:	je     5e673 <probe::main+0x1813>
   5e65e:	shl    $0x3,%r13
   5e662:	mov    $0x8,%edx
   5e667:	mov    %rbx,%rdi
   5e66a:	mov    %r13,%rsi
   5e66d:	call   *0x126a05(%rip)        # 185078 <_DYNAMIC+0x288>
   5e673:	cmpq   $0x0,0x28(%rsp)
   5e679:	je     5e746 <probe::main+0x18e6>
   5e67f:	mov    0x28(%rsp),%rsi
   5e684:	shl    $0x3,%rsi
   5e688:	mov    $0x8,%edx
   5e68d:	mov    0x88(%rsp),%rdi
   5e695:	call   *0x1269dd(%rip)        # 185078 <_DYNAMIC+0x288>
   5e69b:	jmp    5e746 <probe::main+0x18e6>
   5e6a0:	jmp    5e6d8 <probe::main+0x1878>
   5e6a2:	mov    %rax,%r15
   5e6a5:	mov    $0xd,%esi
   5e6aa:	mov    $0x1,%edx
   5e6af:	mov    %r12,%rdi
   5e6b2:	jmp    5e6f5 <probe::main+0x1895>
   5e6b4:	mov    %rax,%r15
   5e6b7:	lea    0x30(%rsp),%rdi
   5e6bc:	call   5cdb0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e6c1:	jmp    5e767 <probe::main+0x1907>
   5e6c6:	mov    %rax,%r15
   5e6c9:	lea    0x90(%rsp),%rdi
   5e6d1:	call   5cdb0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   5e6d6:	jmp    5e746 <probe::main+0x18e6>
   5e6d8:	mov    %rax,%r15
   5e6db:	jmp    5e708 <probe::main+0x18a8>
   5e6dd:	mov    %rax,%r15
   5e6e0:	jmp    5e767 <probe::main+0x1907>
   5e6e5:	mov    %rax,%r15
   5e6e8:	mov    $0xa,%esi
   5e6ed:	mov    $0x1,%edx
   5e6f2:	mov    %rbp,%rdi
   5e6f5:	call   *0x12697d(%rip)        # 185078 <_DYNAMIC+0x288>
   5e6fb:	lea    0xb0(%rsp),%rdi
   5e703:	call   5f940 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value> as core::ops::drop::Drop>::drop>
   5e708:	lea    0xe8(%rsp),%rdi
   5e710:	call   5fc40 <<alloc::vec::Vec<num_bigint::bigint::BigInt> as core::ops::drop::Drop>::drop>
   5e715:	mov    0xe8(%rsp),%rsi
   5e71d:	test   %rsi,%rsi
   5e720:	je     5e739 <probe::main+0x18d9>
   5e722:	mov    0xf0(%rsp),%rdi
   5e72a:	shl    $0x5,%rsi
   5e72e:	mov    $0x8,%edx
   5e733:	call   *0x12693f(%rip)        # 185078 <_DYNAMIC+0x288>
   5e739:	lea    0x90(%rsp),%rdi
   5e741:	call   5cbe0 <core::ptr::drop_glue::<[hyperreal::computable::node::Computable; 2]>>
   5e746:	mov    0xc8(%rsp),%rsi
   5e74e:	test   %rsi,%rsi
   5e751:	je     5e767 <probe::main+0x1907>
   5e753:	shl    $0x3,%rsi
   5e757:	mov    $0x8,%edx
   5e75c:	mov    0x78(%rsp),%rdi
   5e761:	call   *0x126911(%rip)        # 185078 <_DYNAMIC+0x288>
   5e767:	mov    0xd8(%rsp),%rbx
   5e76f:	mov    0xe0(%rsp),%r14
   5e777:	test   %r14,%r14
   5e77a:	jne    5e7a7 <probe::main+0x1947>
   5e77c:	mov    0xd0(%rsp),%rax
   5e784:	test   %rax,%rax
   5e787:	je     5e79f <probe::main+0x193f>
   5e789:	shl    $0x3,%rax
   5e78d:	lea    (%rax,%rax,2),%rsi
   5e791:	mov    $0x8,%edx
   5e796:	mov    %rbx,%rdi
   5e799:	call   *0x1268d9(%rip)        # 185078 <_DYNAMIC+0x288>
   5e79f:	mov    %r15,%rdi
   5e7a2:	call   17f640 <_Unwind_Resume@plt>
   5e7a7:	lea    0x8(%rbx),%r12
   5e7ab:	mov    0x1268c6(%rip),%r13        # 185078 <_DYNAMIC+0x288>
   5e7b2:	jmp    5e7c9 <probe::main+0x1969>
   5e7b4:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5e7c0:	add    $0x18,%r12
   5e7c4:	dec    %r14
   5e7c7:	je     5e77c <probe::main+0x191c>
   5e7c9:	mov    -0x8(%r12),%rsi
   5e7ce:	test   %rsi,%rsi
   5e7d1:	je     5e7c0 <probe::main+0x1960>
   5e7d3:	mov    (%r12),%rdi
   5e7d7:	mov    $0x1,%edx
   5e7dc:	call   *%r13
   5e7df:	jmp    5e7c0 <probe::main+0x1960>
   5e7e1:	call   *0x126889(%rip)        # 185070 <_DYNAMIC+0x280>
