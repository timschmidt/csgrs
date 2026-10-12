
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1850-bound-merge-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

000000000005ec40 <probe::main>:
   5ec40:	push   %rbp
   5ec41:	push   %r15
   5ec43:	push   %r14
   5ec45:	push   %r13
   5ec47:	push   %r12
   5ec49:	push   %rbx
   5ec4a:	sub    $0x1a8,%rsp
   5ec51:	mov    %rsp,%rbx
   5ec54:	mov    %rbx,%rdi
   5ec57:	call   *0x12a7fb(%rip)        # 189458 <_DYNAMIC+0x2d8>
   5ec5d:	movq   $0x1,0x20(%rsp)
   5ec66:	lea    0x138(%rsp),%rdi
   5ec6e:	mov    %rbx,%rsi
   5ec71:	call   5e270 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::skip::Skip<std::env::Args>>>::from_iter>
   5ec76:	mov    0x148(%rsp),%rdi
   5ec7e:	test   %rdi,%rdi
   5ec81:	je     609ae <probe::main+0x1d6e>
   5ec87:	cmp    $0x1,%rdi
   5ec8b:	je     609b7 <probe::main+0x1d77>
   5ec91:	mov    0x140(%rsp),%r9
   5ec99:	mov    0x20(%r9),%rcx
   5ec9d:	mov    0x28(%r9),%rsi
   5eca1:	cmp    $0x1,%rsi
   5eca5:	je     5ecb5 <probe::main+0x75>
   5eca7:	test   %rsi,%rsi
   5ecaa:	je     60a05 <probe::main+0x1dc5>
   5ecb0:	movzbl (%rcx),%edx
   5ecb3:	jmp    5eccc <probe::main+0x8c>
   5ecb5:	movzbl (%rcx),%edx
   5ecb8:	mov    $0x1,%al
   5ecba:	cmp    $0x2b,%edx
   5ecbd:	je     6074f <probe::main+0x1b0f>
   5ecc3:	cmp    $0x2d,%edx
   5ecc6:	je     6074f <probe::main+0x1b0f>
   5eccc:	xor    %eax,%eax
   5ecce:	cmp    $0x2b,%dl
   5ecd1:	sete   %al
   5ecd4:	mov    %rsi,%rdx
   5ecd7:	sub    %rax,%rdx
   5ecda:	add    %rax,%rcx
   5ecdd:	neg    %rax
   5ece0:	cmp    $0x11,%rdx
   5ece4:	jae    5ed27 <probe::main+0xe7>
   5ece6:	test   %rdx,%rdx
   5ece9:	je     5ed79 <probe::main+0x139>
   5ecef:	add    %rax,%rsi
   5ecf2:	neg    %rsi
   5ecf5:	xor    %r15d,%r15d
   5ecf8:	xor    %eax,%eax
   5ecfa:	nopw   0x0(%rax,%rax,1)
   5ed00:	movzbl (%rcx,%rax,1),%edx
   5ed04:	add    $0xffffffd0,%edx
   5ed07:	cmp    $0x9,%edx
   5ed0a:	ja     60988 <probe::main+0x1d48>
   5ed10:	lea    (%r15,%r15,4),%rdi
   5ed14:	mov    %edx,%edx
   5ed16:	lea    (%rdx,%rdi,2),%r15
   5ed1a:	inc    %rax
   5ed1d:	mov    %rsi,%rdx
   5ed20:	add    %rax,%rdx
   5ed23:	jne    5ed00 <probe::main+0xc0>
   5ed25:	jmp    5ed7c <probe::main+0x13c>
   5ed27:	add    %rax,%rsi
   5ed2a:	neg    %rsi
   5ed2d:	xor    %r15d,%r15d
   5ed30:	mov    $0xa,%edi
   5ed35:	xor    %r8d,%r8d
   5ed38:	nopl   0x0(%rax,%rax,1)
   5ed40:	mov    %rsi,%rax
   5ed43:	add    %r8,%rax
   5ed46:	je     5ed7c <probe::main+0x13c>
   5ed48:	mov    %r15,%rax
   5ed4b:	mul    %rdi
   5ed4e:	movzbl (%rcx,%r8,1),%edx
   5ed53:	jo     60745 <probe::main+0x1b05>
   5ed59:	add    $0xffffffd0,%edx
   5ed5c:	cmp    $0xa,%edx
   5ed5f:	jae    60988 <probe::main+0x1d48>
   5ed65:	mov    %rax,%r15
   5ed68:	mov    %edx,%eax
   5ed6a:	inc    %r8
   5ed6d:	add    %rax,%r15
   5ed70:	jae    5ed40 <probe::main+0x100>
   5ed72:	mov    $0x2,%al
   5ed74:	jmp    6074f <probe::main+0x1b0f>
   5ed79:	xor    %r15d,%r15d
   5ed7c:	mov    0x10(%r9),%rax
   5ed80:	add    $0xfffffffffffffff4,%rax
   5ed84:	cmp    $0xa,%rax
   5ed88:	ja     6096a <probe::main+0x1d2a>
   5ed8e:	mov    0x8(%r9),%rcx
   5ed92:	lea    -0x2bb29(%rip),%rdx        # 33270 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x110>
   5ed99:	movslq (%rdx,%rax,4),%rax
   5ed9d:	add    %rdx,%rax
   5eda0:	mov    %r9,0x190(%rsp)
   5eda8:	jmp    *%rax
   5edaa:	movabs $0x746e695f65646977,%rax
   5edb4:	xor    (%rcx),%rax
   5edb7:	mov    0x8(%rcx),%ecx
   5edba:	xor    $0x72656765,%rcx
   5edc1:	or     %rax,%rcx
   5edc4:	jne    6096a <probe::main+0x1d2a>
   5edca:	xor    %ecx,%ecx
   5edcc:	mov    $0xf4241,%eax
   5edd1:	jmp    5ef3b <probe::main+0x2fb>
   5edd6:	movdqu (%rcx),%xmm0
   5edda:	movdqu 0x5(%rcx),%xmm1
   5eddf:	pcmpeqb -0x2bf17(%rip),%xmm1        # 32ed0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x220>
   5ede7:	pcmpeqb -0x2be2f(%rip),%xmm0        # 32fc0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x310>
   5edef:	pand   %xmm1,%xmm0
   5edf3:	pmovmskb %xmm0,%eax
   5edf7:	cmp    $0xffff,%eax
   5edfc:	jne    6096a <probe::main+0x1d2a>
   5ee02:	mov    $0x1,%eax
   5ee07:	jmp    5f01f <probe::main+0x3df>
   5ee0c:	movdqu (%rcx),%xmm0
   5ee10:	movzwl 0x10(%rcx),%eax
   5ee14:	movd   %eax,%xmm1
   5ee18:	pcmpeqb -0x2be50(%rip),%xmm0        # 32fd0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x320>
   5ee20:	pcmpeqb -0x2bd68(%rip),%xmm1        # 330c0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x410>
   5ee28:	pand   %xmm0,%xmm1
   5ee2c:	pmovmskb %xmm1,%eax
   5ee30:	cmp    $0xffff,%eax
   5ee35:	jne    6096a <probe::main+0x1d2a>
   5ee3b:	mov    $0x1,%eax
   5ee40:	jmp    5eefd <probe::main+0x2bd>
   5ee45:	movdqu (%rcx),%xmm0
   5ee49:	pcmpeqb -0x2bd81(%rip),%xmm0        # 330d0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x420>
   5ee51:	pmovmskb %xmm0,%edx
   5ee55:	xor    %eax,%eax
   5ee57:	cmp    $0xffff,%edx
   5ee5d:	je     60541 <probe::main+0x1901>
   5ee63:	movdqu (%rcx),%xmm0
   5ee67:	pcmpeqb -0x2beff(%rip),%xmm0        # 32f70 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x2c0>
   5ee6f:	pmovmskb %xmm0,%ecx
   5ee73:	cmp    $0xffff,%ecx
   5ee79:	jne    6096a <probe::main+0x1d2a>
   5ee7f:	mov    $0x1,%dl
   5ee81:	mov    $0xf4240,%ecx
   5ee86:	mov    %rcx,0x70(%rsp)
   5ee8b:	xor    %ecx,%ecx
   5ee8d:	mov    $0x1,%sil
   5ee90:	mov    %rsi,0x78(%rsp)
   5ee95:	jmp    5f036 <probe::main+0x3f6>
   5ee9a:	movabs $0x6365725f65646977,%rax
   5eea4:	xor    (%rcx),%rax
   5eea7:	movabs $0x6c61636f72706963,%rdx
   5eeb1:	xor    0x7(%rcx),%rdx
   5eeb5:	or     %rax,%rdx
   5eeb8:	jne    6096a <probe::main+0x1d2a>
   5eebe:	mov    $0x1,%dl
   5eec0:	xor    %ecx,%ecx
   5eec2:	mov    $0xf4241,%eax
   5eec7:	jmp    5efdc <probe::main+0x39c>
   5eecc:	movdqu (%rcx),%xmm0
   5eed0:	movdqu 0x3(%rcx),%xmm1
   5eed5:	pcmpeqb -0x2bfed(%rip),%xmm1        # 32ef0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x240>
   5eedd:	pcmpeqb -0x2bda5(%rip),%xmm0        # 33140 <anon.9023bb4654a08944d5043612f590d0ff.344.llvm.2438011856832709387+0x60>
   5eee5:	pand   %xmm1,%xmm0
   5eee9:	pmovmskb %xmm0,%eax
   5eeed:	cmp    $0xffff,%eax
   5eef2:	jne    6096a <probe::main+0x1d2a>
   5eef8:	mov    $0x40,%eax
   5eefd:	xor    %ecx,%ecx
   5eeff:	mov    $0xf4241,%edx
   5ef04:	mov    %rdx,0x70(%rsp)
   5ef09:	xor    %edx,%edx
   5ef0b:	jmp    5f02d <probe::main+0x3ed>
   5ef10:	movabs $0x695f6d756964656d,%rax
   5ef1a:	xor    (%rcx),%rax
   5ef1d:	movabs $0x72656765746e695f,%rdx
   5ef27:	xor    0x6(%rcx),%rdx
   5ef2b:	or     %rax,%rdx
   5ef2e:	jne    6096a <probe::main+0x1d2a>
   5ef34:	xor    %ecx,%ecx
   5ef36:	mov    $0x10001,%eax
   5ef3b:	mov    %rax,0x70(%rsp)
   5ef40:	xor    %edx,%edx
   5ef42:	jmp    5efe1 <probe::main+0x3a1>
   5ef47:	movabs $0x72656765746e695f,%rdx
   5ef51:	movabs $0x6e695f6c6c616d73,%rsi
   5ef5b:	xor    (%rcx),%rsi
   5ef5e:	mov    0x5(%rcx),%rdi
   5ef62:	xor    %rdx,%rdi
   5ef65:	xor    %eax,%eax
   5ef67:	or     %rsi,%rdi
   5ef6a:	je     60552 <probe::main+0x1912>
   5ef70:	movabs $0x6e695f65736e6564,%rsi
   5ef7a:	xor    (%rcx),%rsi
   5ef7d:	xor    0x5(%rcx),%rdx
   5ef81:	or     %rsi,%rdx
   5ef84:	jne    6096a <probe::main+0x1d2a>
   5ef8a:	mov    $0x1,%cl
   5ef8c:	mov    %rcx,0x78(%rsp)
   5ef91:	mov    $0xf4240,%ecx
   5ef96:	mov    %rcx,0x70(%rsp)
   5ef9b:	xor    %ecx,%ecx
   5ef9d:	xor    %edx,%edx
   5ef9f:	jmp    5f036 <probe::main+0x3f6>
   5efa4:	movdqu (%rcx),%xmm0
   5efa8:	movzbl 0x10(%rcx),%eax
   5efac:	movd   %eax,%xmm1
   5efb0:	pcmpeqb -0x2bd68(%rip),%xmm0        # 33250 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0xf0>
   5efb8:	pcmpeqb -0x2c060(%rip),%xmm1        # 32f60 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x2b0>
   5efc0:	pand   %xmm0,%xmm1
   5efc4:	pmovmskb %xmm1,%eax
   5efc8:	cmp    $0xffff,%eax
   5efcd:	jne    6096a <probe::main+0x1d2a>
   5efd3:	mov    $0x1,%dl
   5efd5:	xor    %ecx,%ecx
   5efd7:	mov    $0x10001,%eax
   5efdc:	mov    %rax,0x70(%rsp)
   5efe1:	movq   $0x0,0x78(%rsp)
   5efea:	xor    %eax,%eax
   5efec:	jmp    5f036 <probe::main+0x3f6>
   5efee:	movdqu (%rcx),%xmm0
   5eff2:	movdqu 0x6(%rcx),%xmm1
   5eff7:	pcmpeqb -0x2bfaf(%rip),%xmm1        # 33050 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x3a0>
   5efff:	pcmpeqb -0x2c047(%rip),%xmm0        # 32fc0 <anon.971d68e3f146e1126e2465ffbb582c25.36.llvm.5892873741908922181+0x310>
   5f007:	pand   %xmm1,%xmm0
   5f00b:	pmovmskb %xmm0,%eax
   5f00f:	cmp    $0xffff,%eax
   5f014:	jne    6096a <probe::main+0x1d2a>
   5f01a:	mov    $0x40,%eax
   5f01f:	mov    $0x1,%dl
   5f021:	xor    %ecx,%ecx
   5f023:	mov    $0xf4241,%esi
   5f028:	mov    %rsi,0x70(%rsp)
   5f02d:	movq   $0x0,0x78(%rsp)
   5f036:	mov    %ecx,0xa0(%rsp)
   5f03d:	mov    %dl,0x57(%rsp)
   5f041:	mov    %rax,0xf0(%rsp)
   5f049:	call   *0x12a411(%rip)        # 189460 <_DYNAMIC+0x2e0>
   5f04f:	mov    %rax,0x30(%rsp)
   5f054:	mov    %rdx,0x38(%rsp)
   5f059:	mov    %rsp,%rdi
   5f05c:	lea    0x30(%rsp),%rsi
   5f061:	mov    $0xffffff40,%edx
   5f066:	call   *0x12a3fc(%rip)        # 189468 <_DYNAMIC+0x2e8>
   5f06c:	mov    (%rsp),%rsi
   5f070:	cmp    $0xffffffffffffffff,%rsi
   5f074:	je     60878 <probe::main+0x1c38>
   5f07a:	test   %rsi,%rsi
   5f07d:	je     5f093 <probe::main+0x453>
   5f07f:	mov    0x8(%rsp),%rdi
   5f084:	shl    $0x3,%rsi
   5f088:	mov    $0x8,%edx
   5f08d:	call   *0x12a375(%rip)        # 189408 <_DYNAMIC+0x288>
   5f093:	lea    0x38(%rsp),%r14
   5f098:	mov    0x30(%rsp),%rax
   5f09d:	lock decq (%rax)
   5f0a1:	jne    5f0ae <probe::main+0x46e>
   5f0a3:	lea    0x30(%rsp),%rdi
   5f0a8:	call   *0x12a382(%rip)        # 189430 <_DYNAMIC+0x2b0>
   5f0ae:	mov    0x38(%rsp),%rax
   5f0b3:	test   %rax,%rax
   5f0b6:	je     5f0c7 <probe::main+0x487>
   5f0b8:	lock decq (%rax)
   5f0bc:	jne    5f0c7 <probe::main+0x487>
   5f0be:	mov    %r14,%rdi
   5f0c1:	call   *0x12a371(%rip)        # 189438 <_DYNAMIC+0x2b8>
   5f0c7:	call   *0x12a3a3(%rip)        # 189470 <_DYNAMIC+0x2f0>
   5f0cd:	mov    %rax,0x30(%rsp)
   5f0d2:	mov    %rdx,0x38(%rsp)
   5f0d7:	mov    %rsp,%rdi
   5f0da:	lea    0x30(%rsp),%rsi
   5f0df:	xor    %edx,%edx
   5f0e1:	call   *0x12a381(%rip)        # 189468 <_DYNAMIC+0x2e8>
   5f0e7:	cmpq   $0xffffffffffffffff,(%rsp)
   5f0ec:	je     608b1 <probe::main+0x1c71>
   5f0f2:	lea    0x38(%rsp),%r14
   5f0f7:	movdqu (%rsp),%xmm0
   5f0fc:	movdqu 0x10(%rsp),%xmm1
   5f102:	movdqa %xmm1,0x120(%rsp)
   5f10b:	movdqa %xmm0,0x110(%rsp)
   5f114:	mov    0x30(%rsp),%rax
   5f119:	lock decq (%rax)
   5f11d:	jne    5f12a <probe::main+0x4ea>
   5f11f:	lea    0x30(%rsp),%rdi
   5f124:	call   *0x12a306(%rip)        # 189430 <_DYNAMIC+0x2b0>
   5f12a:	mov    0x38(%rsp),%rax
   5f12f:	test   %rax,%rax
   5f132:	je     5f143 <probe::main+0x503>
   5f134:	lock decq (%rax)
   5f138:	jne    5f143 <probe::main+0x503>
   5f13a:	mov    %r14,%rdi
   5f13d:	call   *0x12a2f5(%rip)        # 189438 <_DYNAMIC+0x2b8>
   5f143:	movzbl 0x128(%rsp),%ebx
   5f14b:	cmpb   $0x0,0x78(%rsp)
   5f150:	mov    %r15,0xc0(%rsp)
   5f158:	je     5f1e4 <probe::main+0x5a4>
   5f15e:	cmpq   $0x0,0x120(%rsp)
   5f167:	je     5f26f <probe::main+0x62f>
   5f16d:	mov    0x70(%rsp),%rdx
   5f172:	mov    %edx,%eax
   5f174:	and    $0x3,%al
   5f176:	shr    $0x6,%edx
   5f179:	movq   $0xffffffffffffffff,(%rsp)
   5f181:	lea    0x110(%rsp),%rcx
   5f189:	mov    %rcx,0x8(%rsp)
   5f18e:	movzbl %al,%ecx
   5f191:	lea    0x30(%rsp),%rdi
   5f196:	mov    %rsp,%rsi
   5f199:	call   *0x12a2d9(%rip)        # 189478 <_DYNAMIC+0x2f8>
   5f19f:	lea    0x8(%rsp),%r12
   5f1a4:	cmp    $0x1,%ebx
   5f1a7:	jne    5f298 <probe::main+0x658>
   5f1ad:	mov    0x30(%rsp),%r15
   5f1b2:	cmp    $0x4,%r15
   5f1b6:	jb     5f373 <probe::main+0x733>
   5f1bc:	mov    0x38(%rsp),%rdi
   5f1c1:	shl    $0x3,%r15
   5f1c5:	mov    $0x8,%edx
   5f1ca:	mov    %r15,%rsi
   5f1cd:	call   *0x12a235(%rip)        # 189408 <_DYNAMIC+0x288>
   5f1d3:	movq   $0x8,0x38(%rsp)
   5f1dc:	xor    %r15d,%r15d
   5f1df:	jmp    5f373 <probe::main+0x733>
   5f1e4:	cmpq   $0x0,0x120(%rsp)
   5f1ed:	je     5f3ca <probe::main+0x78a>
   5f1f3:	mov    0x70(%rsp),%rax
   5f1f8:	lea    -0x1(%rax),%rdx
   5f1fc:	mov    %edx,%eax
   5f1fe:	and    $0x3f,%al
   5f200:	shr    $0x6,%rdx
   5f204:	movq   $0xffffffffffffffff,(%rsp)
   5f20c:	lea    0x110(%rsp),%rcx
   5f214:	mov    %rcx,0x8(%rsp)
   5f219:	movzbl %al,%ecx
   5f21c:	lea    0x30(%rsp),%rdi
   5f221:	mov    %rsp,%rsi
   5f224:	call   *0x12a24e(%rip)        # 189478 <_DYNAMIC+0x2f8>
   5f22a:	lea    0x8(%rsp),%r12
   5f22f:	cmp    $0x1,%ebx
   5f232:	jne    5f3f3 <probe::main+0x7b3>
   5f238:	mov    0x30(%rsp),%r15
   5f23d:	cmp    $0x4,%r15
   5f241:	jb     5f492 <probe::main+0x852>
   5f247:	mov    0x38(%rsp),%rdi
   5f24c:	shl    $0x3,%r15
   5f250:	mov    $0x8,%edx
   5f255:	mov    %r15,%rsi
   5f258:	call   *0x12a1aa(%rip)        # 189408 <_DYNAMIC+0x288>
   5f25e:	movq   $0x8,0x38(%rsp)
   5f267:	xor    %r15d,%r15d
   5f26a:	jmp    5f492 <probe::main+0x852>
   5f26f:	movq   $0x0,0x30(%rsp)
   5f278:	movq   $0x8,0x38(%rsp)
   5f281:	movq   $0x0,0x40(%rsp)
   5f28a:	lea    0x8(%rsp),%r12
   5f28f:	cmp    $0x1,%ebx
   5f292:	je     5f1ad <probe::main+0x56d>
   5f298:	mov    0x30(%rsp),%r15
   5f29d:	mov    0x40(%rsp),%rbp
   5f2a2:	test   %rbp,%rbp
   5f2a5:	je     5f373 <probe::main+0x733>
   5f2ab:	mov    0x38(%rsp),%r14
   5f2b0:	test   %ebx,%ebx
   5f2b2:	je     603d4 <probe::main+0x1794>
   5f2b8:	cmp    $0x1,%ebx
   5f2bb:	je     5f378 <probe::main+0x738>
   5f2c1:	movdqu -0x2bfd9(%rip),%xmm0        # 332f0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x190>
   5f2c9:	movdqa %xmm0,(%rsp)
   5f2ce:	movq   $0x0,0x10(%rsp)
   5f2d7:	mov    %rsp,%rdi
   5f2da:	call   *0x12a1a0(%rip)        # 189480 <_DYNAMIC+0x300>
   5f2e0:	mov    0x8(%rsp),%rax
   5f2e5:	movq   $0x1,(%rax)
   5f2ec:	mov    (%rsp),%r12
   5f2f0:	mov    0x8(%rsp),%r13
   5f2f5:	mov    $0x1,%al
   5f2f7:	cmp    $0x1,%rbp
   5f2fb:	jne    5f30f <probe::main+0x6cf>
   5f2fd:	mov    0x0(%r13),%rax
   5f301:	cmp    %rax,(%r14)
   5f304:	je     604fe <probe::main+0x18be>
   5f30a:	seta   %al
   5f30d:	sbb    $0x0,%al
   5f30f:	test   %al,%al
   5f311:	je     604fe <probe::main+0x18be>
   5f317:	movzbl %al,%eax
   5f31a:	cmp    $0x1,%eax
   5f31d:	jne    6069b <probe::main+0x1a5b>
   5f323:	mov    %r15,(%rsp)
   5f327:	mov    %r14,0x8(%rsp)
   5f32c:	mov    %rbp,0x10(%rsp)
   5f331:	mov    %rsp,%rdi
   5f334:	mov    $0x1,%esi
   5f339:	call   *0x12a149(%rip)        # 189488 <_DYNAMIC+0x308>
   5f33f:	mov    (%rsp),%rsi
   5f343:	mov    0x8(%rsp),%rdi
   5f348:	mov    0x10(%rsp),%rax
   5f34d:	test   %rax,%rax
   5f350:	je     607aa <probe::main+0x1b6a>
   5f356:	mov    %rsi,0x80(%rsp)
   5f35e:	mov    %rdi,0x88(%rsp)
   5f366:	mov    %rax,0x90(%rsp)
   5f36e:	jmp    6068e <probe::main+0x1a4e>
   5f373:	mov    0x38(%rsp),%r14
   5f378:	movdqu -0x2c090(%rip),%xmm0        # 332f0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x190>
   5f380:	movdqa %xmm0,(%rsp)
   5f385:	movq   $0x0,0x10(%rsp)
   5f38e:	mov    %rsp,%rdi
   5f391:	call   *0x12a0e9(%rip)        # 189480 <_DYNAMIC+0x300>
   5f397:	mov    0x8(%rsp),%rax
   5f39c:	movq   $0x1,(%rax)
   5f3a3:	movdqa (%rsp),%xmm0
   5f3a8:	movdqa %xmm0,0x80(%rsp)
   5f3b1:	movq   $0x1,0x90(%rsp)
   5f3bd:	movb   $0x0,0x98(%rsp)
   5f3c5:	jmp    5f4e4 <probe::main+0x8a4>
   5f3ca:	movq   $0x0,0x30(%rsp)
   5f3d3:	movq   $0x8,0x38(%rsp)
   5f3dc:	movq   $0x0,0x40(%rsp)
   5f3e5:	lea    0x8(%rsp),%r12
   5f3ea:	cmp    $0x1,%ebx
   5f3ed:	je     5f238 <probe::main+0x5f8>
   5f3f3:	mov    0x30(%rsp),%r15
   5f3f8:	mov    0x40(%rsp),%rbp
   5f3fd:	test   %rbp,%rbp
   5f400:	je     5f492 <probe::main+0x852>
   5f406:	mov    0x38(%rsp),%r14
   5f40b:	test   %ebx,%ebx
   5f40d:	je     60452 <probe::main+0x1812>
   5f413:	cmp    $0x1,%ebx
   5f416:	je     5f497 <probe::main+0x857>
   5f418:	mov    %r15,(%rsp)
   5f41c:	mov    %r14,0x8(%rsp)
   5f421:	mov    %rbp,0x10(%rsp)
   5f426:	incq   (%r14)
   5f429:	jne    6059a <probe::main+0x195a>
   5f42f:	mov    $0x1,%r15d
   5f435:	cmp    $0x1,%rbp
   5f439:	je     5f46d <probe::main+0x82d>
   5f43b:	shl    $0x3,%rbp
   5f43f:	mov    $0x1,%al
   5f441:	mov    $0x8,%ecx
   5f446:	cs nopw 0x0(%rax,%rax,1)
   5f450:	add    $0xff,%al
   5f452:	adcq   $0x0,(%r14,%rcx,1)
   5f457:	setb   %al
   5f45a:	jae    6059a <probe::main+0x195a>
   5f460:	add    $0x8,%rcx
   5f464:	cmp    %rcx,%rbp
   5f467:	jne    5f450 <probe::main+0x810>
   5f469:	movzbl %al,%r15d
   5f46d:	mov    0x10(%rsp),%rbx
   5f472:	cmp    (%rsp),%rbx
   5f476:	jne    5f481 <probe::main+0x841>
   5f478:	mov    %rsp,%rdi
   5f47b:	call   *0x129fff(%rip)        # 189480 <_DYNAMIC+0x300>
   5f481:	mov    0x8(%rsp),%rax
   5f486:	mov    %r15,(%rax,%rbx,8)
   5f48a:	inc    %rbx
   5f48d:	jmp    6059f <probe::main+0x195f>
   5f492:	mov    0x38(%rsp),%r14
   5f497:	movdqu -0x2c1af(%rip),%xmm0        # 332f0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x190>
   5f49f:	movdqa %xmm0,(%rsp)
   5f4a4:	movq   $0x0,0x10(%rsp)
   5f4ad:	mov    %rsp,%rdi
   5f4b0:	call   *0x129fca(%rip)        # 189480 <_DYNAMIC+0x300>
   5f4b6:	mov    0x8(%rsp),%rax
   5f4bb:	movq   $0x1,(%rax)
   5f4c2:	movdqa (%rsp),%xmm0
   5f4c7:	movdqa %xmm0,0x80(%rsp)
   5f4d0:	movq   $0x1,0x90(%rsp)
   5f4dc:	movb   $0x2,0x98(%rsp)
   5f4e4:	test   %r15,%r15
   5f4e7:	je     5f4fe <probe::main+0x8be>
   5f4e9:	shl    $0x3,%r15
   5f4ed:	mov    $0x8,%edx
   5f4f2:	mov    %r14,%rdi
   5f4f5:	mov    %r15,%rsi
   5f4f8:	call   *0x129f0a(%rip)        # 189408 <_DYNAMIC+0x288>
   5f4fe:	mov    0x90(%rsp),%rax
   5f506:	test   %rax,%rax
   5f509:	je     609c9 <probe::main+0x1d89>
   5f50f:	mov    0x88(%rsp),%rcx
   5f517:	mov    $0x7f,%edx
   5f51c:	bsr    -0x8(%rcx,%rax,8),%rdx
   5f522:	xor    $0x3f,%rdx
   5f526:	shl    $0x6,%rax
   5f52a:	sub    %rdx,%rax
   5f52d:	mov    %rax,0x30(%rsp)
   5f532:	mov    0x70(%rsp),%rcx
   5f537:	mov    %rcx,(%rsp)
   5f53b:	cmp    %rcx,%rax
   5f53e:	mov    0xc0(%rsp),%r12
   5f546:	jne    609db <probe::main+0x1d9b>
   5f54c:	lea    0xf0(%rsp),%rbx
   5f554:	mov    %rbx,0x170(%rsp)
   5f55c:	lea    0x57(%rsp),%rax
   5f561:	mov    %rax,0x178(%rsp)
   5f569:	lea    0x110(%rsp),%rax
   5f571:	mov    %rax,0x180(%rsp)
   5f579:	lea    0x80(%rsp),%rax
   5f581:	mov    %rax,0x188(%rsp)
   5f589:	movq   $0x0,0x10(%rsp)
   5f592:	movq   $0x2,0x18(%rsp)
   5f59b:	lea    0x170(%rsp),%r14
   5f5a3:	mov    %r14,(%rsp)
   5f5a7:	lea    0xa0(%rsp),%r15
   5f5af:	mov    %r15,0x8(%rsp)
   5f5b4:	lea    0xf8(%rsp),%rdi
   5f5bc:	mov    %rsp,%rsi
   5f5bf:	call   5e670 <<alloc::vec::Vec<alloc::string::String> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<alloc::string::String, core::iter::adapters::map::Map<core::ops::range::Range<usize>, probe::main::{closure#1}>>>::from_iter>
   5f5c4:	movq   $0x0,0x150(%rsp)
   5f5d0:	movq   $0x0,0x158(%rsp)
   5f5dc:	mov    0xf0(%rsp),%rax
   5f5e4:	test   %r12,%r12
   5f5e7:	je     5f781 <probe::main+0xb41>
   5f5ed:	mov    0x100(%rsp),%rcx
   5f5f5:	mov    0x108(%rsp),%rsi
   5f5fd:	xor    %r13d,%r13d
   5f600:	xor    %edx,%edx
   5f602:	xor    %ebp,%ebp
   5f604:	jmp    5f64b <probe::main+0xa0b>
   5f606:	cs nopw 0x0(%rax,%rax,1)
   5f610:	mov    %r13,%r10
   5f613:	inc    %rdx
   5f616:	mov    %r10,%r13
   5f619:	shl    $0x5,%r13
   5f61d:	sub    %r10,%r13
   5f620:	add    $0xff,%r13
   5f627:	mov    %r13,0x150(%rsp)
   5f62f:	add    %rdi,%rbp
   5f632:	mov    %rbp,0x158(%rsp)
   5f63a:	mov    0xc0(%rsp),%r12
   5f642:	cmp    %r12,%rdx
   5f645:	je     5f786 <probe::main+0xb46>
   5f64b:	mov    %edx,%r8d
   5f64e:	and    $0x1,%r8d
   5f652:	cmp    $0x1,%rax
   5f656:	mov    $0x0,%edi
   5f65b:	cmovne %r8,%rdi
   5f65f:	cmp    %rsi,%rdi
   5f662:	jae    6098f <probe::main+0x1d4f>
   5f668:	lea    (%rdi,%rdi,2),%r8
   5f66c:	mov    0x10(%rcx,%r8,8),%rdi
   5f671:	test   %rdi,%rdi
   5f674:	je     5f610 <probe::main+0x9d0>
   5f676:	mov    0x8(%rcx,%r8,8),%r8
   5f67b:	mov    %rdi,%r11
   5f67e:	and    $0x7,%r11
   5f682:	je     5f6c0 <probe::main+0xa80>
   5f684:	mov    %r8,%r9
   5f687:	mov    %r13,%r10
   5f68a:	nopw   0x0(%rax,%rax,1)
   5f690:	movzbl (%r9),%r12d
   5f694:	inc    %r9
   5f697:	shl    $0x5,%r10
   5f69b:	sub    %r13,%r10
   5f69e:	add    %r12,%r10
   5f6a1:	mov    %r10,%r13
   5f6a4:	dec    %r11
   5f6a7:	jne    5f690 <probe::main+0xa50>
   5f6a9:	cmp    $0x8,%rdi
   5f6ad:	jb     5f613 <probe::main+0x9d3>
   5f6b3:	jmp    5f6d0 <probe::main+0xa90>
   5f6b5:	data16 cs nopw 0x0(%rax,%rax,1)
   5f6c0:	mov    %r13,%r10
   5f6c3:	mov    %r8,%r9
   5f6c6:	cmp    $0x8,%rdi
   5f6ca:	jb     5f613 <probe::main+0x9d3>
   5f6d0:	add    %rdi,%r8
   5f6d3:	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5f6e0:	movzbl (%r9),%r11d
   5f6e4:	mov    %r10,%r12
   5f6e7:	shl    $0x5,%r12
   5f6eb:	sub    %r10,%r12
   5f6ee:	add    %r11,%r12
   5f6f1:	movzbl 0x1(%r9),%r10d
   5f6f6:	mov    %r12,%r11
   5f6f9:	shl    $0x5,%r11
   5f6fd:	sub    %r12,%r11
   5f700:	add    %r10,%r11
   5f703:	movzbl 0x2(%r9),%r10d
   5f708:	mov    %r11,%r12
   5f70b:	shl    $0x5,%r12
   5f70f:	sub    %r11,%r12
   5f712:	add    %r10,%r12
   5f715:	movzbl 0x3(%r9),%r10d
   5f71a:	mov    %r12,%r11
   5f71d:	shl    $0x5,%r11
   5f721:	sub    %r12,%r11
   5f724:	add    %r10,%r11
   5f727:	movzbl 0x4(%r9),%r10d
   5f72c:	mov    %r11,%r12
   5f72f:	shl    $0x5,%r12
   5f733:	sub    %r11,%r12
   5f736:	add    %r10,%r12
   5f739:	movzbl 0x5(%r9),%r10d
   5f73e:	mov    %r12,%r11
   5f741:	shl    $0x5,%r11
   5f745:	sub    %r12,%r11
   5f748:	add    %r10,%r11
   5f74b:	movzbl 0x6(%r9),%r10d
   5f750:	mov    %r11,%r12
   5f753:	shl    $0x5,%r12
   5f757:	sub    %r11,%r12
   5f75a:	add    %r10,%r12
   5f75d:	movzbl 0x7(%r9),%r11d
   5f762:	add    $0x8,%r9
   5f766:	mov    %r12,%r10
   5f769:	shl    $0x5,%r10
   5f76d:	sub    %r12,%r10
   5f770:	add    %r11,%r10
   5f773:	cmp    %r8,%r9
   5f776:	jne    5f6e0 <probe::main+0xaa0>
   5f77c:	jmp    5f613 <probe::main+0x9d3>
   5f781:	xor    %ebp,%ebp
   5f783:	xor    %r13d,%r13d
   5f786:	mov    %rbp,0xe0(%rsp)
   5f78e:	test   %rax,%rax
   5f791:	cmove  %r12,%rax
   5f795:	movq   $0x0,0x18(%rsp)
   5f79e:	mov    %rax,0x20(%rsp)
   5f7a3:	mov    %r14,(%rsp)
   5f7a7:	mov    %rbx,0x8(%rsp)
   5f7ac:	mov    %r15,0x10(%rsp)
   5f7b1:	lea    0xc8(%rsp),%rdi
   5f7b9:	mov    %rsp,%rsi
   5f7bc:	call   5e760 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as alloc::vec::spec_from_iter_nested::SpecFromIterNested<hyperreal::computable::node::Computable, core::iter::adapters::map::Map<core::ops::range::Range<usize>, probe::main::{closure#2}>>>::from_iter>
   5f7c1:	movq   $0x0,0x160(%rsp)
   5f7cd:	movq   $0x0,0x168(%rsp)
   5f7d9:	call   *0x129cb1(%rip)        # 189490 <_DYNAMIC+0x310>
   5f7df:	mov    %rax,0x198(%rsp)
   5f7e7:	mov    %edx,0x1a0(%rsp)
   5f7ee:	test   %r12,%r12
   5f7f1:	je     5fa76 <probe::main+0xe36>
   5f7f7:	mov    0xd8(%rsp),%r15
   5f7ff:	test   %r15,%r15
   5f802:	je     609f6 <probe::main+0x1db6>
   5f808:	mov    0xd0(%rsp),%rax
   5f810:	mov    %rax,0xe8(%rsp)
   5f818:	xor    %ebp,%ebp
   5f81a:	xor    %r14d,%r14d
   5f81d:	xor    %esi,%esi
   5f81f:	xor    %r12d,%r12d
   5f822:	jmp    5f844 <probe::main+0xc04>
   5f824:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5f830:	inc    %r12
   5f833:	mov    %rbx,%rsi
   5f836:	cmp    0xc0(%rsp),%r12
   5f83e:	je     5fa7b <probe::main+0xe3b>
   5f844:	lea    0x1(%rsi),%rax
   5f848:	cmp    %r15,%rax
   5f84b:	mov    $0x0,%ebx
   5f850:	cmovne %rax,%rbx
   5f854:	mov    %r12,%rax
   5f857:	or     %r15,%rax
   5f85a:	shr    $0x20,%rax
   5f85e:	jne    5f86a <probe::main+0xc2a>
   5f860:	mov    %r12d,%eax
   5f863:	xor    %edx,%edx
   5f865:	div    %r15d
   5f868:	mov    %edx,%esi
   5f86a:	shl    $0x4,%rsi
   5f86e:	add    0xe8(%rsp),%rsi
   5f876:	mov    0xa0(%rsp),%edx
   5f87d:	mov    %rsp,%rdi
   5f880:	call   *0x129be2(%rip)        # 189468 <_DYNAMIC+0x2e8>
   5f886:	cmpq   $0xffffffffffffffff,(%rsp)
   5f88b:	je     606e5 <probe::main+0x1aa5>
   5f891:	movdqu (%rsp),%xmm0
   5f896:	movdqu 0x10(%rsp),%xmm1
   5f89c:	movdqa %xmm1,0x40(%rsp)
   5f8a2:	movdqa %xmm0,0x30(%rsp)
   5f8a8:	lea    0x30(%rsp),%rdi
   5f8ad:	movq   $0x0,0x58(%rsp)
   5f8b6:	movq   $0x1,0x60(%rsp)
   5f8bf:	movq   $0x0,0x68(%rsp)
   5f8c8:	movq   $0x60000020,0x10(%rsp)
   5f8d1:	lea    0x58(%rsp),%rax
   5f8d6:	mov    %rax,(%rsp)
   5f8da:	lea    0x125337(%rip),%rax        # 184c18 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x260>
   5f8e1:	mov    %rax,0x8(%rsp)
   5f8e6:	mov    %rsp,%rsi
   5f8e9:	call   *0x129ba9(%rip)        # 189498 <_DYNAMIC+0x318>
   5f8ef:	test   %al,%al
   5f8f1:	jne    60718 <probe::main+0x1ad8>
   5f8f7:	mov    0x58(%rsp),%rsi
   5f8fc:	mov    0x60(%rsp),%rdi
   5f901:	mov    0x68(%rsp),%rax
   5f906:	add    %rax,%r14
   5f909:	mov    %r14,0x168(%rsp)
   5f911:	test   %rax,%rax
   5f914:	je     5f960 <probe::main+0xd20>
   5f916:	mov    %rax,%r8
   5f919:	and    $0x7,%r8
   5f91d:	je     5f970 <probe::main+0xd30>
   5f91f:	mov    %rdi,%rcx
   5f922:	mov    %rbp,%rdx
   5f925:	data16 cs nopw 0x0(%rax,%rax,1)
   5f930:	movzbl (%rcx),%r9d
   5f934:	inc    %rcx
   5f937:	shl    $0x5,%rdx
   5f93b:	sub    %rbp,%rdx
   5f93e:	add    %r9,%rdx
   5f941:	mov    %rdx,%rbp
   5f944:	dec    %r8
   5f947:	jne    5f930 <probe::main+0xcf0>
   5f949:	cmp    $0x8,%rax
   5f94d:	jae    5f980 <probe::main+0xd40>
   5f94f:	jmp    5fa26 <probe::main+0xde6>
   5f954:	data16 data16 cs nopw 0x0(%rax,%rax,1)
   5f960:	mov    %rbp,%rdx
   5f963:	jmp    5fa26 <probe::main+0xde6>
   5f968:	nopl   0x0(%rax,%rax,1)
   5f970:	mov    %rbp,%rdx
   5f973:	mov    %rdi,%rcx
   5f976:	cmp    $0x8,%rax
   5f97a:	jb     5fa26 <probe::main+0xde6>
   5f980:	add    %rdi,%rax
   5f983:	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   5f990:	movzbl (%rcx),%r8d
   5f994:	mov    %rdx,%r9
   5f997:	shl    $0x5,%r9
   5f99b:	sub    %rdx,%r9
   5f99e:	add    %r8,%r9
   5f9a1:	movzbl 0x1(%rcx),%edx
   5f9a5:	mov    %r9,%r8
   5f9a8:	shl    $0x5,%r8
   5f9ac:	sub    %r9,%r8
   5f9af:	add    %rdx,%r8
   5f9b2:	movzbl 0x2(%rcx),%edx
   5f9b6:	mov    %r8,%r9
   5f9b9:	shl    $0x5,%r9
   5f9bd:	sub    %r8,%r9
   5f9c0:	add    %rdx,%r9
   5f9c3:	movzbl 0x3(%rcx),%edx
   5f9c7:	mov    %r9,%r8
   5f9ca:	shl    $0x5,%r8
   5f9ce:	sub    %r9,%r8
   5f9d1:	add    %rdx,%r8
   5f9d4:	movzbl 0x4(%rcx),%edx
   5f9d8:	mov    %r8,%r9
   5f9db:	shl    $0x5,%r9
   5f9df:	sub    %r8,%r9
   5f9e2:	add    %rdx,%r9
   5f9e5:	movzbl 0x5(%rcx),%edx
   5f9e9:	mov    %r9,%r8
   5f9ec:	shl    $0x5,%r8
   5f9f0:	sub    %r9,%r8
   5f9f3:	add    %rdx,%r8
   5f9f6:	movzbl 0x6(%rcx),%edx
   5f9fa:	mov    %r8,%r9
   5f9fd:	shl    $0x5,%r9
   5fa01:	sub    %r8,%r9
   5fa04:	add    %rdx,%r9
   5fa07:	movzbl 0x7(%rcx),%r8d
   5fa0c:	add    $0x8,%rcx
   5fa10:	mov    %r9,%rdx
   5fa13:	shl    $0x5,%rdx
   5fa17:	sub    %r9,%rdx
   5fa1a:	add    %r8,%rdx
   5fa1d:	cmp    %rax,%rcx
   5fa20:	jne    5f990 <probe::main+0xd50>
   5fa26:	mov    %rdx,%rbp
   5fa29:	shl    $0x5,%rbp
   5fa2d:	sub    %rdx,%rbp
   5fa30:	add    $0xff,%rbp
   5fa37:	mov    %rbp,0x160(%rsp)
   5fa3f:	test   %rsi,%rsi
   5fa42:	je     5fa4f <probe::main+0xe0f>
   5fa44:	mov    $0x1,%edx
   5fa49:	call   *0x1299b9(%rip)        # 189408 <_DYNAMIC+0x288>
   5fa4f:	mov    0x30(%rsp),%rsi
   5fa54:	test   %rsi,%rsi
   5fa57:	je     5f830 <probe::main+0xbf0>
   5fa5d:	mov    0x38(%rsp),%rdi
   5fa62:	shl    $0x3,%rsi
   5fa66:	mov    $0x8,%edx
   5fa6b:	call   *0x129997(%rip)        # 189408 <_DYNAMIC+0x288>
   5fa71:	jmp    5f830 <probe::main+0xbf0>
   5fa76:	xor    %r14d,%r14d
   5fa79:	xor    %ebp,%ebp
   5fa7b:	lea    0x198(%rsp),%rdi
   5fa83:	call   *0x129a17(%rip)        # 1894a0 <_DYNAMIC+0x320>
   5fa89:	mov    %edx,%ecx
   5fa8b:	mov    $0x3b9aca00,%edx
   5fa90:	mul    %rdx
   5fa93:	mov    %rdx,%rbx
   5fa96:	mov    %ecx,%ecx
   5fa98:	add    %rax,%rcx
   5fa9b:	adc    $0x0,%rbx
   5fa9f:	cmp    %r13,%rbp
   5faa2:	jne    608ea <probe::main+0x1caa>
   5faa8:	cmp    0xe0(%rsp),%r14
   5fab0:	jne    60910 <probe::main+0x1cd0>
   5fab6:	mov    %rcx,%rbp
   5fab9:	movq   $0x0,0xa8(%rsp)
   5fac5:	movq   $0x0,0xb8(%rsp)
   5fad1:	call   *0x1298e9(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fad7:	mov    $0x4,%r15d
   5fadd:	mov    $0x4,%edi
   5fae2:	mov    $0x1,%esi
   5fae7:	call   *0x1298db(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5faed:	test   %rax,%rax
   5faf0:	je     6099e <probe::main+0x1d5e>
   5faf6:	mov    %rax,%r12
   5faf9:	movl   $0x65736163,(%rax)
   5faff:	movq   $0x4,0x58(%rsp)
   5fb08:	mov    %rax,0x60(%rsp)
   5fb0d:	movq   $0x4,0x68(%rsp)
   5fb16:	mov    0x190(%rsp),%rax
   5fb1e:	mov    0x10(%rax),%r15
   5fb22:	test   %r15,%r15
   5fb25:	jns    5fb37 <probe::main+0xef7>
   5fb27:	xor    %edi,%edi
   5fb29:	mov    %r15,%rsi
   5fb2c:	call   *0x1298f6(%rip)        # 189428 <_DYNAMIC+0x2a8>
   5fb32:	jmp    60a1a <probe::main+0x1dda>
   5fb37:	je     5fb7c <probe::main+0xf3c>
   5fb39:	mov    0x8(%rax),%rax
   5fb3d:	mov    %rax,0xe8(%rsp)
   5fb45:	call   *0x129875(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fb4b:	mov    $0x1,%esi
   5fb50:	mov    %r15,%rdi
   5fb53:	call   *0x12986f(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fb59:	mov    $0x1,%edi
   5fb5e:	test   %rax,%rax
   5fb61:	je     5fb29 <probe::main+0xee9>
   5fb63:	mov    %rax,%r14
   5fb66:	mov    %rax,%rdi
   5fb69:	mov    0xe8(%rsp),%rsi
   5fb71:	mov    %r15,%rdx
   5fb74:	call   *0x129866(%rip)        # 1893e0 <memcpy@GLIBC_2.14>
   5fb7a:	jmp    5fb82 <probe::main+0xf42>
   5fb7c:	mov    $0x1,%r14d
   5fb82:	movb   $0x3,(%rsp)
   5fb86:	mov    %r15,0x8(%rsp)
   5fb8b:	mov    %r14,0x10(%rsp)
   5fb90:	mov    %r15,0x18(%rsp)
   5fb95:	lea    0x30(%rsp),%rdi
   5fb9a:	lea    0xa8(%rsp),%rsi
   5fba2:	lea    0x58(%rsp),%rdx
   5fba7:	mov    %rsp,%rcx
   5fbaa:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fbaf:	cmpb   $0xff,0x30(%rsp)
   5fbb4:	je     5fbc0 <probe::main+0xf80>
   5fbb6:	lea    0x30(%rsp),%rdi
   5fbbb:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fbc0:	call   *0x1297fa(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fbc6:	mov    $0xa,%r15d
   5fbcc:	mov    $0xa,%edi
   5fbd1:	mov    $0x1,%esi
   5fbd6:	call   *0x1297ec(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fbdc:	test   %rax,%rax
   5fbdf:	je     6099e <probe::main+0x1d5e>
   5fbe5:	movabs $0x6f69746172657469,%rcx
   5fbef:	mov    %rcx,(%rax)
   5fbf2:	movw   $0x736e,0x8(%rax)
   5fbf8:	movq   $0xa,0x58(%rsp)
   5fc01:	mov    %rax,0x60(%rsp)
   5fc06:	movq   $0xa,0x68(%rsp)
   5fc0f:	movb   $0x2,(%rsp)
   5fc13:	movq   $0x0,0x8(%rsp)
   5fc1c:	mov    0xc0(%rsp),%rax
   5fc24:	mov    %rax,0x10(%rsp)
   5fc29:	lea    0x30(%rsp),%rdi
   5fc2e:	lea    0xa8(%rsp),%rsi
   5fc36:	lea    0x58(%rsp),%rdx
   5fc3b:	mov    %rsp,%rcx
   5fc3e:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fc43:	cmpb   $0xff,0x30(%rsp)
   5fc48:	je     5fc54 <probe::main+0x1014>
   5fc4a:	lea    0x30(%rsp),%rdi
   5fc4f:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fc54:	call   *0x129766(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fc5a:	mov    $0x4,%r15d
   5fc60:	mov    $0x4,%edi
   5fc65:	mov    $0x1,%esi
   5fc6a:	call   *0x129758(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fc70:	test   %rax,%rax
   5fc73:	je     6099e <probe::main+0x1d5e>
   5fc79:	movl   $0x73746962,(%rax)
   5fc7f:	movq   $0x4,0x58(%rsp)
   5fc88:	mov    %rax,0x60(%rsp)
   5fc8d:	movq   $0x4,0x68(%rsp)
   5fc96:	movb   $0x2,(%rsp)
   5fc9a:	movq   $0x0,0x8(%rsp)
   5fca3:	mov    0x70(%rsp),%rax
   5fca8:	mov    %rax,0x10(%rsp)
   5fcad:	lea    0x30(%rsp),%rdi
   5fcb2:	lea    0xa8(%rsp),%rsi
   5fcba:	lea    0x58(%rsp),%rdx
   5fcbf:	mov    %rsp,%rcx
   5fcc2:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fcc7:	cmpb   $0xff,0x30(%rsp)
   5fccc:	je     5fcd8 <probe::main+0x1098>
   5fcce:	lea    0x30(%rsp),%rdi
   5fcd3:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fcd8:	call   *0x1296e2(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fcde:	mov    $0x9,%r15d
   5fce4:	mov    $0x9,%edi
   5fce9:	mov    $0x1,%esi
   5fcee:	call   *0x1296d4(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fcf4:	test   %rax,%rax
   5fcf7:	je     6099e <probe::main+0x1d5e>
   5fcfd:	movabs $0x6f69736963657270,%rcx
   5fd07:	mov    %rcx,(%rax)
   5fd0a:	movb   $0x6e,0x8(%rax)
   5fd0e:	movq   $0x9,0x58(%rsp)
   5fd17:	mov    %rax,0x60(%rsp)
   5fd1c:	movq   $0x9,0x68(%rsp)
   5fd25:	movslq 0xa0(%rsp),%rax
   5fd2d:	mov    %rax,%rcx
   5fd30:	shr    $0x3f,%rcx
   5fd34:	movb   $0x2,(%rsp)
   5fd38:	mov    %rcx,0x8(%rsp)
   5fd3d:	mov    %rax,0x10(%rsp)
   5fd42:	lea    0x30(%rsp),%rdi
   5fd47:	lea    0xa8(%rsp),%rsi
   5fd4f:	lea    0x58(%rsp),%rdx
   5fd54:	mov    %rsp,%rcx
   5fd57:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fd5c:	cmpb   $0xff,0x30(%rsp)
   5fd61:	je     5fd6d <probe::main+0x112d>
   5fd63:	lea    0x30(%rsp),%rdi
   5fd68:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fd6d:	call   *0x12964d(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fd73:	mov    $0xa,%r15d
   5fd79:	mov    $0xa,%edi
   5fd7e:	mov    $0x1,%esi
   5fd83:	call   *0x12963f(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fd89:	test   %rax,%rax
   5fd8c:	je     6099e <probe::main+0x1d5e>
   5fd92:	movabs $0x636f727069636572,%rcx
   5fd9c:	mov    %rcx,(%rax)
   5fd9f:	movw   $0x6c61,0x8(%rax)
   5fda5:	movq   $0xa,0x58(%rsp)
   5fdae:	mov    %rax,0x60(%rsp)
   5fdb3:	movq   $0xa,0x68(%rsp)
   5fdbc:	movzbl 0x57(%rsp),%eax
   5fdc1:	movb   $0x1,(%rsp)
   5fdc5:	mov    %al,0x1(%rsp)
   5fdc9:	lea    0x30(%rsp),%rdi
   5fdce:	lea    0xa8(%rsp),%rsi
   5fdd6:	lea    0x58(%rsp),%rdx
   5fddb:	mov    %rsp,%rcx
   5fdde:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fde3:	cmpb   $0xff,0x30(%rsp)
   5fde8:	je     5fdf4 <probe::main+0x11b4>
   5fdea:	lea    0x30(%rsp),%rdi
   5fdef:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fdf4:	call   *0x1295c6(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fdfa:	mov    $0x5,%r15d
   5fe00:	mov    $0x5,%edi
   5fe05:	mov    $0x1,%esi
   5fe0a:	call   *0x1295b8(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fe10:	test   %rax,%rax
   5fe13:	je     6099e <probe::main+0x1d5e>
   5fe19:	movb   $0x65,0x4(%rax)
   5fe1d:	movl   $0x736e6564,(%rax)
   5fe23:	movq   $0x5,0x58(%rsp)
   5fe2c:	mov    %rax,0x60(%rsp)
   5fe31:	movq   $0x5,0x68(%rsp)
   5fe3a:	movb   $0x1,(%rsp)
   5fe3e:	mov    0x78(%rsp),%rax
   5fe43:	mov    %al,0x1(%rsp)
   5fe47:	lea    0x30(%rsp),%rdi
   5fe4c:	lea    0xa8(%rsp),%rsi
   5fe54:	lea    0x58(%rsp),%rdx
   5fe59:	mov    %rsp,%rcx
   5fe5c:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fe61:	cmpb   $0xff,0x30(%rsp)
   5fe66:	je     5fe72 <probe::main+0x1232>
   5fe68:	lea    0x30(%rsp),%rdi
   5fe6d:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5fe72:	call   *0x129548(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5fe78:	mov    $0xc,%r15d
   5fe7e:	mov    $0xc,%edi
   5fe83:	mov    $0x1,%esi
   5fe88:	call   *0x12953a(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5fe8e:	test   %rax,%rax
   5fe91:	je     6099e <probe::main+0x1d5e>
   5fe97:	movabs $0x725f646568636163,%rcx
   5fea1:	mov    %rcx,(%rax)
   5fea4:	movl   $0x73746f6f,0x8(%rax)
   5feab:	movq   $0xc,0x58(%rsp)
   5feb4:	mov    %rax,0x60(%rsp)
   5feb9:	movq   $0xc,0x68(%rsp)
   5fec2:	mov    0xf0(%rsp),%rax
   5feca:	movb   $0x2,(%rsp)
   5fece:	movq   $0x0,0x8(%rsp)
   5fed7:	mov    %rax,0x10(%rsp)
   5fedc:	lea    0x30(%rsp),%rdi
   5fee1:	lea    0xa8(%rsp),%rsi
   5fee9:	lea    0x58(%rsp),%rdx
   5feee:	mov    %rsp,%rcx
   5fef1:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   5fef6:	cmpb   $0xff,0x30(%rsp)
   5fefb:	je     5ff07 <probe::main+0x12c7>
   5fefd:	lea    0x30(%rsp),%rdi
   5ff02:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   5ff07:	call   *0x1294b3(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5ff0d:	mov    $0xa,%r15d
   5ff13:	mov    $0xa,%edi
   5ff18:	mov    $0x1,%esi
   5ff1d:	call   *0x1294a5(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5ff23:	test   %rax,%rax
   5ff26:	je     6099e <probe::main+0x1d5e>
   5ff2c:	mov    %rax,%r12
   5ff2f:	movabs $0x5f64657370616c65,%rax
   5ff39:	mov    %rax,(%r12)
   5ff3d:	movw   $0x736e,0x8(%r12)
   5ff45:	movq   $0xa,0x58(%rsp)
   5ff4e:	mov    %r12,0x60(%rsp)
   5ff53:	movq   $0xa,0x68(%rsp)
   5ff5c:	mov    %rsp,%rdx
   5ff5f:	mov    $0x27,%ecx
   5ff64:	mov    %rbp,%rdi
   5ff67:	mov    %rbx,%rsi
   5ff6a:	call   *0x129538(%rip)        # 1894a8 <_DYNAMIC+0x328>
   5ff70:	mov    %rdx,%r14
   5ff73:	test   %rdx,%rdx
   5ff76:	jns    5ff8c <probe::main+0x134c>
   5ff78:	xor    %r15d,%r15d
   5ff7b:	mov    %r15,%rdi
   5ff7e:	mov    %r14,%rsi
   5ff81:	call   *0x1294a1(%rip)        # 189428 <_DYNAMIC+0x2a8>
   5ff87:	jmp    60a1a <probe::main+0x1dda>
   5ff8c:	test   %r14,%r14
   5ff8f:	je     5fff6 <probe::main+0x13b6>
   5ff91:	mov    %rax,%rbx
   5ff94:	call   *0x129426(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5ff9a:	mov    $0x1,%r15d
   5ffa0:	mov    $0x1,%esi
   5ffa5:	mov    %r14,%rdi
   5ffa8:	call   *0x12941a(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5ffae:	test   %rax,%rax
   5ffb1:	je     5ff7b <probe::main+0x133b>
   5ffb3:	mov    %rax,%rbp
   5ffb6:	mov    %rax,%rdi
   5ffb9:	mov    %rbx,%rsi
   5ffbc:	mov    %r14,%rdx
   5ffbf:	call   *0x12941b(%rip)        # 1893e0 <memcpy@GLIBC_2.14>
   5ffc5:	call   *0x1293f5(%rip)        # 1893c0 <_DYNAMIC+0x240>
   5ffcb:	mov    $0x1,%esi
   5ffd0:	mov    %r14,%rdi
   5ffd3:	call   *0x1293ef(%rip)        # 1893c8 <_DYNAMIC+0x248>
   5ffd9:	test   %rax,%rax
   5ffdc:	je     60a0c <probe::main+0x1dcc>
   5ffe2:	mov    %rax,%rbx
   5ffe5:	mov    %rax,%rdi
   5ffe8:	mov    %rbp,%rsi
   5ffeb:	mov    %r14,%rdx
   5ffee:	call   *0x1293ec(%rip)        # 1893e0 <memcpy@GLIBC_2.14>
   5fff4:	jmp    60000 <probe::main+0x13c0>
   5fff6:	mov    $0x1,%ebp
   5fffb:	mov    $0x1,%ebx
   60000:	mov    0xe0(%rsp),%r12
   60008:	movb   $0x3,(%rsp)
   6000c:	mov    %r14,0x8(%rsp)
   60011:	mov    %rbx,0x10(%rsp)
   60016:	mov    %r14,0x18(%rsp)
   6001b:	lea    0x30(%rsp),%rdi
   60020:	lea    0xa8(%rsp),%rsi
   60028:	lea    0x58(%rsp),%rdx
   6002d:	mov    %rsp,%rcx
   60030:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   60035:	cmpb   $0xff,0x30(%rsp)
   6003a:	je     60046 <probe::main+0x1406>
   6003c:	lea    0x30(%rsp),%rdi
   60041:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   60046:	test   %r14,%r14
   60049:	je     6005c <probe::main+0x141c>
   6004b:	mov    $0x1,%edx
   60050:	mov    %rbp,%rdi
   60053:	mov    %r14,%rsi
   60056:	call   *0x1293ac(%rip)        # 189408 <_DYNAMIC+0x288>
   6005c:	call   *0x12935e(%rip)        # 1893c0 <_DYNAMIC+0x240>
   60062:	mov    $0x8,%r15d
   60068:	mov    $0x8,%edi
   6006d:	mov    $0x1,%esi
   60072:	call   *0x129350(%rip)        # 1893c8 <_DYNAMIC+0x248>
   60078:	test   %rax,%rax
   6007b:	je     6099e <probe::main+0x1d5e>
   60081:	movabs $0x6d75736b63656863,%rcx
   6008b:	mov    %rcx,(%rax)
   6008e:	movq   $0x8,0x58(%rsp)
   60097:	mov    %rax,0x60(%rsp)
   6009c:	movq   $0x8,0x68(%rsp)
   600a5:	movb   $0x2,(%rsp)
   600a9:	movq   $0x0,0x8(%rsp)
   600b2:	mov    %r13,0x10(%rsp)
   600b7:	lea    0x30(%rsp),%rdi
   600bc:	lea    0xa8(%rsp),%rsi
   600c4:	lea    0x58(%rsp),%rdx
   600c9:	mov    %rsp,%rcx
   600cc:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   600d1:	cmpb   $0xff,0x30(%rsp)
   600d6:	je     600e2 <probe::main+0x14a2>
   600d8:	lea    0x30(%rsp),%rdi
   600dd:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   600e2:	call   *0x1292d8(%rip)        # 1893c0 <_DYNAMIC+0x240>
   600e8:	mov    $0xc,%r15d
   600ee:	mov    $0xc,%edi
   600f3:	mov    $0x1,%esi
   600f8:	call   *0x1292ca(%rip)        # 1893c8 <_DYNAMIC+0x248>
   600fe:	test   %rax,%rax
   60101:	je     6099e <probe::main+0x1d5e>
   60107:	movabs $0x625f74757074756f,%rcx
   60111:	mov    %rcx,(%rax)
   60114:	movl   $0x73657479,0x8(%rax)
   6011b:	movq   $0xc,0x58(%rsp)
   60124:	mov    %rax,0x60(%rsp)
   60129:	movq   $0xc,0x68(%rsp)
   60132:	movb   $0x2,(%rsp)
   60136:	movq   $0x0,0x8(%rsp)
   6013f:	mov    %r12,0x10(%rsp)
   60144:	lea    0x30(%rsp),%rdi
   60149:	lea    0xa8(%rsp),%rsi
   60151:	lea    0x58(%rsp),%rdx
   60156:	mov    %rsp,%rcx
   60159:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   6015e:	cmpb   $0xff,0x30(%rsp)
   60163:	je     6016f <probe::main+0x152f>
   60165:	lea    0x30(%rsp),%rdi
   6016a:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   6016f:	call   *0x12924b(%rip)        # 1893c0 <_DYNAMIC+0x240>
   60175:	mov    $0xa,%r15d
   6017b:	mov    $0xa,%edi
   60180:	mov    $0x1,%esi
   60185:	call   *0x12923d(%rip)        # 1893c8 <_DYNAMIC+0x248>
   6018b:	test   %rax,%rax
   6018e:	je     6099e <probe::main+0x1d5e>
   60194:	mov    %rax,%r14
   60197:	movabs $0x636e657265666572,%rax
   601a1:	mov    %rax,(%r14)
   601a4:	movw   $0x7365,0x8(%r14)
   601ab:	movq   $0xa,0x58(%rsp)
   601b4:	mov    %r14,0x60(%rsp)
   601b9:	movq   $0xa,0x68(%rsp)
   601c2:	mov    %rsp,%rdi
   601c5:	lea    0xf8(%rsp),%rsi
   601cd:	call   61c20 <<serde_json::value::ser::Serializer as serde_core::ser::Serializer>::collect_seq::<&alloc::vec::Vec<alloc::string::String>>>
   601d2:	cmpb   $0xff,(%rsp)
   601d6:	je     60936 <probe::main+0x1cf6>
   601dc:	movdqu (%rsp),%xmm0
   601e1:	movdqu 0x10(%rsp),%xmm1
   601e7:	movdqa %xmm1,0x40(%rsp)
   601ed:	movdqa %xmm0,0x30(%rsp)
   601f3:	mov    %rsp,%rbx
   601f6:	lea    0xa8(%rsp),%rsi
   601fe:	lea    0x58(%rsp),%rdx
   60203:	lea    0x30(%rsp),%rcx
   60208:	mov    %rbx,%rdi
   6020b:	call   62370 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value>>::insert>
   60210:	cmpb   $0xff,(%rsp)
   60214:	je     6021e <probe::main+0x15de>
   60216:	mov    %rsp,%rdi
   60219:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   6021e:	movdqu 0xa8(%rsp),%xmm0
   60227:	movdqu %xmm0,0x8(%rsp)
   6022d:	mov    0xb8(%rsp),%rax
   60235:	mov    %rax,0x18(%rsp)
   6023a:	movb   $0x5,(%rsp)
   6023e:	mov    %rbx,0x30(%rsp)
   60243:	mov    0x129266(%rip),%rax        # 1894b0 <_DYNAMIC+0x330>
   6024a:	mov    %rax,0x38(%rsp)
   6024f:	lea    -0x3009b(%rip),%rdi        # 301bb <GCC_except_table0+0x34f>
   60256:	lea    0x30(%rsp),%rsi
   6025b:	call   *0x129257(%rip)        # 1894b8 <_DYNAMIC+0x338>
   60261:	mov    %rsp,%rdi
   60264:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   60269:	lea    0xc8(%rsp),%rdi
   60271:	call   5e860 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   60276:	mov    0xc8(%rsp),%rsi
   6027e:	test   %rsi,%rsi
   60281:	je     6029a <probe::main+0x165a>
   60283:	mov    0xd0(%rsp),%rdi
   6028b:	shl    $0x4,%rsi
   6028f:	mov    $0x8,%edx
   60294:	call   *0x12916e(%rip)        # 189408 <_DYNAMIC+0x288>
   6029a:	mov    0x108(%rsp),%rbx
   602a2:	test   %rbx,%rbx
   602a5:	je     602df <probe::main+0x169f>
   602a7:	mov    0x100(%rsp),%r14
   602af:	add    $0x8,%r14
   602b3:	mov    0x12914e(%rip),%r15        # 189408 <_DYNAMIC+0x288>
   602ba:	jmp    602c9 <probe::main+0x1689>
   602bc:	nopl   0x0(%rax)
   602c0:	add    $0x18,%r14
   602c4:	dec    %rbx
   602c7:	je     602df <probe::main+0x169f>
   602c9:	mov    -0x8(%r14),%rsi
   602cd:	test   %rsi,%rsi
   602d0:	je     602c0 <probe::main+0x1680>
   602d2:	mov    (%r14),%rdi
   602d5:	mov    $0x1,%edx
   602da:	call   *%r15
   602dd:	jmp    602c0 <probe::main+0x1680>
   602df:	mov    0xf8(%rsp),%rax
   602e7:	test   %rax,%rax
   602ea:	je     60307 <probe::main+0x16c7>
   602ec:	mov    0x100(%rsp),%rdi
   602f4:	shl    $0x3,%rax
   602f8:	lea    (%rax,%rax,2),%rsi
   602fc:	mov    $0x8,%edx
   60301:	call   *0x129101(%rip)        # 189408 <_DYNAMIC+0x288>
   60307:	mov    0x80(%rsp),%rsi
   6030f:	test   %rsi,%rsi
   60312:	je     6032b <probe::main+0x16eb>
   60314:	mov    0x88(%rsp),%rdi
   6031c:	shl    $0x3,%rsi
   60320:	mov    $0x8,%edx
   60325:	call   *0x1290dd(%rip)        # 189408 <_DYNAMIC+0x288>
   6032b:	mov    0x110(%rsp),%rsi
   60333:	test   %rsi,%rsi
   60336:	je     6034f <probe::main+0x170f>
   60338:	mov    0x118(%rsp),%rdi
   60340:	shl    $0x3,%rsi
   60344:	mov    $0x8,%edx
   60349:	call   *0x1290b9(%rip)        # 189408 <_DYNAMIC+0x288>
   6034f:	mov    0x140(%rsp),%rbx
   60357:	mov    0x148(%rsp),%r14
   6035f:	test   %r14,%r14
   60362:	je     6039f <probe::main+0x175f>
   60364:	lea    0x8(%rbx),%r15
   60368:	mov    0x129099(%rip),%r12        # 189408 <_DYNAMIC+0x288>
   6036f:	jmp    60389 <probe::main+0x1749>
   60371:	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   60380:	add    $0x18,%r15
   60384:	dec    %r14
   60387:	je     6039f <probe::main+0x175f>
   60389:	mov    -0x8(%r15),%rsi
   6038d:	test   %rsi,%rsi
   60390:	je     60380 <probe::main+0x1740>
   60392:	mov    (%r15),%rdi
   60395:	mov    $0x1,%edx
   6039a:	call   *%r12
   6039d:	jmp    60380 <probe::main+0x1740>
   6039f:	mov    0x138(%rsp),%rax
   603a7:	test   %rax,%rax
   603aa:	je     603c2 <probe::main+0x1782>
   603ac:	shl    $0x3,%rax
   603b0:	lea    (%rax,%rax,2),%rsi
   603b4:	mov    $0x8,%edx
   603b9:	mov    %rbx,%rdi
   603bc:	call   *0x129046(%rip)        # 189408 <_DYNAMIC+0x288>
   603c2:	add    $0x1a8,%rsp
   603c9:	pop    %rbx
   603ca:	pop    %r12
   603cc:	pop    %r13
   603ce:	pop    %r14
   603d0:	pop    %r15
   603d2:	pop    %rbp
   603d3:	ret
   603d4:	mov    %r15,(%rsp)
   603d8:	mov    %r14,0x8(%rsp)
   603dd:	mov    %rbp,0x10(%rsp)
   603e2:	incq   (%r14)
   603e5:	jne    60561 <probe::main+0x1921>
   603eb:	mov    $0x1,%r15d
   603f1:	cmp    $0x1,%rbp
   603f5:	je     6042d <probe::main+0x17ed>
   603f7:	shl    $0x3,%rbp
   603fb:	mov    $0x1,%al
   603fd:	mov    $0x8,%ecx
   60402:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   60410:	add    $0xff,%al
   60412:	adcq   $0x0,(%r14,%rcx,1)
   60417:	setb   %al
   6041a:	jae    60561 <probe::main+0x1921>
   60420:	add    $0x8,%rcx
   60424:	cmp    %rcx,%rbp
   60427:	jne    60410 <probe::main+0x17d0>
   60429:	movzbl %al,%r15d
   6042d:	mov    0x10(%rsp),%rbx
   60432:	cmp    (%rsp),%rbx
   60436:	jne    60441 <probe::main+0x1801>
   60438:	mov    %rsp,%rdi
   6043b:	call   *0x12903f(%rip)        # 189480 <_DYNAMIC+0x300>
   60441:	mov    0x8(%rsp),%rax
   60446:	mov    %r15,(%rax,%rbx,8)
   6044a:	inc    %rbx
   6044d:	jmp    60566 <probe::main+0x1926>
   60452:	movdqu -0x2d16a(%rip),%xmm0        # 332f0 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x190>
   6045a:	movdqa %xmm0,(%rsp)
   6045f:	movq   $0x0,0x10(%rsp)
   60468:	mov    %rsp,%rdi
   6046b:	call   *0x12900f(%rip)        # 189480 <_DYNAMIC+0x300>
   60471:	mov    0x8(%rsp),%rax
   60476:	movq   $0x1,(%rax)
   6047d:	mov    (%rsp),%r12
   60481:	mov    0x8(%rsp),%r13
   60486:	mov    $0x1,%al
   60488:	cmp    $0x1,%rbp
   6048c:	jne    6049c <probe::main+0x185c>
   6048e:	mov    0x0(%r13),%rax
   60492:	cmp    %rax,(%r14)
   60495:	je     604fe <probe::main+0x18be>
   60497:	seta   %al
   6049a:	sbb    $0x0,%al
   6049c:	test   %al,%al
   6049e:	je     604fe <probe::main+0x18be>
   604a0:	movzbl %al,%eax
   604a3:	cmp    $0x1,%eax
   604a6:	jne    6064b <probe::main+0x1a0b>
   604ac:	mov    %r15,(%rsp)
   604b0:	mov    %r14,0x8(%rsp)
   604b5:	mov    %rbp,0x10(%rsp)
   604ba:	mov    %rsp,%rdi
   604bd:	mov    $0x1,%esi
   604c2:	call   *0x128fc0(%rip)        # 189488 <_DYNAMIC+0x308>
   604c8:	mov    (%rsp),%rsi
   604cc:	mov    0x8(%rsp),%rdi
   604d1:	mov    0x10(%rsp),%rax
   604d6:	test   %rax,%rax
   604d9:	je     6077a <probe::main+0x1b3a>
   604df:	mov    %rsi,0x80(%rsp)
   604e7:	mov    %rdi,0x88(%rsp)
   604ef:	mov    %rax,0x90(%rsp)
   604f7:	xor    %ebx,%ebx
   604f9:	jmp    60854 <probe::main+0x1c14>
   604fe:	movups -0x2ced5(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   60505:	movaps %xmm0,0x90(%rsp)
   6050d:	movdqu -0x2cef5(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   60515:	movdqa %xmm0,0x80(%rsp)
   6051e:	test   %r12,%r12
   60521:	je     5f4e4 <probe::main+0x8a4>
   60527:	shl    $0x3,%r12
   6052b:	mov    $0x8,%edx
   60530:	mov    %r13,%rdi
   60533:	mov    %r12,%rsi
   60536:	call   *0x128ecc(%rip)        # 189408 <_DYNAMIC+0x288>
   6053c:	jmp    5f4e4 <probe::main+0x8a4>
   60541:	mov    $0x1,%dl
   60543:	mov    $0xffffff80,%ecx
   60548:	mov    $0x3,%esi
   6054d:	jmp    5f028 <probe::main+0x3e8>
   60552:	mov    $0xffffff80,%ecx
   60557:	mov    $0x3,%edx
   6055c:	jmp    5ef04 <probe::main+0x2c4>
   60561:	mov    0x10(%rsp),%rbx
   60566:	mov    (%rsp),%rsi
   6056a:	mov    0x8(%rsp),%rdi
   6056f:	test   %rbx,%rbx
   60572:	je     605d2 <probe::main+0x1992>
   60574:	mov    %rsi,0x80(%rsp)
   6057c:	mov    %rdi,0x88(%rsp)
   60584:	mov    %rbx,0x90(%rsp)
   6058c:	xor    %ebx,%ebx
   6058e:	mov    %bl,0x98(%rsp)
   60595:	jmp    5f4fe <probe::main+0x8be>
   6059a:	mov    0x10(%rsp),%rbx
   6059f:	mov    (%rsp),%rsi
   605a3:	mov    0x8(%rsp),%rdi
   605a8:	test   %rbx,%rbx
   605ab:	je     60614 <probe::main+0x19d4>
   605ad:	mov    %rsi,0x80(%rsp)
   605b5:	mov    %rdi,0x88(%rsp)
   605bd:	mov    %rbx,0x90(%rsp)
   605c5:	movb   $0x2,0x98(%rsp)
   605cd:	jmp    5f4fe <probe::main+0x8be>
   605d2:	movups -0x2cfa9(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   605d9:	movaps %xmm0,0x90(%rsp)
   605e1:	movdqu -0x2cfc9(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   605e9:	movdqa %xmm0,0x80(%rsp)
   605f2:	mov    $0x1,%bl
   605f4:	test   %rsi,%rsi
   605f7:	je     6058e <probe::main+0x194e>
   605f9:	shl    $0x3,%rsi
   605fd:	mov    $0x8,%edx
   60602:	call   *0x128e00(%rip)        # 189408 <_DYNAMIC+0x288>
   60608:	mov    %bl,0x98(%rsp)
   6060f:	jmp    5f4fe <probe::main+0x8be>
   60614:	movups -0x2cfeb(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   6061b:	movaps %xmm0,0x90(%rsp)
   60623:	movdqu -0x2d00b(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   6062b:	movdqa %xmm0,0x80(%rsp)
   60634:	test   %rsi,%rsi
   60637:	je     5f4fe <probe::main+0x8be>
   6063d:	shl    $0x3,%rsi
   60641:	mov    $0x8,%edx
   60646:	jmp    5f4f8 <probe::main+0x8b8>
   6064b:	mov    %r15,(%rsp)
   6064f:	mov    %r14,0x8(%rsp)
   60654:	mov    %rbp,0x10(%rsp)
   60659:	lea    0x30(%rsp),%rdi
   6065e:	mov    %rsp,%rsi
   60661:	call   60e50 <<u32 as core::ops::arith::Sub<num_bigint::biguint::BigUint>>::sub>
   60666:	cmpq   $0x0,0x40(%rsp)
   6066c:	je     607d4 <probe::main+0x1b94>
   60672:	mov    0x40(%rsp),%rax
   60677:	mov    %rax,0x90(%rsp)
   6067f:	movdqu 0x30(%rsp),%xmm0
   60685:	movdqa %xmm0,0x80(%rsp)
   6068e:	movb   $0x2,0x98(%rsp)
   60696:	jmp    6085b <probe::main+0x1c1b>
   6069b:	mov    %r15,(%rsp)
   6069f:	mov    %r14,0x8(%rsp)
   606a4:	mov    %rbp,0x10(%rsp)
   606a9:	lea    0x30(%rsp),%rdi
   606ae:	mov    %rsp,%rsi
   606b1:	call   60e50 <<u32 as core::ops::arith::Sub<num_bigint::biguint::BigUint>>::sub>
   606b6:	cmpq   $0x0,0x40(%rsp)
   606bc:	je     60814 <probe::main+0x1bd4>
   606c2:	mov    0x40(%rsp),%rax
   606c7:	mov    %rax,0x90(%rsp)
   606cf:	movdqu 0x30(%rsp),%xmm0
   606d5:	movdqa %xmm0,0x80(%rsp)
   606de:	xor    %ebx,%ebx
   606e0:	jmp    60854 <probe::main+0x1c14>
   606e5:	movzbl 0x8(%rsp),%eax
   606ea:	mov    %al,0x58(%rsp)
   606ee:	lea    -0x2d459(%rip),%rdi        # 3329c <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x13c>
   606f5:	lea    0x124484(%rip),%rcx        # 184b80 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x1c8>
   606fc:	lea    0x12444d(%rip),%r8        # 184b50 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x198>
   60703:	lea    0x58(%rsp),%rdx
   60708:	mov    $0x2b,%esi
   6070d:	call   *0x128dad(%rip)        # 1894c0 <_DYNAMIC+0x340>
   60713:	jmp    60a1a <probe::main+0x1dda>
   60718:	lea    -0x2d09f(%rip),%rdi        # 33680 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x60>
   6071f:	lea    0x1244ba(%rip),%rcx        # 184be0 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x228>
   60726:	lea    0x12451b(%rip),%r8        # 184c48 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x290>
   6072d:	lea    0xa7(%rsp),%rdx
   60735:	mov    $0x37,%esi
   6073a:	call   *0x128d80(%rip)        # 1894c0 <_DYNAMIC+0x340>
   60740:	jmp    60a1a <probe::main+0x1dda>
   60745:	add    $0xd0,%dl
   60748:	xor    %eax,%eax
   6074a:	cmp    $0xa,%dl
   6074d:	adc    $0x1,%al
   6074f:	mov    %al,(%rsp)
   60752:	lea    -0x2d4bd(%rip),%rdi        # 3329c <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x13c>
   60759:	lea    0x124460(%rip),%rcx        # 184bc0 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x208>
   60760:	lea    0x124311(%rip),%r8        # 184a78 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0xc0>
   60767:	mov    %rsp,%rdx
   6076a:	mov    $0x2b,%esi
   6076f:	call   *0x128d4b(%rip)        # 1894c0 <_DYNAMIC+0x340>
   60775:	jmp    60a1a <probe::main+0x1dda>
   6077a:	movups -0x2d151(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   60781:	movaps %xmm0,0x90(%rsp)
   60789:	movdqu -0x2d171(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   60791:	movdqa %xmm0,0x80(%rsp)
   6079a:	mov    $0x1,%bl
   6079c:	test   %rsi,%rsi
   6079f:	jne    60845 <probe::main+0x1c05>
   607a5:	jmp    60854 <probe::main+0x1c14>
   607aa:	movups -0x2d181(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   607b1:	movaps %xmm0,0x90(%rsp)
   607b9:	movdqu -0x2d1a1(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   607c1:	movdqa %xmm0,0x80(%rsp)
   607ca:	test   %rsi,%rsi
   607cd:	jne    60803 <probe::main+0x1bc3>
   607cf:	jmp    6085b <probe::main+0x1c1b>
   607d4:	movups -0x2d1ab(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   607db:	movaps %xmm0,0x90(%rsp)
   607e3:	movdqu -0x2d1cb(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   607eb:	movdqa %xmm0,0x80(%rsp)
   607f4:	mov    0x30(%rsp),%rsi
   607f9:	test   %rsi,%rsi
   607fc:	je     6085b <probe::main+0x1c1b>
   607fe:	mov    0x38(%rsp),%rdi
   60803:	shl    $0x3,%rsi
   60807:	mov    $0x8,%edx
   6080c:	call   *0x128bf6(%rip)        # 189408 <_DYNAMIC+0x288>
   60812:	jmp    6085b <probe::main+0x1c1b>
   60814:	movups -0x2d1eb(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   6081b:	movaps %xmm0,0x90(%rsp)
   60823:	movdqu -0x2d20b(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   6082b:	movdqa %xmm0,0x80(%rsp)
   60834:	mov    0x30(%rsp),%rsi
   60839:	mov    $0x1,%bl
   6083b:	test   %rsi,%rsi
   6083e:	je     60854 <probe::main+0x1c14>
   60840:	mov    0x38(%rsp),%rdi
   60845:	shl    $0x3,%rsi
   60849:	mov    $0x8,%edx
   6084e:	call   *0x128bb4(%rip)        # 189408 <_DYNAMIC+0x288>
   60854:	mov    %bl,0x98(%rsp)
   6085b:	test   %r12,%r12
   6085e:	je     5f4fe <probe::main+0x8be>
   60864:	shl    $0x3,%r12
   60868:	mov    $0x8,%edx
   6086d:	mov    %r13,%rdi
   60870:	mov    %r12,%rsi
   60873:	jmp    5f4f8 <probe::main+0x8b8>
   60878:	movzbl 0x8(%rsp),%eax
   6087d:	mov    %al,0x170(%rsp)
   60884:	lea    -0x2d5ef(%rip),%rdi        # 3329c <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x13c>
   6088b:	lea    0x1242ee(%rip),%rcx        # 184b80 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x1c8>
   60892:	lea    0x12420f(%rip),%r8        # 184aa8 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0xf0>
   60899:	lea    0x170(%rsp),%rdx
   608a1:	mov    $0x2b,%esi
   608a6:	call   *0x128c14(%rip)        # 1894c0 <_DYNAMIC+0x340>
   608ac:	jmp    60a1a <probe::main+0x1dda>
   608b1:	movzbl 0x8(%rsp),%eax
   608b6:	mov    %al,0x170(%rsp)
   608bd:	lea    -0x2d628(%rip),%rdi        # 3329c <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x13c>
   608c4:	lea    0x1242b5(%rip),%rcx        # 184b80 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x1c8>
   608cb:	lea    0x1241ee(%rip),%r8        # 184ac0 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x108>
   608d2:	lea    0x170(%rsp),%rdx
   608da:	mov    $0x2b,%esi
   608df:	call   *0x128bdb(%rip)        # 1894c0 <_DYNAMIC+0x340>
   608e5:	jmp    60a1a <probe::main+0x1dda>
   608ea:	lea    0x1241ff(%rip),%r9        # 184af0 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x138>
   608f1:	lea    0x160(%rsp),%rsi
   608f9:	lea    0x150(%rsp),%rdx
   60901:	xor    %edi,%edi
   60903:	xor    %ecx,%ecx
   60905:	call   *0x128bbd(%rip)        # 1894c8 <_DYNAMIC+0x348>
   6090b:	jmp    60a1a <probe::main+0x1dda>
   60910:	lea    0x1241f1(%rip),%r9        # 184b08 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x150>
   60917:	lea    0x168(%rsp),%rsi
   6091f:	lea    0x158(%rsp),%rdx
   60927:	xor    %edi,%edi
   60929:	xor    %ecx,%ecx
   6092b:	call   *0x128b9f(%rip)        # 1894d0 <_DYNAMIC+0x350>
   60931:	jmp    60a1a <probe::main+0x1dda>
   60936:	mov    0x8(%rsp),%rax
   6093b:	mov    %rax,0x30(%rsp)
   60940:	lea    -0x2d6ab(%rip),%rdi        # 3329c <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x13c>
   60947:	lea    0x124252(%rip),%rcx        # 184ba0 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x1e8>
   6094e:	lea    0x1241cb(%rip),%r8        # 184b20 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x168>
   60955:	lea    0x30(%rsp),%rdx
   6095a:	mov    $0x2b,%esi
   6095f:	call   *0x128b5b(%rip)        # 1894c0 <_DYNAMIC+0x340>
   60965:	jmp    60a1a <probe::main+0x1dda>
   6096a:	lea    -0x2d711(%rip),%rdi        # 33260 <anon.9023bb4654a08944d5043612f590d0ff.345.llvm.2438011856832709387+0x100>
   60971:	lea    0x124118(%rip),%rdx        # 184a90 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0xd8>
   60978:	mov    $0x21,%esi
   6097d:	call   *0x128b55(%rip)        # 1894d8 <_DYNAMIC+0x358>
   60983:	jmp    60a1a <probe::main+0x1dda>
   60988:	mov    $0x1,%al
   6098a:	jmp    6074f <probe::main+0x1b0f>
   6098f:	lea    0x1241d2(%rip),%rdx        # 184b68 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x1b0>
   60996:	call   *0x128b44(%rip)        # 1894e0 <_DYNAMIC+0x360>
   6099c:	jmp    60a1a <probe::main+0x1dda>
   6099e:	mov    $0x1,%edi
   609a3:	mov    %r15,%rsi
   609a6:	call   *0x128a7c(%rip)        # 189428 <_DYNAMIC+0x2a8>
   609ac:	jmp    60a1a <probe::main+0x1dda>
   609ae:	lea    0x124093(%rip),%rdx        # 184a48 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x90>
   609b5:	jmp    609be <probe::main+0x1d7e>
   609b7:	lea    0x1240a2(%rip),%rdx        # 184a60 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0xa8>
   609be:	mov    %rdi,%rsi
   609c1:	call   *0x128b19(%rip)        # 1894e0 <_DYNAMIC+0x360>
   609c7:	jmp    60a1a <probe::main+0x1dda>
   609c9:	movq   $0x0,0x30(%rsp)
   609d2:	mov    0x70(%rsp),%rax
   609d7:	mov    %rax,(%rsp)
   609db:	lea    0x1240f6(%rip),%r9        # 184ad8 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x120>
   609e2:	lea    0x30(%rsp),%rsi
   609e7:	mov    %rsp,%rdx
   609ea:	xor    %edi,%edi
   609ec:	xor    %ecx,%ecx
   609ee:	call   *0x128ad4(%rip)        # 1894c8 <_DYNAMIC+0x348>
   609f4:	jmp    60a1a <probe::main+0x1dda>
   609f6:	lea    0x12413b(%rip),%rdi        # 184b38 <anon.650effda27756233549fe4650fad983e.5.llvm.3527033185240374034+0x180>
   609fd:	call   *0x128ae5(%rip)        # 1894e8 <_DYNAMIC+0x368>
   60a03:	jmp    60a1a <probe::main+0x1dda>
   60a05:	xor    %eax,%eax
   60a07:	jmp    6074f <probe::main+0x1b0f>
   60a0c:	mov    $0x1,%edi
   60a11:	mov    %r14,%rsi
   60a14:	call   *0x128a0e(%rip)        # 189428 <_DYNAMIC+0x2a8>
   60a1a:	ud2
   60a1c:	jmp    60a1e <probe::main+0x1dde>
   60a1e:	mov    %rax,%rbx
   60a21:	mov    (%rsp),%rsi
   60a25:	test   %rsi,%rsi
   60a28:	je     60d06 <probe::main+0x20c6>
   60a2e:	mov    0x8(%rsp),%rdi
   60a33:	jmp    60cf7 <probe::main+0x20b7>
   60a38:	jmp    60a3a <probe::main+0x1dfa>
   60a3a:	mov    %rax,%rbx
   60a3d:	jmp    60a61 <probe::main+0x1e21>
   60a3f:	jmp    60a41 <probe::main+0x1e01>
   60a41:	mov    %rax,%rbx
   60a44:	mov    (%rsp),%rsi
   60a48:	test   %rsi,%rsi
   60a4b:	je     60a61 <probe::main+0x1e21>
   60a4d:	mov    0x8(%rsp),%rdi
   60a52:	shl    $0x3,%rsi
   60a56:	mov    $0x8,%edx
   60a5b:	call   *0x1289a7(%rip)        # 189408 <_DYNAMIC+0x288>
   60a61:	test   %r12,%r12
   60a64:	je     60d06 <probe::main+0x20c6>
   60a6a:	shl    $0x3,%r12
   60a6e:	mov    $0x8,%edx
   60a73:	mov    %r13,%rdi
   60a76:	mov    %r12,%rsi
   60a79:	jmp    60d00 <probe::main+0x20c0>
   60a7e:	jmp    60ae9 <probe::main+0x1ea9>
   60a80:	jmp    60ae9 <probe::main+0x1ea9>
   60a82:	mov    %rax,%rbx
   60a85:	mov    $0xa,%esi
   60a8a:	mov    $0x1,%edx
   60a8f:	mov    %r12,%rdi
   60a92:	call   *0x128970(%rip)        # 189408 <_DYNAMIC+0x288>
   60a98:	jmp    60ad7 <probe::main+0x1e97>
   60a9a:	jmp    60acb <probe::main+0x1e8b>
   60a9c:	mov    %rax,%rbx
   60a9f:	jmp    60c85 <probe::main+0x2045>
   60aa4:	mov    %rax,%rbx
   60aa7:	mov    %rsp,%rdi
   60aaa:	call   5ea70 <core::ptr::drop_glue::<serde_json::value::Value>>
   60aaf:	jmp    60c78 <probe::main+0x2038>
   60ab4:	jmp    60ba1 <probe::main+0x1f61>
   60ab9:	mov    %rax,%rbx
   60abc:	jmp    60bc5 <probe::main+0x1f85>
   60ac1:	jmp    60ba1 <probe::main+0x1f61>
   60ac6:	jmp    60ba1 <probe::main+0x1f61>
   60acb:	mov    %rax,%rbx
   60ace:	test   %r14,%r14
   60ad1:	je     60c6b <probe::main+0x202b>
   60ad7:	mov    $0x1,%edx
   60adc:	mov    %rbp,%rdi
   60adf:	mov    %r14,%rsi
   60ae2:	jmp    60c65 <probe::main+0x2025>
   60ae7:	jmp    60ae9 <probe::main+0x1ea9>
   60ae9:	mov    %rax,%rbx
   60aec:	mov    (%rsp),%rsi
   60af0:	test   %rsi,%rsi
   60af3:	je     60b08 <probe::main+0x1ec8>
   60af5:	mov    (%r12),%rdi
   60af9:	shl    $0x3,%rsi
   60afd:	mov    $0x8,%edx
   60b02:	call   *0x128900(%rip)        # 189408 <_DYNAMIC+0x288>
   60b08:	test   %r15,%r15
   60b0b:	je     60d06 <probe::main+0x20c6>
   60b11:	shl    $0x3,%r15
   60b15:	mov    $0x8,%edx
   60b1a:	mov    %r14,%rdi
   60b1d:	mov    %r15,%rsi
   60b20:	jmp    60d00 <probe::main+0x20c0>
   60b25:	jmp    60ba1 <probe::main+0x1f61>
   60b27:	jmp    60ba1 <probe::main+0x1f61>
   60b29:	jmp    60ba1 <probe::main+0x1f61>
   60b2b:	jmp    60ba1 <probe::main+0x1f61>
   60b2d:	jmp    60ba1 <probe::main+0x1f61>
   60b2f:	jmp    60ba1 <probe::main+0x1f61>
   60b31:	jmp    60ba1 <probe::main+0x1f61>
   60b33:	mov    %rax,%rbx
   60b36:	mov    0x38(%rsp),%rax
   60b3b:	test   %rax,%rax
   60b3e:	je     60d06 <probe::main+0x20c6>
   60b44:	lock decq (%rax)
   60b48:	jne    60d06 <probe::main+0x20c6>
   60b4e:	mov    %r14,%rdi
   60b51:	call   *0x1288e1(%rip)        # 189438 <_DYNAMIC+0x2b8>
   60b57:	jmp    60d06 <probe::main+0x20c6>
   60b5c:	call   *0x12889e(%rip)        # 189400 <_DYNAMIC+0x280>
   60b62:	mov    %rax,%rbx
   60b65:	mov    0x38(%rsp),%rax
   60b6a:	test   %rax,%rax
   60b6d:	je     60d2a <probe::main+0x20ea>
   60b73:	lock decq (%rax)
   60b77:	jne    60d2a <probe::main+0x20ea>
   60b7d:	mov    %r14,%rdi
   60b80:	call   *0x1288b2(%rip)        # 189438 <_DYNAMIC+0x2b8>
   60b86:	jmp    60d2a <probe::main+0x20ea>
   60b8b:	call   *0x12886f(%rip)        # 189400 <_DYNAMIC+0x280>
   60b91:	mov    %rax,%rbx
   60b94:	jmp    60d06 <probe::main+0x20c6>
   60b99:	mov    %rax,%rbx
   60b9c:	jmp    60ce2 <probe::main+0x20a2>
   60ba1:	mov    %rax,%rbx
   60ba4:	jmp    60c6b <probe::main+0x202b>
   60ba9:	jmp    60c01 <probe::main+0x1fc1>
   60bab:	jmp    60c3e <probe::main+0x1ffe>
   60bb0:	mov    %rax,%rbx
   60bb3:	jmp    60ca9 <probe::main+0x2069>
   60bb8:	mov    %rax,%rbx
   60bbb:	lea    0x30(%rsp),%rdi
   60bc0:	call   5e9d0 <core::ptr::drop_glue::<serde_json::error::Error>>
   60bc5:	mov    $0xa,%esi
   60bca:	mov    $0x1,%edx
   60bcf:	mov    %r14,%rdi
   60bd2:	jmp    60c65 <probe::main+0x2025>
   60bd7:	call   *0x128823(%rip)        # 189400 <_DYNAMIC+0x280>
   60bdd:	mov    %rax,%rbx
   60be0:	lea    0x30(%rsp),%rdi
   60be5:	call   5ebc0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   60bea:	jmp    60d2a <probe::main+0x20ea>
   60bef:	mov    %rax,%rbx
   60bf2:	lea    0x30(%rsp),%rdi
   60bf7:	call   5ebc0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   60bfc:	jmp    60d2a <probe::main+0x20ea>
   60c01:	mov    %rax,%rbx
   60c04:	mov    0x58(%rsp),%rsi
   60c09:	test   %rsi,%rsi
   60c0c:	je     60c1e <probe::main+0x1fde>
   60c0e:	mov    0x60(%rsp),%rdi
   60c13:	mov    $0x1,%edx
   60c18:	call   *0x1287ea(%rip)        # 189408 <_DYNAMIC+0x288>
   60c1e:	mov    0x30(%rsp),%rsi
   60c23:	test   %rsi,%rsi
   60c26:	je     60c78 <probe::main+0x2038>
   60c28:	mov    0x38(%rsp),%rdi
   60c2d:	shl    $0x3,%rsi
   60c31:	mov    $0x8,%edx
   60c36:	call   *0x1287cc(%rip)        # 189408 <_DYNAMIC+0x288>
   60c3c:	jmp    60c78 <probe::main+0x2038>
   60c3e:	mov    %rax,%rbx
   60c41:	jmp    60c78 <probe::main+0x2038>
   60c43:	mov    %rax,%rbx
   60c46:	jmp    60d2a <probe::main+0x20ea>
   60c4b:	mov    %rax,%rbx
   60c4e:	mov    $0xa,%esi
   60c53:	jmp    60c5d <probe::main+0x201d>
   60c55:	mov    %rax,%rbx
   60c58:	mov    $0x4,%esi
   60c5d:	mov    $0x1,%edx
   60c62:	mov    %r12,%rdi
   60c65:	call   *0x12879d(%rip)        # 189408 <_DYNAMIC+0x288>
   60c6b:	lea    0xa8(%rsp),%rdi
   60c73:	call   62a50 <<alloc::collections::btree::map::BTreeMap<alloc::string::String, serde_json::value::Value> as core::ops::drop::Drop>::drop>
   60c78:	lea    0xc8(%rsp),%rdi
   60c80:	call   5e860 <<alloc::vec::Vec<hyperreal::computable::node::Computable> as core::ops::drop::Drop>::drop>
   60c85:	mov    0xc8(%rsp),%rsi
   60c8d:	test   %rsi,%rsi
   60c90:	je     60ca9 <probe::main+0x2069>
   60c92:	mov    0xd0(%rsp),%rdi
   60c9a:	shl    $0x4,%rsi
   60c9e:	mov    $0x8,%edx
   60ca3:	call   *0x12875f(%rip)        # 189408 <_DYNAMIC+0x288>
   60ca9:	mov    0x108(%rsp),%r14
   60cb1:	test   %r14,%r14
   60cb4:	jne    60da1 <probe::main+0x2161>
   60cba:	mov    0xf8(%rsp),%rax
   60cc2:	test   %rax,%rax
   60cc5:	je     60ce2 <probe::main+0x20a2>
   60cc7:	mov    0x100(%rsp),%rdi
   60ccf:	shl    $0x3,%rax
   60cd3:	lea    (%rax,%rax,2),%rsi
   60cd7:	mov    $0x8,%edx
   60cdc:	call   *0x128726(%rip)        # 189408 <_DYNAMIC+0x288>
   60ce2:	mov    0x80(%rsp),%rsi
   60cea:	test   %rsi,%rsi
   60ced:	je     60d06 <probe::main+0x20c6>
   60cef:	mov    0x88(%rsp),%rdi
   60cf7:	shl    $0x3,%rsi
   60cfb:	mov    $0x8,%edx
   60d00:	call   *0x128702(%rip)        # 189408 <_DYNAMIC+0x288>
   60d06:	mov    0x110(%rsp),%rsi
   60d0e:	test   %rsi,%rsi
   60d11:	je     60d2a <probe::main+0x20ea>
   60d13:	mov    0x118(%rsp),%rdi
   60d1b:	shl    $0x3,%rsi
   60d1f:	mov    $0x8,%edx
   60d24:	call   *0x1286de(%rip)        # 189408 <_DYNAMIC+0x288>
   60d2a:	mov    0x140(%rsp),%r14
   60d32:	mov    0x148(%rsp),%r15
   60d3a:	test   %r15,%r15
   60d3d:	jne    60d6a <probe::main+0x212a>
   60d3f:	mov    0x138(%rsp),%rax
   60d47:	test   %rax,%rax
   60d4a:	je     60d62 <probe::main+0x2122>
   60d4c:	shl    $0x3,%rax
   60d50:	lea    (%rax,%rax,2),%rsi
   60d54:	mov    $0x8,%edx
   60d59:	mov    %r14,%rdi
   60d5c:	call   *0x1286a6(%rip)        # 189408 <_DYNAMIC+0x288>
   60d62:	mov    %rbx,%rdi
   60d65:	call   183780 <_Unwind_Resume@plt>
   60d6a:	lea    0x8(%r14),%r12
   60d6e:	mov    0x128693(%rip),%r13        # 189408 <_DYNAMIC+0x288>
   60d75:	jmp    60d89 <probe::main+0x2149>
   60d77:	nopw   0x0(%rax,%rax,1)
   60d80:	add    $0x18,%r12
   60d84:	dec    %r15
   60d87:	je     60d3f <probe::main+0x20ff>
   60d89:	mov    -0x8(%r12),%rsi
   60d8e:	test   %rsi,%rsi
   60d91:	je     60d80 <probe::main+0x2140>
   60d93:	mov    (%r12),%rdi
   60d97:	mov    $0x1,%edx
   60d9c:	call   *%r13
   60d9f:	jmp    60d80 <probe::main+0x2140>
   60da1:	mov    0x100(%rsp),%r15
   60da9:	add    $0x8,%r15
   60dad:	mov    0x128654(%rip),%r12        # 189408 <_DYNAMIC+0x288>
   60db4:	jmp    60dcd <probe::main+0x218d>
   60db6:	cs nopw 0x0(%rax,%rax,1)
   60dc0:	add    $0x18,%r15
   60dc4:	dec    %r14
   60dc7:	je     60cba <probe::main+0x207a>
   60dcd:	mov    -0x8(%r15),%rsi
   60dd1:	test   %rsi,%rsi
   60dd4:	je     60dc0 <probe::main+0x2180>
   60dd6:	mov    (%r15),%rdi
   60dd9:	mov    $0x1,%edx
   60dde:	call   *%r12
   60de1:	jmp    60dc0 <probe::main+0x2180>
   60de3:	mov    0xc8(%rsp),%rsi
   60deb:	test   %rsi,%rsi
   60dee:	je     60e0d <probe::main+0x21cd>
   60df0:	mov    0xd0(%rsp),%rdi
   60df8:	shl    $0x4,%rsi
   60dfc:	mov    $0x8,%edx
   60e01:	call   *0x128601(%rip)        # 189408 <_DYNAMIC+0x288>
   60e07:	call   *0x1285f3(%rip)        # 189400 <_DYNAMIC+0x280>
   60e0d:	call   *0x1285ed(%rip)        # 189400 <_DYNAMIC+0x280>
