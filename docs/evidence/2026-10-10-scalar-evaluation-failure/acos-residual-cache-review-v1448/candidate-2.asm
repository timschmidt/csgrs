
target/typed-api-prototype-v1437-acos-residual-targeted-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bae60 <<hyperreal::computable::node::EvaluationCache>::at_precision>:
   bae60:	push   %rbp
   bae61:	push   %r15
   bae63:	push   %r14
   bae65:	push   %r13
   bae67:	push   %r12
   bae69:	push   %rbx
   bae6a:	sub    $0x68,%rsp
   bae6e:	mov    %rdi,%r14
   bae71:	mov    (%rsi),%rbx
   bae74:	test   %rbx,%rbx
   bae77:	je     baeb7 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x57>
   bae79:	mov    (%rbx),%eax
   bae7b:	cmp    $0x3ffffffd,%eax
   bae80:	ja     bb1b5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bae86:	lea    0x1(%rax),%ecx
   bae89:	lock cmpxchg %ecx,(%rbx)
   bae8d:	jne    bb1b5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bae93:	movzbl 0x8(%rbx),%eax
   bae97:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bae9c:	je     bb1d3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x373>
   baea2:	lea    0x10(%rbx),%r15
   baea6:	sub    0x30(%rbx),%edx
   baea9:	jge    baec3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x63>
   baeab:	movq   $0xffffffffffffffff,(%r14)
   baeb2:	jmp    bb194 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   baeb7:	movq   $0xffffffffffffffff,(%r14)
   baebe:	jmp    bb1f0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   baec3:	jne    baf36 <<hyperreal::computable::node::EvaluationCache>::at_precision+0xd6>
   baec5:	mov    0x20(%rbx),%rbp
   baec9:	lea    0x0(,%rbp,8),%r15
   baed1:	mov    %rbp,%rax
   baed4:	shr    $0x3d,%rax
   baed8:	setne  %al
   baedb:	movabs $0x7ffffffffffffff8,%rcx
   baee5:	cmp    %rcx,%r15
   baee8:	seta   %cl
   baeeb:	or     %al,%cl
   baeed:	jne    baf65 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x105>
   baeef:	movzbl 0x28(%rbx),%eax
   baef3:	mov    %al,0xc(%rsp)
   baef7:	mov    0x18(%rbx),%rsi
   baefb:	test   %r15,%r15
   baefe:	je     baff0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x190>
   baf04:	mov    %rsi,0x58(%rsp)
   baf09:	call   *0xcbb71(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   baf0f:	mov    $0x8,%esi
   baf14:	mov    %r15,%rdi
   baf17:	call   *0xcbb6b(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   baf1d:	test   %rax,%rax
   baf20:	je     bb25c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   baf26:	mov    %rax,%r13
   baf29:	mov    %rbp,%r12
   baf2c:	mov    0x58(%rsp),%rsi
   baf31:	jmp    baff9 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x199>
   baf36:	cmp    $0x1,%edx
   baf39:	jne    baf72 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x112>
   baf3b:	mov    0x20(%rbx),%r12
   baf3f:	lea    0x0(,%r12,8),%r15
   baf47:	mov    %r12,%rax
   baf4a:	shr    $0x3d,%rax
   baf4e:	setne  %al
   baf51:	movabs $0x7ffffffffffffff8,%rcx
   baf5b:	cmp    %rcx,%r15
   baf5e:	seta   %cl
   baf61:	or     %al,%cl
   baf63:	je     bafbd <<hyperreal::computable::node::EvaluationCache>::at_precision+0x15d>
   baf65:	xor    %edi,%edi
   baf67:	mov    %r15,%rsi
   baf6a:	call   *0xcbb68(%rip)        # 186ad8 <_DYNAMIC+0x308>
   baf70:	ud2
   baf72:	mov    0x20(%rbx),%rax
   baf76:	test   %rax,%rax
   baf79:	je     bafa0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x140>
   baf7b:	mov    0x18(%rbx),%rcx
   baf7f:	mov    %edx,%esi
   baf81:	mov    $0x7f,%edi
   baf86:	bsr    -0x8(%rcx,%rax,8),%rdi
   baf8c:	xor    $0x3f,%rdi
   baf90:	shl    $0x6,%rax
   baf94:	sub    %rdi,%rax
   baf97:	cmp    %rsi,%rax
   baf9a:	jae    bb023 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1c3>
   bafa0:	movups -0x864d7(%rip),%xmm0        # 34ad0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067+0x10>
   bafa7:	movaps %xmm0,0x20(%rsp)
   bafac:	movups -0x864f3(%rip),%xmm0        # 34ac0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067>
   bafb3:	movaps %xmm0,0x10(%rsp)
   bafb8:	jmp    bb181 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x321>
   bafbd:	movzbl 0x28(%rbx),%ebp
   bafc1:	mov    0x18(%rbx),%r13
   bafc5:	test   %r15,%r15
   bafc8:	je     bb068 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x208>
   bafce:	call   *0xcbaac(%rip)        # 186a80 <_DYNAMIC+0x2b0>
   bafd4:	mov    $0x8,%esi
   bafd9:	mov    %r15,%rdi
   bafdc:	call   *0xcbaa6(%rip)        # 186a88 <_DYNAMIC+0x2b8>
   bafe2:	test   %rax,%rax
   bafe5:	je     bb25c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   bafeb:	mov    %r12,%rsi
   bafee:	jmp    bb06f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x20f>
   baff0:	mov    $0x8,%r13d
   baff6:	xor    %r12d,%r12d
   baff9:	test   %rbp,%rbp
   baffc:	je     bb00a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1aa>
   baffe:	mov    %r13,%rdi
   bb001:	mov    %r15,%rdx
   bb004:	call   *0xcba86(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   bb00a:	mov    %r12,(%r14)
   bb00d:	mov    %r13,0x8(%r14)
   bb011:	mov    %rbp,0x10(%r14)
   bb015:	movzbl 0xc(%rsp),%eax
   bb01a:	mov    %al,0x18(%r14)
   bb01e:	jmp    bb194 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bb023:	dec    %edx
   bb025:	mov    %r15,%rdi
   bb028:	mov    %rdx,%r12
   bb02b:	mov    %edx,%esi
   bb02d:	call   125220 <num_bigint::bigint::shift::shr_round_down::<u32>>
   bb032:	mov    %eax,%ebp
   bb034:	cmpq   $0x0,0x20(%rbx)
   bb039:	je     bb0a7 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x247>
   bb03b:	mov    %r12,%rdx
   bb03e:	mov    %edx,%eax
   bb040:	and    $0x3f,%al
   bb042:	shr    $0x6,%edx
   bb045:	movq   $0xffffffffffffffff,0x10(%rsp)
   bb04e:	mov    %r15,0x18(%rsp)
   bb053:	movzbl %al,%ecx
   bb056:	lea    0x38(%rsp),%rdi
   bb05b:	lea    0x10(%rsp),%rsi
   bb060:	call   *0xcbbda(%rip)        # 186c40 <_DYNAMIC+0x470>
   bb066:	jmp    bb0b4 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x254>
   bb068:	mov    $0x8,%eax
   bb06d:	xor    %esi,%esi
   bb06f:	test   %r12,%r12
   bb072:	je     bb09f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x23f>
   bb074:	mov    %r13,%rcx
   bb077:	mov    %rax,%r13
   bb07a:	mov    %rax,%rdi
   bb07d:	mov    %ebp,0xc(%rsp)
   bb081:	mov    %rsi,%rbp
   bb084:	mov    %rcx,%rsi
   bb087:	mov    %r15,%rdx
   bb08a:	call   *0xcba00(%rip)        # 186a90 <memcpy@GLIBC_2.14>
   bb090:	mov    %r13,%rax
   bb093:	mov    %rbp,%rsi
   bb096:	mov    0xc(%rsp),%ebp
   bb09a:	jmp    bb13a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bb09f:	xor    %r12d,%r12d
   bb0a2:	jmp    bb13a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bb0a7:	lea    0x38(%rsp),%rdi
   bb0ac:	mov    %r15,%rsi
   bb0af:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   bb0b4:	test   %bpl,%bpl
   bb0b7:	je     bb0ca <<hyperreal::computable::node::EvaluationCache>::at_precision+0x26a>
   bb0b9:	lea    0x10(%rsp),%rdi
   bb0be:	lea    0x38(%rsp),%rsi
   bb0c3:	call   bf610 <<num_bigint::biguint::BigUint as core::ops::arith::Add<u32>>::add>
   bb0c8:	jmp    bb0de <<hyperreal::computable::node::EvaluationCache>::at_precision+0x27e>
   bb0ca:	mov    0x48(%rsp),%rax
   bb0cf:	mov    %rax,0x20(%rsp)
   bb0d4:	movups 0x38(%rsp),%xmm0
   bb0d9:	movaps %xmm0,0x10(%rsp)
   bb0de:	movzbl 0x28(%rbx),%eax
   bb0e2:	cmp    $0x1,%al
   bb0e4:	jne    bb0f9 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x299>
   bb0e6:	mov    0x10(%rsp),%rsi
   bb0eb:	mov    $0x1,%bpl
   bb0ee:	cmp    $0x4,%rsi
   bb0f2:	jae    bb113 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2b3>
   bb0f4:	xor    %r12d,%r12d
   bb0f7:	jmp    bb135 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bb0f9:	mov    0x10(%rsp),%rsi
   bb0fe:	mov    0x20(%rsp),%r12
   bb103:	test   %r12,%r12
   bb106:	movzbl %al,%eax
   bb109:	mov    $0x1,%ebp
   bb10e:	cmovne %eax,%ebp
   bb111:	jmp    bb135 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bb113:	mov    0x18(%rsp),%rdi
   bb118:	shl    $0x3,%rsi
   bb11c:	mov    $0x8,%edx
   bb121:	call   *0xcb8e9(%rip)        # 186a10 <_DYNAMIC+0x240>
   bb127:	movq   $0x8,0x18(%rsp)
   bb130:	xor    %r12d,%r12d
   bb133:	xor    %esi,%esi
   bb135:	mov    0x18(%rsp),%rax
   bb13a:	mov    %rsi,0x10(%rsp)
   bb13f:	mov    %rax,0x18(%rsp)
   bb144:	mov    %r12,0x20(%rsp)
   bb149:	mov    %bpl,0x28(%rsp)
   bb14e:	mov    0xfcef4(%rip),%ecx        # 1b8048 <hyperreal::computable::node::signed::ONE+0x20>
   bb154:	test   %ecx,%ecx
   bb156:	jne    bb216 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3b6>
   bb15c:	lea    0xfcec5(%rip),%rdx        # 1b8028 <hyperreal::computable::node::signed::ONE>
   bb163:	lea    0x38(%rsp),%rdi
   bb168:	lea    0x10(%rsp),%rsi
   bb16d:	call   bf2c0 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bb172:	lea    0x10(%rsp),%rdi
   bb177:	lea    0x38(%rsp),%rsi
   bb17c:	call   c07e0 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bb181:	movaps 0x10(%rsp),%xmm0
   bb186:	movaps 0x20(%rsp),%xmm1
   bb18b:	movups %xmm1,0x10(%r14)
   bb190:	movups %xmm0,(%r14)
   bb194:	mov    $0xffffffff,%esi
   bb199:	lock xadd %esi,(%rbx)
   bb19d:	dec    %esi
   bb19f:	mov    %esi,%eax
   bb1a1:	and    $0xbfffffff,%eax
   bb1a6:	neg    %eax
   bb1a8:	jno    bb1f0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bb1aa:	mov    %rbx,%rdi
   bb1ad:	call   *0xcbbc5(%rip)        # 186d78 <_DYNAMIC+0x5a8>
   bb1b3:	jmp    bb1f0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bb1b5:	mov    %rbx,%rdi
   bb1b8:	mov    %rdx,%r15
   bb1bb:	call   *0xcbbbf(%rip)        # 186d80 <_DYNAMIC+0x5b0>
   bb1c1:	mov    %r15,%rdx
   bb1c4:	movzbl 0x8(%rbx),%eax
   bb1c8:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bb1cd:	jne    baea2 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42>
   bb1d3:	movq   $0xffffffffffffffff,(%r14)
   bb1da:	mov    $0xffffffff,%esi
   bb1df:	lock xadd %esi,(%rbx)
   bb1e3:	dec    %esi
   bb1e5:	mov    %esi,%eax
   bb1e7:	and    $0xbfffffff,%eax
   bb1ec:	neg    %eax
   bb1ee:	jo     bb1ff <<hyperreal::computable::node::EvaluationCache>::at_precision+0x39f>
   bb1f0:	add    $0x68,%rsp
   bb1f4:	pop    %rbx
   bb1f5:	pop    %r12
   bb1f7:	pop    %r13
   bb1f9:	pop    %r14
   bb1fb:	pop    %r15
   bb1fd:	pop    %rbp
   bb1fe:	ret
   bb1ff:	mov    %rbx,%rdi
   bb202:	add    $0x68,%rsp
   bb206:	pop    %rbx
   bb207:	pop    %r12
   bb209:	pop    %r13
   bb20b:	pop    %r14
   bb20d:	pop    %r15
   bb20f:	pop    %rbp
   bb210:	jmp    *0xcbb62(%rip)        # 186d78 <_DYNAMIC+0x5a8>
   bb216:	mov    %rax,%r12
   bb219:	mov    %rsi,%r15
   bb21c:	lea    0xfce05(%rip),%rax        # 1b8028 <hyperreal::computable::node::signed::ONE>
   bb223:	mov    %rax,0x60(%rsp)
   bb228:	lea    0x60(%rsp),%rax
   bb22d:	mov    %rax,0x38(%rsp)
   bb232:	lea    0xfce0f(%rip),%rdi        # 1b8048 <hyperreal::computable::node::signed::ONE+0x20>
   bb239:	lea    0xc7268(%rip),%rcx        # 1824a8 <anon.7b75c566c6a805bace2b01974d435009.3.llvm.16730574089061123067>
   bb240:	lea    0xc7249(%rip),%r8        # 182490 <anon.7b75c566c6a805bace2b01974d435009.2.llvm.16730574089061123067>
   bb247:	lea    0x38(%rsp),%rdx
   bb24c:	mov    $0x1,%esi
   bb251:	call   *0xcb941(%rip)        # 186b98 <_DYNAMIC+0x3c8>
   bb257:	jmp    bb15c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2fc>
   bb25c:	mov    $0x8,%edi
   bb261:	jmp    baf67 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x107>
   bb266:	mov    %rax,%r14
   bb269:	test   %r15,%r15
   bb26c:	je     bb28a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bb26e:	mov    %r15,%rsi
   bb271:	shl    $0x3,%rsi
   bb275:	mov    $0x8,%edx
   bb27a:	mov    %r12,%rdi
   bb27d:	call   *0xcb78d(%rip)        # 186a10 <_DYNAMIC+0x240>
   bb283:	jmp    bb28a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bb285:	jmp    bb287 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x427>
   bb287:	mov    %rax,%r14
   bb28a:	mov    $0xffffffff,%esi
   bb28f:	lock xadd %esi,(%rbx)
   bb293:	dec    %esi
   bb295:	mov    %esi,%eax
   bb297:	and    $0xbfffffff,%eax
   bb29c:	neg    %eax
   bb29e:	jo     bb2a8 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x448>
   bb2a0:	mov    %r14,%rdi
   bb2a3:	call   180f40 <_Unwind_Resume@plt>
   bb2a8:	mov    %rbx,%rdi
   bb2ab:	call   *0xcbac7(%rip)        # 186d78 <_DYNAMIC+0x5a8>
   bb2b1:	jmp    bb2a0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x440>
   bb2b3:	call   *0xcb75f(%rip)        # 186a18 <_DYNAMIC+0x248>
