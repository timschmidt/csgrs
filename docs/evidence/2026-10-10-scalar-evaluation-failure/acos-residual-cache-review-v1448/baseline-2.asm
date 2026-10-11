
target/typed-api-prototype-v1418-acos-warm-baseline-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bade0 <<hyperreal::computable::node::EvaluationCache>::at_precision>:
   bade0:	push   %rbp
   bade1:	push   %r15
   bade3:	push   %r14
   bade5:	push   %r13
   bade7:	push   %r12
   bade9:	push   %rbx
   badea:	sub    $0x68,%rsp
   badee:	mov    %rdi,%r14
   badf1:	mov    (%rsi),%rbx
   badf4:	test   %rbx,%rbx
   badf7:	je     bae37 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x57>
   badf9:	mov    (%rbx),%eax
   badfb:	cmp    $0x3ffffffd,%eax
   bae00:	ja     bb135 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bae06:	lea    0x1(%rax),%ecx
   bae09:	lock cmpxchg %ecx,(%rbx)
   bae0d:	jne    bb135 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bae13:	movzbl 0x8(%rbx),%eax
   bae17:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bae1c:	je     bb153 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x373>
   bae22:	lea    0x10(%rbx),%r15
   bae26:	sub    0x30(%rbx),%edx
   bae29:	jge    bae43 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x63>
   bae2b:	movq   $0xffffffffffffffff,(%r14)
   bae32:	jmp    bb114 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bae37:	movq   $0xffffffffffffffff,(%r14)
   bae3e:	jmp    bb170 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bae43:	jne    baeb6 <<hyperreal::computable::node::EvaluationCache>::at_precision+0xd6>
   bae45:	mov    0x20(%rbx),%rbp
   bae49:	lea    0x0(,%rbp,8),%r15
   bae51:	mov    %rbp,%rax
   bae54:	shr    $0x3d,%rax
   bae58:	setne  %al
   bae5b:	movabs $0x7ffffffffffffff8,%rcx
   bae65:	cmp    %rcx,%r15
   bae68:	seta   %cl
   bae6b:	or     %al,%cl
   bae6d:	jne    baee5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x105>
   bae6f:	movzbl 0x28(%rbx),%eax
   bae73:	mov    %al,0xc(%rsp)
   bae77:	mov    0x18(%rbx),%rsi
   bae7b:	test   %r15,%r15
   bae7e:	je     baf70 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x190>
   bae84:	mov    %rsi,0x58(%rsp)
   bae89:	call   *0xcb7a9(%rip)        # 186638 <_DYNAMIC+0x2b0>
   bae8f:	mov    $0x8,%esi
   bae94:	mov    %r15,%rdi
   bae97:	call   *0xcb7a3(%rip)        # 186640 <_DYNAMIC+0x2b8>
   bae9d:	test   %rax,%rax
   baea0:	je     bb1dc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   baea6:	mov    %rax,%r13
   baea9:	mov    %rbp,%r12
   baeac:	mov    0x58(%rsp),%rsi
   baeb1:	jmp    baf79 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x199>
   baeb6:	cmp    $0x1,%edx
   baeb9:	jne    baef2 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x112>
   baebb:	mov    0x20(%rbx),%r12
   baebf:	lea    0x0(,%r12,8),%r15
   baec7:	mov    %r12,%rax
   baeca:	shr    $0x3d,%rax
   baece:	setne  %al
   baed1:	movabs $0x7ffffffffffffff8,%rcx
   baedb:	cmp    %rcx,%r15
   baede:	seta   %cl
   baee1:	or     %al,%cl
   baee3:	je     baf3d <<hyperreal::computable::node::EvaluationCache>::at_precision+0x15d>
   baee5:	xor    %edi,%edi
   baee7:	mov    %r15,%rsi
   baeea:	call   *0xcb7a0(%rip)        # 186690 <_DYNAMIC+0x308>
   baef0:	ud2
   baef2:	mov    0x20(%rbx),%rax
   baef6:	test   %rax,%rax
   baef9:	je     baf20 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x140>
   baefb:	mov    0x18(%rbx),%rcx
   baeff:	mov    %edx,%esi
   baf01:	mov    $0x7f,%edi
   baf06:	bsr    -0x8(%rcx,%rax,8),%rdi
   baf0c:	xor    $0x3f,%rdi
   baf10:	shl    $0x6,%rax
   baf14:	sub    %rdi,%rax
   baf17:	cmp    %rsi,%rax
   baf1a:	jae    bafa3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1c3>
   baf20:	movups -0x864a7(%rip),%xmm0        # 34a80 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061+0x10>
   baf27:	movaps %xmm0,0x20(%rsp)
   baf2c:	movups -0x864c3(%rip),%xmm0        # 34a70 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061>
   baf33:	movaps %xmm0,0x10(%rsp)
   baf38:	jmp    bb101 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x321>
   baf3d:	movzbl 0x28(%rbx),%ebp
   baf41:	mov    0x18(%rbx),%r13
   baf45:	test   %r15,%r15
   baf48:	je     bafe8 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x208>
   baf4e:	call   *0xcb6e4(%rip)        # 186638 <_DYNAMIC+0x2b0>
   baf54:	mov    $0x8,%esi
   baf59:	mov    %r15,%rdi
   baf5c:	call   *0xcb6de(%rip)        # 186640 <_DYNAMIC+0x2b8>
   baf62:	test   %rax,%rax
   baf65:	je     bb1dc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   baf6b:	mov    %r12,%rsi
   baf6e:	jmp    bafef <<hyperreal::computable::node::EvaluationCache>::at_precision+0x20f>
   baf70:	mov    $0x8,%r13d
   baf76:	xor    %r12d,%r12d
   baf79:	test   %rbp,%rbp
   baf7c:	je     baf8a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1aa>
   baf7e:	mov    %r13,%rdi
   baf81:	mov    %r15,%rdx
   baf84:	call   *0xcb6be(%rip)        # 186648 <memcpy@GLIBC_2.14>
   baf8a:	mov    %r12,(%r14)
   baf8d:	mov    %r13,0x8(%r14)
   baf91:	mov    %rbp,0x10(%r14)
   baf95:	movzbl 0xc(%rsp),%eax
   baf9a:	mov    %al,0x18(%r14)
   baf9e:	jmp    bb114 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bafa3:	dec    %edx
   bafa5:	mov    %r15,%rdi
   bafa8:	mov    %rdx,%r12
   bafab:	mov    %edx,%esi
   bafad:	call   124df0 <num_bigint::bigint::shift::shr_round_down::<u32>>
   bafb2:	mov    %eax,%ebp
   bafb4:	cmpq   $0x0,0x20(%rbx)
   bafb9:	je     bb027 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x247>
   bafbb:	mov    %r12,%rdx
   bafbe:	mov    %edx,%eax
   bafc0:	and    $0x3f,%al
   bafc2:	shr    $0x6,%edx
   bafc5:	movq   $0xffffffffffffffff,0x10(%rsp)
   bafce:	mov    %r15,0x18(%rsp)
   bafd3:	movzbl %al,%ecx
   bafd6:	lea    0x38(%rsp),%rdi
   bafdb:	lea    0x10(%rsp),%rsi
   bafe0:	call   *0xcb812(%rip)        # 1867f8 <_DYNAMIC+0x470>
   bafe6:	jmp    bb034 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x254>
   bafe8:	mov    $0x8,%eax
   bafed:	xor    %esi,%esi
   bafef:	test   %r12,%r12
   baff2:	je     bb01f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x23f>
   baff4:	mov    %r13,%rcx
   baff7:	mov    %rax,%r13
   baffa:	mov    %rax,%rdi
   baffd:	mov    %ebp,0xc(%rsp)
   bb001:	mov    %rsi,%rbp
   bb004:	mov    %rcx,%rsi
   bb007:	mov    %r15,%rdx
   bb00a:	call   *0xcb638(%rip)        # 186648 <memcpy@GLIBC_2.14>
   bb010:	mov    %r13,%rax
   bb013:	mov    %rbp,%rsi
   bb016:	mov    0xc(%rsp),%ebp
   bb01a:	jmp    bb0ba <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bb01f:	xor    %r12d,%r12d
   bb022:	jmp    bb0ba <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bb027:	lea    0x38(%rsp),%rdi
   bb02c:	mov    %r15,%rsi
   bb02f:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   bb034:	test   %bpl,%bpl
   bb037:	je     bb04a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x26a>
   bb039:	lea    0x10(%rsp),%rdi
   bb03e:	lea    0x38(%rsp),%rsi
   bb043:	call   bf590 <<num_bigint::biguint::BigUint as core::ops::arith::Add<u32>>::add>
   bb048:	jmp    bb05e <<hyperreal::computable::node::EvaluationCache>::at_precision+0x27e>
   bb04a:	mov    0x48(%rsp),%rax
   bb04f:	mov    %rax,0x20(%rsp)
   bb054:	movups 0x38(%rsp),%xmm0
   bb059:	movaps %xmm0,0x10(%rsp)
   bb05e:	movzbl 0x28(%rbx),%eax
   bb062:	cmp    $0x1,%al
   bb064:	jne    bb079 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x299>
   bb066:	mov    0x10(%rsp),%rsi
   bb06b:	mov    $0x1,%bpl
   bb06e:	cmp    $0x4,%rsi
   bb072:	jae    bb093 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2b3>
   bb074:	xor    %r12d,%r12d
   bb077:	jmp    bb0b5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bb079:	mov    0x10(%rsp),%rsi
   bb07e:	mov    0x20(%rsp),%r12
   bb083:	test   %r12,%r12
   bb086:	movzbl %al,%eax
   bb089:	mov    $0x1,%ebp
   bb08e:	cmovne %eax,%ebp
   bb091:	jmp    bb0b5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bb093:	mov    0x18(%rsp),%rdi
   bb098:	shl    $0x3,%rsi
   bb09c:	mov    $0x8,%edx
   bb0a1:	call   *0xcb521(%rip)        # 1865c8 <_DYNAMIC+0x240>
   bb0a7:	movq   $0x8,0x18(%rsp)
   bb0b0:	xor    %r12d,%r12d
   bb0b3:	xor    %esi,%esi
   bb0b5:	mov    0x18(%rsp),%rax
   bb0ba:	mov    %rsi,0x10(%rsp)
   bb0bf:	mov    %rax,0x18(%rsp)
   bb0c4:	mov    %r12,0x20(%rsp)
   bb0c9:	mov    %bpl,0x28(%rsp)
   bb0ce:	mov    0xfcb2c(%rip),%ecx        # 1b7c00 <hyperreal::computable::node::signed::ONE+0x20>
   bb0d4:	test   %ecx,%ecx
   bb0d6:	jne    bb196 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3b6>
   bb0dc:	lea    0xfcafd(%rip),%rdx        # 1b7be0 <hyperreal::computable::node::signed::ONE>
   bb0e3:	lea    0x38(%rsp),%rdi
   bb0e8:	lea    0x10(%rsp),%rsi
   bb0ed:	call   bf240 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bb0f2:	lea    0x10(%rsp),%rdi
   bb0f7:	lea    0x38(%rsp),%rsi
   bb0fc:	call   c0760 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bb101:	movaps 0x10(%rsp),%xmm0
   bb106:	movaps 0x20(%rsp),%xmm1
   bb10b:	movups %xmm1,0x10(%r14)
   bb110:	movups %xmm0,(%r14)
   bb114:	mov    $0xffffffff,%esi
   bb119:	lock xadd %esi,(%rbx)
   bb11d:	dec    %esi
   bb11f:	mov    %esi,%eax
   bb121:	and    $0xbfffffff,%eax
   bb126:	neg    %eax
   bb128:	jno    bb170 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bb12a:	mov    %rbx,%rdi
   bb12d:	call   *0xcb7fd(%rip)        # 186930 <_DYNAMIC+0x5a8>
   bb133:	jmp    bb170 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bb135:	mov    %rbx,%rdi
   bb138:	mov    %rdx,%r15
   bb13b:	call   *0xcb7f7(%rip)        # 186938 <_DYNAMIC+0x5b0>
   bb141:	mov    %r15,%rdx
   bb144:	movzbl 0x8(%rbx),%eax
   bb148:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bb14d:	jne    bae22 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42>
   bb153:	movq   $0xffffffffffffffff,(%r14)
   bb15a:	mov    $0xffffffff,%esi
   bb15f:	lock xadd %esi,(%rbx)
   bb163:	dec    %esi
   bb165:	mov    %esi,%eax
   bb167:	and    $0xbfffffff,%eax
   bb16c:	neg    %eax
   bb16e:	jo     bb17f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x39f>
   bb170:	add    $0x68,%rsp
   bb174:	pop    %rbx
   bb175:	pop    %r12
   bb177:	pop    %r13
   bb179:	pop    %r14
   bb17b:	pop    %r15
   bb17d:	pop    %rbp
   bb17e:	ret
   bb17f:	mov    %rbx,%rdi
   bb182:	add    $0x68,%rsp
   bb186:	pop    %rbx
   bb187:	pop    %r12
   bb189:	pop    %r13
   bb18b:	pop    %r14
   bb18d:	pop    %r15
   bb18f:	pop    %rbp
   bb190:	jmp    *0xcb79a(%rip)        # 186930 <_DYNAMIC+0x5a8>
   bb196:	mov    %rax,%r12
   bb199:	mov    %rsi,%r15
   bb19c:	lea    0xfca3d(%rip),%rax        # 1b7be0 <hyperreal::computable::node::signed::ONE>
   bb1a3:	mov    %rax,0x60(%rsp)
   bb1a8:	lea    0x60(%rsp),%rax
   bb1ad:	mov    %rax,0x38(%rsp)
   bb1b2:	lea    0xfca47(%rip),%rdi        # 1b7c00 <hyperreal::computable::node::signed::ONE+0x20>
   bb1b9:	lea    0xc6eb8(%rip),%rcx        # 182078 <anon.7b75c566c6a805bace2b01974d435009.3.llvm.17861709745643738061>
   bb1c0:	lea    0xc6e99(%rip),%r8        # 182060 <anon.7b75c566c6a805bace2b01974d435009.2.llvm.17861709745643738061>
   bb1c7:	lea    0x38(%rsp),%rdx
   bb1cc:	mov    $0x1,%esi
   bb1d1:	call   *0xcb579(%rip)        # 186750 <_DYNAMIC+0x3c8>
   bb1d7:	jmp    bb0dc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2fc>
   bb1dc:	mov    $0x8,%edi
   bb1e1:	jmp    baee7 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x107>
   bb1e6:	mov    %rax,%r14
   bb1e9:	test   %r15,%r15
   bb1ec:	je     bb20a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bb1ee:	mov    %r15,%rsi
   bb1f1:	shl    $0x3,%rsi
   bb1f5:	mov    $0x8,%edx
   bb1fa:	mov    %r12,%rdi
   bb1fd:	call   *0xcb3c5(%rip)        # 1865c8 <_DYNAMIC+0x240>
   bb203:	jmp    bb20a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bb205:	jmp    bb207 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x427>
   bb207:	mov    %rax,%r14
   bb20a:	mov    $0xffffffff,%esi
   bb20f:	lock xadd %esi,(%rbx)
   bb213:	dec    %esi
   bb215:	mov    %esi,%eax
   bb217:	and    $0xbfffffff,%eax
   bb21c:	neg    %eax
   bb21e:	jo     bb228 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x448>
   bb220:	mov    %r14,%rdi
   bb223:	call   180b10 <_Unwind_Resume@plt>
   bb228:	mov    %rbx,%rdi
   bb22b:	call   *0xcb6ff(%rip)        # 186930 <_DYNAMIC+0x5a8>
   bb231:	jmp    bb220 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x440>
   bb233:	call   *0xcb397(%rip)        # 1865d0 <_DYNAMIC+0x248>
