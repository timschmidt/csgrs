
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v2015-nth-root-enclosure-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bb700 <hyperreal::computable::node::scale_at_precision_ref>:
   bb700:	push   %rbp
   bb701:	push   %r15
   bb703:	push   %r14
   bb705:	push   %r13
   bb707:	push   %r12
   bb709:	push   %rbx
   bb70a:	sub    $0x58,%rsp
   bb70e:	mov    %rdi,%rbx
   bb711:	test   %edx,%edx
   bb713:	jle    bb756 <hyperreal::computable::node::scale_at_precision_ref+0x56>
   bb715:	cmp    $0x1,%edx
   bb718:	jne    bb78f <hyperreal::computable::node::scale_at_precision_ref+0x8f>
   bb71a:	mov    0x10(%rsi),%r15
   bb71e:	lea    0x0(,%r15,8),%r12
   bb726:	mov    %r15,%rax
   bb729:	shr    $0x3d,%rax
   bb72d:	setne  %al
   bb730:	movabs $0x7ffffffffffffff8,%rcx
   bb73a:	cmp    %rcx,%r12
   bb73d:	seta   %cl
   bb740:	or     %al,%cl
   bb742:	je     bb8c1 <hyperreal::computable::node::scale_at_precision_ref+0x1c1>
   bb748:	xor    %ebp,%ebp
   bb74a:	mov    %rbp,%rdi
   bb74d:	mov    %r12,%rsi
   bb750:	call   *0xc99ba(%rip)        # 185110 <_DYNAMIC+0x320>
   bb756:	mov    0x10(%rsi),%rbp
   bb75a:	lea    0x0(,%rbp,8),%r14
   bb762:	mov    %rbp,%rax
   bb765:	shr    $0x3d,%rax
   bb769:	setne  %al
   bb76c:	movabs $0x7ffffffffffffff8,%rcx
   bb776:	cmp    %rcx,%r14
   bb779:	seta   %cl
   bb77c:	or     %al,%cl
   bb77e:	je     bb7d8 <hyperreal::computable::node::scale_at_precision_ref+0xd8>
   bb780:	xor    %r13d,%r13d
   bb783:	mov    %r13,%rdi
   bb786:	mov    %r14,%rsi
   bb789:	call   *0xc9981(%rip)        # 185110 <_DYNAMIC+0x320>
   bb78f:	mov    0x10(%rsi),%rax
   bb793:	test   %rax,%rax
   bb796:	je     bb7be <hyperreal::computable::node::scale_at_precision_ref+0xbe>
   bb798:	mov    0x8(%rsi),%rcx
   bb79c:	mov    %edx,%edi
   bb79e:	mov    $0x7f,%r8d
   bb7a4:	bsr    -0x8(%rcx,%rax,8),%r8
   bb7aa:	xor    $0x3f,%r8
   bb7ae:	shl    $0x6,%rax
   bb7b2:	sub    %r8,%rax
   bb7b5:	cmp    %rdi,%rax
   bb7b8:	jae    bb90f <hyperreal::computable::node::scale_at_precision_ref+0x20f>
   bb7be:	movups -0x88a35(%rip),%xmm0        # 32d90 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958+0x10>
   bb7c5:	movups %xmm0,0x10(%rbx)
   bb7c9:	movups -0x88a50(%rip),%xmm0        # 32d80 <anon.cc8a35c2c4be5681ab7929d48bff7af3.53.llvm.589490126919994958>
   bb7d0:	movups %xmm0,(%rbx)
   bb7d3:	jmp    bbb35 <hyperreal::computable::node::scale_at_precision_ref+0x435>
   bb7d8:	movzbl 0x18(%rsi),%eax
   bb7dc:	mov    %al,0x7(%rsp)
   bb7e0:	mov    0x8(%rsi),%rsi
   bb7e4:	test   %r14,%r14
   bb7e7:	je     bb926 <hyperreal::computable::node::scale_at_precision_ref+0x226>
   bb7ed:	mov    %rsi,0x48(%rsp)
   bb7f2:	mov    %rdx,%r15
   bb7f5:	call   *0xc9835(%rip)        # 185030 <_DYNAMIC+0x240>
   bb7fb:	mov    $0x8,%r13d
   bb801:	mov    $0x8,%esi
   bb806:	mov    %r14,%rdi
   bb809:	call   *0xc9829(%rip)        # 185038 <_DYNAMIC+0x248>
   bb80f:	test   %rax,%rax
   bb812:	je     bb783 <hyperreal::computable::node::scale_at_precision_ref+0x83>
   bb818:	mov    %rax,%r12
   bb81b:	mov    %rbp,%r13
   bb81e:	mov    %r15,%rdx
   bb821:	mov    0x48(%rsp),%rsi
   bb826:	test   %rbp,%rbp
   bb829:	je     bb938 <hyperreal::computable::node::scale_at_precision_ref+0x238>
   bb82f:	neg    %edx
   bb831:	mov    %r12,%rdi
   bb834:	mov    %rdx,%r15
   bb837:	mov    %r14,%rdx
   bb83a:	call   *0xc9810(%rip)        # 185050 <memcpy@GLIBC_2.14>
   bb840:	mov    %r15d,%eax
   bb843:	and    $0x3f,%al
   bb845:	shr    $0x6,%r15d
   bb849:	mov    %r13,0x28(%rsp)
   bb84e:	mov    %r12,0x30(%rsp)
   bb853:	mov    %rbp,0x38(%rsp)
   bb858:	movzbl %al,%ecx
   bb85b:	lea    0x8(%rsp),%rdi
   bb860:	lea    0x28(%rsp),%rsi
   bb865:	mov    %r15,%rdx
   bb868:	call   *0xc984a(%rip)        # 1850b8 <_DYNAMIC+0x2c8>
   bb86e:	movzbl 0x7(%rsp),%eax
   bb873:	cmp    $0x1,%al
   bb875:	jne    bb958 <hyperreal::computable::node::scale_at_precision_ref+0x258>
   bb87b:	movq   $0x0,0x18(%rsp)
   bb884:	mov    0x8(%rsp),%rsi
   bb889:	mov    $0x1,%bpl
   bb88c:	cmp    $0x4,%rsi
   bb890:	jb     bb969 <hyperreal::computable::node::scale_at_precision_ref+0x269>
   bb896:	mov    0x10(%rsp),%rdi
   bb89b:	shl    $0x3,%rsi
   bb89f:	mov    $0x8,%edx
   bb8a4:	call   *0xc97ce(%rip)        # 185078 <_DYNAMIC+0x288>
   bb8aa:	movq   $0x8,0x10(%rsp)
   bb8b3:	movq   $0x0,0x8(%rsp)
   bb8bc:	jmp    bb969 <hyperreal::computable::node::scale_at_precision_ref+0x269>
   bb8c1:	movzbl 0x18(%rsi),%r13d
   bb8c6:	mov    0x8(%rsi),%rsi
   bb8ca:	test   %r12,%r12
   bb8cd:	je     bb98c <hyperreal::computable::node::scale_at_precision_ref+0x28c>
   bb8d3:	mov    %rsi,%r14
   bb8d6:	call   *0xc9754(%rip)        # 185030 <_DYNAMIC+0x240>
   bb8dc:	mov    $0x8,%ebp
   bb8e1:	mov    $0x8,%esi
   bb8e6:	mov    %r12,%rdi
   bb8e9:	call   *0xc9749(%rip)        # 185038 <_DYNAMIC+0x248>
   bb8ef:	test   %rax,%rax
   bb8f2:	je     bb74a <hyperreal::computable::node::scale_at_precision_ref+0x4a>
   bb8f8:	mov    %r15,%rbp
   bb8fb:	mov    %r14,%rsi
   bb8fe:	mov    %rax,%r14
   bb901:	test   %r15,%r15
   bb904:	jne    bb99d <hyperreal::computable::node::scale_at_precision_ref+0x29d>
   bb90a:	jmp    bbadd <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bb90f:	dec    %edx
   bb911:	movzbl 0x18(%rsi),%r12d
   bb916:	test   %r12b,%r12b
   bb919:	je     bb9ae <hyperreal::computable::node::scale_at_precision_ref+0x2ae>
   bb91f:	xor    %ebp,%ebp
   bb921:	jmp    bb9d7 <hyperreal::computable::node::scale_at_precision_ref+0x2d7>
   bb926:	mov    $0x8,%r12d
   bb92c:	xor    %r13d,%r13d
   bb92f:	test   %rbp,%rbp
   bb932:	jne    bb82f <hyperreal::computable::node::scale_at_precision_ref+0x12f>
   bb938:	mov    %r13,0x8(%rsp)
   bb93d:	mov    %r12,0x10(%rsp)
   bb942:	movq   $0x0,0x18(%rsp)
   bb94b:	movzbl 0x7(%rsp),%eax
   bb950:	cmp    $0x1,%al
   bb952:	je     bb87b <hyperreal::computable::node::scale_at_precision_ref+0x17b>
   bb958:	cmpq   $0x0,0x18(%rsp)
   bb95e:	movzbl %al,%eax
   bb961:	mov    $0x1,%ebp
   bb966:	cmovne %eax,%ebp
   bb969:	mov    0x18(%rsp),%rax
   bb96e:	mov    %rax,0x10(%rbx)
   bb972:	mov    0x8(%rsp),%rax
   bb977:	mov    %rax,(%rbx)
   bb97a:	mov    0x10(%rsp),%rax
   bb97f:	mov    %rax,0x8(%rbx)
   bb983:	mov    %bpl,0x18(%rbx)
   bb987:	jmp    bbb35 <hyperreal::computable::node::scale_at_precision_ref+0x435>
   bb98c:	mov    $0x8,%r14d
   bb992:	xor    %ebp,%ebp
   bb994:	test   %r15,%r15
   bb997:	je     bbadd <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bb99d:	mov    %r14,%rdi
   bb9a0:	mov    %r12,%rdx
   bb9a3:	call   *0xc96a7(%rip)        # 185050 <memcpy@GLIBC_2.14>
   bb9a9:	jmp    bbaf3 <hyperreal::computable::node::scale_at_precision_ref+0x3f3>
   bb9ae:	mov    %rdx,%r15
   bb9b1:	mov    %rsi,%r14
   bb9b4:	mov    %rsi,%rdi
   bb9b7:	call   *0xc9a7b(%rip)        # 185438 <_DYNAMIC+0x648>
   bb9bd:	test   $0x1,%al
   bb9bf:	je     bbb84 <hyperreal::computable::node::scale_at_precision_ref+0x484>
   bb9c5:	mov    %rdx,%rcx
   bb9c8:	mov    %r15,%rdx
   bb9cb:	mov    %edx,%eax
   bb9cd:	cmp    %rax,%rcx
   bb9d0:	setb   %bpl
   bb9d4:	mov    %r14,%rsi
   bb9d7:	mov    %edx,%eax
   bb9d9:	and    $0x3f,%al
   bb9db:	shr    $0x6,%edx
   bb9de:	movq   $0xffffffffffffffff,0x28(%rsp)
   bb9e7:	mov    %rsi,0x30(%rsp)
   bb9ec:	movzbl %al,%ecx
   bb9ef:	lea    0x8(%rsp),%rdi
   bb9f4:	lea    0x28(%rsp),%rsi
   bb9f9:	call   *0xc9839(%rip)        # 185238 <_DYNAMIC+0x448>
   bb9ff:	test   %bpl,%bpl
   bba02:	je     bbaa2 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bba08:	mov    0x18(%rsp),%rax
   bba0d:	test   %rax,%rax
   bba10:	jne    bba3f <hyperreal::computable::node::scale_at_precision_ref+0x33f>
   bba12:	cmpq   $0x0,0x8(%rsp)
   bba18:	jne    bba25 <hyperreal::computable::node::scale_at_precision_ref+0x325>
   bba1a:	lea    0x8(%rsp),%rdi
   bba1f:	call   *0xc969b(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   bba25:	mov    0x10(%rsp),%rax
   bba2a:	movq   $0x0,(%rax)
   bba31:	movq   $0x1,0x18(%rsp)
   bba3a:	mov    $0x1,%eax
   bba3f:	mov    0x10(%rsp),%rcx
   bba44:	incq   (%rcx)
   bba47:	jne    bbaa2 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bba49:	mov    $0x1,%r14d
   bba4f:	cmp    $0x1,%rax
   bba53:	je     bba7a <hyperreal::computable::node::scale_at_precision_ref+0x37a>
   bba55:	shl    $0x3,%rax
   bba59:	mov    $0x1,%dl
   bba5b:	mov    $0x8,%esi
   bba60:	add    $0xff,%dl
   bba63:	adcq   $0x0,(%rcx,%rsi,1)
   bba68:	setb   %dl
   bba6b:	jae    bbaa2 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bba6d:	add    $0x8,%rsi
   bba71:	cmp    %rsi,%rax
   bba74:	jne    bba60 <hyperreal::computable::node::scale_at_precision_ref+0x360>
   bba76:	movzbl %dl,%r14d
   bba7a:	mov    0x18(%rsp),%r15
   bba7f:	cmp    0x8(%rsp),%r15
   bba84:	jne    bba91 <hyperreal::computable::node::scale_at_precision_ref+0x391>
   bba86:	lea    0x8(%rsp),%rdi
   bba8b:	call   *0xc962f(%rip)        # 1850c0 <_DYNAMIC+0x2d0>
   bba91:	mov    0x10(%rsp),%rax
   bba96:	mov    %r14,(%rax,%r15,8)
   bba9a:	inc    %r15
   bba9d:	mov    %r15,0x18(%rsp)
   bbaa2:	mov    0x10(%rsp),%r14
   bbaa7:	mov    0x18(%rsp),%r15
   bbaac:	mov    0x8(%rsp),%rbp
   bbab1:	cmp    $0x1,%r12b
   bbab5:	jne    bbae2 <hyperreal::computable::node::scale_at_precision_ref+0x3e2>
   bbab7:	mov    $0x1,%r13b
   bbaba:	cmp    $0x4,%rbp
   bbabe:	jb     bbadd <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bbac0:	shl    $0x3,%rbp
   bbac4:	mov    $0x8,%edx
   bbac9:	mov    %r14,%rdi
   bbacc:	mov    %rbp,%rsi
   bbacf:	call   *0xc95a3(%rip)        # 185078 <_DYNAMIC+0x288>
   bbad5:	xor    %ebp,%ebp
   bbad7:	mov    $0x8,%r14d
   bbadd:	xor    %r15d,%r15d
   bbae0:	jmp    bbaf3 <hyperreal::computable::node::scale_at_precision_ref+0x3f3>
   bbae2:	test   %r15,%r15
   bbae5:	movzbl %r12b,%eax
   bbae9:	mov    $0x1,%r13d
   bbaef:	cmovne %eax,%r13d
   bbaf3:	mov    %rbp,0x28(%rsp)
   bbaf8:	mov    %r14,0x30(%rsp)
   bbafd:	mov    %r15,0x38(%rsp)
   bbb02:	mov    %r13b,0x40(%rsp)
   bbb07:	mov    0xfab4b(%rip),%eax        # 1b6658 <hyperreal::computable::node::signed::ONE+0x20>
   bbb0d:	test   %eax,%eax
   bbb0f:	jne    bbb47 <hyperreal::computable::node::scale_at_precision_ref+0x447>
   bbb11:	lea    0xfab20(%rip),%rdx        # 1b6638 <hyperreal::computable::node::signed::ONE>
   bbb18:	lea    0x8(%rsp),%r14
   bbb1d:	lea    0x28(%rsp),%rsi
   bbb22:	mov    %r14,%rdi
   bbb25:	call   be400 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bbb2a:	mov    %rbx,%rdi
   bbb2d:	mov    %r14,%rsi
   bbb30:	call   bf920 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bbb35:	mov    %rbx,%rax
   bbb38:	add    $0x58,%rsp
   bbb3c:	pop    %rbx
   bbb3d:	pop    %r12
   bbb3f:	pop    %r13
   bbb41:	pop    %r14
   bbb43:	pop    %r15
   bbb45:	pop    %rbp
   bbb46:	ret
   bbb47:	lea    0xfaaea(%rip),%rax        # 1b6638 <hyperreal::computable::node::signed::ONE>
   bbb4e:	mov    %rax,0x50(%rsp)
   bbb53:	lea    0x50(%rsp),%rax
   bbb58:	mov    %rax,0x8(%rsp)
   bbb5d:	lea    0xfaaf4(%rip),%rdi        # 1b6658 <hyperreal::computable::node::signed::ONE+0x20>
   bbb64:	lea    0xc4fc5(%rip),%rcx        # 180b30 <anon.f651f877c5f1b0c83bd435f3f6e617b1.3.llvm.9350274297799828109>
   bbb6b:	lea    0xc4fa6(%rip),%r8        # 180b18 <anon.f651f877c5f1b0c83bd435f3f6e617b1.2.llvm.9350274297799828109>
   bbb72:	lea    0x8(%rsp),%rdx
   bbb77:	mov    $0x1,%esi
   bbb7c:	call   *0xc960e(%rip)        # 185190 <_DYNAMIC+0x3a0>
   bbb82:	jmp    bbb11 <hyperreal::computable::node::scale_at_precision_ref+0x411>
   bbb84:	lea    -0x82fd3(%rip),%rdi        # 38bb8 <anon.837dca1aa7b2ef2d7e2c41242f94404e.2.llvm.16824341230851169513>
   bbb8b:	lea    0xc670e(%rip),%rdx        # 1822a0 <anon.837dca1aa7b2ef2d7e2c41242f94404e.4.llvm.16824341230851169513>
   bbb92:	mov    $0x1c,%esi
   bbb97:	call   *0xc96cb(%rip)        # 185268 <_DYNAMIC+0x478>
   bbb9d:	mov    %r14,%rdi
   bbba0:	mov    %rax,%rbx
   bbba3:	test   %rbp,%rbp
   bbba6:	jne    bbbbc <hyperreal::computable::node::scale_at_precision_ref+0x4bc>
   bbba8:	jmp    bbbce <hyperreal::computable::node::scale_at_precision_ref+0x4ce>
   bbbaa:	mov    %rax,%rbx
   bbbad:	mov    0x8(%rsp),%rbp
   bbbb2:	test   %rbp,%rbp
   bbbb5:	je     bbbce <hyperreal::computable::node::scale_at_precision_ref+0x4ce>
   bbbb7:	mov    0x10(%rsp),%rdi
   bbbbc:	shl    $0x3,%rbp
   bbbc0:	mov    $0x8,%edx
   bbbc5:	mov    %rbp,%rsi
   bbbc8:	call   *0xc94aa(%rip)        # 185078 <_DYNAMIC+0x288>
   bbbce:	mov    %rbx,%rdi
   bbbd1:	call   17f640 <_Unwind_Resume@plt>
