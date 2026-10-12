
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1774-domain-predicate-integer-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bb760 <hyperreal::computable::node::scale_at_precision_ref>:
   bb760:	push   %rbp
   bb761:	push   %r15
   bb763:	push   %r14
   bb765:	push   %r13
   bb767:	push   %r12
   bb769:	push   %rbx
   bb76a:	sub    $0x58,%rsp
   bb76e:	mov    %rdi,%rbx
   bb771:	test   %edx,%edx
   bb773:	jle    bb7b6 <hyperreal::computable::node::scale_at_precision_ref+0x56>
   bb775:	cmp    $0x1,%edx
   bb778:	jne    bb7ef <hyperreal::computable::node::scale_at_precision_ref+0x8f>
   bb77a:	mov    0x10(%rsi),%r15
   bb77e:	lea    0x0(,%r15,8),%r12
   bb786:	mov    %r15,%rax
   bb789:	shr    $0x3d,%rax
   bb78d:	setne  %al
   bb790:	movabs $0x7ffffffffffffff8,%rcx
   bb79a:	cmp    %rcx,%r12
   bb79d:	seta   %cl
   bb7a0:	or     %al,%cl
   bb7a2:	je     bb921 <hyperreal::computable::node::scale_at_precision_ref+0x1c1>
   bb7a8:	xor    %ebp,%ebp
   bb7aa:	mov    %rbp,%rdi
   bb7ad:	mov    %r12,%rsi
   bb7b0:	call   *0xc9c9a(%rip)        # 185450 <_DYNAMIC+0x320>
   bb7b6:	mov    0x10(%rsi),%rbp
   bb7ba:	lea    0x0(,%rbp,8),%r14
   bb7c2:	mov    %rbp,%rax
   bb7c5:	shr    $0x3d,%rax
   bb7c9:	setne  %al
   bb7cc:	movabs $0x7ffffffffffffff8,%rcx
   bb7d6:	cmp    %rcx,%r14
   bb7d9:	seta   %cl
   bb7dc:	or     %al,%cl
   bb7de:	je     bb838 <hyperreal::computable::node::scale_at_precision_ref+0xd8>
   bb7e0:	xor    %r13d,%r13d
   bb7e3:	mov    %r13,%rdi
   bb7e6:	mov    %r14,%rsi
   bb7e9:	call   *0xc9c61(%rip)        # 185450 <_DYNAMIC+0x320>
   bb7ef:	mov    0x10(%rsi),%rax
   bb7f3:	test   %rax,%rax
   bb7f6:	je     bb81e <hyperreal::computable::node::scale_at_precision_ref+0xbe>
   bb7f8:	mov    0x8(%rsi),%rcx
   bb7fc:	mov    %edx,%edi
   bb7fe:	mov    $0x7f,%r8d
   bb804:	bsr    -0x8(%rcx,%rax,8),%r8
   bb80a:	xor    $0x3f,%r8
   bb80e:	shl    $0x6,%rax
   bb812:	sub    %r8,%rax
   bb815:	cmp    %rdi,%rax
   bb818:	jae    bb96f <hyperreal::computable::node::scale_at_precision_ref+0x20f>
   bb81e:	movups -0x88a55(%rip),%xmm0        # 32dd0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   bb825:	movups %xmm0,0x10(%rbx)
   bb829:	movups -0x88a70(%rip),%xmm0        # 32dc0 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   bb830:	movups %xmm0,(%rbx)
   bb833:	jmp    bbb95 <hyperreal::computable::node::scale_at_precision_ref+0x435>
   bb838:	movzbl 0x18(%rsi),%eax
   bb83c:	mov    %al,0x7(%rsp)
   bb840:	mov    0x8(%rsi),%rsi
   bb844:	test   %r14,%r14
   bb847:	je     bb986 <hyperreal::computable::node::scale_at_precision_ref+0x226>
   bb84d:	mov    %rsi,0x48(%rsp)
   bb852:	mov    %rdx,%r15
   bb855:	call   *0xc9b15(%rip)        # 185370 <_DYNAMIC+0x240>
   bb85b:	mov    $0x8,%r13d
   bb861:	mov    $0x8,%esi
   bb866:	mov    %r14,%rdi
   bb869:	call   *0xc9b09(%rip)        # 185378 <_DYNAMIC+0x248>
   bb86f:	test   %rax,%rax
   bb872:	je     bb7e3 <hyperreal::computable::node::scale_at_precision_ref+0x83>
   bb878:	mov    %rax,%r12
   bb87b:	mov    %rbp,%r13
   bb87e:	mov    %r15,%rdx
   bb881:	mov    0x48(%rsp),%rsi
   bb886:	test   %rbp,%rbp
   bb889:	je     bb998 <hyperreal::computable::node::scale_at_precision_ref+0x238>
   bb88f:	neg    %edx
   bb891:	mov    %r12,%rdi
   bb894:	mov    %rdx,%r15
   bb897:	mov    %r14,%rdx
   bb89a:	call   *0xc9af0(%rip)        # 185390 <memcpy@GLIBC_2.14>
   bb8a0:	mov    %r15d,%eax
   bb8a3:	and    $0x3f,%al
   bb8a5:	shr    $0x6,%r15d
   bb8a9:	mov    %r13,0x28(%rsp)
   bb8ae:	mov    %r12,0x30(%rsp)
   bb8b3:	mov    %rbp,0x38(%rsp)
   bb8b8:	movzbl %al,%ecx
   bb8bb:	lea    0x8(%rsp),%rdi
   bb8c0:	lea    0x28(%rsp),%rsi
   bb8c5:	mov    %r15,%rdx
   bb8c8:	call   *0xc9b2a(%rip)        # 1853f8 <_DYNAMIC+0x2c8>
   bb8ce:	movzbl 0x7(%rsp),%eax
   bb8d3:	cmp    $0x1,%al
   bb8d5:	jne    bb9b8 <hyperreal::computable::node::scale_at_precision_ref+0x258>
   bb8db:	movq   $0x0,0x18(%rsp)
   bb8e4:	mov    0x8(%rsp),%rsi
   bb8e9:	mov    $0x1,%bpl
   bb8ec:	cmp    $0x4,%rsi
   bb8f0:	jb     bb9c9 <hyperreal::computable::node::scale_at_precision_ref+0x269>
   bb8f6:	mov    0x10(%rsp),%rdi
   bb8fb:	shl    $0x3,%rsi
   bb8ff:	mov    $0x8,%edx
   bb904:	call   *0xc9aae(%rip)        # 1853b8 <_DYNAMIC+0x288>
   bb90a:	movq   $0x8,0x10(%rsp)
   bb913:	movq   $0x0,0x8(%rsp)
   bb91c:	jmp    bb9c9 <hyperreal::computable::node::scale_at_precision_ref+0x269>
   bb921:	movzbl 0x18(%rsi),%r13d
   bb926:	mov    0x8(%rsi),%rsi
   bb92a:	test   %r12,%r12
   bb92d:	je     bb9ec <hyperreal::computable::node::scale_at_precision_ref+0x28c>
   bb933:	mov    %rsi,%r14
   bb936:	call   *0xc9a34(%rip)        # 185370 <_DYNAMIC+0x240>
   bb93c:	mov    $0x8,%ebp
   bb941:	mov    $0x8,%esi
   bb946:	mov    %r12,%rdi
   bb949:	call   *0xc9a29(%rip)        # 185378 <_DYNAMIC+0x248>
   bb94f:	test   %rax,%rax
   bb952:	je     bb7aa <hyperreal::computable::node::scale_at_precision_ref+0x4a>
   bb958:	mov    %r15,%rbp
   bb95b:	mov    %r14,%rsi
   bb95e:	mov    %rax,%r14
   bb961:	test   %r15,%r15
   bb964:	jne    bb9fd <hyperreal::computable::node::scale_at_precision_ref+0x29d>
   bb96a:	jmp    bbb3d <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bb96f:	dec    %edx
   bb971:	movzbl 0x18(%rsi),%r12d
   bb976:	test   %r12b,%r12b
   bb979:	je     bba0e <hyperreal::computable::node::scale_at_precision_ref+0x2ae>
   bb97f:	xor    %ebp,%ebp
   bb981:	jmp    bba37 <hyperreal::computable::node::scale_at_precision_ref+0x2d7>
   bb986:	mov    $0x8,%r12d
   bb98c:	xor    %r13d,%r13d
   bb98f:	test   %rbp,%rbp
   bb992:	jne    bb88f <hyperreal::computable::node::scale_at_precision_ref+0x12f>
   bb998:	mov    %r13,0x8(%rsp)
   bb99d:	mov    %r12,0x10(%rsp)
   bb9a2:	movq   $0x0,0x18(%rsp)
   bb9ab:	movzbl 0x7(%rsp),%eax
   bb9b0:	cmp    $0x1,%al
   bb9b2:	je     bb8db <hyperreal::computable::node::scale_at_precision_ref+0x17b>
   bb9b8:	cmpq   $0x0,0x18(%rsp)
   bb9be:	movzbl %al,%eax
   bb9c1:	mov    $0x1,%ebp
   bb9c6:	cmovne %eax,%ebp
   bb9c9:	mov    0x18(%rsp),%rax
   bb9ce:	mov    %rax,0x10(%rbx)
   bb9d2:	mov    0x8(%rsp),%rax
   bb9d7:	mov    %rax,(%rbx)
   bb9da:	mov    0x10(%rsp),%rax
   bb9df:	mov    %rax,0x8(%rbx)
   bb9e3:	mov    %bpl,0x18(%rbx)
   bb9e7:	jmp    bbb95 <hyperreal::computable::node::scale_at_precision_ref+0x435>
   bb9ec:	mov    $0x8,%r14d
   bb9f2:	xor    %ebp,%ebp
   bb9f4:	test   %r15,%r15
   bb9f7:	je     bbb3d <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bb9fd:	mov    %r14,%rdi
   bba00:	mov    %r12,%rdx
   bba03:	call   *0xc9987(%rip)        # 185390 <memcpy@GLIBC_2.14>
   bba09:	jmp    bbb53 <hyperreal::computable::node::scale_at_precision_ref+0x3f3>
   bba0e:	mov    %rdx,%r15
   bba11:	mov    %rsi,%r14
   bba14:	mov    %rsi,%rdi
   bba17:	call   *0xc9d5b(%rip)        # 185778 <_DYNAMIC+0x648>
   bba1d:	test   $0x1,%al
   bba1f:	je     bbbe4 <hyperreal::computable::node::scale_at_precision_ref+0x484>
   bba25:	mov    %rdx,%rcx
   bba28:	mov    %r15,%rdx
   bba2b:	mov    %edx,%eax
   bba2d:	cmp    %rax,%rcx
   bba30:	setb   %bpl
   bba34:	mov    %r14,%rsi
   bba37:	mov    %edx,%eax
   bba39:	and    $0x3f,%al
   bba3b:	shr    $0x6,%edx
   bba3e:	movq   $0xffffffffffffffff,0x28(%rsp)
   bba47:	mov    %rsi,0x30(%rsp)
   bba4c:	movzbl %al,%ecx
   bba4f:	lea    0x8(%rsp),%rdi
   bba54:	lea    0x28(%rsp),%rsi
   bba59:	call   *0xc9b19(%rip)        # 185578 <_DYNAMIC+0x448>
   bba5f:	test   %bpl,%bpl
   bba62:	je     bbb02 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bba68:	mov    0x18(%rsp),%rax
   bba6d:	test   %rax,%rax
   bba70:	jne    bba9f <hyperreal::computable::node::scale_at_precision_ref+0x33f>
   bba72:	cmpq   $0x0,0x8(%rsp)
   bba78:	jne    bba85 <hyperreal::computable::node::scale_at_precision_ref+0x325>
   bba7a:	lea    0x8(%rsp),%rdi
   bba7f:	call   *0xc997b(%rip)        # 185400 <_DYNAMIC+0x2d0>
   bba85:	mov    0x10(%rsp),%rax
   bba8a:	movq   $0x0,(%rax)
   bba91:	movq   $0x1,0x18(%rsp)
   bba9a:	mov    $0x1,%eax
   bba9f:	mov    0x10(%rsp),%rcx
   bbaa4:	incq   (%rcx)
   bbaa7:	jne    bbb02 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bbaa9:	mov    $0x1,%r14d
   bbaaf:	cmp    $0x1,%rax
   bbab3:	je     bbada <hyperreal::computable::node::scale_at_precision_ref+0x37a>
   bbab5:	shl    $0x3,%rax
   bbab9:	mov    $0x1,%dl
   bbabb:	mov    $0x8,%esi
   bbac0:	add    $0xff,%dl
   bbac3:	adcq   $0x0,(%rcx,%rsi,1)
   bbac8:	setb   %dl
   bbacb:	jae    bbb02 <hyperreal::computable::node::scale_at_precision_ref+0x3a2>
   bbacd:	add    $0x8,%rsi
   bbad1:	cmp    %rsi,%rax
   bbad4:	jne    bbac0 <hyperreal::computable::node::scale_at_precision_ref+0x360>
   bbad6:	movzbl %dl,%r14d
   bbada:	mov    0x18(%rsp),%r15
   bbadf:	cmp    0x8(%rsp),%r15
   bbae4:	jne    bbaf1 <hyperreal::computable::node::scale_at_precision_ref+0x391>
   bbae6:	lea    0x8(%rsp),%rdi
   bbaeb:	call   *0xc990f(%rip)        # 185400 <_DYNAMIC+0x2d0>
   bbaf1:	mov    0x10(%rsp),%rax
   bbaf6:	mov    %r14,(%rax,%r15,8)
   bbafa:	inc    %r15
   bbafd:	mov    %r15,0x18(%rsp)
   bbb02:	mov    0x10(%rsp),%r14
   bbb07:	mov    0x18(%rsp),%r15
   bbb0c:	mov    0x8(%rsp),%rbp
   bbb11:	cmp    $0x1,%r12b
   bbb15:	jne    bbb42 <hyperreal::computable::node::scale_at_precision_ref+0x3e2>
   bbb17:	mov    $0x1,%r13b
   bbb1a:	cmp    $0x4,%rbp
   bbb1e:	jb     bbb3d <hyperreal::computable::node::scale_at_precision_ref+0x3dd>
   bbb20:	shl    $0x3,%rbp
   bbb24:	mov    $0x8,%edx
   bbb29:	mov    %r14,%rdi
   bbb2c:	mov    %rbp,%rsi
   bbb2f:	call   *0xc9883(%rip)        # 1853b8 <_DYNAMIC+0x288>
   bbb35:	xor    %ebp,%ebp
   bbb37:	mov    $0x8,%r14d
   bbb3d:	xor    %r15d,%r15d
   bbb40:	jmp    bbb53 <hyperreal::computable::node::scale_at_precision_ref+0x3f3>
   bbb42:	test   %r15,%r15
   bbb45:	movzbl %r12b,%eax
   bbb49:	mov    $0x1,%r13d
   bbb4f:	cmovne %eax,%r13d
   bbb53:	mov    %rbp,0x28(%rsp)
   bbb58:	mov    %r14,0x30(%rsp)
   bbb5d:	mov    %r15,0x38(%rsp)
   bbb62:	mov    %r13b,0x40(%rsp)
   bbb67:	mov    0xfae2b(%rip),%eax        # 1b6998 <hyperreal::computable::node::signed::ONE+0x20>
   bbb6d:	test   %eax,%eax
   bbb6f:	jne    bbba7 <hyperreal::computable::node::scale_at_precision_ref+0x447>
   bbb71:	lea    0xfae00(%rip),%rdx        # 1b6978 <hyperreal::computable::node::signed::ONE>
   bbb78:	lea    0x8(%rsp),%r14
   bbb7d:	lea    0x28(%rsp),%rsi
   bbb82:	mov    %r14,%rdi
   bbb85:	call   be460 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bbb8a:	mov    %rbx,%rdi
   bbb8d:	mov    %r14,%rsi
   bbb90:	call   bf980 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bbb95:	mov    %rbx,%rax
   bbb98:	add    $0x58,%rsp
   bbb9c:	pop    %rbx
   bbb9d:	pop    %r12
   bbb9f:	pop    %r13
   bbba1:	pop    %r14
   bbba3:	pop    %r15
   bbba5:	pop    %rbp
   bbba6:	ret
   bbba7:	lea    0xfadca(%rip),%rax        # 1b6978 <hyperreal::computable::node::signed::ONE>
   bbbae:	mov    %rax,0x50(%rsp)
   bbbb3:	lea    0x50(%rsp),%rax
   bbbb8:	mov    %rax,0x8(%rsp)
   bbbbd:	lea    0xfadd4(%rip),%rdi        # 1b6998 <hyperreal::computable::node::signed::ONE+0x20>
   bbbc4:	lea    0xc5275(%rip),%rcx        # 180e40 <anon.9023bb4654a08944d5043612f590d0ff.3.llvm.1768662447242434102>
   bbbcb:	lea    0xc5256(%rip),%r8        # 180e28 <anon.9023bb4654a08944d5043612f590d0ff.2.llvm.1768662447242434102>
   bbbd2:	lea    0x8(%rsp),%rdx
   bbbd7:	mov    $0x1,%esi
   bbbdc:	call   *0xc98ee(%rip)        # 1854d0 <_DYNAMIC+0x3a0>
   bbbe2:	jmp    bbb71 <hyperreal::computable::node::scale_at_precision_ref+0x411>
   bbbe4:	lea    -0x82ffb(%rip),%rdi        # 38bf0 <anon.837dca1aa7b2ef2d7e2c41242f94404e.2.llvm.9447362419440166577>
   bbbeb:	lea    0xc69ee(%rip),%rdx        # 1825e0 <anon.837dca1aa7b2ef2d7e2c41242f94404e.4.llvm.9447362419440166577>
   bbbf2:	mov    $0x1c,%esi
   bbbf7:	call   *0xc99ab(%rip)        # 1855a8 <_DYNAMIC+0x478>
   bbbfd:	mov    %r14,%rdi
   bbc00:	mov    %rax,%rbx
   bbc03:	test   %rbp,%rbp
   bbc06:	jne    bbc1c <hyperreal::computable::node::scale_at_precision_ref+0x4bc>
   bbc08:	jmp    bbc2e <hyperreal::computable::node::scale_at_precision_ref+0x4ce>
   bbc0a:	mov    %rax,%rbx
   bbc0d:	mov    0x8(%rsp),%rbp
   bbc12:	test   %rbp,%rbp
   bbc15:	je     bbc2e <hyperreal::computable::node::scale_at_precision_ref+0x4ce>
   bbc17:	mov    0x10(%rsp),%rdi
   bbc1c:	shl    $0x3,%rbp
   bbc20:	mov    $0x8,%edx
   bbc25:	mov    %rbp,%rsi
   bbc28:	call   *0xc978a(%rip)        # 1853b8 <_DYNAMIC+0x288>
   bbc2e:	mov    %rbx,%rdi
   bbc31:	call   17f950 <_Unwind_Resume@plt>
