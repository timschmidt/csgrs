
target/typed-api-prototype-v1437-acos-residual-targeted-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

0000000000105590 <hyperreal::computable::approximation::acos_positive_rational>:
  105590:	push   %rbp
  105591:	push   %r15
  105593:	push   %r14
  105595:	push   %r13
  105597:	push   %r12
  105599:	push   %rbx
  10559a:	sub    $0xa8,%rsp
  1055a1:	mov    %rdi,%rbx
  1055a4:	cmp    $0xffefffff,%ecx
  1055aa:	jg     1055b2 <hyperreal::computable::approximation::acos_positive_rational+0x22>
  1055ac:	movb   $0xc,0x8(%rbx)
  1055b0:	jmp    10562b <hyperreal::computable::approximation::acos_positive_rational+0x9b>
  1055b2:	mov    %ecx,%ebp
  1055b4:	mov    %rsi,%r12
  1055b7:	mov    (%rdx),%r13
  1055ba:	mov    0x60(%r13),%eax
  1055be:	mov    %rdx,%r15
  1055c1:	test   %al,%al
  1055c3:	jns    1055d5 <hyperreal::computable::approximation::acos_positive_rational+0x45>
  1055c5:	lea    0x40(%r13),%r15
  1055c9:	mov    0x50(%r13),%eax
  1055cd:	test   %eax,%eax
  1055cf:	jne    105c2d <hyperreal::computable::approximation::acos_positive_rational+0x69d>
  1055d5:	mov    (%r15),%r14
  1055d8:	mov    0x60(%r13),%eax
  1055dc:	mov    %rdx,%r15
  1055df:	test   %al,%al
  1055e1:	jns    1055f3 <hyperreal::computable::approximation::acos_positive_rational+0x63>
  1055e3:	lea    0x40(%r13),%r15
  1055e7:	mov    0x50(%r13),%eax
  1055eb:	test   %eax,%eax
  1055ed:	jne    105c43 <hyperreal::computable::approximation::acos_positive_rational+0x6b3>
  1055f3:	mov    (%r15),%rcx
  1055f6:	mov    0x20(%r14),%rax
  1055fa:	cmp    0x38(%rcx),%rax
  1055fe:	jne    105625 <hyperreal::computable::approximation::acos_positive_rational+0x95>
  105600:	mov    0x30(%rcx),%rcx
  105604:	mov    0x18(%r14),%rsi
  105608:	shl    $0x3,%rax
  10560c:	nopl   0x0(%rax)
  105610:	test   %rax,%rax
  105613:	je     105644 <hyperreal::computable::approximation::acos_positive_rational+0xb4>
  105615:	mov    -0x8(%rcx,%rax,1),%rdi
  10561a:	cmp    %rdi,-0x8(%rsi,%rax,1)
  10561f:	lea    -0x8(%rax),%rax
  105623:	je     105610 <hyperreal::computable::approximation::acos_positive_rational+0x80>
  105625:	jbe    105644 <hyperreal::computable::approximation::acos_positive_rational+0xb4>
  105627:	movb   $0x5,0x8(%rbx)
  10562b:	movq   $0xffffffffffffffff,(%rbx)
  105632:	add    $0xa8,%rsp
  105639:	pop    %rbx
  10563a:	pop    %r12
  10563c:	pop    %r13
  10563e:	pop    %r14
  105640:	pop    %r15
  105642:	pop    %rbp
  105643:	ret
  105644:	mov    0x60(%r13),%eax
  105648:	mov    %rdx,%r15
  10564b:	test   %al,%al
  10564d:	jns    10565f <hyperreal::computable::approximation::acos_positive_rational+0xcf>
  10564f:	lea    0x40(%r13),%r15
  105653:	mov    0x50(%r13),%eax
  105657:	test   %eax,%eax
  105659:	jne    105c74 <hyperreal::computable::approximation::acos_positive_rational+0x6e4>
  10565f:	mov    (%r15),%r14
  105662:	mov    0x60(%r13),%eax
  105666:	mov    %rdx,%r15
  105669:	test   %al,%al
  10566b:	jns    10567d <hyperreal::computable::approximation::acos_positive_rational+0xed>
  10566d:	lea    0x40(%r13),%r15
  105671:	mov    0x50(%r13),%eax
  105675:	test   %eax,%eax
  105677:	jne    105c8a <hyperreal::computable::approximation::acos_positive_rational+0x6fa>
  10567d:	mov    (%r15),%rcx
  105680:	mov    0x20(%r14),%rax
  105684:	cmp    0x38(%rcx),%rax
  105688:	jne    1056ad <hyperreal::computable::approximation::acos_positive_rational+0x11d>
  10568a:	mov    0x30(%rcx),%rsi
  10568e:	mov    0x18(%r14),%rdi
  105692:	shl    $0x3,%rax
  105696:	mov    %rdx,%r14
  105699:	mov    %rax,%rdx
  10569c:	call   *0x8151e(%rip)        # 186bc0 <bcmp@GLIBC_2.2.5>
  1056a2:	mov    %r14,%rdx
  1056a5:	test   %eax,%eax
  1056a7:	je     10578e <hyperreal::computable::approximation::acos_positive_rational+0x1fe>
  1056ad:	test   %ebp,%ebp
  1056af:	jle    1056ec <hyperreal::computable::approximation::acos_positive_rational+0x15c>
  1056b1:	cmp    $0x1,%ebp
  1056b4:	jne    105764 <hyperreal::computable::approximation::acos_positive_rational+0x1d4>
  1056ba:	call   *0x813c0(%rip)        # 186a80 <_DYNAMIC+0x2b0>
  1056c0:	mov    $0x8,%edi
  1056c5:	mov    $0x8,%esi
  1056ca:	call   *0x813b8(%rip)        # 186a88 <_DYNAMIC+0x2b8>
  1056d0:	test   %rax,%rax
  1056d3:	je     105cbb <hyperreal::computable::approximation::acos_positive_rational+0x72b>
  1056d9:	movq   $0x1,(%rax)
  1056e0:	mov    $0x1,%edx
  1056e5:	mov    $0x2,%cl
  1056e7:	jmp    10576d <hyperreal::computable::approximation::acos_positive_rational+0x1dd>
  1056ec:	mov    0x60(%r13),%eax
  1056f0:	mov    %rdx,%r15
  1056f3:	test   %al,%al
  1056f5:	jns    105707 <hyperreal::computable::approximation::acos_positive_rational+0x177>
  1056f7:	lea    0x40(%r13),%r15
  1056fb:	mov    0x50(%r13),%eax
  1056ff:	test   %eax,%eax
  105701:	jne    105ccb <hyperreal::computable::approximation::acos_positive_rational+0x73b>
  105707:	mov    %r12,0x48(%rsp)
  10570c:	mov    (%r15),%r12
  10570f:	mov    0x60(%r13),%eax
  105713:	mov    %rdx,%rsi
  105716:	test   %al,%al
  105718:	jns    10572a <hyperreal::computable::approximation::acos_positive_rational+0x19a>
  10571a:	lea    0x40(%r13),%rsi
  10571e:	mov    0x50(%r13),%eax
  105722:	test   %eax,%eax
  105724:	jne    105ce1 <hyperreal::computable::approximation::acos_positive_rational+0x751>
  10572a:	mov    0x38(%r12),%r14
  10572f:	lea    0x0(,%r14,8),%r15
  105737:	mov    %r14,%rax
  10573a:	shr    $0x3d,%rax
  10573e:	setne  %al
  105741:	movabs $0x7ffffffffffffff8,%rcx
  10574b:	cmp    %rcx,%r15
  10574e:	seta   %cl
  105751:	or     %al,%cl
  105753:	je     1057af <hyperreal::computable::approximation::acos_positive_rational+0x21f>
  105755:	xor    %r12d,%r12d
  105758:	mov    %r12,%rdi
  10575b:	mov    %r15,%rsi
  10575e:	call   *0x81374(%rip)        # 186ad8 <_DYNAMIC+0x308>
  105764:	mov    $0x8,%eax
  105769:	mov    $0x1,%cl
  10576b:	xor    %edx,%edx
  10576d:	mov    %rdx,(%rbx)
  105770:	mov    %rax,0x8(%rbx)
  105774:	mov    %rdx,0x10(%rbx)
  105778:	mov    %cl,0x18(%rbx)
  10577b:	movl   $0x0,0x19(%rbx)
  105782:	movl   $0x0,0x1c(%rbx)
  105789:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  10578e:	cmpb   $0x0,0x64(%r13)
  105793:	je     1057fd <hyperreal::computable::approximation::acos_positive_rational+0x26d>
  105795:	movups -0xd0ccc(%rip),%xmm0        # 34ad0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067+0x10>
  10579c:	movups %xmm0,0x10(%rbx)
  1057a0:	movups -0xd0ce7(%rip),%xmm0        # 34ac0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067>
  1057a7:	movups %xmm0,(%rbx)
  1057aa:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  1057af:	mov    %rdx,0x68(%rsp)
  1057b4:	mov    (%rsi),%rdx
  1057b7:	mov    0x30(%r12),%rsi
  1057bc:	test   %r15,%r15
  1057bf:	je     10582f <hyperreal::computable::approximation::acos_positive_rational+0x29f>
  1057c1:	mov    %rsi,0x70(%rsp)
  1057c6:	mov    %rdx,0x18(%rsp)
  1057cb:	call   *0x812af(%rip)        # 186a80 <_DYNAMIC+0x2b0>
  1057d1:	mov    $0x8,%r12d
  1057d7:	mov    $0x8,%esi
  1057dc:	mov    %r15,%rdi
  1057df:	call   *0x812a3(%rip)        # 186a88 <_DYNAMIC+0x2b8>
  1057e5:	test   %rax,%rax
  1057e8:	je     105758 <hyperreal::computable::approximation::acos_positive_rational+0x1c8>
  1057ee:	mov    %r14,%r12
  1057f1:	mov    0x18(%rsp),%rdx
  1057f6:	mov    0x70(%rsp),%rsi
  1057fb:	jmp    105837 <hyperreal::computable::approximation::acos_positive_rational+0x2a7>
  1057fd:	call   *0x81265(%rip)        # 186a68 <_DYNAMIC+0x298>
  105803:	mov    %rax,0x20(%rsp)
  105808:	mov    %rdx,0x28(%rsp)
  10580d:	lea    0x20(%rsp),%rsi
  105812:	mov    %rbx,%rdi
  105815:	mov    %r12,%rdx
  105818:	mov    %ebp,%ecx
  10581a:	call   *0x81478(%rip)        # 186c98 <_DYNAMIC+0x4c8>
  105820:	lea    0x20(%rsp),%rdi
  105825:	call   e8b30 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  10582a:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  10582f:	mov    $0x8,%eax
  105834:	xor    %r12d,%r12d
  105837:	add    $0x10,%rdx
  10583b:	test   %r14,%r14
  10583e:	je     10585c <hyperreal::computable::approximation::acos_positive_rational+0x2cc>
  105840:	mov    %rax,%rdi
  105843:	mov    %rdx,0x18(%rsp)
  105848:	mov    %r15,%rdx
  10584b:	mov    %rax,%r15
  10584e:	call   *0x8123c(%rip)        # 186a90 <memcpy@GLIBC_2.14>
  105854:	mov    %r15,%rax
  105857:	mov    0x18(%rsp),%rdx
  10585c:	mov    %r12,0x20(%rsp)
  105861:	mov    %rax,0x28(%rsp)
  105866:	mov    %r14,0x30(%rsp)
  10586b:	lea    0x78(%rsp),%rdi
  105870:	lea    0x20(%rsp),%rsi
  105875:	call   *0x81535(%rip)        # 186db0 <_DYNAMIC+0x5e0>
  10587b:	mov    0x60(%r13),%eax
  10587f:	mov    0x68(%rsp),%rsi
  105884:	mov    %rsi,%r15
  105887:	test   %al,%al
  105889:	jns    10589b <hyperreal::computable::approximation::acos_positive_rational+0x30b>
  10588b:	lea    0x40(%r13),%r15
  10588f:	mov    0x50(%r13),%eax
  105893:	test   %eax,%eax
  105895:	jne    105d30 <hyperreal::computable::approximation::acos_positive_rational+0x7a0>
  10589b:	mov    (%r15),%r15
  10589e:	mov    0x60(%r13),%eax
  1058a2:	test   %al,%al
  1058a4:	js     1058ab <hyperreal::computable::approximation::acos_positive_rational+0x31b>
  1058a6:	mov    %rsi,%r12
  1058a9:	jmp    1058bb <hyperreal::computable::approximation::acos_positive_rational+0x32b>
  1058ab:	lea    0x40(%r13),%r12
  1058af:	mov    0x50(%r13),%eax
  1058b3:	test   %eax,%eax
  1058b5:	jne    105d42 <hyperreal::computable::approximation::acos_positive_rational+0x7b2>
  1058bb:	mov    (%r12),%r14
  1058bf:	mov    %r15,0x18(%rsp)
  1058c4:	mov    0x38(%r15),%r12
  1058c8:	mov    0x20(%r14),%r15
  1058cc:	cmp    %r15,%r12
  1058cf:	jae    105943 <hyperreal::computable::approximation::acos_positive_rational+0x3b3>
  1058d1:	mov    0x18(%r14),%rax
  1058d5:	mov    %rax,0x68(%rsp)
  1058da:	lea    0x0(,%r15,8),%r14
  1058e2:	call   *0x81198(%rip)        # 186a80 <_DYNAMIC+0x2b0>
  1058e8:	mov    $0x8,%esi
  1058ed:	mov    %r14,%rdi
  1058f0:	call   *0x81192(%rip)        # 186a88 <_DYNAMIC+0x2b8>
  1058f6:	test   %rax,%rax
  1058f9:	je     105e3f <hyperreal::computable::approximation::acos_positive_rational+0x8af>
  1058ff:	mov    0x18(%rsp),%r12
  105904:	add    $0x28,%r12
  105908:	mov    %r15,0x20(%rsp)
  10590d:	mov    %rax,0x28(%rsp)
  105912:	mov    %rax,%rdi
  105915:	mov    0x68(%rsp),%rsi
  10591a:	mov    %r14,%rdx
  10591d:	call   *0x8116d(%rip)        # 186a90 <memcpy@GLIBC_2.14>
  105923:	mov    %r15,0x30(%rsp)
  105928:	lea    0x90(%rsp),%rdi
  105930:	lea    0x20(%rsp),%rsi
  105935:	mov    %r12,%rdx
  105938:	call   *0x813ca(%rip)        # 186d08 <_DYNAMIC+0x538>
  10593e:	jmp    1059d2 <hyperreal::computable::approximation::acos_positive_rational+0x442>
  105943:	test   %r12,%r12
  105946:	je     10599d <hyperreal::computable::approximation::acos_positive_rational+0x40d>
  105948:	mov    0x18(%rsp),%rax
  10594d:	mov    0x30(%rax),%rax
  105951:	mov    %rax,0x18(%rsp)
  105956:	lea    0x0(,%r12,8),%r15
  10595e:	call   *0x8111c(%rip)        # 186a80 <_DYNAMIC+0x2b0>
  105964:	mov    $0x8,%esi
  105969:	mov    %r15,%rdi
  10596c:	call   *0x81116(%rip)        # 186a88 <_DYNAMIC+0x2b8>
  105972:	test   %rax,%rax
  105975:	je     105e44 <hyperreal::computable::approximation::acos_positive_rational+0x8b4>
  10597b:	mov    %r12,0x20(%rsp)
  105980:	mov    %rax,0x28(%rsp)
  105985:	mov    %rax,%rdi
  105988:	mov    0x18(%rsp),%rsi
  10598d:	mov    %r15,%rdx
  105990:	call   *0x810fa(%rip)        # 186a90 <memcpy@GLIBC_2.14>
  105996:	mov    %r12,0x30(%rsp)
  10599b:	jmp    1059b8 <hyperreal::computable::approximation::acos_positive_rational+0x428>
  10599d:	movq   $0x0,0x20(%rsp)
  1059a6:	movq   $0x8,0x28(%rsp)
  1059af:	movq   $0x0,0x30(%rsp)
  1059b8:	add    $0x10,%r14
  1059bc:	lea    0x90(%rsp),%rdi
  1059c4:	lea    0x20(%rsp),%rsi
  1059c9:	mov    %r14,%rdx
  1059cc:	call   *0x81336(%rip)        # 186d08 <_DYNAMIC+0x538>
  1059d2:	cmpb   $0x0,0x64(%r13)
  1059d7:	mov    0x90(%rsp),%rsi
  1059df:	mov    0x98(%rsp),%rdi
  1059e7:	mov    0x78(%rsp),%rdx
  1059ec:	mov    0x80(%rsp),%r8
  1059f4:	mov    %rdx,%r15
  1059f7:	cmove  %rsi,%r15
  1059fb:	mov    %r8,%r12
  1059fe:	cmove  %rdi,%r12
  105a02:	mov    0xa0(%rsp),%rax
  105a0a:	mov    0x88(%rsp),%r9
  105a12:	mov    %r9,%rcx
  105a15:	cmove  %rax,%rcx
  105a19:	cmove  %rdx,%rsi
  105a1d:	cmove  %r8,%rdi
  105a21:	cmove  %r9,%rax
  105a25:	test   %rax,%rax
  105a28:	je     105de7 <hyperreal::computable::approximation::acos_positive_rational+0x857>
  105a2e:	mov    %r15,0x20(%rsp)
  105a33:	mov    %r12,0x28(%rsp)
  105a38:	mov    %rcx,0x30(%rsp)
  105a3d:	mov    %rsi,0x50(%rsp)
  105a42:	mov    %rdi,0x58(%rsp)
  105a47:	mov    %rax,0x60(%rsp)
  105a4c:	cmp    $0x1,%rcx
  105a50:	mov    $0x2,%edi
  105a55:	sbb    $0x0,%edi
  105a58:	lea    0x20(%rsp),%rsi
  105a5d:	lea    0x50(%rsp),%rdx
  105a62:	call   d65a0 <<hyperreal::rational::arithmetic::Rational>::from_fraction_parts_reduced>
  105a67:	mov    %rax,0x40(%rsp)
  105a6c:	lea    0x20(%rsp),%rdi
  105a71:	mov    $0x1,%esi
  105a76:	mov    $0x4,%edx
  105a7b:	call   *0x8118f(%rip)        # 186c10 <_DYNAMIC+0x440>
  105a81:	mov    0x48(%rsp),%r12
  105a86:	cmpb   $0x1,0x20(%rsp)
  105a8b:	je     105cfd <hyperreal::computable::approximation::acos_positive_rational+0x76d>
  105a91:	mov    0x28(%rsp),%r14
  105a96:	mov    %r14,0x50(%rsp)
  105a9b:	lea    0x40(%rsp),%rdi
  105aa0:	lea    0x50(%rsp),%rsi
  105aa5:	call   *0x81175(%rip)        # 186c20 <_DYNAMIC+0x450>
  105aab:	cmp    $0xfe,%al
  105aad:	setne  %cl
  105ab0:	test   %al,%al
  105ab2:	setle  %dl
  105ab5:	lock decq (%r14)
  105ab9:	sete   %al
  105abc:	test   %dl,%cl
  105abe:	je     105b4a <hyperreal::computable::approximation::acos_positive_rational+0x5ba>
  105ac4:	test   %al,%al
  105ac6:	je     105ad3 <hyperreal::computable::approximation::acos_positive_rational+0x543>
  105ac8:	lea    0x50(%rsp),%rdi
  105acd:	call   *0x81025(%rip)        # 186af8 <_DYNAMIC+0x328>
  105ad3:	mov    %fs:0x0,%rax
  105adc:	lea    -0xd8(%rax),%rax
  105ae3:	cmpb   $0x1,0x98(%rax)
  105aea:	jne    105d4f <hyperreal::computable::approximation::acos_positive_rational+0x7bf>
  105af0:	cmpq   $0x0,(%rax)
  105af4:	jne    105d8d <hyperreal::computable::approximation::acos_positive_rational+0x7fd>
  105afa:	movq   $0xffffffffffffffff,(%rax)
  105b01:	cmpb   $0x0,0x90(%rax)
  105b08:	jne    105d9f <hyperreal::computable::approximation::acos_positive_rational+0x80f>
  105b0e:	xor    %ecx,%ecx
  105b10:	mov    %rcx,(%rax)
  105b13:	dec    %ebp
  105b15:	mov    (%r12),%rsi
  105b19:	lea    0x40(%rsp),%rdx
  105b1e:	mov    %rbx,%rdi
  105b21:	mov    %ebp,%ecx
  105b23:	xor    %r8d,%r8d
  105b26:	call   108a10 <hyperreal::computable::approximation::atan_sqrt_rational_small>
  105b2b:	mov    0x40(%rsp),%rax
  105b30:	lock decq (%rax)
  105b34:	jne    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105b3a:	lea    0x40(%rsp),%rdi
  105b3f:	call   *0x80fb3(%rip)        # 186af8 <_DYNAMIC+0x328>
  105b45:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105b4a:	test   %al,%al
  105b4c:	je     105b59 <hyperreal::computable::approximation::acos_positive_rational+0x5c9>
  105b4e:	lea    0x50(%rsp),%rdi
  105b53:	call   *0x80f9f(%rip)        # 186af8 <_DYNAMIC+0x328>
  105b59:	mov    0x40(%rsp),%rdi
  105b5e:	call   *0x80ed4(%rip)        # 186a38 <_DYNAMIC+0x268>
  105b64:	mov    %rax,%rdi
  105b67:	xor    %esi,%esi
  105b69:	call   *0x81171(%rip)        # 186ce0 <_DYNAMIC+0x510>
  105b6f:	mov    %rax,%rdi
  105b72:	mov    %rdx,%rsi
  105b75:	call   *0x811b5(%rip)        # 186d30 <_DYNAMIC+0x560>
  105b7b:	mov    %rax,%rdi
  105b7e:	mov    %rdx,%rsi
  105b81:	mov    $0x1,%edx
  105b86:	call   70d30 <<hyperreal::computable::node::Computable>::shift_left_with_work::<false>>
  105b8b:	test   %rax,%rax
  105b8e:	je     105d74 <hyperreal::computable::approximation::acos_positive_rational+0x7e4>
  105b94:	mov    %rax,%r15
  105b97:	mov    %rax,0x50(%rsp)
  105b9c:	mov    %rdx,0x58(%rsp)
  105ba1:	lea    0x20(%rsp),%rdi
  105ba6:	lea    0x50(%rsp),%rsi
  105bab:	mov    %r12,%rdx
  105bae:	mov    %ebp,%ecx
  105bb0:	call   *0x810e2(%rip)        # 186c98 <_DYNAMIC+0x4c8>
  105bb6:	mov    0x20(%rsp),%r12
  105bbb:	movzbl 0x28(%rsp),%ebp
  105bc0:	cmp    $0xffffffffffffffff,%r12
  105bc4:	je     105c13 <hyperreal::computable::approximation::acos_positive_rational+0x683>
  105bc6:	lea    0x58(%rsp),%r14
  105bcb:	mov    0x38(%rsp),%rax
  105bd0:	mov    %rax,0x18(%rbx)
  105bd4:	movups 0x29(%rsp),%xmm0
  105bd9:	movups %xmm0,0x9(%rbx)
  105bdd:	lock decq (%r15)
  105be1:	jne    105bee <hyperreal::computable::approximation::acos_positive_rational+0x65e>
  105be3:	lea    0x50(%rsp),%rdi
  105be8:	call   *0x80e32(%rip)        # 186a20 <_DYNAMIC+0x250>
  105bee:	mov    0x58(%rsp),%rax
  105bf3:	test   %rax,%rax
  105bf6:	je     105c07 <hyperreal::computable::approximation::acos_positive_rational+0x677>
  105bf8:	lock decq (%rax)
  105bfc:	jne    105c07 <hyperreal::computable::approximation::acos_positive_rational+0x677>
  105bfe:	mov    %r14,%rdi
  105c01:	call   *0x80e21(%rip)        # 186a28 <_DYNAMIC+0x258>
  105c07:	mov    %r12,(%rbx)
  105c0a:	mov    %bpl,0x8(%rbx)
  105c0e:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105c13:	mov    %bpl,0x8(%rbx)
  105c17:	movq   $0xffffffffffffffff,(%rbx)
  105c1e:	lea    0x50(%rsp),%rdi
  105c23:	call   e8b30 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105c28:	jmp    105632 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105c2d:	mov    %r15,%rdi
  105c30:	mov    %rdx,%rsi
  105c33:	mov    %rdx,%r14
  105c36:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105c3b:	mov    %r14,%rdx
  105c3e:	jmp    1055d5 <hyperreal::computable::approximation::acos_positive_rational+0x45>
  105c43:	mov    %r15,%rdi
  105c46:	mov    %rdx,%rsi
  105c49:	mov    %r12,0x48(%rsp)
  105c4e:	mov    %rdx,%r12
  105c51:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105c56:	mov    %r12,%rdx
  105c59:	mov    0x48(%rsp),%r12
  105c5e:	mov    (%r15),%rcx
  105c61:	mov    0x20(%r14),%rax
  105c65:	cmp    0x38(%rcx),%rax
  105c69:	je     105600 <hyperreal::computable::approximation::acos_positive_rational+0x70>
  105c6f:	jmp    105625 <hyperreal::computable::approximation::acos_positive_rational+0x95>
  105c74:	mov    %r15,%rdi
  105c77:	mov    %rdx,%rsi
  105c7a:	mov    %rdx,%r14
  105c7d:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105c82:	mov    %r14,%rdx
  105c85:	jmp    10565f <hyperreal::computable::approximation::acos_positive_rational+0xcf>
  105c8a:	mov    %r15,%rdi
  105c8d:	mov    %rdx,%rsi
  105c90:	mov    %r12,0x48(%rsp)
  105c95:	mov    %rdx,%r12
  105c98:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105c9d:	mov    %r12,%rdx
  105ca0:	mov    0x48(%rsp),%r12
  105ca5:	mov    (%r15),%rcx
  105ca8:	mov    0x20(%r14),%rax
  105cac:	cmp    0x38(%rcx),%rax
  105cb0:	je     10568a <hyperreal::computable::approximation::acos_positive_rational+0xfa>
  105cb6:	jmp    1056ad <hyperreal::computable::approximation::acos_positive_rational+0x11d>
  105cbb:	mov    $0x8,%edi
  105cc0:	mov    $0x8,%esi
  105cc5:	call   *0x80e4d(%rip)        # 186b18 <_DYNAMIC+0x348>
  105ccb:	mov    %r15,%rdi
  105cce:	mov    %rdx,%rsi
  105cd1:	mov    %rdx,%r14
  105cd4:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105cd9:	mov    %r14,%rdx
  105cdc:	jmp    105707 <hyperreal::computable::approximation::acos_positive_rational+0x177>
  105ce1:	mov    %rsi,%r15
  105ce4:	mov    %rsi,%rdi
  105ce7:	mov    %rdx,%rsi
  105cea:	mov    %rdx,%r14
  105ced:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105cf2:	mov    %r15,%rsi
  105cf5:	mov    %r14,%rdx
  105cf8:	jmp    10572a <hyperreal::computable::approximation::acos_positive_rational+0x19a>
  105cfd:	movzbl 0x21(%rsp),%eax
  105d02:	mov    %al,0x17(%rsp)
  105d06:	lea    -0xcd4c6(%rip),%rdi        # 38847 <anon.b304514175607a3d12c9b319a66702cc.180.llvm.12648000514137966701+0xec>
  105d0d:	lea    0x7d7f4(%rip),%rcx        # 183508 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.18232575825031054135+0x490>
  105d14:	lea    0x7ddbd(%rip),%r8        # 183ad8 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.18232575825031054135+0xa60>
  105d1b:	lea    0x17(%rsp),%rdx
  105d20:	mov    $0x13,%esi
  105d25:	call   *0x80d1d(%rip)        # 186a48 <_DYNAMIC+0x278>
  105d2b:	jmp    105e52 <hyperreal::computable::approximation::acos_positive_rational+0x8c2>
  105d30:	mov    %r15,%rdi
  105d33:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105d38:	mov    0x68(%rsp),%rsi
  105d3d:	jmp    10589b <hyperreal::computable::approximation::acos_positive_rational+0x30b>
  105d42:	mov    %r12,%rdi
  105d45:	call   12c9be <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105d4a:	jmp    1058bb <hyperreal::computable::approximation::acos_positive_rational+0x32b>
  105d4f:	mov    %rax,%rdi
  105d52:	xor    %esi,%esi
  105d54:	call   129a30 <<std::sys::thread_local::native::lazy::Storage<core::cell::RefCell<hyperreal::dispatch_trace::TraceState>, ()>>::get_or_init_slow::<hyperreal::dispatch_trace::TRACE_STATE::__rust_std_internal_init_fn>>
  105d59:	test   %rax,%rax
  105d5c:	jne    105af0 <hyperreal::computable::approximation::acos_positive_rational+0x560>
  105d62:	lea    0x7dfe7(%rip),%rdi        # 183d50 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
  105d69:	call   *0x80e21(%rip)        # 186b90 <_DYNAMIC+0x3c0>
  105d6f:	jmp    105e52 <hyperreal::computable::approximation::acos_positive_rational+0x8c2>
  105d74:	lea    -0xd08fb(%rip),%rdi        # 35480 <anon.7b75c566c6a805bace2b01974d435009.248.llvm.16730574089061123067>
  105d7b:	lea    0x7cc5e(%rip),%rdx        # 1829e0 <anon.7b75c566c6a805bace2b01974d435009.249.llvm.16730574089061123067>
  105d82:	mov    $0x21,%esi
  105d87:	call   *0x80ee3(%rip)        # 186c70 <_DYNAMIC+0x4a0>
  105d8d:	lea    0x7e054(%rip),%rdi        # 183de8 <anon.953f3db8f5a996f677fb6a53175227a4.21.llvm.13083929455518204517>
  105d94:	call   *0x80e5e(%rip)        # 186bf8 <_DYNAMIC+0x428>
  105d9a:	jmp    105e52 <hyperreal::computable::approximation::acos_positive_rational+0x8c2>
  105d9f:	mov    %rax,%r14
  105da2:	lea    0x8(%rax),%rdi
  105da6:	movq   $0x17,(%rsp)
  105dae:	lea    -0xcd597(%rip),%rsi        # 3881e <anon.b304514175607a3d12c9b319a66702cc.180.llvm.12648000514137966701+0xc3>
  105db5:	lea    -0xd2e98(%rip),%rcx        # 32f24 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.15839081146404610231+0x20c>
  105dbc:	lea    -0xcd569(%rip),%r9        # 3885a <anon.b304514175607a3d12c9b319a66702cc.180.llvm.12648000514137966701+0xff>
  105dc3:	mov    $0x11,%edx
  105dc8:	mov    $0x4,%r8d
  105dce:	call   *0x80e14(%rip)        # 186be8 <_DYNAMIC+0x418>
  105dd4:	mov    %r14,%rax
  105dd7:	mov    (%r14),%rcx
  105dda:	inc    %rcx
  105ddd:	mov    0x48(%rsp),%r12
  105de2:	jmp    105b10 <hyperreal::computable::approximation::acos_positive_rational+0x580>
  105de7:	test   %rsi,%rsi
  105dea:	je     105dfb <hyperreal::computable::approximation::acos_positive_rational+0x86b>
  105dec:	shl    $0x3,%rsi
  105df0:	mov    $0x8,%edx
  105df5:	call   *0x80c15(%rip)        # 186a10 <_DYNAMIC+0x240>
  105dfb:	test   %r15,%r15
  105dfe:	je     105e15 <hyperreal::computable::approximation::acos_positive_rational+0x885>
  105e00:	shl    $0x3,%r15
  105e04:	mov    $0x8,%edx
  105e09:	mov    %r12,%rdi
  105e0c:	mov    %r15,%rsi
  105e0f:	call   *0x80bfb(%rip)        # 186a10 <_DYNAMIC+0x240>
  105e15:	movb   $0x2,0x20(%rsp)
  105e1a:	lea    -0xcd452(%rip),%rdi        # 389cf <anon.b304514175607a3d12c9b319a66702cc.180.llvm.12648000514137966701+0x274>
  105e21:	lea    0x7d6e0(%rip),%rcx        # 183508 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.18232575825031054135+0x490>
  105e28:	lea    0x7dc91(%rip),%r8        # 183ac0 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.18232575825031054135+0xa48>
  105e2f:	lea    0x20(%rsp),%rdx
  105e34:	mov    $0x40,%esi
  105e39:	call   *0x80c09(%rip)        # 186a48 <_DYNAMIC+0x278>
  105e3f:	mov    %r14,%rsi
  105e42:	jmp    105e47 <hyperreal::computable::approximation::acos_positive_rational+0x8b7>
  105e44:	mov    %r15,%rsi
  105e47:	mov    $0x8,%edi
  105e4c:	call   *0x80c86(%rip)        # 186ad8 <_DYNAMIC+0x308>
  105e52:	ud2
  105e54:	mov    %rax,%rbx
  105e57:	incq   (%r14)
  105e5a:	jmp    105eee <hyperreal::computable::approximation::acos_positive_rational+0x95e>
  105e5f:	mov    %rax,%rbx
  105e62:	mov    0x58(%rsp),%rax
  105e67:	test   %rax,%rax
  105e6a:	je     105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105e70:	lock decq (%rax)
  105e74:	jne    105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105e7a:	mov    %r14,%rdi
  105e7d:	call   *0x80ba5(%rip)        # 186a28 <_DYNAMIC+0x258>
  105e83:	jmp    105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105e85:	call   *0x80b8d(%rip)        # 186a18 <_DYNAMIC+0x248>
  105e8b:	mov    %rax,%rbx
  105e8e:	lea    0x20(%rsp),%rdi
  105e93:	call   e8b30 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105e98:	jmp    105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105e9a:	mov    %rax,%rbx
  105e9d:	lea    0x50(%rsp),%rdi
  105ea2:	call   e8b30 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105ea7:	jmp    105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105ea9:	mov    %rax,%rbx
  105eac:	lock decq (%r14)
  105eb0:	jne    105eee <hyperreal::computable::approximation::acos_positive_rational+0x95e>
  105eb2:	lea    0x50(%rsp),%rdi
  105eb7:	call   *0x80c3b(%rip)        # 186af8 <_DYNAMIC+0x328>
  105ebd:	jmp    105eee <hyperreal::computable::approximation::acos_positive_rational+0x95e>
  105ebf:	mov    %rax,%rbx
  105ec2:	mov    0x78(%rsp),%rsi
  105ec7:	test   %rsi,%rsi
  105eca:	je     105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105ecc:	mov    0x80(%rsp),%rdi
  105ed4:	shl    $0x3,%rsi
  105ed8:	mov    $0x8,%edx
  105edd:	call   *0x80b2d(%rip)        # 186a10 <_DYNAMIC+0x240>
  105ee3:	mov    %rbx,%rdi
  105ee6:	call   180f40 <_Unwind_Resume@plt>
  105eeb:	mov    %rax,%rbx
  105eee:	mov    0x40(%rsp),%rax
  105ef3:	lock decq (%rax)
  105ef7:	jne    105f04 <hyperreal::computable::approximation::acos_positive_rational+0x974>
  105ef9:	lea    0x40(%rsp),%rdi
  105efe:	call   *0x80bf4(%rip)        # 186af8 <_DYNAMIC+0x328>
  105f04:	mov    %rbx,%rdi
  105f07:	call   180f40 <_Unwind_Resume@plt>
  105f0c:	call   *0x80b06(%rip)        # 186a18 <_DYNAMIC+0x248>
