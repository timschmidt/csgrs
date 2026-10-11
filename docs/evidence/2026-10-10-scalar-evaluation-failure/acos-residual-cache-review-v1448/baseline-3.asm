
target/typed-api-prototype-v1418-acos-warm-baseline-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

0000000000105510 <hyperreal::computable::approximation::acos_positive_rational>:
  105510:	push   %rbp
  105511:	push   %r15
  105513:	push   %r14
  105515:	push   %r13
  105517:	push   %r12
  105519:	push   %rbx
  10551a:	sub    $0x58,%rsp
  10551e:	mov    %rdi,%rbx
  105521:	cmp    $0xffefffff,%ecx
  105527:	jg     10552f <hyperreal::computable::approximation::acos_positive_rational+0x1f>
  105529:	movb   $0xc,0x8(%rbx)
  10552d:	jmp    1055ab <hyperreal::computable::approximation::acos_positive_rational+0x9b>
  10552f:	mov    %ecx,%ebp
  105531:	mov    %rsi,%r14
  105534:	mov    (%rdx),%r15
  105537:	mov    0x60(%r15),%eax
  10553b:	mov    %rdx,%r12
  10553e:	test   %al,%al
  105540:	jns    105552 <hyperreal::computable::approximation::acos_positive_rational+0x42>
  105542:	lea    0x40(%r15),%r12
  105546:	mov    0x50(%r15),%eax
  10554a:	test   %eax,%eax
  10554c:	jne    1055fe <hyperreal::computable::approximation::acos_positive_rational+0xee>
  105552:	mov    (%r12),%r13
  105556:	mov    0x60(%r15),%eax
  10555a:	mov    %rdx,%r12
  10555d:	test   %al,%al
  10555f:	jns    105571 <hyperreal::computable::approximation::acos_positive_rational+0x61>
  105561:	lea    0x40(%r15),%r12
  105565:	mov    0x50(%r15),%eax
  105569:	test   %eax,%eax
  10556b:	jne    105614 <hyperreal::computable::approximation::acos_positive_rational+0x104>
  105571:	mov    (%r12),%rcx
  105575:	mov    0x20(%r13),%rax
  105579:	cmp    0x38(%rcx),%rax
  10557d:	jne    1055a5 <hyperreal::computable::approximation::acos_positive_rational+0x95>
  10557f:	mov    0x30(%rcx),%rcx
  105583:	mov    0x18(%r13),%rsi
  105587:	shl    $0x3,%rax
  10558b:	nopl   0x0(%rax,%rax,1)
  105590:	test   %rax,%rax
  105593:	je     1055c1 <hyperreal::computable::approximation::acos_positive_rational+0xb1>
  105595:	mov    -0x8(%rcx,%rax,1),%rdi
  10559a:	cmp    %rdi,-0x8(%rsi,%rax,1)
  10559f:	lea    -0x8(%rax),%rax
  1055a3:	je     105590 <hyperreal::computable::approximation::acos_positive_rational+0x80>
  1055a5:	jbe    1055c1 <hyperreal::computable::approximation::acos_positive_rational+0xb1>
  1055a7:	movb   $0x5,0x8(%rbx)
  1055ab:	movq   $0xffffffffffffffff,(%rbx)
  1055b2:	add    $0x58,%rsp
  1055b6:	pop    %rbx
  1055b7:	pop    %r12
  1055b9:	pop    %r13
  1055bb:	pop    %r14
  1055bd:	pop    %r15
  1055bf:	pop    %rbp
  1055c0:	ret
  1055c1:	mov    0x60(%r15),%eax
  1055c5:	mov    %rdx,%r12
  1055c8:	test   %al,%al
  1055ca:	jns    1055d8 <hyperreal::computable::approximation::acos_positive_rational+0xc8>
  1055cc:	lea    0x40(%r15),%r12
  1055d0:	mov    0x50(%r15),%eax
  1055d4:	test   %eax,%eax
  1055d6:	jne    105640 <hyperreal::computable::approximation::acos_positive_rational+0x130>
  1055d8:	mov    (%r12),%r13
  1055dc:	mov    0x60(%r15),%eax
  1055e0:	test   %al,%al
  1055e2:	js     10565d <hyperreal::computable::approximation::acos_positive_rational+0x14d>
  1055e4:	mov    %rdx,%r12
  1055e7:	mov    (%r12),%rax
  1055eb:	mov    0x20(%r13),%rdx
  1055ef:	cmp    0x38(%rax),%rdx
  1055f3:	je     10567b <hyperreal::computable::approximation::acos_positive_rational+0x16b>
  1055f9:	jmp    105695 <hyperreal::computable::approximation::acos_positive_rational+0x185>
  1055fe:	mov    %r12,%rdi
  105601:	mov    %rdx,%rsi
  105604:	mov    %rdx,%r13
  105607:	call   12c58e <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  10560c:	mov    %r13,%rdx
  10560f:	jmp    105552 <hyperreal::computable::approximation::acos_positive_rational+0x42>
  105614:	mov    %r12,%rdi
  105617:	mov    %rdx,%rsi
  10561a:	mov    %rdx,0x50(%rsp)
  10561f:	call   12c58e <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105624:	mov    0x50(%rsp),%rdx
  105629:	mov    (%r12),%rcx
  10562d:	mov    0x20(%r13),%rax
  105631:	cmp    0x38(%rcx),%rax
  105635:	je     10557f <hyperreal::computable::approximation::acos_positive_rational+0x6f>
  10563b:	jmp    1055a5 <hyperreal::computable::approximation::acos_positive_rational+0x95>
  105640:	mov    %r12,%rdi
  105643:	mov    %rdx,%rsi
  105646:	mov    %rdx,%r13
  105649:	call   12c58e <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  10564e:	mov    %r13,%rdx
  105651:	mov    (%r12),%r13
  105655:	mov    0x60(%r15),%eax
  105659:	test   %al,%al
  10565b:	jns    1055e4 <hyperreal::computable::approximation::acos_positive_rational+0xd4>
  10565d:	lea    0x40(%r15),%r12
  105661:	mov    0x50(%r15),%eax
  105665:	test   %eax,%eax
  105667:	jne    10595e <hyperreal::computable::approximation::acos_positive_rational+0x44e>
  10566d:	mov    (%r12),%rax
  105671:	mov    0x20(%r13),%rdx
  105675:	cmp    0x38(%rax),%rdx
  105679:	jne    105695 <hyperreal::computable::approximation::acos_positive_rational+0x185>
  10567b:	mov    0x30(%rax),%rsi
  10567f:	mov    0x18(%r13),%rdi
  105683:	shl    $0x3,%rdx
  105687:	call   *0x810eb(%rip)        # 186778 <bcmp@GLIBC_2.2.5>
  10568d:	test   %eax,%eax
  10568f:	je     105824 <hyperreal::computable::approximation::acos_positive_rational+0x314>
  105695:	test   %ebp,%ebp
  105697:	jle    1056d4 <hyperreal::computable::approximation::acos_positive_rational+0x1c4>
  105699:	cmp    $0x1,%ebp
  10569c:	jne    1057fa <hyperreal::computable::approximation::acos_positive_rational+0x2ea>
  1056a2:	call   *0x80f90(%rip)        # 186638 <_DYNAMIC+0x2b0>
  1056a8:	mov    $0x8,%edi
  1056ad:	mov    $0x8,%esi
  1056b2:	call   *0x80f88(%rip)        # 186640 <_DYNAMIC+0x2b8>
  1056b8:	test   %rax,%rax
  1056bb:	je     1059b0 <hyperreal::computable::approximation::acos_positive_rational+0x4a0>
  1056c1:	movq   $0x1,(%rax)
  1056c8:	mov    $0x1,%edx
  1056cd:	mov    $0x2,%cl
  1056cf:	jmp    105803 <hyperreal::computable::approximation::acos_positive_rational+0x2f3>
  1056d4:	call   *0x810ce(%rip)        # 1867a8 <_DYNAMIC+0x420>
  1056da:	lock incq (%r15)
  1056de:	jle    105a08 <hyperreal::computable::approximation::acos_positive_rational+0x4f8>
  1056e4:	mov    %rax,%rdi
  1056e7:	mov    %r15,%rsi
  1056ea:	call   e3620 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Sub>::sub>
  1056ef:	mov    %rax,%r12
  1056f2:	mov    %rax,0x30(%rsp)
  1056f7:	call   *0x810ab(%rip)        # 1867a8 <_DYNAMIC+0x420>
  1056fd:	lock incq (%r15)
  105701:	jle    105a08 <hyperreal::computable::approximation::acos_positive_rational+0x4f8>
  105707:	mov    %rax,%rdi
  10570a:	mov    %r15,%rsi
  10570d:	call   e27e0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
  105712:	mov    %r12,%rdi
  105715:	mov    %rax,%rsi
  105718:	call   e5460 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Div>::div>
  10571d:	mov    %rax,0x18(%rsp)
  105722:	lea    0x30(%rsp),%rdi
  105727:	mov    $0x1,%esi
  10572c:	mov    $0x4,%edx
  105731:	call   *0x81091(%rip)        # 1867c8 <_DYNAMIC+0x440>
  105737:	cmpb   $0x1,0x30(%rsp)
  10573c:	je     105980 <hyperreal::computable::approximation::acos_positive_rational+0x470>
  105742:	mov    0x38(%rsp),%r15
  105747:	mov    %r15,0x20(%rsp)
  10574c:	lea    0x18(%rsp),%rdi
  105751:	lea    0x20(%rsp),%rsi
  105756:	call   *0x8107c(%rip)        # 1867d8 <_DYNAMIC+0x450>
  10575c:	cmp    $0xfe,%al
  10575e:	setne  %cl
  105761:	test   %al,%al
  105763:	setle  %dl
  105766:	lock decq (%r15)
  10576a:	sete   %al
  10576d:	test   %dl,%cl
  10576f:	je     105849 <hyperreal::computable::approximation::acos_positive_rational+0x339>
  105775:	test   %al,%al
  105777:	je     105784 <hyperreal::computable::approximation::acos_positive_rational+0x274>
  105779:	lea    0x20(%rsp),%rdi
  10577e:	call   *0x80f2c(%rip)        # 1866b0 <_DYNAMIC+0x328>
  105784:	mov    %fs:0x0,%rax
  10578d:	lea    -0xd8(%rax),%rax
  105794:	cmpb   $0x1,0x98(%rax)
  10579b:	jne    1059c0 <hyperreal::computable::approximation::acos_positive_rational+0x4b0>
  1057a1:	cmpq   $0x0,(%rax)
  1057a5:	jne    1059fb <hyperreal::computable::approximation::acos_positive_rational+0x4eb>
  1057ab:	movq   $0xffffffffffffffff,(%rax)
  1057b2:	cmpb   $0x0,0x90(%rax)
  1057b9:	jne    105a0a <hyperreal::computable::approximation::acos_positive_rational+0x4fa>
  1057bf:	xor    %ecx,%ecx
  1057c1:	mov    %rcx,(%rax)
  1057c4:	dec    %ebp
  1057c6:	mov    (%r14),%rsi
  1057c9:	lea    0x18(%rsp),%rdx
  1057ce:	mov    %rbx,%rdi
  1057d1:	mov    %ebp,%ecx
  1057d3:	xor    %r8d,%r8d
  1057d6:	call   1085e0 <hyperreal::computable::approximation::atan_sqrt_rational_small>
  1057db:	mov    0x18(%rsp),%rax
  1057e0:	lock decq (%rax)
  1057e4:	jne    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  1057ea:	lea    0x18(%rsp),%rdi
  1057ef:	call   *0x80ebb(%rip)        # 1866b0 <_DYNAMIC+0x328>
  1057f5:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  1057fa:	mov    $0x8,%eax
  1057ff:	mov    $0x1,%cl
  105801:	xor    %edx,%edx
  105803:	mov    %rdx,(%rbx)
  105806:	mov    %rax,0x8(%rbx)
  10580a:	mov    %rdx,0x10(%rbx)
  10580e:	mov    %cl,0x18(%rbx)
  105811:	movl   $0x0,0x19(%rbx)
  105818:	movl   $0x0,0x1c(%rbx)
  10581f:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105824:	cmpb   $0x0,0x64(%r15)
  105829:	je     105912 <hyperreal::computable::approximation::acos_positive_rational+0x402>
  10582f:	movups -0xd0db6(%rip),%xmm0        # 34a80 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061+0x10>
  105836:	movups %xmm0,0x10(%rbx)
  10583a:	movups -0xd0dd1(%rip),%xmm0        # 34a70 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061>
  105841:	movups %xmm0,(%rbx)
  105844:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105849:	test   %al,%al
  10584b:	je     105858 <hyperreal::computable::approximation::acos_positive_rational+0x348>
  10584d:	lea    0x20(%rsp),%rdi
  105852:	call   *0x80e58(%rip)        # 1866b0 <_DYNAMIC+0x328>
  105858:	mov    0x18(%rsp),%rdi
  10585d:	call   *0x80d8d(%rip)        # 1865f0 <_DYNAMIC+0x268>
  105863:	mov    %rax,%rdi
  105866:	xor    %esi,%esi
  105868:	call   *0x8102a(%rip)        # 186898 <_DYNAMIC+0x510>
  10586e:	mov    %rax,%rdi
  105871:	mov    %rdx,%rsi
  105874:	call   *0x8106e(%rip)        # 1868e8 <_DYNAMIC+0x560>
  10587a:	mov    %rax,%rdi
  10587d:	mov    %rdx,%rsi
  105880:	mov    $0x1,%edx
  105885:	call   70cb0 <<hyperreal::computable::node::Computable>::shift_left_with_work::<false>>
  10588a:	test   %rax,%rax
  10588d:	je     1059e2 <hyperreal::computable::approximation::acos_positive_rational+0x4d2>
  105893:	mov    %rax,%r15
  105896:	mov    %rax,0x20(%rsp)
  10589b:	mov    %rdx,0x28(%rsp)
  1058a0:	lea    0x30(%rsp),%rdi
  1058a5:	lea    0x20(%rsp),%rsi
  1058aa:	mov    %r14,%rdx
  1058ad:	mov    %ebp,%ecx
  1058af:	call   *0x80f9b(%rip)        # 186850 <_DYNAMIC+0x4c8>
  1058b5:	mov    0x30(%rsp),%r12
  1058ba:	movzbl 0x38(%rsp),%ebp
  1058bf:	cmp    $0xffffffffffffffff,%r12
  1058c3:	je     105944 <hyperreal::computable::approximation::acos_positive_rational+0x434>
  1058c5:	lea    0x28(%rsp),%r14
  1058ca:	mov    0x48(%rsp),%rax
  1058cf:	mov    %rax,0x18(%rbx)
  1058d3:	movups 0x39(%rsp),%xmm0
  1058d8:	movups %xmm0,0x9(%rbx)
  1058dc:	lock decq (%r15)
  1058e0:	jne    1058ed <hyperreal::computable::approximation::acos_positive_rational+0x3dd>
  1058e2:	lea    0x20(%rsp),%rdi
  1058e7:	call   *0x80ceb(%rip)        # 1865d8 <_DYNAMIC+0x250>
  1058ed:	mov    0x28(%rsp),%rax
  1058f2:	test   %rax,%rax
  1058f5:	je     105906 <hyperreal::computable::approximation::acos_positive_rational+0x3f6>
  1058f7:	lock decq (%rax)
  1058fb:	jne    105906 <hyperreal::computable::approximation::acos_positive_rational+0x3f6>
  1058fd:	mov    %r14,%rdi
  105900:	call   *0x80cda(%rip)        # 1865e0 <_DYNAMIC+0x258>
  105906:	mov    %r12,(%rbx)
  105909:	mov    %bpl,0x8(%rbx)
  10590d:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105912:	call   *0x80d08(%rip)        # 186620 <_DYNAMIC+0x298>
  105918:	mov    %rax,0x30(%rsp)
  10591d:	mov    %rdx,0x38(%rsp)
  105922:	lea    0x30(%rsp),%rsi
  105927:	mov    %rbx,%rdi
  10592a:	mov    %r14,%rdx
  10592d:	mov    %ebp,%ecx
  10592f:	call   *0x80f1b(%rip)        # 186850 <_DYNAMIC+0x4c8>
  105935:	lea    0x30(%rsp),%rdi
  10593a:	call   e8ab0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  10593f:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  105944:	mov    %bpl,0x8(%rbx)
  105948:	movq   $0xffffffffffffffff,(%rbx)
  10594f:	lea    0x20(%rsp),%rdi
  105954:	call   e8ab0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105959:	jmp    1055b2 <hyperreal::computable::approximation::acos_positive_rational+0xa2>
  10595e:	mov    %r12,%rdi
  105961:	mov    %rdx,%rsi
  105964:	call   12c58e <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
  105969:	mov    (%r12),%rax
  10596d:	mov    0x20(%r13),%rdx
  105971:	cmp    0x38(%rax),%rdx
  105975:	je     10567b <hyperreal::computable::approximation::acos_positive_rational+0x16b>
  10597b:	jmp    105695 <hyperreal::computable::approximation::acos_positive_rational+0x185>
  105980:	movzbl 0x31(%rsp),%eax
  105985:	mov    %al,0x17(%rsp)
  105989:	lea    -0xcd199(%rip),%rdi        # 387f7 <anon.b304514175607a3d12c9b319a66702cc.180.llvm.2928830664922062821+0xec>
  105990:	lea    0x7d741(%rip),%rcx        # 1830d8 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.11764028182252023629+0x490>
  105997:	lea    0x7dcf2(%rip),%r8        # 183690 <anon.cc8a35c2c4be5681ab7929d48bff7af3.227.llvm.11764028182252023629+0xa48>
  10599e:	lea    0x17(%rsp),%rdx
  1059a3:	mov    $0x13,%esi
  1059a8:	call   *0x80c52(%rip)        # 186600 <_DYNAMIC+0x278>
  1059ae:	jmp    105a08 <hyperreal::computable::approximation::acos_positive_rational+0x4f8>
  1059b0:	mov    $0x8,%edi
  1059b5:	mov    $0x8,%esi
  1059ba:	call   *0x80d10(%rip)        # 1866d0 <_DYNAMIC+0x348>
  1059c0:	mov    %rax,%rdi
  1059c3:	xor    %esi,%esi
  1059c5:	call   129600 <<std::sys::thread_local::native::lazy::Storage<core::cell::RefCell<hyperreal::dispatch_trace::TraceState>, ()>>::get_or_init_slow::<hyperreal::dispatch_trace::TRACE_STATE::__rust_std_internal_init_fn>>
  1059ca:	test   %rax,%rax
  1059cd:	jne    1057a1 <hyperreal::computable::approximation::acos_positive_rational+0x291>
  1059d3:	lea    0x7df2e(%rip),%rdi        # 183908 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
  1059da:	call   *0x80d68(%rip)        # 186748 <_DYNAMIC+0x3c0>
  1059e0:	jmp    105a08 <hyperreal::computable::approximation::acos_positive_rational+0x4f8>
  1059e2:	lea    -0xd05b9(%rip),%rdi        # 35430 <anon.7b75c566c6a805bace2b01974d435009.248.llvm.17861709745643738061>
  1059e9:	lea    0x7cbc0(%rip),%rdx        # 1825b0 <anon.7b75c566c6a805bace2b01974d435009.249.llvm.17861709745643738061>
  1059f0:	mov    $0x21,%esi
  1059f5:	call   *0x80e2d(%rip)        # 186828 <_DYNAMIC+0x4a0>
  1059fb:	lea    0x7df9e(%rip),%rdi        # 1839a0 <anon.953f3db8f5a996f677fb6a53175227a4.21.llvm.13083929455518204517>
  105a02:	call   *0x80da8(%rip)        # 1867b0 <_DYNAMIC+0x428>
  105a08:	ud2
  105a0a:	mov    %rax,%r15
  105a0d:	lea    0x8(%rax),%rdi
  105a11:	movq   $0x17,(%rsp)
  105a19:	lea    -0xcd252(%rip),%rsi        # 387ce <anon.b304514175607a3d12c9b319a66702cc.180.llvm.2928830664922062821+0xc3>
  105a20:	lea    -0xd2b53(%rip),%rcx        # 32ed4 <anon.c3a302fb892fb1604dd2767d56b20e73.37.llvm.8018543666394721898+0x20c>
  105a27:	lea    -0xcd224(%rip),%r9        # 3880a <anon.b304514175607a3d12c9b319a66702cc.180.llvm.2928830664922062821+0xff>
  105a2e:	mov    $0x11,%edx
  105a33:	mov    $0x4,%r8d
  105a39:	call   *0x80d61(%rip)        # 1867a0 <_DYNAMIC+0x418>
  105a3f:	mov    %r15,%rax
  105a42:	mov    (%r15),%rcx
  105a45:	inc    %rcx
  105a48:	jmp    1057c1 <hyperreal::computable::approximation::acos_positive_rational+0x2b1>
  105a4d:	mov    %rax,%rbx
  105a50:	incq   (%r15)
  105a53:	jmp    105ac9 <hyperreal::computable::approximation::acos_positive_rational+0x5b9>
  105a55:	mov    %rax,%rbx
  105a58:	mov    0x28(%rsp),%rax
  105a5d:	test   %rax,%rax
  105a60:	je     105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105a62:	lock decq (%rax)
  105a66:	jne    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105a68:	mov    %r14,%rdi
  105a6b:	call   *0x80b6f(%rip)        # 1865e0 <_DYNAMIC+0x258>
  105a71:	jmp    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105a73:	call   *0x80b57(%rip)        # 1865d0 <_DYNAMIC+0x248>
  105a79:	mov    %rax,%rbx
  105a7c:	lea    0x30(%rsp),%rdi
  105a81:	call   e8ab0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105a86:	jmp    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105a88:	mov    %rax,%rbx
  105a8b:	lea    0x20(%rsp),%rdi
  105a90:	call   e8ab0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
  105a95:	jmp    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105a97:	mov    %rax,%rbx
  105a9a:	lock decq (%r15)
  105a9e:	jne    105ac9 <hyperreal::computable::approximation::acos_positive_rational+0x5b9>
  105aa0:	lea    0x20(%rsp),%rdi
  105aa5:	call   *0x80c05(%rip)        # 1866b0 <_DYNAMIC+0x328>
  105aab:	jmp    105ac9 <hyperreal::computable::approximation::acos_positive_rational+0x5b9>
  105aad:	jmp    105aaf <hyperreal::computable::approximation::acos_positive_rational+0x59f>
  105aaf:	mov    %rax,%rbx
  105ab2:	lock decq (%r12)
  105ab7:	jne    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105ab9:	lea    0x30(%rsp),%rdi
  105abe:	call   *0x80bec(%rip)        # 1866b0 <_DYNAMIC+0x328>
  105ac4:	jmp    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105ac6:	mov    %rax,%rbx
  105ac9:	mov    0x18(%rsp),%rax
  105ace:	lock decq (%rax)
  105ad2:	jne    105adf <hyperreal::computable::approximation::acos_positive_rational+0x5cf>
  105ad4:	lea    0x18(%rsp),%rdi
  105ad9:	call   *0x80bd1(%rip)        # 1866b0 <_DYNAMIC+0x328>
  105adf:	mov    %rbx,%rdi
  105ae2:	call   180b10 <_Unwind_Resume@plt>
  105ae7:	call   *0x80ae3(%rip)        # 1865d0 <_DYNAMIC+0x248>
