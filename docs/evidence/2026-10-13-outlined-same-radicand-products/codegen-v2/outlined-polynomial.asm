
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/outlined-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cf780 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply>:
   cf780:	55                   	push   %rbp
   cf781:	41 57                	push   %r15
   cf783:	41 56                	push   %r14
   cf785:	41 55                	push   %r13
   cf787:	41 54                	push   %r12
   cf789:	53                   	push   %rbx
   cf78a:	48 81 ec d8 01 00 00 	sub    $0x1d8,%rsp
   cf791:	49 89 fe             	mov    %rdi,%r14
   cf794:	48 c7 44 24 40 00 00 	movq   $0x0,0x40(%rsp)
   cf79b:	00 00 
   cf79d:	48 c7 44 24 48 08 00 	movq   $0x8,0x48(%rsp)
   cf7a4:	00 00 
   cf7a6:	48 c7 44 24 50 00 00 	movq   $0x0,0x50(%rsp)
   cf7ad:	00 00 
   cf7af:	48 8b 42 08          	mov    0x8(%rdx),%rax
   cf7b3:	48 89 04 24          	mov    %rax,(%rsp)
   cf7b7:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cf7bc:	4c 8b 62 10          	mov    0x10(%rdx),%r12
   cf7c0:	49 83 fc 01          	cmp    $0x1,%r12
   cf7c4:	0f 85 b8 01 00 00    	jne    cf982 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x202>
   cf7ca:	48 8b 06             	mov    (%rsi),%rax
   cf7cd:	48 8b 5e 08          	mov    0x8(%rsi),%rbx
   cf7d1:	48 8b 4e 10          	mov    0x10(%rsi),%rcx
   cf7d5:	49 89 cd             	mov    %rcx,%r13
   cf7d8:	49 c1 e5 06          	shl    $0x6,%r13
   cf7dc:	4a 8d 14 2b          	lea    (%rbx,%r13,1),%rdx
   cf7e0:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cf7e5:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf7ea:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cf7ef:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
   cf7f4:	48 85 c9             	test   %rcx,%rcx
   cf7f7:	0f 84 64 01 00 00    	je     cf961 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1e1>
   cf7fd:	48 89 54 24 60       	mov    %rdx,0x60(%rsp)
   cf802:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cf807:	48 8b 04 24          	mov    (%rsp),%rax
   cf80b:	48 83 c0 08          	add    $0x8,%rax
   cf80f:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cf814:	4c 8d bc 24 98 00 00 	lea    0x98(%rsp),%r15
   cf81b:	00 
   cf81c:	48 83 c3 40          	add    $0x40,%rbx
   cf820:	48 8d ac 24 00 01 00 	lea    0x100(%rsp),%rbp
   cf827:	00 
   cf828:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cf82f:	00 
   cf830:	48 8b 43 c8          	mov    -0x38(%rbx),%rax
   cf834:	48 83 f8 ff          	cmp    $0xffffffffffffffff,%rax
   cf838:	0f 84 19 01 00 00    	je     cf957 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1d7>
   cf83e:	44 0f b7 73 c0       	movzwl -0x40(%rbx),%r14d
   cf843:	48 89 84 24 90 00 00 	mov    %rax,0x90(%rsp)
   cf84a:	00 
   cf84b:	0f 10 43 d0          	movups -0x30(%rbx),%xmm0
   cf84f:	0f 10 4b e0          	movups -0x20(%rbx),%xmm1
   cf853:	0f 10 53 f0          	movups -0x10(%rbx),%xmm2
   cf857:	41 0f 11 57 20       	movups %xmm2,0x20(%r15)
   cf85c:	41 0f 11 4f 10       	movups %xmm1,0x10(%r15)
   cf861:	41 0f 11 07          	movups %xmm0,(%r15)
   cf865:	48 8b 04 24          	mov    (%rsp),%rax
   cf869:	66 44 03 30          	add    (%rax),%r14w
   cf86d:	0f 80 7b 06 00 00    	jo     cfeee <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x76e>
   cf873:	48 8b 84 24 c0 00 00 	mov    0xc0(%rsp),%rax
   cf87a:	00 
   cf87b:	48 89 84 24 30 01 00 	mov    %rax,0x130(%rsp)
   cf882:	00 
   cf883:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cf88a:	00 
   cf88b:	48 8b 8c 24 98 00 00 	mov    0x98(%rsp),%rcx
   cf892:	00 
   cf893:	0f 10 84 24 a0 00 00 	movups 0xa0(%rsp),%xmm0
   cf89a:	00 
   cf89b:	0f 10 8c 24 b0 00 00 	movups 0xb0(%rsp),%xmm1
   cf8a2:	00 
   cf8a3:	0f 29 8c 24 20 01 00 	movaps %xmm1,0x120(%rsp)
   cf8aa:	00 
   cf8ab:	0f 29 84 24 10 01 00 	movaps %xmm0,0x110(%rsp)
   cf8b2:	00 
   cf8b3:	48 89 84 24 00 01 00 	mov    %rax,0x100(%rsp)
   cf8ba:	00 
   cf8bb:	48 89 8c 24 08 01 00 	mov    %rcx,0x108(%rsp)
   cf8c2:	00 
   cf8c3:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cf8ca:	00 
   cf8cb:	48 89 ee             	mov    %rbp,%rsi
   cf8ce:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
   cf8d3:	e8 88 de ff ff       	call   cd760 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cf8d8:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cf8df:	00 ff 
   cf8e1:	0f 84 08 05 00 00    	je     cfdef <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf8e7:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cf8ee:	00 
   cf8ef:	48 89 84 24 d0 01 00 	mov    %rax,0x1d0(%rsp)
   cf8f6:	00 
   cf8f7:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cf8fe:	00 
   cf8ff:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cf906:	00 
   cf907:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cf90e:	00 
   cf90f:	0f 29 94 24 c0 01 00 	movaps %xmm2,0x1c0(%rsp)
   cf916:	00 
   cf917:	0f 29 8c 24 b0 01 00 	movaps %xmm1,0x1b0(%rsp)
   cf91e:	00 
   cf91f:	0f 29 84 24 a0 01 00 	movaps %xmm0,0x1a0(%rsp)
   cf926:	00 
   cf927:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf92c:	44 89 f6             	mov    %r14d,%esi
   cf92f:	48 8d 94 24 a0 01 00 	lea    0x1a0(%rsp),%rdx
   cf936:	00 
   cf937:	e8 b4 f2 ff ff       	call   cebf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cf93c:	84 c0                	test   %al,%al
   cf93e:	0f 84 ab 04 00 00    	je     cfdef <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf944:	48 83 c3 40          	add    $0x40,%rbx
   cf948:	49 83 c5 c0          	add    $0xffffffffffffffc0,%r13
   cf94c:	0f 85 de fe ff ff    	jne    cf830 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0xb0>
   cf952:	48 8b 5c 24 60       	mov    0x60(%rsp),%rbx
   cf957:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf95c:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cf961:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf966:	e8 45 e1 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf96b:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cf970:	49 89 46 10          	mov    %rax,0x10(%r14)
   cf974:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cf979:	41 0f 11 06          	movups %xmm0,(%r14)
   cf97d:	e9 03 05 00 00       	jmp    cfe85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cf982:	48 8b 0e             	mov    (%rsi),%rcx
   cf985:	48 8b 46 08          	mov    0x8(%rsi),%rax
   cf989:	48 8b 56 10          	mov    0x10(%rsi),%rdx
   cf98d:	48 89 d6             	mov    %rdx,%rsi
   cf990:	48 c1 e6 06          	shl    $0x6,%rsi
   cf994:	48 01 c6             	add    %rax,%rsi
   cf997:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   cf99c:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf9a1:	48 89 4c 24 28       	mov    %rcx,0x28(%rsp)
   cf9a6:	48 89 b4 24 88 00 00 	mov    %rsi,0x88(%rsp)
   cf9ad:	00 
   cf9ae:	48 89 74 24 30       	mov    %rsi,0x30(%rsp)
   cf9b3:	48 85 d2             	test   %rdx,%rdx
   cf9b6:	0f 84 d1 03 00 00    	je     cfd8d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x60d>
   cf9bc:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cf9c3:	00 
   cf9c4:	4c 89 e1             	mov    %r12,%rcx
   cf9c7:	48 c1 e1 06          	shl    $0x6,%rcx
   cf9cb:	48 03 0c 24          	add    (%rsp),%rcx
   cf9cf:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
   cf9d4:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cf9d9:	48 8d 50 40          	lea    0x40(%rax),%rdx
   cf9dd:	48 8b 48 08          	mov    0x8(%rax),%rcx
   cf9e1:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
   cf9e5:	0f 84 9d 03 00 00    	je     cfd88 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x608>
   cf9eb:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
   cf9f0:	0f b7 10             	movzwl (%rax),%edx
   cf9f3:	66 89 54 24 38       	mov    %dx,0x38(%rsp)
   cf9f8:	48 89 8c 24 90 00 00 	mov    %rcx,0x90(%rsp)
   cf9ff:	00 
   cfa00:	0f 10 40 10          	movups 0x10(%rax),%xmm0
   cfa04:	0f 10 48 20          	movups 0x20(%rax),%xmm1
   cfa08:	0f 10 50 30          	movups 0x30(%rax),%xmm2
   cfa0c:	0f 11 56 20          	movups %xmm2,0x20(%rsi)
   cfa10:	0f 11 4e 10          	movups %xmm1,0x10(%rsi)
   cfa14:	0f 11 06             	movups %xmm0,(%rsi)
   cfa17:	4d 85 e4             	test   %r12,%r12
   cfa1a:	0f 84 30 03 00 00    	je     cfd50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cfa20:	4c 8b b4 24 c0 00 00 	mov    0xc0(%rsp),%r14
   cfa27:	00 
   cfa28:	48 8b 9c 24 a0 00 00 	mov    0xa0(%rsp),%rbx
   cfa2f:	00 
   cfa30:	48 89 dd             	mov    %rbx,%rbp
   cfa33:	48 c1 e5 04          	shl    $0x4,%rbp
   cfa37:	48 89 d8             	mov    %rbx,%rax
   cfa3a:	48 c1 e8 3c          	shr    $0x3c,%rax
   cfa3e:	0f 94 c0             	sete   %al
   cfa41:	48 b9 f9 ff ff ff ff 	movabs $0x7ffffffffffffff9,%rcx
   cfa48:	ff ff 7f 
   cfa4b:	48 39 cd             	cmp    %rcx,%rbp
   cfa4e:	0f 92 c1             	setb   %cl
   cfa51:	84 c8                	test   %cl,%al
   cfa53:	0f 84 72 03 00 00    	je     cfdcb <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x64b>
   cfa59:	48 85 ed             	test   %rbp,%rbp
   cfa5c:	0f 84 ab 01 00 00    	je     cfc0d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x48d>
   cfa62:	48 8b 84 24 98 00 00 	mov    0x98(%rsp),%rax
   cfa69:	00 
   cfa6a:	48 89 84 24 40 01 00 	mov    %rax,0x140(%rsp)
   cfa71:	00 
   cfa72:	4c 8b 3c 24          	mov    (%rsp),%r15
   cfa76:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cfa7d:	00 00 00 
   cfa80:	45 0f b7 2f          	movzwl (%r15),%r13d
   cfa84:	66 44 03 6c 24 38    	add    0x38(%rsp),%r13w
   cfa8a:	0f 80 7c 03 00 00    	jo     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfa90:	f0 49 ff 06          	lock incq (%r14)
   cfa94:	0f 8e 53 03 00 00    	jle    cfded <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfa9a:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cfa9f:	ff 15 d3 f9 09 00    	call   *0x9f9d3(%rip)        # 16f478 <_DYNAMIC+0x338>
   cfaa5:	be 08 00 00 00       	mov    $0x8,%esi
   cfaaa:	48 89 ef             	mov    %rbp,%rdi
   cfaad:	ff 15 cd f9 09 00    	call   *0x9f9cd(%rip)        # 16f480 <_DYNAMIC+0x340>
   cfab3:	48 85 c0             	test   %rax,%rax
   cfab6:	0f 84 55 04 00 00    	je     cff11 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x791>
   cfabc:	48 85 db             	test   %rbx,%rbx
   cfabf:	74 48                	je     cfb09 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cfac1:	31 c9                	xor    %ecx,%ecx
   cfac3:	48 89 da             	mov    %rbx,%rdx
   cfac6:	48 8b b4 24 40 01 00 	mov    0x140(%rsp),%rsi
   cfacd:	00 
   cface:	66 90                	xchg   %ax,%ax
   cfad0:	48 39 cd             	cmp    %rcx,%rbp
   cfad3:	74 34                	je     cfb09 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cfad5:	48 8b 3e             	mov    (%rsi),%rdi
   cfad8:	4c 8b 46 08          	mov    0x8(%rsi),%r8
   cfadc:	f0 48 ff 07          	lock incq (%rdi)
   cfae0:	0f 8e 07 03 00 00    	jle    cfded <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfae6:	f0 49 ff 00          	lock incq (%r8)
   cfaea:	0f 8e fd 02 00 00    	jle    cfded <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfaf0:	48 ff ca             	dec    %rdx
   cfaf3:	48 83 c6 10          	add    $0x10,%rsi
   cfaf7:	48 89 3c 08          	mov    %rdi,(%rax,%rcx,1)
   cfafb:	4c 89 44 08 08       	mov    %r8,0x8(%rax,%rcx,1)
   cfb00:	48 83 c1 10          	add    $0x10,%rcx
   cfb04:	48 85 d2             	test   %rdx,%rdx
   cfb07:	75 c7                	jne    cfad0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x350>
   cfb09:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
   cfb0e:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
   cfb13:	48 89 9c 24 80 00 00 	mov    %rbx,0x80(%rsp)
   cfb1a:	00 
   cfb1b:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cfb22:	00 
   cfb23:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cfb2a:	00 
   cfb2b:	e8 70 32 04 00       	call   112da0 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cfb30:	49 8d 57 08          	lea    0x8(%r15),%rdx
   cfb34:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cfb3b:	00 
   cfb3c:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cfb41:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cfb48:	00 
   cfb49:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cfb50:	00 
   cfb51:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cfb58:	00 
   cfb59:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cfb60:	00 
   cfb61:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cfb68:	00 
   cfb69:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cfb6d:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cfb74:	00 
   cfb75:	0f 11 01             	movups %xmm0,(%rcx)
   cfb78:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cfb7f:	00 
   cfb80:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cfb87:	00 
   cfb88:	e8 d3 db ff ff       	call   cd760 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cfb8d:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cfb94:	00 ff 
   cfb96:	0f 84 70 02 00 00    	je     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfb9c:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cfba3:	00 
   cfba4:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cfbab:	00 
   cfbac:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cfbb3:	00 
   cfbb4:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cfbbb:	00 
   cfbbc:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cfbc3:	00 
   cfbc4:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cfbcb:	00 
   cfbcc:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cfbd3:	00 
   cfbd4:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cfbdb:	00 
   cfbdc:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cfbe1:	44 89 ee             	mov    %r13d,%esi
   cfbe4:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cfbeb:	00 
   cfbec:	e8 ff ef ff ff       	call   cebf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cfbf1:	84 c0                	test   %al,%al
   cfbf3:	0f 84 13 02 00 00    	je     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfbf9:	49 83 c7 40          	add    $0x40,%r15
   cfbfd:	4c 3b 7c 24 60       	cmp    0x60(%rsp),%r15
   cfc02:	0f 85 78 fe ff ff    	jne    cfa80 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x300>
   cfc08:	e9 43 01 00 00       	jmp    cfd50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cfc0d:	4c 8b 2c 24          	mov    (%rsp),%r13
   cfc11:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cfc18:	0f 1f 84 00 00 00 00 
   cfc1f:	00 
   cfc20:	41 0f b7 5d 00       	movzwl 0x0(%r13),%ebx
   cfc25:	66 03 5c 24 38       	add    0x38(%rsp),%bx
   cfc2a:	0f 80 dc 01 00 00    	jo     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfc30:	f0 49 ff 06          	lock incq (%r14)
   cfc34:	0f 8e b3 01 00 00    	jle    cfded <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfc3a:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cfc3f:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
   cfc46:	00 00 
   cfc48:	48 c7 44 24 78 08 00 	movq   $0x8,0x78(%rsp)
   cfc4f:	00 00 
   cfc51:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
   cfc58:	00 00 00 00 00 
   cfc5d:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cfc64:	00 
   cfc65:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cfc6c:	00 
   cfc6d:	e8 2e 31 04 00       	call   112da0 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cfc72:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cfc79:	00 
   cfc7a:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cfc7f:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cfc86:	00 
   cfc87:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cfc8e:	00 
   cfc8f:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cfc96:	00 
   cfc97:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cfc9e:	00 
   cfc9f:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cfca6:	00 
   cfca7:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cfcab:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cfcb2:	00 
   cfcb3:	0f 11 01             	movups %xmm0,(%rcx)
   cfcb6:	49 83 c5 08          	add    $0x8,%r13
   cfcba:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cfcc1:	00 
   cfcc2:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cfcc9:	00 
   cfcca:	4c 89 ea             	mov    %r13,%rdx
   cfccd:	e8 8e da ff ff       	call   cd760 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cfcd2:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cfcd9:	00 ff 
   cfcdb:	0f 84 2b 01 00 00    	je     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfce1:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cfce8:	00 
   cfce9:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cfcf0:	00 
   cfcf1:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cfcf8:	00 
   cfcf9:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cfd00:	00 
   cfd01:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cfd08:	00 
   cfd09:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cfd10:	00 
   cfd11:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cfd18:	00 
   cfd19:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cfd20:	00 
   cfd21:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cfd26:	89 de                	mov    %ebx,%esi
   cfd28:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cfd2f:	00 
   cfd30:	e8 bb ee ff ff       	call   cebf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cfd35:	84 c0                	test   %al,%al
   cfd37:	0f 84 cf 00 00 00    	je     cfe0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfd3d:	49 83 c5 38          	add    $0x38,%r13
   cfd41:	4c 3b 6c 24 60       	cmp    0x60(%rsp),%r13
   cfd46:	0f 85 d4 fe ff ff    	jne    cfc20 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x4a0>
   cfd4c:	0f 1f 40 00          	nopl   0x0(%rax)
   cfd50:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cfd57:	00 
   cfd58:	e8 a3 10 fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cfd5d:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cfd62:	48 89 c8             	mov    %rcx,%rax
   cfd65:	48 3b 8c 24 88 00 00 	cmp    0x88(%rsp),%rcx
   cfd6c:	00 
   cfd6d:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cfd72:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cfd79:	00 
   cfd7a:	0f 85 59 fc ff ff    	jne    cf9d9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x259>
   cfd80:	48 8b 94 24 88 00 00 	mov    0x88(%rsp),%rdx
   cfd87:	00 
   cfd88:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
   cfd8d:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfd92:	e8 19 dd 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfd97:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cfd9c:	49 89 46 10          	mov    %rax,0x10(%r14)
   cfda0:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cfda5:	41 0f 11 06          	movups %xmm0,(%r14)
   cfda9:	49 ff c4             	inc    %r12
   cfdac:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfdb0:	49 83 fc 01          	cmp    $0x1,%r12
   cfdb4:	0f 84 ed 00 00 00    	je     cfea7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cfdba:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfdbe:	49 ff cc             	dec    %r12
   cfdc1:	e8 5a 1d 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfdc6:	48 89 df             	mov    %rbx,%rdi
   cfdc9:	eb e5                	jmp    cfdb0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x630>
   cfdcb:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfdd0:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfdd5:	48 8b 04 24          	mov    (%rsp),%rax
   cfdd9:	0f b7 4c 24 38       	movzwl 0x38(%rsp),%ecx
   cfdde:	66 03 08             	add    (%rax),%cx
   cfde1:	70 33                	jo     cfe16 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x696>
   cfde3:	f0 49 ff 06          	lock incq (%r14)
   cfde7:	0f 8f ec 00 00 00    	jg     cfed9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x759>
   cfded:	0f 0b                	ud2
   cfdef:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cfdf4:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfdf9:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfe00:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfe05:	e8 a6 dc 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfe0a:	eb 2d                	jmp    cfe39 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6b9>
   cfe0c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfe11:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfe16:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfe1b:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfe22:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cfe29:	00 
   cfe2a:	e8 d1 0f fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cfe2f:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfe34:	e8 77 dc 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfe39:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfe3e:	4c 8b 74 24 50       	mov    0x50(%rsp),%r14
   cfe43:	49 ff c6             	inc    %r14
   cfe46:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cfe4d:	00 00 00 
   cfe50:	49 83 fe 01          	cmp    $0x1,%r14
   cfe54:	74 11                	je     cfe67 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6e7>
   cfe56:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfe5a:	49 ff ce             	dec    %r14
   cfe5d:	e8 be 1c 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfe62:	48 89 df             	mov    %rbx,%rdi
   cfe65:	eb e9                	jmp    cfe50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6d0>
   cfe67:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cfe6c:	48 85 f6             	test   %rsi,%rsi
   cfe6f:	74 14                	je     cfe85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cfe71:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfe76:	48 c1 e6 06          	shl    $0x6,%rsi
   cfe7a:	ba 08 00 00 00       	mov    $0x8,%edx
   cfe7f:	ff 15 7b f5 09 00    	call   *0x9f57b(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cfe85:	49 ff c4             	inc    %r12
   cfe88:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfe8c:	0f 1f 40 00          	nopl   0x0(%rax)
   cfe90:	49 83 fc 01          	cmp    $0x1,%r12
   cfe94:	74 11                	je     cfea7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cfe96:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfe9a:	49 ff cc             	dec    %r12
   cfe9d:	e8 7e 1c 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfea2:	48 89 df             	mov    %rbx,%rdi
   cfea5:	eb e9                	jmp    cfe90 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x710>
   cfea7:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   cfeac:	48 8b 30             	mov    (%rax),%rsi
   cfeaf:	48 85 f6             	test   %rsi,%rsi
   cfeb2:	74 13                	je     cfec7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x747>
   cfeb4:	48 c1 e6 06          	shl    $0x6,%rsi
   cfeb8:	ba 08 00 00 00       	mov    $0x8,%edx
   cfebd:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfec1:	ff 15 39 f5 09 00    	call   *0x9f539(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cfec7:	48 81 c4 d8 01 00 00 	add    $0x1d8,%rsp
   cfece:	5b                   	pop    %rbx
   cfecf:	41 5c                	pop    %r12
   cfed1:	41 5d                	pop    %r13
   cfed3:	41 5e                	pop    %r14
   cfed5:	41 5f                	pop    %r15
   cfed7:	5d                   	pop    %rbp
   cfed8:	c3                   	ret
   cfed9:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cfede:	31 ff                	xor    %edi,%edi
   cfee0:	48 89 ee             	mov    %rbp,%rsi
   cfee3:	ff 15 87 f5 09 00    	call   *0x9f587(%rip)        # 16f470 <_DYNAMIC+0x330>
   cfee9:	e9 ff fe ff ff       	jmp    cfded <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfeee:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cfef3:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfef8:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfeff:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cff06:	00 
   cff07:	e8 f4 0e fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cff0c:	e9 ef fe ff ff       	jmp    cfe00 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x680>
   cff11:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cff16:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cff1b:	bf 08 00 00 00       	mov    $0x8,%edi
   cff20:	eb be                	jmp    cfee0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x760>
   cff22:	49 89 c7             	mov    %rax,%r15
   cff25:	eb 5c                	jmp    cff83 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x803>
   cff27:	49 89 c7             	mov    %rax,%r15
   cff2a:	e9 56 01 00 00       	jmp    d0085 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cff2f:	49 89 c7             	mov    %rax,%r15
   cff32:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cff37:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cff3c:	e9 44 01 00 00       	jmp    d0085 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cff41:	eb 08                	jmp    cff4b <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7cb>
   cff43:	49 89 c7             	mov    %rax,%r15
   cff46:	e9 44 01 00 00       	jmp    d008f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cff4b:	49 89 c7             	mov    %rax,%r15
   cff4e:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cff53:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cff58:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cff5d:	e8 6e fb fd ff       	call   afad0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cff62:	e9 00 01 00 00       	jmp    d0067 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8e7>
   cff67:	eb 00                	jmp    cff69 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7e9>
   cff69:	49 89 c7             	mov    %rax,%r15
   cff6c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cff71:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cff76:	e9 fd 00 00 00       	jmp    d0078 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   cff7b:	49 89 c7             	mov    %rax,%r15
   cff7e:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cff83:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cff88:	e8 23 db 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cff8d:	e9 fd 00 00 00       	jmp    d008f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cff92:	49 89 c7             	mov    %rax,%r15
   cff95:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
   cff9c:	00 00 00 00 
   cffa0:	49 ff cc             	dec    %r12
   cffa3:	0f 84 8d 00 00 00    	je     d0036 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   cffa9:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   cffad:	48 89 df             	mov    %rbx,%rdi
   cffb0:	e8 6b 1b 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cffb5:	4c 89 f3             	mov    %r14,%rbx
   cffb8:	eb e6                	jmp    cffa0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x820>
   cffba:	ff 15 28 f4 09 00    	call   *0x9f428(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   cffc0:	49 89 c7             	mov    %rax,%r15
   cffc3:	66 66 66 66 2e 0f 1f 	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cffca:	84 00 00 00 00 00 
   cffd0:	49 ff ce             	dec    %r14
   cffd3:	74 11                	je     cffe6 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x866>
   cffd5:	4c 8d 63 40          	lea    0x40(%rbx),%r12
   cffd9:	48 89 df             	mov    %rbx,%rdi
   cffdc:	e8 3f 1b 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cffe1:	4c 89 e3             	mov    %r12,%rbx
   cffe4:	eb ea                	jmp    cffd0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x850>
   cffe6:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cffeb:	48 85 f6             	test   %rsi,%rsi
   cffee:	0f 84 a5 00 00 00    	je     d0099 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   cfff4:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfff9:	48 c1 e6 06          	shl    $0x6,%rsi
   cfffd:	ba 08 00 00 00       	mov    $0x8,%edx
   d0002:	ff 15 f8 f3 09 00    	call   *0x9f3f8(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   d0008:	e9 8c 00 00 00       	jmp    d0099 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   d000d:	ff 15 d5 f3 09 00    	call   *0x9f3d5(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   d0013:	49 89 c7             	mov    %rax,%r15
   d0016:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   d001d:	00 00 00 
   d0020:	49 ff cc             	dec    %r12
   d0023:	74 11                	je     d0036 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   d0025:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   d0029:	48 89 df             	mov    %rbx,%rdi
   d002c:	e8 ef 1a 04 00       	call   111b20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   d0031:	4c 89 f3             	mov    %r14,%rbx
   d0034:	eb ea                	jmp    d0020 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8a0>
   d0036:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   d003b:	48 8b 30             	mov    (%rax),%rsi
   d003e:	48 85 f6             	test   %rsi,%rsi
   d0041:	74 60                	je     d00a3 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x923>
   d0043:	48 c1 e6 06          	shl    $0x6,%rsi
   d0047:	ba 08 00 00 00       	mov    $0x8,%edx
   d004c:	48 8b 3c 24          	mov    (%rsp),%rdi
   d0050:	ff 15 aa f3 09 00    	call   *0x9f3aa(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   d0056:	4c 89 ff             	mov    %r15,%rdi
   d0059:	e8 22 90 09 00       	call   169080 <_Unwind_Resume@plt>
   d005e:	ff 15 84 f3 09 00    	call   *0x9f384(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   d0064:	49 89 c7             	mov    %rax,%r15
   d0067:	f0 49 ff 0e          	lock decq (%r14)
   d006b:	75 0b                	jne    d0078 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   d006d:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   d0072:	ff 15 68 f3 09 00    	call   *0x9f368(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   d0078:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   d007f:	00 
   d0080:	e8 7b 0d fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   d0085:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   d008a:	e8 21 da 03 00       	call   10dab0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   d008f:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   d0094:	e8 07 fb fd ff       	call   afba0 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   d0099:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   d009e:	e8 fd fa fd ff       	call   afba0 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   d00a3:	4c 89 ff             	mov    %r15,%rdi
   d00a6:	e8 d5 8f 09 00       	call   169080 <_Unwind_Resume@plt>
   d00ab:	ff 15 37 f3 09 00    	call   *0x9f337(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   d00b1:	ff 15 31 f3 09 00    	call   *0x9f331(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
